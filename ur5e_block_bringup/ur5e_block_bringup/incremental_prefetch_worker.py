"""Prepare one future block from private snapshots, without a ROS node.

The subprocess has no action/service clients. It invokes the local C++ planner
and the same interpolation, finger-grid and wrist-sweep certification routines
as foreground planning. The caller must compare the sealed context, committed
scene and measured start before installing this speculative result.
"""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from moveit_msgs.msg import PlanningScene, RobotState, RobotTrajectory
from rclpy.serialization import deserialize_message, serialize_message
import yaml

from .air_task import AirTask, ARM, HAND, MAX_FILE_BYTES, atomic_write, digest
from .collision_worker import CollisionWorker
from .incremental_state import POLICY, recipe_layout, validate_state
from .physical_motion import incremental_motion_profile, SURFACE_CONTACT_POLICY
from .physical_stack import (PhysicalStackTask, JOINTS, GRASP_POLICY,
                             artifact_path, arm_execution_profile,
                             scene_fingerprint, trajectory_duration, validate_recipe)
from . import physical_replay as replay


class Cancelled(RuntimeError):
    pass


class OfflineCertifier:
    """Only pure physical-task methods; deliberately not a Node subclass."""
    normalize_arm = PhysicalStackTask.normalize_arm
    motion_samples = PhysicalStackTask.motion_samples
    tracking_motion_samples = PhysicalStackTask.tracking_motion_samples
    certify_motion = PhysicalStackTask.certify_motion
    trajectory_fingerprint = staticmethod(PhysicalStackTask.trajectory_fingerprint)
    seal_recipe = PhysicalStackTask.seal_recipe
    stretch = staticmethod(AirTask.stretch)

    def __init__(self, request, check_stop):
        self.path = Path(request['output_dir'])/'recipe.json'
        self.bounds = request['bounds']
        self.joint_velocity_limits = request['joint_velocity_limits']
        self.joint_acceleration_limits = request['joint_acceleration_limits']
        self.check_stop = check_stop

    def read_message(self, name, kind):
        return deserialize_message(artifact_path(self.path.parent, name).read_bytes(), kind)


def regular_input(value):
    path = Path(value)
    if not path.is_absolute() or path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError('Offline planning requires bounded regular absolute input files')
    return path


def validate_request(request):
    directory = Path(request['output_dir'])
    if not directory.is_absolute() or directory.is_symlink() or not directory.is_dir():
        raise ValueError('Offline output directory must be a private absolute directory')
    for name in ('recipe.json', 'recipe.json.meta.json', 'result.json'):
        if (directory/name).exists() or (directory/name).is_symlink():
            raise ValueError('Offline output directory already contains a result')
    validate_state(request['expected_state'])
    params, context = request['planner_parameters'], request['context']
    for parameter, key in (('robot_description', 'model_sha256'),
                           ('robot_description_semantic', 'semantic_sha256')):
        if not isinstance(params.get(parameter), str) or digest(params[parameter].encode()) != context.get(key):
            raise ValueError('Offline model differs from its execution context')
    if (context.get('recipe_policy') != POLICY
            or context.get('replay_policy') != replay.POLICY
            or context.get('grasp_policy') != GRASP_POLICY
            or context.get('arm_execution_profile') != arm_execution_profile()
            or context.get('incremental_motion_profile') != incremental_motion_profile()
            or context.get('surface_contact_policy') != SURFACE_CONTACT_POLICY
            or context.get('finger_joint_bounds') != list(request['bounds'][HAND])):
        raise ValueError('Offline execution policy differs from the current policy')
    for name in JOINTS:
        bounds = request['bounds'][name]
        if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in bounds)
                or bounds[0] >= bounds[1]):
            raise ValueError('Invalid offline joint bounds')
    for key in ('joint_velocity_limits', 'joint_acceleration_limits'):
        for name in ARM:
            value = request[key][name]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid offline joint timing limit')
    scene = deserialize_message(regular_input(request['scene_file']).read_bytes(), PlanningScene)
    state = deserialize_message(regular_input(request['state_file']).read_bytes(), RobotState)
    if scene.is_diff or scene.robot_state.is_diff or state.is_diff:
        raise ValueError('Offline planning requires full snapshots')
    if (len(state.joint_state.name) != len(state.joint_state.position)
            or len(set(state.joint_state.name)) != len(state.joint_state.name)):
        raise ValueError('Offline start has malformed joints')
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    if any(name not in values or not math.isfinite(values[name])
           or not request['bounds'][name][0] <= values[name] <= request['bounds'][name][1] for name in JOINTS):
        raise ValueError('Offline start has missing or out-of-bounds joints')
    if state.attached_collision_objects or scene.robot_state.attached_collision_objects:
        raise ValueError('Next-block planning requires the predicted empty-gripper state')
    return scene, [values[name] for name in JOINTS]


def planner_parameters(request, raw):
    params = copy.deepcopy(request['planner_parameters'])
    params.update(stack_mode='plan', empty_hand_test=False, empty_xy_scan=False,
                  plate_alignment_test=False, physical_stack_task=True, incremental_stack=True,
                  recipe_manifest_file=str(raw), display_trajectory_file='',
                  offline_planning_scene_file=request['scene_file'],
                  offline_robot_state_file=request['state_file'])
    for name in ('trajectory_file', 'selection_file', 'calibration_file', 'physical_profile',
                 'workspace_clear', 'allow_mock_hardware', 'execution_mode', 'preparation_id'):
        params.pop(name, None)
    return {name: value for name, value in params.items() if value != []}


def run_planner(request, raw, check_stop):
    parameters_file = raw.with_suffix('.params.yaml')
    parameters_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': planner_parameters(request, raw)}}))
    executable = Path(get_package_prefix('ur5e_a3_grid'))/'lib/ur5e_a3_grid/stack_blocks'
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID='46', ROS_LOCALHOST_ONLY='1', RCUTILS_LOGGING_USE_STDOUT='0')
    timeout = request.get('planner_timeout_seconds', 300.)
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 1 <= timeout <= 600:
        raise ValueError('Invalid offline planner timeout')
    # Inherit the worker's process group, so parent cancellation also reaches
    # native work even if Python is interrupted while starting a child.
    with (raw.parent/'planner.log').open('wb') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(parameters_file)],
                                   stdout=log, stderr=subprocess.STDOUT, env=environment)
        try:
            deadline = time.monotonic()+timeout
            while process.poll() is None:
                check_stop()
                if time.monotonic() > deadline:
                    raise TimeoutError('Offline next-block planner timed out')
                time.sleep(.02)
            check_stop()
            if process.returncode != 0:
                raise RuntimeError('Offline next-block planner failed; see '+str(raw.parent/'planner.log'))
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.)


def prepare(request, check_stop=lambda: None, planner=run_planner, worker_factory=CollisionWorker):
    began = time.monotonic()
    initial, initial_joints = validate_request(request)
    context = copy.deepcopy(request['context'])
    task = OfflineCertifier(request, check_stop)
    raw = task.path.parent/'raw.json'
    check_stop()
    planner(request, raw, check_stop)
    search_seconds = time.monotonic()-began
    result = json.loads(regular_input(str(raw)).read_text())
    if result.get('incremental_status') in ('no_remaining_blocks', 'no_feasible_direct_pick'):
        state = validate_state(result['state'])
        if state != request['expected_state']:
            raise ValueError('Offline terminal state differs from predicted committed state')
        if result['incremental_status'] == 'no_remaining_blocks' and state['remaining_sources']:
            raise ValueError('Offline planner omitted remaining blocks')
        return dict(status='finished', reason=result['incremental_status'], state=state,
                    search_seconds=search_seconds)
    recipe = result
    recipe.update(arm_execution_profile=arm_execution_profile(), grasp_policy=GRASP_POLICY)
    recipe_layout(recipe)
    steps = validate_recipe(recipe)
    if recipe['incremental_state_before'] != request['expected_state']:
        raise ValueError('Offline recipe starts in a different block layout')
    if any(abs(a-b) > 1e-8 for a, b in zip(initial_joints, recipe['initial_joints'])):
        raise ValueError('Offline recipe starts at different robot joints')
    initial_hash = scene_fingerprint(initial)
    if scene_fingerprint(task.read_message(recipe['initial_scene_file'], PlanningScene)) != initial_hash:
        raise ValueError('Offline planner changed its input scene')
    samples, duration, previous = 0, 0., initial_hash
    validation_began = time.monotonic()
    params = request['planner_parameters']
    worker = worker_factory({'robot_description': params['robot_description'],
                             'robot_description_semantic': params['robot_description_semantic'],
                             'bounded_surface_contacts': True}, task.path.parent, check_stop)
    try:
        for step, holding in replay.holding_steps(steps):
            check_stop()
            before = task.read_message(step['scene_before_file'], PlanningScene)
            after = task.read_message(step['scene_after_file'], PlanningScene)
            before_hash, after_hash = scene_fingerprint(before), scene_fingerprint(after)
            if before_hash != previous:
                raise ValueError('Offline collision scene chain is discontinuous')
            step.update(scene_before_sha256=before_hash, scene_after_sha256=after_hash)
            if step['kind'] != 'scene':
                if before_hash != after_hash:
                    raise ValueError('Offline motion changes collision geometry')
                trajectory = task.read_message(step['file'], RobotTrajectory)
                if step['kind'] == 'arm':
                    trajectory, step['metrics'] = task.normalize_arm(trajectory, step)
                    name = step['file']+'.validated.cdr'
                    atomic_write(task.path.parent/name, serialize_message(trajectory))
                    step['file'] = name
                samples += task.certify_motion(worker, before, step, trajectory, holding)
                step['duration'] = trajectory_duration(trajectory)
                duration += step['duration']
            previous = after_hash
    finally:
        worker.close()
    check_stop()
    validation_seconds = time.monotonic()-validation_began
    recipe.update(path_kind='physical_stack', validated_samples=samples, duration=duration,
                  planning_timing=dict(search_seconds=search_seconds, validation_seconds=validation_seconds,
                                       collision_backend='local_cpp', total_seconds=time.monotonic()-began))
    names = {recipe['initial_scene_file'], recipe['initial_state_file']}
    names.update(step[key] for step in steps for key in replay.ARTIFACT_FIELDS if step.get(key))
    payloads = {name: artifact_path(task.path.parent, name).read_bytes() for name in names}
    metadata, encoded = task.seal_recipe(recipe, payloads, context, initial_hash)
    atomic_write(task.path, encoded)
    atomic_write(task.path.with_name(task.path.name+'.meta.json'), (json.dumps(metadata, indent=2)+'\n').encode())
    # Re-open the exact files the parent will consume, including all checksums.
    loaded_metadata, loaded_recipe, _ = replay.read_package(task.path.parent)
    replay.check_certificates(loaded_recipe, loaded_metadata)
    return dict(status='ready', package_directory=str(task.path.parent), plan_id=metadata['plan_id'],
                validated_samples=samples, duration=duration, **recipe['planning_timing'])


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: python3 -m ur5e_block_bringup.incremental_prefetch_worker request.json')
    request = json.loads(regular_input(sys.argv[1]).read_text())
    cancelled = False

    def signal_stop(_signum, _frame):
        nonlocal cancelled
        cancelled = True

    def check_stop():
        if cancelled:
            raise Cancelled('Offline next-block preparation cancelled')

    signal.signal(signal.SIGINT, signal_stop)
    signal.signal(signal.SIGTERM, signal_stop)
    try:
        # Execution feedback and watchdog callbacks retain scheduling priority.
        os.nice(5)
        result = prepare(request, check_stop)
        check_stop()
        atomic_write(Path(request['output_dir'])/'result.json', (json.dumps(result, sort_keys=True)+'\n').encode())
        print('STACK_PREFETCH_WORKER '+json.dumps(result, sort_keys=True), flush=True)
    except Exception as error:
        print('STACK_PREFETCH_WORKER_ERROR '+str(error), file=sys.stderr, flush=True)
        raise SystemExit(2 if isinstance(error, Cancelled) else 1)


if __name__ == '__main__':
    main()
