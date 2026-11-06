"""Offline preparation must preserve the foreground collision contract."""
import copy
import json
from types import SimpleNamespace

import pytest
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState, RobotTrajectory
from rclpy.serialization import serialize_message
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5e_block_bringup import incremental_prefetch_worker as worker
from ur5e_block_bringup import physical_replay as replay
from ur5e_block_bringup import physical_stack as physical
from ur5e_block_bringup.incremental_state import POLICY
from ur5e_block_bringup.physical_motion import incremental_motion_profile, SURFACE_CONTACT_POLICY
from test_incremental_stack import one_block


class Checker:
    calls = []
    closed = False

    def __init__(self, parameters, directory, poll):
        self.poll = poll
        assert parameters['bounded_surface_contacts'] is True

    def validate(self, scene, samples):
        self.poll()
        self.calls.append(samples)
        return len(samples)

    def close(self):
        type(self).closed = True


def fixture(tmp_path):
    recipe = one_block()
    model, semantic = '<robot name="test"/>', '<robot name="test"/>'
    task = SimpleNamespace(model=model, semantic=semantic, bounds={name: [-7., 7.] for name in physical.JOINTS},
                           recipe_policy=lambda: POLICY)
    task.bounds[physical.HAND] = [0., .8]
    context = physical.PhysicalStackTask.package_context(task, {'selection': 'test'}, 'mock')
    context.update(incremental_motion_profile=incremental_motion_profile(), surface_contact_policy=SURFACE_CONTACT_POLICY)
    scene = PlanningScene()
    state = RobotState()
    state.joint_state.name = physical.JOINTS
    state.joint_state.position = list(map(float, recipe['initial_joints']))
    scene.robot_state = state
    (tmp_path/'input_scene.cdr').write_bytes(serialize_message(scene))
    (tmp_path/'input_state.cdr').write_bytes(serialize_message(state))
    request = dict(output_dir=str(tmp_path), planner_parameters=dict(robot_description=model, robot_description_semantic=semantic),
                   context=context, bounds=task.bounds,
                   joint_velocity_limits={name: 3. for name in physical.ARM},
                   joint_acceleration_limits={name: 1. for name in physical.ARM},
                   scene_file=str(tmp_path/'input_scene.cdr'), state_file=str(tmp_path/'input_state.cdr'),
                   expected_state=recipe['incremental_state_before'])

    def planner(_request, raw, poll):
        poll()
        for name in ('initial.cdr', 'before.cdr', 'after.cdr'):
            (tmp_path/name).write_bytes(serialize_message(scene))
        (tmp_path/'state.cdr').write_bytes(serialize_message(state))
        for step in recipe['steps']:
            trajectory = RobotTrajectory()
            arm = step['kind'] == 'arm'
            trajectory.joint_trajectory.joint_names = physical.ARM if arm else [physical.HAND]
            for index, joints in enumerate((step['start_joints'], step['end_joints'])):
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in (joints[:6] if arm else joints[-1:])]
                point.velocities = [0.] * len(point.positions)
                point.accelerations = [0.] * len(point.positions)
                point.time_from_start.sec = index
                trajectory.joint_trajectory.points.append(point)
            (tmp_path/step['file']).write_bytes(serialize_message(trajectory))
        raw.write_text(json.dumps(recipe))
    Checker.calls = []
    Checker.closed = False
    return request, recipe, planner


def test_offline_package_uses_identical_samples_and_valid_certificates(tmp_path):
    request, original, planner = fixture(tmp_path)
    result = worker.prepare(request, planner=planner, worker_factory=Checker)
    metadata, recipe, payloads = replay.read_package(tmp_path)
    replay.check_certificates(recipe, metadata)
    assert result['status'] == 'ready'
    assert recipe['incremental_state_before'] == request['expected_state']
    assert metadata['input_sha256'] == request['context']['input_sha256']
    assert result['validated_samples'] == sum(len(samples) for samples in Checker.calls)
    assert Checker.closed
    assert len([s for s in recipe['steps'] if s['kind'] == 'arm']) == 4
    facade = worker.OfflineCertifier(request, lambda: None)
    for step, holding in replay.holding_steps(recipe['steps']):
        if step['kind'] == 'scene':
            continue
        trajectory = facade.read_message(step['file'], RobotTrajectory)
        fingers = replay.finger_shapes(step, holding, request['bounds'][physical.HAND])
        path = facade.tracking_motion_samples(step, trajectory) if step['kind'] == 'arm' else [(0., step['start_joints'])]
        assert step['collision_certificate']['checked_samples'] == len(path)*len(fingers)
    assert payloads


@pytest.mark.parametrize('change', ['context', 'state', 'joints', 'scene'])
def test_predicted_context_or_start_mismatch_never_publishes_package(tmp_path, change):
    request, recipe, planner = fixture(tmp_path)
    if change == 'context':
        request['context']['model_sha256'] = 'bad'
    elif change == 'state':
        recipe['incremental_state_before']['remaining_sources'].reverse()
        request['expected_state'] = copy.deepcopy(recipe['incremental_state_before'])
        request['expected_state']['remaining_sources'].reverse()
    elif change == 'joints':
        # Change all joints continuously so recipe structure is still valid.
        recipe['initial_joints'][0] += .01
        for step in recipe['steps']:
            step['start_joints'][0] += .01
            step['end_joints'][0] += .01
    else:
        previous = planner
        def planner(req, raw, poll):
            previous(req, raw, poll)
            scene = PlanningScene()
            item = CollisionObject()
            item.id = 'foreign-change'
            scene.world.collision_objects = [item]
            (tmp_path/'initial.cdr').write_bytes(serialize_message(scene))
    with pytest.raises(ValueError):
        worker.prepare(request, planner=planner, worker_factory=Checker)
    assert not (tmp_path/'recipe.json.meta.json').exists()


def test_partial_collision_acknowledgement_never_seals(tmp_path):
    request, _, planner = fixture(tmp_path)
    class Incomplete(Checker):
        def validate(self, scene, samples):
            return len(samples)-1
    with pytest.raises(RuntimeError, match='complete finger grid'):
        worker.prepare(request, planner=planner, worker_factory=Incomplete)
    assert Incomplete.closed
    assert not (tmp_path/'recipe.json.meta.json').exists()


def test_cancel_during_validation_closes_checker_without_publishing(tmp_path):
    request, _, planner = fixture(tmp_path)
    stopped = False
    def stop():
        if stopped:
            raise worker.Cancelled('test stop')
    class StopAfterSample(Checker):
        def validate(self, scene, samples):
            nonlocal stopped
            stopped = True
            return len(samples)
    with pytest.raises(worker.Cancelled):
        worker.prepare(request, stop, planner=planner, worker_factory=StopAfterSample)
    assert StopAfterSample.closed
    assert not (tmp_path/'recipe.json.meta.json').exists()


def test_planner_is_forced_to_offline_plan_without_execution_settings(tmp_path):
    request, _, _ = fixture(tmp_path)
    request['planner_parameters'].update(stack_mode='execute', workspace_clear=True, execution_mode='continuous',
                                         offline_planning_scene_file='/wrong', empty_hand_test=True)
    parameters = worker.planner_parameters(request, tmp_path/'raw.json')
    assert parameters['stack_mode'] == 'plan'
    assert parameters['physical_stack_task'] is parameters['incremental_stack'] is True
    assert parameters['empty_hand_test'] is False
    assert parameters['offline_planning_scene_file'] == request['scene_file']
    assert parameters['offline_robot_state_file'] == request['state_file']
    assert 'workspace_clear' not in parameters and 'execution_mode' not in parameters


@pytest.mark.parametrize('reason', ['no_remaining_blocks', 'no_feasible_direct_pick'])
def test_terminal_result_must_preserve_predicted_state(tmp_path, reason):
    request, _, _ = fixture(tmp_path)
    result = dict(incremental_status=reason, state=request['expected_state'])
    def planner(req, raw, poll):
        raw.write_text(json.dumps(result))
    if reason == 'no_remaining_blocks':
        with pytest.raises(ValueError, match='omitted remaining'):
            worker.prepare(request, planner=planner)
    else:
        assert worker.prepare(request, planner=planner)['status'] == 'finished'
    assert not (tmp_path/'recipe.json').exists()
