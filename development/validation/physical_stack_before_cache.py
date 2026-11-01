"""Reviewed physical pick/place recipes, separate from the mock-only executor.

The C++ planner exports every arm, finger and scene step. This node validates
the complete recipe, restores the initial scene, seals the reviewed files, and
dispatches only operator-approved goals to the existing hardware controllers.
"""
import copy
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.signals import SignalHandlerOptions
from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import DynamicJointState, JointTolerance
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, PlanningScene, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID as GoalUUID
import yaml

from .air_task import (AirTask, ARM, HAND, CONTROLLER, MAX_FILE_BYTES,
                       START_TOLERANCE, atomic_write, digest, file_digest)
from .air_scene import scene_fingerprint as strict_scene_fingerprint
from .air_trajectory import sample_trajectory, trajectory_metrics
from .physical_held import clear_held_fault, read_held_fault, write_held_fault
from rosidl_runtime_py.convert import message_to_ordereddict

JOINTS = ARM+[HAND]
HAND_TARGETS = {'open': .4, 'grasp': .73, 'release_first': .70, 'release_clearance': .60}
ARM_SPEEDS = {'approach': .05, 'lift': .20, 'retreat': .20, 'transfer': .20}
REVIEW_TTL = 3600.
HAND_START_TOLERANCE = .015
SCENE_HASH_POLICY = 'physical_normalized_pose_round12_v1'


def stop_with_reason(task, code, **details):
    """Keep the first cause, so cancellation does not hide the failed monitor."""
    if getattr(task, 'stop_reason', None) is None:
        task.stop_reason = dict(code=code, **details)
        print('STACK_STOP_REASON '+json.dumps(task.stop_reason, sort_keys=True, default=str), flush=True)
    task.stopped = True


def normalized_collision_data(value):
    """Normalize Eigen/ROS pose round trips to sub-nanometre precision.

    This affects comparison only. The original collision shapes and trajectory
    samples are sent unchanged to MoveIt. No millimetre-scale scene difference
    is ignored, and q/-q quaternion representations describe the same rotation.
    """
    if hasattr(value, 'get_fields_and_field_types'):
        value = message_to_ordereddict(value)
    if isinstance(value, dict):
        if set(value) == {'x', 'y', 'z', 'w'} and all(isinstance(v, (int, float)) for v in value.values()):
            q = [float(value[k]) for k in ('x', 'y', 'z', 'w')]
            norm = math.sqrt(sum(v*v for v in q))
            if math.isfinite(norm) and norm > 1e-15:
                q = [round(v/norm, 12) for v in q]
                significant = next((v for v in reversed(q) if abs(v) > 1e-12), 1.)
                if significant < 0.:
                    q = [-v for v in q]
                return dict(zip(('x', 'y', 'z', 'w'), q))
        return {key: normalized_collision_data({k: v for k, v in item.items() if k != 'stamp'}
                                               if key == 'header' and isinstance(item, dict) else item)
                for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [normalized_collision_data(item) for item in value]
    if isinstance(value, float):
        return round(value, 12)
    return value


def scene_fingerprint(scene):
    return strict_scene_fingerprint(normalized_collision_data(scene))


def reconcile_snapshot(live, desired):
    """Build explicit owned-object diffs; MoveIt's full-message path may merge.

    Acknowledged removals precede additions, including the world object which
    MoveIt automatically creates when an attached body is detached. Unchanged
    foreign objects are retained, and changed foreign geometry aborts the write.
    """
    live_world = {item.id: item for item in live.world.collision_objects}
    goal_world = {item.id: item for item in desired.world.collision_objects}
    live_attached = {item.object.id: item for item in live.robot_state.attached_collision_objects}
    goal_attached = {item.object.id: item for item in desired.robot_state.attached_collision_objects}
    def key(value):
        return normalized_collision_data(value)
    if key(live.world.octomap) != key(desired.world.octomap):
        raise RuntimeError('External collision geometry changed; occupancy data will not be overwritten')
    for current, goal in ((live_world, goal_world), (live_attached, goal_attached)):
        foreign_current = {name: key(item) for name, item in current.items() if not name.startswith('ur5e_stack/')}
        foreign_goal = {name: key(item) for name, item in goal.items() if not name.startswith('ur5e_stack/')}
        if foreign_current != foreign_goal:
            raise RuntimeError('External collision geometry changed; it will not be deleted or overwritten')
    remove_attached = {name for name, item in live_attached.items() if name.startswith('ur5e_stack/')
                       and (name not in goal_attached or key(item) != key(goal_attached[name]))}
    remove_world = {name for name, item in live_world.items() if name.startswith('ur5e_stack/')
                    and (name not in goal_world or key(item) != key(goal_world[name]))} | remove_attached
    remove = PlanningScene()
    remove.is_diff = remove.robot_state.is_diff = True
    for name in sorted(remove_attached):
        body = copy.deepcopy(live_attached[name])
        body.object.operation = CollisionObject.REMOVE
        remove.robot_state.attached_collision_objects.append(body)
    for name in sorted(remove_world):
        item = CollisionObject()
        item.id, item.operation = name, CollisionObject.REMOVE
        remove.world.collision_objects.append(item)
    add = copy.deepcopy(desired)
    add.is_diff = add.robot_state.is_diff = True
    add.world.collision_objects = [copy.deepcopy(item) for name, item in goal_world.items()
                                   if name.startswith('ur5e_stack/') and
                                   (name not in live_world or name in remove_world or key(item) != key(live_world[name]))]
    add.robot_state.attached_collision_objects = [copy.deepcopy(item) for name, item in goal_attached.items()
                                                  if name.startswith('ur5e_stack/') and
                                                  (name not in live_attached or name in remove_attached or
                                                   key(item) != key(live_attached[name]))]
    return remove, add


def artifact_path(directory, name):
    """Recipes can refer only to regular sibling files, never arbitrary paths."""
    if not isinstance(name, str) or not name or Path(name).name != name or name in ('.', '..'):
        raise ValueError('Invalid recipe artifact basename')
    path = directory/name
    if path.is_symlink() or not path.is_file() or path.resolve().parent != directory.resolve():
        raise ValueError('Recipe artifact must be a regular sibling file')
    if path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError('Recipe artifact exceeds the size limit')
    return path


def finite_state(values):
    return (isinstance(values, list) and len(values) == 7 and
            all(not isinstance(q, bool) and isinstance(q, (int, float)) and math.isfinite(q) for q in values))


def validate_recipe(recipe):
    """Enforce the physical sequence before any ROS service or action is used."""
    if (not isinstance(recipe, dict) or type(recipe.get('schema')) is not int or recipe['schema'] != 1
            or recipe.get('joint_names') != JOINTS):
        raise ValueError('Unsupported physical recipe schema/joints')
    steps = recipe.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= 10000:
        raise ValueError('A physical recipe must contain bounded steps')
    if not finite_state(recipe.get('initial_joints')):
        raise ValueError('Missing measured physical recipe start')
    if (type(recipe.get('target_cell')) is not int or not 1 <= recipe['target_cell'] <= 100
            or type(recipe.get('moves')) is not int or recipe['moves'] < 1
            or type(recipe.get('foundation_layers')) is not int or recipe['foundation_layers'] != 1
            or type(recipe.get('layers')) is not int):
        raise ValueError('Physical stacking requires one existing foundation block and integer counts')
    expected = recipe['initial_joints']
    carrying = attached = False
    release_stage = 0
    grasps = clears = 0
    for index, step in enumerate(steps, 1):
        if (not isinstance(step, dict) or type(step.get('index')) is not int
                or step['index'] != index or step.get('joint_names') != JOINTS):
            raise ValueError('Physical recipe steps are out of order')
        if not isinstance(step.get('label'), str) or not 0 < len(step['label']) <= 200:
            raise ValueError('Invalid physical recipe label')
        start, end = step.get('start_joints'), step.get('end_joints')
        if not finite_state(start) or not finite_state(end) or max(abs(a-b) for a, b in zip(start, expected)) > 1e-7:
            raise ValueError('Physical recipe joint state is discontinuous')
        kind, phase = step.get('kind'), step.get('phase')
        if kind == 'arm':
            velocity, acceleration = step.get('velocity_scaling'), step.get('acceleration_scaling')
            if (phase not in ARM_SPEEDS or isinstance(velocity, bool) or not isinstance(velocity, (int, float))
                    or not math.isfinite(velocity) or abs(velocity-ARM_SPEEDS[phase]) > 1e-9):
                raise ValueError('Arm phase does not use the reviewed 0.05/0.20 speed scaling')
            if (isinstance(acceleration, bool) or not isinstance(acceleration, (int, float))
                    or not math.isfinite(acceleration) or abs(acceleration-.05) > 1e-9):
                raise ValueError('Physical arm acceleration scaling must remain 0.05')
            if release_stage or (carrying and not attached):
                raise ValueError('Arm motion precedes attachment or the completed 0.70 -> 0.60 release')
            if abs(start[-1]-end[-1]) > 1e-9:
                raise ValueError('Arm motion must hold the gripper shape')
        elif kind == 'hand':
            target = step.get('hand_target')
            if (phase not in HAND_TARGETS or isinstance(target, bool) or not isinstance(target, (int, float))
                    or not math.isfinite(target) or abs(target-HAND_TARGETS[phase]) > 1e-9):
                raise ValueError('Unexpected physical gripper target')
            if max(abs(a-b) for a, b in zip(start[:6], end[:6])) > 1e-9:
                raise ValueError('A gripper step must hold all six arm joints')
            if abs(end[-1]-HAND_TARGETS[phase]) > 1e-8:
                raise ValueError('Gripper endpoint differs from its reviewed target')
            if phase == 'grasp':
                if carrying or release_stage:
                    raise ValueError('Cannot grasp while already carrying a block')
                carrying = True
                grasps += 1
            elif phase == 'release_first':
                if not carrying or not attached or release_stage:
                    raise ValueError('First release must follow an attached grasp')
                release_stage = 1
            elif phase == 'release_clearance':
                if release_stage != 2 or carrying or attached:
                    raise ValueError('Clearance release must follow the stationary place event')
                release_stage = 0
                clears += 1
            elif carrying or release_stage:
                raise ValueError('Initial opening cannot release a carried block')
        elif kind == 'scene':
            if max(abs(a-b) for a, b in zip(start, end)) > 1e-9:
                raise ValueError('Scene events must not move robot joints')
            if phase == 'attach':
                if not carrying or attached or release_stage:
                    raise ValueError('Attachment must follow a successful grasp')
                attached = True
            elif phase == 'place':
                if not carrying or not attached or release_stage != 1:
                    raise ValueError('Place must follow the 0.70 release at the stationary endpoint')
                carrying = attached = False
                release_stage = 2
            elif phase != 'scene':
                raise ValueError('Unknown physical scene event')
        else:
            raise ValueError('Unknown physical recipe step kind')
        expected = end
    if carrying or attached or release_stage or grasps != clears or grasps < 1:
        raise ValueError('Physical recipe ends without a complete place/release sequence')
    if recipe.get('moves') != grasps or recipe.get('layers') != grasps+recipe.get('foundation_layers', 0):
        raise ValueError('Physical recipe block/layer counts differ')
    return steps


def trajectory_duration(trajectory):
    if not trajectory.joint_trajectory.points:
        raise ValueError('Empty physical trajectory')
    stamp = trajectory.joint_trajectory.points[-1].time_from_start
    return stamp.sec+stamp.nanosec*1e-9


class PhysicalStackTask(AirTask):
    def __init__(self):
        super().__init__()
        if self.has_parameter('recipe_manifest_file') and self.param('recipe_manifest_file'):
            self.path = Path(self.param('recipe_manifest_file')).expanduser()
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.meta_path = self.path.with_name(self.path.name+'.meta.json')
        self.path_kind = 'physical_stack'
        self.stack_lock = None
        self.pending_identity = None
        self.sensor, self.sensor_sequence, self.sensor_received = {}, -1, -math.inf
        self.sensor_subscription = self.create_subscription(
            DynamicJointState, '/gripper/dynamic_joint_states', self.sensor_feedback, qos_profile_sensor_data)
        self.carrying = False
        self.holding_reference = None
        self.release_stage = 0
        self.execution_started = False
        self.active_recipe = None
        self.grasp_attempt = None
        self.scene_request_pending = False
        self.stop_reason = None
        self.monitor_clients = {}
        self.joint_velocity_limits = {}
        tree = ET.fromstring(self.model)
        self.joint_acceleration_limits = {}
        for name in ARM:
            self.joint_velocity_limits[name] = float(tree.find(f"joint[@name='{name}']/limit").get('velocity'))
            prefix = 'robot_description_planning.joint_limits.'+name+'.'
            if self.has_parameter(prefix+'has_acceleration_limits') and self.param(prefix+'has_acceleration_limits'):
                self.joint_acceleration_limits[name] = float(self.param(prefix+'max_acceleration'))
            else:
                # MoveIt's time parameterization uses 1rad/s² when no joint
                # acceleration bound is supplied. Retain that conservative base.
                self.joint_acceleration_limits[name] = 1.

    def sensor_feedback(self, message):
        if len(message.joint_names) != len(message.interface_values) or HAND not in message.joint_names:
            return
        item = message.interface_values[message.joint_names.index(HAND)]
        if len(item.interface_names) != len(item.values) or len(set(item.interface_names)) != len(item.interface_names):
            return
        record = dict(zip(item.interface_names, item.values))
        required = ('position', 'raw_position', 'grip_state', 'force_percent', 'communication_age', 'sample_sequence')
        if any(name not in record or not math.isfinite(record[name]) for name in required):
            return
        q, raw, grip, force, age, sequence = [record[name] for name in required]
        if (not 0 <= q <= .93 or not 0 <= raw <= 1000 or raw != int(raw)
                or grip not in (0, 1, 2, 3) or not 20 <= force <= 100 or not 0 <= age <= .5
                or sequence != int(sequence) or sequence <= self.sensor_sequence
                or abs(q-.93*(1000-raw)/1000) > .0015):
            return
        self.sensor, self.sensor_sequence = record, int(sequence)
        self.sensor_received = time.monotonic()-age

    def sensor_fresh(self, after=None):
        return self.mock or (time.monotonic()-self.sensor_received < .5 and
                             (after is None or self.sensor_sequence > after))

    def holding_ok(self):
        if not self.carrying or self.release_stage:
            return True
        if not self.sensor_fresh() or (not self.mock and self.sensor.get('grip_state') != 2):
            return False
        return self.holding_reference is not None and abs(self.values.get(HAND, math.inf)-self.holding_reference) <= .003

    def measured_stop(self, seconds=4.):
        began = None
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            now = time.monotonic()
            quiet = self.fresh() and self.sensor_fresh() and all(name in self.velocities and now-self.velocities[name][1] < .5
                        and abs(self.velocities[name][0]) < .01 for name in JOINTS)
            began = now if quiet and began is None else began if quiet else None
            if began is not None and now-began >= .3:
                return True
        return False

    def acquire_stack_lock(self):
        if self.stack_lock is None:
            domain = os.environ.get('ROS_DOMAIN_ID', '0')
            self.stack_lock = os.open(f'/tmp/ur5e_stack_{domain}.lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
            fcntl.flock(self.stack_lock, fcntl.LOCK_EX|fcntl.LOCK_NB)

    def check_stop(self):
        if self.stopped:
            if self.goal is None:
                self.preserve_contact_after_stop()
            reason = getattr(self, 'stop_reason', None)
            raise RuntimeError('Physical pick/place stopped; current holding state preserved'+
                               ('; '+json.dumps(reason, sort_keys=True) if reason else ''))

    def request_stop(self, request, reply):
        stop_with_reason(self, 'OPERATOR_STOP')
        return super().request_stop(request, reply)

    def apply_snapshot(self, message, cancel=True, full=True):
        """Apply reviewed geometry while retaining freshly measured joint state."""
        if getattr(self, 'scene_request_pending', False):
            raise RuntimeError('An earlier planning-scene write has no acknowledgement; further writes/execution are blocked')
        if full:
            live = self.read_scene_for_reconciliation(cancel=cancel)
            remove, snapshot = reconcile_snapshot(live, message)
            if remove.robot_state.attached_collision_objects:
                detach = PlanningScene()
                detach.is_diff = detach.robot_state.is_diff = True
                detach.robot_state.attached_collision_objects = remove.robot_state.attached_collision_objects
                self.apply_scene_message(detach, cancel=False)
            if remove.world.collision_objects:
                cleanup = PlanningScene()
                cleanup.is_diff = cleanup.robot_state.is_diff = True
                cleanup.world.collision_objects = remove.world.collision_objects
                self.apply_scene_message(cleanup, cancel=False)
            # Stop must not leave the transaction after removing a carried
            # body but before adding the reviewed replacement geometry.
            self.apply_scene_message(snapshot, cancel=False)
            if not cancel:
                fault = read_held_fault()
                if fault is not None and str(fault.get('cause', '')).startswith('SCENE_RPC_PENDING:'):
                    clear_held_fault()
            if cancel:
                self.check_stop()
        else:
            self.apply_scene_message(message, cancel=cancel)

    def read_scene_for_reconciliation(self, cancel=True):
        request = GetPlanningScene.Request()
        request.components.components = 1023
        client = self.create_client(GetPlanningScene, '/get_planning_scene')
        try:
            self.spin_until(client.service_is_ready, 10., cancel=cancel)
            future = client.call_async(request)
            self.spin_until(future.done, 10., cancel=cancel)
            reply = future.result()
            if reply is None:
                raise RuntimeError('Unable to read the current scene for an exact geometry transaction')
            return reply.scene
        finally:
            self.destroy_client(client)

    def apply_scene_message(self, message, cancel=True):
        snapshot = copy.deepcopy(message)
        snapshot.is_diff = snapshot.robot_state.is_diff = True
        snapshot.robot_state.joint_state.name = JOINTS
        snapshot.robot_state.joint_state.position = [self.values.get(name, 0.) for name in JOINTS]
        snapshot.robot_state.joint_state.velocity = []
        snapshot.robot_state.joint_state.effort = []
        client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        try:
            self.spin_until(client.service_is_ready, 10., cancel=cancel)
            request = ApplyPlanningScene.Request()
            request.scene = snapshot
            future = client.call_async(request)
            # Once a scene mutation is sent, Stop may latch but must not
            # overtake its acknowledgement with a later restoration request.
            # Keep this process/lock alive if the server stalls; no physical
            # goal can be dispatched while this write remains unresolved.
            self.scene_request_pending = True
            began, announced = time.monotonic(), False
            while not future.done():
                rclpy.spin_once(self, timeout_sec=.02)
                if time.monotonic()-began > 15. and not announced:
                    write_held_fault('SCENE_RPC_PENDING: waiting for a planning-scene write acknowledgement', digest(self.model.encode()))
                    print('STACK_SCENE_RESTORE_PENDING waiting for earlier scene write acknowledgement', flush=True)
                    announced = True
            try:
                reply = future.result()
            except Exception:
                write_held_fault('SCENE_RPC_PENDING: planning-scene write transport failed; inspect/restart the planning server', digest(self.model.encode()))
                raise RuntimeError('Planning-scene write acknowledgement was lost; execution is blocked')
            if reply is None:
                write_held_fault('SCENE_RPC_PENDING: no planning-scene write acknowledgement was returned', digest(self.model.encode()))
                raise RuntimeError('Planning-scene write acknowledgement is missing; execution is blocked')
            self.scene_request_pending = False
            if not reply.success:
                raise RuntimeError('Unable to apply/restore the reviewed planning geometry')
            if cancel:
                self.check_stop()
        finally:
            self.destroy_client(client)

    def read_message(self, name, kind):
        return deserialize_message(artifact_path(self.path.parent, name).read_bytes(), kind)

    def planner(self, output):
        output.unlink(missing_ok=True)
        params = {name: parameter.value for name, parameter in self.get_parameters_by_prefix('').items()}
        params.update(stack_mode='plan', empty_hand_test=False, empty_xy_scan=False,
                      plate_alignment_test=False, physical_stack_task=True,
                      recipe_manifest_file=str(output), display_trajectory_file='')
        for name in ('trajectory_file', 'selection_file', 'calibration_file', 'physical_profile',
                     'workspace_clear', 'allow_mock_hardware', 'execution_mode', 'preparation_id'):
            params.pop(name, None)
        params = {name: value for name, value in params.items() if value != []}
        params_file = output.with_name(output.name+'.planner.yaml')
        params_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        self.child = subprocess.Popen(
            ['ros2', 'run', 'ur5e_a3_grid', 'stack_blocks', '--ros-args', '--params-file', str(params_file),
             '-r', '/ur5e_a3_stack/stop:=/ur5e_physical_stack/planner_stop'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        lines = queue.Queue()
        def read():
            for line in self.child.stdout:
                lines.put(line)
        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        interrupted, problem = False, ''
        try:
            while self.child.poll() is None or reader.is_alive() or not lines.empty():
                rclpy.spin_once(self, timeout_sec=.02)
                if self.stopped and self.child.poll() is None and not interrupted:
                    self.child.send_signal(signal.SIGINT)
                    interrupted = True
                while not lines.empty():
                    line = lines.get_nowait()
                    if 'STACK_ERROR ' in line:
                        problem = line.split('STACK_ERROR ', 1)[1].strip()
                    print(line, end='', flush=True)
            self.check_stop()
            if self.child.returncode or not output.is_file():
                raise RuntimeError(problem or 'The complete physical recipe could not be planned')
        finally:
            if self.child.poll() is None:
                self.child.send_signal(signal.SIGINT)
                self.child.wait(timeout=15.)
            self.child = None
            params_file.unlink(missing_ok=True)

    def normalize_arm(self, trajectory, step):
        """Bound the exact quintic by each joint's requested phase scaling."""
        result = copy.deepcopy(trajectory)
        result.joint_trajectory.header.stamp.sec = 0
        result.joint_trajectory.header.stamp.nanosec = 0
        for point in result.joint_trajectory.points:
            point.effort = []
        metrics = trajectory_metrics(result)
        velocity = step['velocity_scaling']
        acceleration = step['acceleration_scaling']
        scale = max([1.] + [value/(self.joint_velocity_limits[name]*velocity)
                     for name, value in zip(ARM, metrics['joint_velocity_maxima'])] +
                    [math.sqrt(value/(self.joint_acceleration_limits[name]*acceleration))
                     for name, value in zip(ARM, metrics['joint_acceleration_maxima'])])
        if scale > 1.:
            self.stretch(result, scale*1.0001)
        metrics = trajectory_metrics(result)
        for point in (result.joint_trajectory.points[0], result.joint_trajectory.points[-1]):
            if max(abs(value) for value in point.velocities+point.accelerations) > 1e-8:
                raise ValueError('Every physical arm step must start/end at rest')
        return result, metrics

    def motion_samples(self, step, trajectory, actual_hand=None):
        if step['kind'] == 'arm':
            hand = step['start_joints'][-1] if actual_hand is None else actual_hand
            samples = [(stamp, list(values)+[hand]) for stamp, values in
                       sample_trajectory(trajectory, max_dt=.05, max_joint_step=.01)]
        else:
            names = list(trajectory.joint_trajectory.joint_names)
            if names != [HAND] or len(trajectory.joint_trajectory.points) < 2:
                raise ValueError('A finger step must contain one gripper joint')
            points = trajectory.joint_trajectory.points
            if (abs(points[0].positions[0]-step['start_joints'][-1]) > 1e-8 or
                    abs(points[-1].positions[0]-step['hand_target']) > 1e-8):
                raise ValueError('Finger trajectory differs from reviewed endpoints')
            start, end = points[0].positions[0], points[-1].positions[0]
            count = max(1, math.ceil(abs(end-start)/.005))
            samples = [(trajectory_duration(trajectory)*i/count,
                        step['start_joints'][:6]+[start+(end-start)*i/count]) for i in range(count+1)]
        for _, values in samples:
            if any(not self.bounds[name][0]-1e-9 <= q <= self.bounds[name][1]+1e-9
                   for name, q in zip(JOINTS, values)):
                raise ValueError('Physical recipe exceeds a URDF joint limit')
        if max(abs(a-b) for a, b in zip(samples[0][1][:6], step['start_joints'][:6])) > 1e-7:
            raise ValueError('Physical trajectory start differs from reviewed state')
        if max(abs(a-b) for a, b in zip(samples[-1][1][:6], step['end_joints'][:6])) > 1e-7:
            raise ValueError('Physical trajectory endpoint differs from reviewed state')
        return samples

    def validate_samples(self, samples):
        # A bounded window avoids overflowing Humble's service queue while
        # checking every controller-interpolated sample and every finger sweep.
        client = self.create_client(GetStateValidity, '/check_state_validity')
        pending, sent, checked = [], 0, 0
        try:
            self.spin_until(client.service_is_ready)
            while sent < len(samples) or pending:
                self.check_stop()
                while sent < len(samples) and len(pending) < 8:
                    request = GetStateValidity.Request()
                    request.robot_state.is_diff = True
                    request.robot_state.joint_state.name = JOINTS
                    request.robot_state.joint_state.position = [float(value) for value in samples[sent][1]]
                    pending.append((time.monotonic(), client.call_async(request)))
                    sent += 1
                rclpy.spin_once(self, timeout_sec=.005)
                for _ in range(len(pending)):
                    began, future = pending.pop(0)
                    if not future.done():
                        if time.monotonic()-began > 10.:
                            raise TimeoutError('Physical recipe collision validation timed out')
                        pending.append((began, future))
                        continue
                    reply = future.result()
                    if reply is None or not reply.valid:
                        raise RuntimeError('집기·쌓기 경로의 충돌 또는 관절 한계 검사에 실패했습니다.')
                    checked += 1
        finally:
            for _, future in pending:
                client.remove_pending_request(future)
            self.destroy_client(client)
        return checked

    def plan(self):
        self.spin_until(self.fresh)
        self.check_model()
        kind = self.hardware(require_controller=True)
        self.gripper_runtime()
        previous_fault = read_held_fault()
        if previous_fault is not None and str(previous_fault.get('cause', '')).startswith('SCENE_RPC_PENDING:'):
            raise RuntimeError('이전 계획 장면 쓰기의 응답이 확인되지 않았습니다. 계획 서버 상태를 복구한 뒤 다시 시작하세요.')
        if not self.measured_stop() or not self.sensor_fresh() or (not self.mock and self.sensor.get('grip_state') != 1):
            raise RuntimeError('그리퍼가 비어 있고 정지한 센서 상태를 확인한 뒤 새 경로를 계획하세요. 잡고 있는 블록은 먼저 안전하게 내려놓으세요.')
        clear_held_fault()
        inputs = self.input_hashes()
        raw = self.path.with_name(self.path.name+'.raw.json')
        self.meta_path.unlink(missing_ok=True)
        self.planner(raw)
        self.acquire_stack_lock()
        recipe = json.loads(raw.read_text())
        steps = validate_recipe(recipe)
        initial = self.read_message(recipe['initial_scene_file'], PlanningScene)
        hashes, samples_checked, duration = {}, 0, 0.
        try:
            for step in steps:
                self.check_stop()
                before = self.read_message(step['scene_before_file'], PlanningScene)
                after = self.read_message(step['scene_after_file'], PlanningScene)
                self.apply_snapshot(before)
                step['scene_before_sha256'] = scene_fingerprint(self.scene())
                if step['kind'] != 'scene':
                    trajectory = self.read_message(step['file'], RobotTrajectory)
                    if step['kind'] == 'arm':
                        trajectory, metrics = self.normalize_arm(trajectory, step)
                        step['metrics'] = metrics
                        validated_name = step['file']+'.validated.cdr'
                        atomic_write(self.path.parent/validated_name, serialize_message(trajectory))
                        step['file'] = validated_name
                    samples_checked += self.validate_samples(self.motion_samples(step, trajectory))
                    step['duration'] = trajectory_duration(trajectory)
                    duration += step['duration']
                self.apply_snapshot(after)
                step['scene_after_sha256'] = scene_fingerprint(self.scene())
                if step['index'] % 10 == 0:
                    print(f'STACK_PHYSICAL_VALIDATING step={step["index"]} total={len(steps)} samples={samples_checked}', flush=True)
        finally:
            # Stop or validation failure cannot leave the live display in a
            # future attachment/tower scene. No robot commands are sent here.
            self.apply_snapshot(initial, cancel=False)
        self.check_stop()
        if inputs != self.input_hashes():
            raise RuntimeError('선택 배치나 보정 설정이 계획 중 바뀌었습니다. 다시 계획하세요.')
        initial_fingerprint = scene_fingerprint(self.scene())
        if initial_fingerprint != steps[0]['scene_before_sha256']:
            raise RuntimeError('Initial planning geometry was not restored exactly; recipe was not sealed')
        for previous, following in zip(steps, steps[1:]):
            if previous['scene_after_sha256'] != following['scene_before_sha256']:
                raise RuntimeError('Planned collision/attachment scene chain is discontinuous; recipe was not sealed')
        for step in steps:
            if step['kind'] != 'scene' and step['scene_before_sha256'] != step['scene_after_sha256']:
                raise RuntimeError('A motion unexpectedly changes collision geometry; recipe was not sealed')
        for name in [recipe['initial_scene_file'], recipe.get('initial_state_file')]+[
                value for step in steps for key, value in step.items()
                if key in ('file', 'scene_before_file', 'scene_after_file')]:
            if name:
                hashes[name] = file_digest(artifact_path(self.path.parent, name))
        recipe['path_kind'] = 'physical_stack'
        recipe['validated_samples'] = samples_checked
        recipe['duration'] = duration
        data = (json.dumps(recipe, sort_keys=True, allow_nan=False)+'\n').encode()
        metadata = dict(schema=1, path_kind='physical_stack', created_at=time.time(),
                        scene_hash_policy=SCENE_HASH_POLICY,
                        valid_for_seconds=REVIEW_TTL, hardware_class=kind,
                        input_sha256=inputs, model_sha256=digest(self.model.encode()),
                        semantic_sha256=digest(self.semantic.encode()),
                        recipe_sha256=digest(data), artifact_sha256=hashes,
                        scene_sha256=initial_fingerprint, validated_samples=samples_checked,
                        duration=duration, motion_steps=sum(s['kind'] != 'scene' for s in steps))
        metadata['plan_id'] = digest(json.dumps(metadata, sort_keys=True).encode())[:16]
        atomic_write(self.path, data)
        atomic_write(self.meta_path, (json.dumps(metadata, indent=2)+'\n').encode())
        self.display.publish(self.recipe_display(recipe, steps))
        print('STACK_PHYSICAL_PLAN '+json.dumps(metadata, sort_keys=True), flush=True)
        print('STACK_STATUS phase=PLAN_READY moved=0 total='+str(recipe['moves'])+' target='+str(recipe['target_cell'])+' layers='+str(recipe['foundation_layers']), flush=True)

    def recipe_display(self, recipe, steps):
        display = DisplayTrajectory()
        display.trajectory_start.joint_state.name = JOINTS
        start = steps[0]['start_joints'] if steps else recipe['initial_joints']
        display.trajectory_start.joint_state.position = [float(value) for value in start]
        combined = RobotTrajectory()
        combined.joint_trajectory.joint_names = JOINTS
        offset = 0
        for step in steps:
            if step['kind'] == 'scene':
                continue
            trajectory = self.read_message(step['file'], RobotTrajectory)
            for index, source in enumerate(trajectory.joint_trajectory.points):
                if combined.joint_trajectory.points and index == 0:
                    continue
                point = JointTrajectoryPoint()
                point.positions = [float(value) for value in
                                   (list(source.positions)+[step['start_joints'][-1]] if step['kind'] == 'arm'
                                    else step['start_joints'][:6]+list(source.positions))]
                point.velocities = list(source.velocities)+[0.] if step['kind'] == 'arm' else [0.]*6+list(source.velocities)
                point.accelerations = list(source.accelerations)+[0.] if step['kind'] == 'arm' else [0.]*6+list(source.accelerations)
                nanos = source.time_from_start.sec*10**9+source.time_from_start.nanosec+offset
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanos, 10**9)
                combined.joint_trajectory.points.append(point)
            offset += round(trajectory_duration(trajectory)*10**9)
        display.trajectory = [combined]
        return display

    def load_recipe(self):
        if read_held_fault() is not None:
            raise RuntimeError('이전 작업의 잡힌 블록 상태를 복구한 뒤 새 경로를 계획하세요.')
        if self.param('workspace_clear') is not True:
            raise RuntimeError('작업 공간과 블록 배치를 확인한 뒤 실제 집기·쌓기를 승인하세요.')
        if not self.path.is_file() or self.path.stat().st_size > MAX_FILE_BYTES:
            raise RuntimeError('먼저 실제 집기·쌓기 전체 경로를 계획하세요.')
        metadata = json.loads(self.meta_path.read_text())
        unsigned = {key: value for key, value in metadata.items() if key != 'plan_id'}
        if metadata.get('plan_id') != digest(json.dumps(unsigned, sort_keys=True).encode())[:16]:
            raise RuntimeError('Reviewed physical recipe metadata changed')
        if metadata.get('schema') != 1 or metadata.get('path_kind') != 'physical_stack':
            raise RuntimeError('이 파일은 검증된 실제 집기·쌓기 경로가 아닙니다.')
        if metadata.get('scene_hash_policy') != SCENE_HASH_POLICY:
            raise RuntimeError('충돌 장면 검증 형식이 바뀌었습니다. 실제 집기·쌓기 경로를 다시 계획하세요.')
        self.check_files(metadata)
        recipe = json.loads(self.path.read_text())
        validate_recipe(recipe)
        return metadata, recipe

    def check_files(self, metadata):
        if (metadata.get('input_sha256') != self.input_hashes()
                or metadata.get('model_sha256') != digest(self.model.encode())
                or metadata.get('semantic_sha256') != digest(self.semantic.encode())
                or metadata.get('recipe_sha256') != file_digest(self.path)
                or not 0 <= time.time()-metadata.get('created_at', 0) <= metadata.get('valid_for_seconds', 0)
                or metadata.get('valid_for_seconds') != REVIEW_TTL):
            raise RuntimeError('집기·쌓기 경로가 오래됐거나 배치·보정·파일이 바뀌었습니다. 다시 계획하세요.')
        artifacts = metadata.get('artifact_sha256')
        if not isinstance(artifacts, dict) or not artifacts:
            raise RuntimeError('Reviewed physical recipe artifacts are missing')
        for name, expected in artifacts.items():
            if file_digest(artifact_path(self.path.parent, name)) != expected:
                raise RuntimeError('Reviewed physical recipe artifact changed')

    def guard(self, metadata, step, expected_scene):
        self.check_stop()
        self.check_files(metadata)
        self.check_model()
        if self.hardware(require_controller=True) != metadata['hardware_class']:
            raise RuntimeError('Reviewed physical hardware changed')
        self.gripper_runtime()
        request = GetParameters.Request()
        request.names = ['allow_trajectory_execution']
        reply = self.call(GetParameters, '/move_group/get_parameters', request)
        if not reply.values or not reply.values[0].bool_value:
            raise RuntimeError('실물 연결을 allow_execution:=true로 시작하세요.')
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('주변 충돌 장면이 바뀌었습니다. 현재 자세에서 다시 계획하세요.')
        if not self.measured_stop() or not self.fresh() or not self.sensor_fresh():
            raise RuntimeError('팔·그리퍼의 최신 피드백과 정지를 확인하지 못했습니다.')
        if any(abs(self.values[name]-q) > START_TOLERANCE for name, q in zip(ARM, step['start_joints'])):
            raise RuntimeError('팔이 검토한 집기·쌓기 시작 위치에서 바뀌었습니다.')
        hand_reference = self.holding_reference if self.carrying and not self.release_stage else step['start_joints'][-1]
        hand_tolerance = .003 if self.carrying and not self.release_stage else HAND_START_TOLERANCE
        if abs(self.values[HAND]-hand_reference) > hand_tolerance:
            raise RuntimeError('그리퍼가 검토한 집기·쌓기 시작 위치에서 바뀌었습니다.')
        if not self.holding_ok():
            raise RuntimeError('그리퍼에서 블록 잡힘을 유지하는지 확인하지 못했습니다.')
        if (not self.mock and not self.carrying and self.release_stage == 0
                and self.sensor.get('grip_state') != 1):
            raise RuntimeError('빈 그리퍼 센서 상태를 확인한 뒤 새 집기·쌓기를 시작하세요.')
        self.check_stop()

    def live_arm_validation(self, step, trajectory, expected_scene):
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('Live scene changed before arm validation')
        self.validate_samples(self.motion_samples(step, trajectory, actual_hand=self.values[HAND]))
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('Live scene changed during arm validation')

    def execute(self):
        self.acquire_stack_lock()
        metadata, recipe = self.load_recipe()
        self.active_recipe = recipe
        steps = recipe['steps']
        expected_scene = metadata['scene_sha256']
        self.spin_until(self.fresh)
        self.guard(metadata, steps[0], expected_scene)
        motions = [step for step in steps if step['kind'] != 'scene']
        if self.execution_mode == 'continuous':
            display = self.recipe_display(recipe, steps)
            preview = display.trajectory[0]
            approval_metadata = dict(metadata, gripper_position=self.values[HAND])
            self.await_approval(display, approval_metadata, preview, 1, 1,
                                f'실제 집기·쌓기 전체 경로 · 블록 {recipe["moves"]}개',
                                context=lambda *_: self.guard(metadata, steps[0], expected_scene))
            self.guard(metadata, steps[0], expected_scene)
        motion_index, completed = 0, 0
        for step in steps:
            self.guard(metadata, step, expected_scene)
            if expected_scene != step['scene_before_sha256']:
                raise RuntimeError('Physical recipe scene sequence is discontinuous')
            if step['kind'] == 'scene':
                if step['phase'] == 'attach' and not self.holding_ok():
                    raise RuntimeError('블록 잡힘 확인 없이 팔을 들어 올릴 수 없습니다.')
                self.apply_snapshot(self.read_message(step['scene_after_file'], PlanningScene))
                expected_scene = scene_fingerprint(self.scene())
                if expected_scene != step['scene_after_sha256']:
                    raise RuntimeError('Reviewed physical attachment/place scene differs')
                # A delayed scene RPC is resolved only after its acknowledged
                # geometry is confirmed against this exact reviewed snapshot.
                fault = read_held_fault()
                if (not self.scene_request_pending and fault is not None
                        and str(fault.get('cause', '')).startswith('SCENE_RPC_PENDING:')):
                    clear_held_fault()
                if step['phase'] == 'place':
                    self.carrying = False
                    self.release_stage = 2
                    completed = int(step.get('completed', completed))
                    print(f'STACK_STATUS phase=PLACED moved={completed} total={recipe["moves"]} target={recipe["target_cell"]} layers={completed+recipe["foundation_layers"]}', flush=True)
                continue
            motion_index += 1
            trajectory = self.read_message(step['file'], RobotTrajectory)
            if step['kind'] == 'arm':
                self.live_arm_validation(step, trajectory, expected_scene)
            else:
                # Contact can stop the physical fingers before 0.73. A release
                # is checked from that actual contact position, not a fictitious
                # fully closed starting shape.
                actual_step = copy.deepcopy(step)
                actual_step['start_joints'][-1] = self.values[HAND]
                actual_trajectory = copy.deepcopy(trajectory)
                actual_trajectory.joint_trajectory.points[0].positions = [self.values[HAND]]
                self.validate_samples(self.motion_samples(actual_step, actual_trajectory))
            if self.execution_mode == 'step':
                display = self.recipe_display(recipe, [step])
                approval_metadata = dict(metadata, gripper_position=self.values[HAND])
                self.await_approval(display, approval_metadata, display.trajectory[0], motion_index, len(motions), step['label'],
                                    context=lambda *_: self.guard(metadata, step, expected_scene))
                self.guard(metadata, step, expected_scene)
            # An operator pause can change grasp/contact state. Revalidate the
            # actual steady finger geometry after approval as well as before it.
            if step['kind'] == 'arm':
                self.live_arm_validation(step, trajectory, expected_scene)
            self.guard(metadata, step, expected_scene)
            self.run_motion(step, trajectory, metadata, expected_scene,
                            motion_index if self.execution_mode == 'step' else 1,
                            len(motions) if self.execution_mode == 'step' else 1)
            if step['phase'] == 'grasp':
                self.carrying = True
                self.holding_reference = self.values[HAND]
            elif step['phase'] == 'release_first':
                self.release_stage = 1
            elif step['phase'] == 'release_clearance':
                self.release_stage = 0
                self.holding_reference = None
            expected_scene = step['scene_after_sha256']
        if self.execution_mode == 'continuous':
            self.phase('STACK_STEP_COMPLETED', index=1, total=1, label='실제 집기·쌓기 전체 경로')
        print(f'STACK_STATUS phase=DONE moved={recipe["moves"]} total={recipe["moves"]} target={recipe["target_cell"]} layers={recipe["layers"]}', flush=True)

    def run_motion(self, step, trajectory, metadata, scene_hash, index, total):
        hand = step['kind'] == 'hand'
        action = GripperCommand if hand else FollowJointTrajectory
        endpoint = self.gripper_runtime() if hand else f'/{CONTROLLER}/follow_joint_trajectory'
        client = ActionClient(self, action, endpoint)
        self.spin_until(client.server_is_ready)
        self.guard(metadata, step, scene_hash)
        goal = action.Goal()
        if hand:
            goal.command.position = float(step['hand_target'])
            goal.command.max_effort = 0.
        else:
            goal.trajectory = trajectory.joint_trajectory
            goal.path_tolerance = [JointTolerance(name=name, position=.005) for name in ARM]
            goal.goal_tolerance = [JointTolerance(name=name, position=.005) for name in ARM]
            goal.goal_time_tolerance = Duration(sec=5)
        sequence = self.sensor_sequence
        before = {name: self.values[name] for name in JOINTS}
        if step['phase'] == 'grasp':
            candidates = [item for item in self.active_recipe['steps']
                          if item['index'] > step['index'] and item['phase'] == 'attach']
            if not candidates:
                raise RuntimeError('Reviewed grasp is missing its following attachment')
            self.grasp_attempt = {'step': step, 'sensor_sequence': sequence, 'arm': [before[name] for name in ARM],
                                  'attachment_snapshot': self.read_message(candidates[0]['scene_after_file'], PlanningScene)}
        goal_id = GoalUUID(uuid=list(uuid.uuid4().bytes))
        self.result_future = None
        try:
            queries = self.flight_queries(metadata, scene_hash)
            self.prewarm_flight_queries(queries)
            if not self.measured_stop(seconds=.6):
                stop_with_reason(self, 'STANDSTILL_UNCONFIRMED_AFTER_MONITOR_PREWARM')
            # Warm-up spins callbacks, so bind dispatch to the same stationary
            # arm/finger state which was reviewed before discovery began.
            if any(abs(self.values.get(name, math.inf)-before[name]) >
                   (START_TOLERANCE if name in ARM else .003) for name in JOINTS):
                stop_with_reason(self, 'START_STATE_CHANGED_DURING_MONITOR_PREWARM')
            if not self.holding_ok():
                stop_with_reason(self, 'HOLDING_CONTACT_LOST', stage='monitor_prewarm')
            if self.input_hashes() != metadata['input_sha256'] or file_digest(self.path) != metadata['recipe_sha256']:
                stop_with_reason(self, 'RECIPE_CONTEXT_CHANGED', stage='monitor_prewarm')
            self.check_stop()
            sequence = self.sensor_sequence
            if step['phase'] == 'grasp':
                self.grasp_attempt['sensor_sequence'] = sequence
            self.pending_identity = (action, endpoint, goal_id)
            self.phase('STACK_STEP_EXECUTING', index=index, total=total, label=step['label'])
            began = time.monotonic()
            try:
                self.goal_pending = client.send_goal_async(goal, goal_uuid=goal_id)
                while not self.goal_pending.done():
                    rclpy.spin_once(self, timeout_sec=.02)
                    self.poll_flight_queries(queries)
                    if not self.fresh():
                        stop_with_reason(self, 'JOINT_FEEDBACK_STALE', stage='goal_acknowledgement')
                    elif not self.sensor_fresh():
                        stop_with_reason(self, 'GRIPPER_SENSOR_STALE', stage='goal_acknowledgement')
                    if time.monotonic()-began > 15.:
                        stop_with_reason(self, 'GOAL_ACKNOWLEDGEMENT_TIMEOUT', service=endpoint,
                                         elapsed=round(time.monotonic()-began, 3))
                    if self.stopped:
                        self.recover_pending()
                        raise RuntimeError('Physical motion stopped while awaiting goal acknowledgement')
                self.goal = self.goal_pending.result()
                if self.goal is None:
                    raise RuntimeError('No physical goal acknowledgement')
            except Exception as error:
                stop_with_reason(self, 'GOAL_ACKNOWLEDGEMENT_FAILED', service=endpoint, error=str(error))
                self.recover_pending()
                reason = getattr(self, 'stop_reason', None)
                raise RuntimeError('Physical goal acceptance failed; exact goal stop was recovered'+
                                   ('; '+json.dumps(reason, sort_keys=True) if reason else ''))
            if not self.goal.accepted:
                self.goal = self.goal_pending = self.pending_identity = None
                raise RuntimeError('The physical controller rejected the reviewed motion')
            self.pending_identity = None
            self.result_future = self.goal.get_result_async()
            cancelled_at, checked_at = -math.inf, time.monotonic()
            timeout = 20. if hand else trajectory_duration(trajectory)*3.+30.
            while not self.result_future.done():
                rclpy.spin_once(self, timeout_sec=.02)
                now = time.monotonic()
                if not self.fresh():
                    stop_with_reason(self, 'JOINT_FEEDBACK_STALE', stage='motion')
                elif not self.sensor_fresh():
                    stop_with_reason(self, 'GRIPPER_SENSOR_STALE', stage='motion',
                                     sample_sequence=self.sensor_sequence)
                elif now-began > timeout:
                    stop_with_reason(self, 'MOTION_TIMEOUT', service=endpoint,
                                     elapsed=round(now-began, 3), limit=timeout)
                if hand:
                    if any(abs(self.values.get(name, math.inf)-before[name]) > START_TOLERANCE for name in ARM):
                        stop_with_reason(self, 'ARM_MOVED_DURING_HAND_COMMAND')
                elif abs(self.values.get(HAND, math.inf)-before[HAND]) > .003:
                    stop_with_reason(self, 'HAND_MOVED_DURING_ARM_COMMAND',
                                     expected=before[HAND], actual=self.values.get(HAND))
                elif not self.holding_ok():
                    stop_with_reason(self, 'HOLDING_CONTACT_LOST', stage='motion',
                                     grip_state=self.sensor.get('grip_state'))
                self.poll_flight_queries(queries)
                if now-checked_at > 1. and not self.stopped:
                    if (self.input_hashes() != metadata['input_sha256']
                            or file_digest(self.path) != metadata['recipe_sha256']):
                        stop_with_reason(self, 'RECIPE_CONTEXT_CHANGED', stage='motion')
                    checked_at = now
                if self.stopped and now-cancelled_at > .5:
                    self.goal.cancel_goal_async()
                    cancelled_at = now
            result = self.result_future.result()
            quiet = self.measured_stop()
            if quiet and result.status in (4, 5, 6):
                self.goal = self.goal_pending = None
                status = 'STACK_STOPPED_CONFIRMED' if self.stopped else 'STACK_MOTION_SETTLED'
                print(status+' physical motion terminal and arm/gripper standstill', flush=True)
            if self.stopped or not quiet or not self.fresh():
                reason = getattr(self, 'stop_reason', None)
                raise RuntimeError('Actual pick/place motion stopped; current scene preserved'+
                                   ('; '+json.dumps(reason, sort_keys=True) if reason else ''))
            if hand:
                action_result = getattr(result, 'result', None)
                hand_result = dict(
                    phase=step['phase'], target=step['hand_target'], start=before[HAND],
                    measured=self.values[HAND], position_error=self.values[HAND]-step['hand_target'],
                    status=result.status, result_position=getattr(action_result, 'position', None),
                    reached_goal=getattr(action_result, 'reached_goal', None),
                    stalled=getattr(action_result, 'stalled', None),
                    grip_state=self.sensor.get('grip_state'), raw_position=self.sensor.get('raw_position'),
                    force_percent=self.sensor.get('force_percent'), sample_sequence=self.sensor_sequence,
                    command_sample_sequence=sequence, communication_age=self.sensor.get('communication_age'))
                print('STACK_GRIPPER_RESULT '+json.dumps(hand_result, sort_keys=True), flush=True)
                if not self.sensor_fresh(after=sequence):
                    raise RuntimeError('그리퍼 명령 뒤 새로운 실제 센서 표본을 확인하지 못했습니다. '+
                                       json.dumps(hand_result, sort_keys=True))
                if step['phase'] == 'grasp':
                    contact = self.mock or self.sensor.get('grip_state') == 2
                    if (result.status not in (4, 6) or not contact
                            or not max(before[HAND], .70)+.001 < self.values[HAND] <= .733):
                        raise RuntimeError('0.73 잡기 뒤 실제 블록 잡힘을 확인하지 못했습니다. 팔을 들어 올리지 않습니다.')
                elif (result.status != 4 or abs(self.values[HAND]-step['hand_target']) > .003
                      or result.result.stalled or not result.result.reached_goal):
                    raise RuntimeError('검토한 그리퍼 열림 위치를 확인하지 못했습니다. '+
                                       json.dumps(hand_result, sort_keys=True))
                if step['phase'] in ('open', 'release_clearance') and not self.mock and self.sensor.get('grip_state') != 1:
                    raise RuntimeError('0.60 열림 뒤 블록 놓임을 확인하지 못했습니다. 팔은 정지 상태를 유지합니다.')
            elif (result.status != 4 or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
                  or any(abs(self.values[name]-q) > START_TOLERANCE for name, q in zip(ARM, step['end_joints']))):
                raise RuntimeError('Actual arm endpoint differs from its reviewed path')
            if self.execution_mode == 'step':
                self.phase('STACK_STEP_COMPLETED', index=index, total=total, label=step['label'])
        finally:
            if 'queries' in locals():
                self.destroy_flight_queries(queries)
            self.cancel_remaining_goal()
            if self.stopped and step['phase'] == 'grasp':
                self.preserve_contact_after_stop()
            client.destroy()

    def flight_queries(self, metadata, scene_hash):
        """Fresh predicates/futures per motion, using clients warmed before goals."""
        expected = 'mock_components/GenericSystem' if self.mock else 'ur_robot_driver/URPositionHardwareInterface'
        grip_expected = 'mock_components/GenericSystem' if self.mock else 'dh_gripper_driver/DHGripperHardwareInterface'
        grip_manager = '/controller_manager' if self.mock else '/gripper/controller_manager'
        request_scene = GetPlanningScene.Request()
        request_scene.components.components = 1023
        queries = []
        def add(kind, service, request, period, valid, describe=None):
            if not hasattr(self, 'monitor_clients'):
                self.monitor_clients = {}
            if service not in self.monitor_clients:
                self.monitor_clients[service] = self.create_client(kind, service)
            queries.append(dict(client=self.monitor_clients[service], service=service, request=request,
                                period=period, valid=valid, describe=describe, future=None,
                                sent=-math.inf, verified=False, last_ok=-math.inf))
        add(ListControllers, '/controller_manager/list_controllers', ListControllers.Request(), .5,
            lambda reply: any(c.name == CONTROLLER and c.state == 'active' for c in reply.controller),
            lambda reply: dict(controllers={c.name: c.state for c in reply.controller}))
        add(ListControllers, grip_manager+'/list_controllers', ListControllers.Request(), .5,
            lambda reply: any(c.name == 'dh_ag95_gripper_controller' and c.state == 'active' for c in reply.controller),
            lambda reply: dict(controllers={c.name: c.state for c in reply.controller}))
        add(ListHardwareComponents, '/controller_manager/list_hardware_components', ListHardwareComponents.Request(), 2.,
            lambda reply: any(c.class_type == expected and c.state.id == 3 for c in reply.component),
            lambda reply: dict(hardware=[dict(plugin=c.class_type, state=c.state.id) for c in reply.component]))
        add(ListHardwareComponents, grip_manager+'/list_hardware_components', ListHardwareComponents.Request(), 2.,
            lambda reply: any(c.class_type == grip_expected and c.state.id == 3 for c in reply.component),
            lambda reply: dict(hardware=[dict(plugin=c.class_type, state=c.state.id) for c in reply.component]))
        add(GetPlanningScene, '/get_planning_scene', request_scene, .5,
            lambda reply: scene_fingerprint(reply.scene) == scene_hash,
            lambda reply: dict(expected_sha256=scene_hash, actual_sha256=scene_fingerprint(reply.scene)))
        for owner, names, texts in [('/robot_state_publisher', ['robot_description'], [self.model]),
                                    ('/move_group', ['robot_description', 'robot_description_semantic'], [self.model, self.semantic])]:
            request = GetParameters.Request()
            request.names = names
            add(GetParameters, owner+'/get_parameters', request, 2.,
                lambda reply, expected=texts: len(reply.values) == len(expected) and
                all(value.string_value == text for value, text in zip(reply.values, expected)))
        return queries

    def prewarm_flight_queries(self, queries):
        """Discover and acknowledge every current-context monitor while stationary.

        Cold DDS discovery may exceed the two-second moving watchdog. The
        stationary warm-up has a ten-second bound, then the same clients retain
        the original two-second request deadline throughout actual movement.
        """
        began = time.monotonic()
        while True:
            self.check_stop()
            now = time.monotonic()
            if not self.fresh():
                stop_with_reason(self, 'JOINT_FEEDBACK_STALE', stage='monitor_prewarm')
            elif not self.sensor_fresh():
                stop_with_reason(self, 'GRIPPER_SENSOR_STALE', stage='monitor_prewarm')
            self.poll_flight_queries(queries, warming=True)
            self.check_stop()
            if all(query['verified'] and now-query['last_ok'] <= 2. and
                   (query['future'] is None or now-query['sent'] <= 2.) for query in queries):
                # Remove any in-flight warm-up reads before a new motion/event
                # can change its expected scene. Late replies cannot satisfy a
                # new future: rclpy matches the service request sequence ID.
                self.destroy_flight_queries(queries)
                for query in queries:
                    query['sent'] = -math.inf
                print('STACK_MONITORS_READY '+json.dumps(dict(services=len(queries),
                      elapsed=round(time.monotonic()-began, 3))), flush=True)
                return
            if now-began > 10.:
                pending = [query['service'] for query in queries if not query['verified']]
                stop_with_reason(self, 'MONITOR_PREWARM_TIMEOUT', services=pending,
                                 elapsed=round(now-began, 3))
                self.check_stop()
            rclpy.spin_once(self, timeout_sec=.02)

    def poll_flight_queries(self, queries, warming=False):
        now = time.monotonic()
        for query in queries:
            future = query['future']
            service = query.get('service', '<unknown service>')
            if future is not None:
                if future.done():
                    try:
                        reply = future.result()
                        if reply is None or not query['valid'](reply):
                            details = query.get('describe')
                            stop_with_reason(self, 'MONITOR_REPLY_INVALID', service=service,
                                             elapsed=round(now-query['sent'], 3),
                                             observed=details(reply) if details and reply is not None else None)
                        else:
                            query['verified'], query['last_ok'] = True, now
                    except Exception as error:
                        stop_with_reason(self, 'MONITOR_RPC_FAILED', service=service,
                                         elapsed=round(now-query['sent'], 3), error=str(error))
                    query['future'] = None
                elif now-query['sent'] > (10. if warming else 2.):
                    stop_with_reason(self, 'MONITOR_RPC_TIMEOUT', service=service,
                                     elapsed=round(now-query['sent'], 3), moving=not warming)
            elif now-query['sent'] >= query['period'] and not self.stopped:
                try:
                    if not query['client'].service_is_ready():
                        if not warming:
                            stop_with_reason(self, 'MONITOR_SERVICE_UNAVAILABLE', service=service)
                    else:
                        query['future'] = query['client'].call_async(query['request'])
                        query['sent'] = now
                except Exception as error:
                    stop_with_reason(self, 'MONITOR_RPC_FAILED', service=service, error=str(error))

    def destroy_flight_queries(self, queries):
        """Forget per-motion reads; retain discovered clients for the recipe."""
        for query in queries:
            if query['future'] is not None:
                query['client'].remove_pending_request(query['future'])
                query['future'] = None

    def close_monitor_clients(self):
        for client in getattr(self, 'monitor_clients', {}).values():
            self.destroy_client(client)
        self.monitor_clients = {}

    def preserve_contact_after_stop(self):
        """Do not erase a physically held cube when Stop arrives near attachment."""
        attempt = self.grasp_attempt
        if (self.mock or attempt is None or self.active_recipe is None
                or not self.sensor_fresh(after=attempt['sensor_sequence'])
                or self.sensor.get('grip_state') != 2 or not self.fresh()
                or any(abs(self.values[name]-q) > .003 for name, q in zip(ARM, attempt['arm']))):
            return
        snapshot = attempt['attachment_snapshot']
        update = PlanningScene()
        update.is_diff = update.robot_state.is_diff = True
        for attached in snapshot.robot_state.attached_collision_objects:
            if attached.object.id.startswith('ur5e_stack/block_'):
                remove = CollisionObject()
                remove.id, remove.operation = attached.object.id, CollisionObject.REMOVE
                update.world.collision_objects.append(remove)
                update.robot_state.attached_collision_objects.append(copy.deepcopy(attached))
        if not update.robot_state.attached_collision_objects:
            write_held_fault('Stopped grasp contact needs manual holding-state recovery', digest(self.model.encode()))
            raise RuntimeError('Physical holding state needs operator recovery; no attachment geometry exists')
        write_held_fault('Physical grasp stopped while holding a block', digest(self.model.encode()))
        # Apply only the conservative attachment, retaining any newly observed
        # external collision objects rather than restoring an older full world.
        self.apply_snapshot(update, cancel=False, full=False)
        self.carrying, self.holding_reference = True, self.values[HAND]
        print('STACK_PHYSICAL_HOLD_PRESERVED block attached after stopped grasp; place it safely before replanning', flush=True)

    def recover_pending(self):
        """Cancel/query only the UUID chosen before an ambiguous action send."""
        if self.pending_identity is None:
            return
        self.stopped = True
        action, endpoint, goal_id = self.pending_identity
        result_type = action.Impl.GetResultService
        cancel = self.create_client(CancelGoal, endpoint+'/_action/cancel_goal')
        result = self.create_client(result_type, endpoint+'/_action/get_result')
        cancel_future = result_future = None
        last_cancel = last_result = announced = -math.inf
        terminal = False
        try:
            while True:
                now = time.monotonic()
                if not terminal and now-last_cancel > 2. and cancel.service_is_ready():
                    if cancel_future is not None:
                        cancel.remove_pending_request(cancel_future)
                    request = CancelGoal.Request()
                    request.goal_info.goal_id = goal_id
                    try:
                        cancel_future = cancel.call_async(request)
                    except Exception:
                        cancel_future = None
                    last_cancel = now
                if result_future is not None and result_future.done():
                    try:
                        response = result_future.result()
                        terminal = response is not None and response.status in (4, 5, 6)
                    except Exception:
                        terminal = False
                    result_future = None
                if not terminal and (result_future is None or now-last_result > 10.) and result.service_is_ready():
                    if result_future is not None:
                        result.remove_pending_request(result_future)
                    request = result_type.Request()
                    request.goal_id = goal_id
                    try:
                        result_future = result.call_async(request)
                    except Exception:
                        result_future = None
                    last_result = now
                if terminal and self.measured_stop(seconds=.6) and self.fresh():
                    self.goal = self.goal_pending = self.result_future = self.pending_identity = None
                    print('STACK_STOPPED_CONFIRMED exact physical goal UUID terminal and standstill', flush=True)
                    return
                if now-announced > 5.:
                    print('STACK_STOP_PENDING recovering exact physical action UUID', flush=True)
                    announced = now
                rclpy.spin_once(self, timeout_sec=.05)
        finally:
            for client, future in ((cancel, cancel_future), (result, result_future)):
                if future is not None:
                    client.remove_pending_request(future)
                self.destroy_client(client)

    def cancel_remaining_goal(self):
        if self.pending_identity is not None and self.goal is None:
            self.recover_pending()
        super().cancel_remaining_goal()

    def run(self):
        if self.mode == 'physical_plan':
            self.plan()
        elif self.mode == 'physical_run':
            self.execute()
        else:
            raise ValueError('Physical stack mode must be physical_plan or physical_run')


def main():
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdigit():
        raise ValueError('ROS_DOMAIN_ID must be numeric')
    lock = os.open(f'/tmp/ur5e_air_task_{domain}.lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
    fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    task = PhysicalStackTask()
    signal.signal(signal.SIGINT, lambda *_: stop_with_reason(task, 'PROCESS_SIGNAL', signal='SIGINT'))
    signal.signal(signal.SIGTERM, lambda *_: stop_with_reason(task, 'PROCESS_SIGNAL', signal='SIGTERM'))
    code = 0
    try:
        task.run()
    except Exception as error:
        print('STACK_ERROR '+str(error), flush=True)
        code = 1
    finally:
        task.cancel_remaining_goal()
        if code:
            if not task.mock and (task.carrying or task.release_stage > 0):
                write_held_fault('Physical pick/place failed or stopped while a block may remain held', digest(task.model.encode()))
            try:
                task.preserve_contact_after_stop()
            except Exception as error:
                print('STACK_PHYSICAL_HOLD_RECOVERY_PENDING '+str(error), flush=True)
        if code and task.goal_pending is None and task.measured_stop():
            print('STACK_STOPPED_CONFIRMED', flush=True)
        if task.stack_lock is not None:
            os.close(task.stack_lock)
        task.close_monitor_clients()
        task.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(lock)
    raise SystemExit(code)
