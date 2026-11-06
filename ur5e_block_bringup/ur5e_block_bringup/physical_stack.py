"""Reviewed physical pick/place recipes, separate from the mock-only executor.

The C++ planner exports every arm, finger and scene step. This node validates
the complete recipe, restores the initial scene, seals the reviewed files, and
dispatches only operator-approved goals to the existing hardware controllers.
"""
import copy
from array import array
import fcntl
from functools import lru_cache
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
from shape_msgs.msg import Mesh
from unique_identifier_msgs.msg import UUID as GoalUUID
import yaml

from .air_task import (AirTask, ARM, HAND, CONTROLLER, MAX_FILE_BYTES,
                       START_TOLERANCE, atomic_write, digest, file_digest)
from .air_scene import scene_fingerprint as strict_scene_fingerprint
from .air_trajectory import sample_trajectory, trajectory_metrics
from .physical_held import clear_held_fault, read_held_fault, write_held_fault
from .physical_motion import (ARM_SPEEDS, CONTACT_CLEARANCE, APPROACH_CLEARANCE,
                              incremental_arm_scaling)
from .incremental_state import ROUTING_POLICY as INCREMENTAL_ROUTING_POLICY
from .collision_worker import CollisionWorker
from . import physical_replay as replay
from rosidl_runtime_py.convert import message_to_ordereddict

JOINTS = ARM+[HAND]
HAND_TARGETS = {'open': .4, 'grasp': .73, 'release_first': .70, 'release_clearance': .60}
SIDE_ENTRY_CLEARANCE = .010
REVIEW_TTL = 3600.
HAND_START_TOLERANCE = .015
SCENE_HASH_POLICY = 'physical_mesh_digest_pose_round12_v2'
RECIPE_POLICY = 'physical_nearest_target_buffer_v18'
BUFFER_SELECTION_POLICY = 'nearest_target_feasible_v1'
PREPARABLE_POLICIES = (RECIPE_POLICY,)
GRASP_POLICY = 'command_completion'
HAND_HOLD_TOLERANCE = .005
HAND_COMMAND_TOLERANCE = .003
HAND_ENVELOPE_STEP = .005
ARM_PATH_TOLERANCE = .1
TRANSFER_WRIST_SWEEP_LIMIT = .010
ARM_GOAL_POSITION_TOLERANCE = START_TOLERANCE
ARM_GOAL_VELOCITY_TOLERANCE = .01
ARM_GOAL_TIME_TOLERANCE = 5
ARM_TRACKING_SWEEP_STEP = .005
PLANNER_INTERRUPT_GRACE = 2.
PLANNER_TERMINATE_GRACE = 2.
PLANNER_KILL_GRACE = 1.


def arm_execution_profile(path_tolerance=None):
    """The reviewed tracking and stationary-boundary contract, in joint order."""
    path_tolerance = ARM_PATH_TOLERANCE if path_tolerance is None else path_tolerance
    return dict(
        joint_names=list(ARM),
        phase_path_position={phase: [path_tolerance]*len(ARM) for phase in ARM_SPEEDS},
        goal_position=ARM_GOAL_POSITION_TOLERANCE,
        goal_velocity=ARM_GOAL_VELOCITY_TOLERANCE,
        goal_time_seconds=ARM_GOAL_TIME_TOLERANCE,
        start_position=START_TOLERANCE,
        transfer_sweep_joint=ARM[-1], transfer_sweep_limit=TRANSFER_WRIST_SWEEP_LIMIT,
        transfer_sweep_step=ARM_TRACKING_SWEEP_STEP)


def arm_result_record(step, result, measured, quiet, last_feedback=None, execution_profile=None):
    """Keep native tracking/goal faults separate from a successful-end mismatch."""
    native = getattr(result, 'result', None)
    status, code = getattr(result, 'status', None), getattr(native, 'error_code', None)
    positions = {}
    endpoint_ok = True
    for name, target in zip(ARM, step['end_joints']):
        actual = measured.get(name)
        finite = isinstance(actual, (int, float)) and math.isfinite(actual)
        delta = float(actual-target) if finite else None
        endpoint_ok = endpoint_ok and delta is not None and abs(delta) <= START_TOLERANCE
        positions[name] = dict(target=target, actual=actual if finite else None, error=delta)
    native_faults = {
        FollowJointTrajectory.Result.INVALID_GOAL: 'ARM_INVALID_GOAL',
        FollowJointTrajectory.Result.INVALID_JOINTS: 'ARM_INVALID_JOINTS',
        FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: 'ARM_OLD_HEADER_TIMESTAMP',
        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: 'ARM_PATH_TOLERANCE_VIOLATED',
        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: 'ARM_GOAL_TOLERANCE_VIOLATED'}
    if code in native_faults:
        classification = native_faults[code]
    elif status != 4 or code != FollowJointTrajectory.Result.SUCCESSFUL:
        classification = 'ARM_ACTION_CANCELED' if status == 5 else 'ARM_ACTION_FAILED'
    elif not quiet:
        classification = 'ARM_STANDSTILL_UNCONFIRMED'
    elif not endpoint_ok:
        classification = 'ARM_ENDPOINT_MISMATCH'
    else:
        classification = 'SUCCESS'
    profile = arm_execution_profile() if execution_profile is None else execution_profile
    return dict(classification=classification, step=step['index'], phase=step['phase'], label=step['label'],
                status=status, error_code=code, error_string=getattr(native, 'error_string', ''),
                path_position_tolerance=dict(zip(ARM, profile['phase_path_position'][step['phase']])),
                goal_position_tolerance=profile['goal_position'], goal_velocity_tolerance=profile['goal_velocity'],
                endpoint_position_tolerance=START_TOLERANCE, goal_time_seconds=profile['goal_time_seconds'],
                quiet=bool(quiet), joints=positions, last_feedback=last_feedback)


def stop_with_reason(task, code, **details):
    """Keep the first cause, so cancellation does not hide the failed monitor."""
    if getattr(task, 'stop_reason', None) is None:
        task.stop_reason = dict(code=code, **details)
        print('STACK_STOP_REASON '+json.dumps(task.stop_reason, sort_keys=True, default=str), flush=True)
    task.stopped = True


@lru_cache(maxsize=16)
def normalized_mesh_digest(encoded):
    """Cache work, never geometry: every call keys the current full mesh bytes.

    Unchanged plate vertices/triangles used to traverse Python canonicalization
    repeatedly in every scene monitor, starving fresh sensor callbacks. A byte
    change always misses; the digest retains the existing round12 equivalence.
    """
    mesh = deserialize_message(encoded, Mesh)
    canonical = normalized_collision_data(message_to_ordereddict(mesh))
    return digest(json.dumps(canonical, sort_keys=True, allow_nan=False).encode())


def normalized_collision_data(value):
    """Normalize Eigen/ROS pose round trips to sub-nanometre precision.

    This affects comparison only. The original collision shapes and trajectory
    samples are sent unchanged to MoveIt. No millimetre-scale scene difference
    is ignored, and q/-q quaternion representations describe the same rotation.
    """
    if isinstance(value, Mesh):
        return dict(normalized_mesh_sha256=normalized_mesh_digest(serialize_message(value)))
    if hasattr(value, 'get_fields_and_field_types'):
        value = {name: getattr(value, name) for name in value.get_fields_and_field_types()}
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
        result = {}
        for key, item in value.items():
            if key == 'meshes' and isinstance(item, (list, tuple)):
                result[key] = [normalized_collision_data(mesh) if isinstance(mesh, Mesh) else
                               dict(normalized_mesh_sha256=digest(json.dumps(
                                   normalized_collision_data(mesh), sort_keys=True, allow_nan=False).encode()))
                               for mesh in item]
            else:
                result[key] = normalized_collision_data(
                    {k: v for k, v in item.items() if k != 'stamp'}
                    if key == 'header' and isinstance(item, dict) else item)
                if key == 'header' and isinstance(result[key], dict):
                    result[key].pop('stamp', None)
        return result
    if isinstance(value, bytes):
        return ''.join(chr(item) for item in value)
    if isinstance(value, (list, tuple, array)):
        return [normalized_collision_data(item) for item in value]
    if hasattr(value, 'tolist'):
        return normalized_collision_data(value.tolist())
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
    if 'arm_execution_profile' in recipe and recipe['arm_execution_profile'] != arm_execution_profile():
        raise ValueError('Physical recipe arm tracking/goal/start profile differs from the reviewed policy')
    if 'grasp_policy' in recipe and recipe['grasp_policy'] != GRASP_POLICY:
        raise ValueError('Physical recipe grasp command policy differs from the reviewed policy')
    clearance = recipe.get('placement_approach_clearance')
    if (recipe.get('placement_policy') != 'raised_tower_hover_setdown_v2'
            or recipe.get('placement_mode') not in ('side', 'vertical')
            or type(recipe.get('vertical_placement_layers')) is not int
            or not 0 <= recipe['vertical_placement_layers'] <= 100
            or isinstance(clearance, bool) or not isinstance(clearance, (int, float))
            or not math.isfinite(clearance) or not .08 <= clearance <= .20):
        raise ValueError('Physical recipe requires the raised tower placement policy and 80..200mm clearance')
    if recipe.get('side_entry_clearance') != SIDE_ENTRY_CLEARANCE:
        raise ValueError('Physical side placement requires the reviewed 10mm hover and vertical setdown')
    if recipe.get('physical_approach_clearance') != APPROACH_CLEARANCE:
        raise ValueError(f'Physical descent requires the reviewed final {APPROACH_CLEARANCE*1000:g}mm slow approach')
    if any(recipe.get(key) != ARM_SPEEDS['descent_fast'] for key in ('descent_velocity_scaling', 'descent_acceleration_scaling')):
        raise ValueError(f'Physical fast descent requires reviewed {ARM_SPEEDS["descent_fast"]:.2f} velocity and acceleration')
    if recipe.get('physical_contact_clearance') != CONTACT_CLEARANCE:
        raise ValueError(f'Physical recipe needs the reviewed {CONTACT_CLEARANCE*1000:g}mm contact clearance')
    for key, expected_scaling in [('approach_acceleration_scaling', ARM_SPEEDS['approach']),
                                  ('transfer_acceleration_scaling', ARM_SPEEDS['transfer'])]:
        scaling = recipe.get(key)
        if (isinstance(scaling, bool) or not isinstance(scaling, (int, float))
                or not math.isfinite(scaling) or abs(scaling-expected_scaling) > 1e-9):
            raise ValueError(f'Physical recipe needs reviewed phase acceleration scaling '
                             f'{ARM_SPEEDS["approach"]:.2f}/{ARM_SPEEDS["transfer"]:.2f}')
    incremental = recipe.get('routing_policy') == INCREMENTAL_ROUTING_POLICY
    if (type(recipe.get('target_cell')) is not int or not 1 <= recipe['target_cell'] <= 100
            or type(recipe.get('moves')) is not int or recipe['moves'] < 1
            or type(recipe.get('foundation_layers')) is not int
            or not (1 <= recipe['foundation_layers'] <= 99 if incremental else recipe['foundation_layers'] == 1)
            or type(recipe.get('layers')) is not int):
        raise ValueError('Physical stacking requires one existing foundation block and integer counts')
    sources = recipe.get('source_cells')
    if recipe.get('buffer_selection_policy') != BUFFER_SELECTION_POLICY:
        raise ValueError('Physical recipe requires nearest-target feasible temporary supports')
    if (recipe.get('routing_policy') not in ('open_gripper_buffer_v1', INCREMENTAL_ROUTING_POLICY)
            or not isinstance(sources, list) or len(sources) != recipe['moves']
            or any(type(cell) is not int or not 1 <= cell <= 100 or cell == recipe['target_cell'] for cell in sources)
            or len(set(sources)) != len(sources)
            or type(recipe.get('transport_moves')) is not int
            or type(recipe.get('buffer_moves')) is not int
            or not 0 <= recipe['buffer_moves'] <= recipe['moves']
            or recipe['transport_moves'] != recipe['moves']+recipe['buffer_moves']):
        raise ValueError('Physical recipe requires the open-gripper buffer routing policy and consistent transport counts')
    towers = {cell: [cell] for cell in sources}
    towers[recipe['target_cell']] = [recipe['target_cell']]
    incremental_after = None
    if incremental:
        from .incremental_state import recipe_layout
        towers, incremental_after = recipe_layout(recipe)
    buffered, finished = set(), set()
    temporary_places = 0
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
            raise ValueError(f'Physical recipe joint state is discontinuous at step {index}: {step.get("label", "")}')
        kind, phase = step.get('kind'), step.get('phase')
        if kind == 'arm':
            velocity, acceleration = step.get('velocity_scaling'), step.get('acceleration_scaling')
            expected_velocity, expected_acceleration = (incremental_arm_scaling(step.get('motion_role'), recipe['layers'])
                if incremental else (ARM_SPEEDS.get(phase),)*2)
            if (phase not in ARM_SPEEDS or isinstance(velocity, bool) or not isinstance(velocity, (int, float))
                    or not math.isfinite(velocity) or abs(velocity-expected_velocity) > 1e-9):
                raise ValueError('Arm phase does not use the reviewed contact/transfer speed scaling')
            if (isinstance(acceleration, bool) or not isinstance(acceleration, (int, float))
                    or not math.isfinite(acceleration) or abs(acceleration-expected_acceleration) > 1e-9):
                raise ValueError('Physical arm acceleration must match its reviewed contact/transfer phase speed')
            if release_stage or (carrying and not attached):
                raise ValueError('Arm motion precedes attachment or the completed 0.70 -> 0.60 release')
            if abs(start[-1]-end[-1]) > 1e-9:
                raise ValueError('Arm motion must hold the gripper shape')
            if phase in ('approach','descent_fast') and not carrying and abs(start[-1]-HAND_TARGETS['open']) > 1e-8:
                raise ValueError('Pickup approach must keep the gripper open at 0.40rad')
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
                raise ValueError('Opening cannot release a carried block')
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
                block, destination, temporary = step.get('block_id'), step.get('destination_cell'), step.get('temporary')
                if (type(block) is not int or block not in sources or block in finished
                        or type(destination) is not int or destination not in towers
                        or type(temporary) is not bool or temporary != (destination != recipe['target_cell'])):
                    raise ValueError('Invalid block or destination in physical routing event')
                origin = next((cell for cell, tower in towers.items() if block in tower), None)
                if origin is None or origin == destination or not towers[origin] or towers[origin][-1] != block:
                    raise ValueError('Cannot pick a covered or already placed block')
                if temporary:
                    if block in buffered or len(towers[origin]) != 1 or len(towers[destination]) != 1:
                        raise ValueError('Temporary placement needs an exposed single-block support and a recoverable top')
                    buffered.add(block)
                    temporary_places += 1
                else:
                    finished.add(block)
                towers[origin].pop()
                towers[destination].append(block)
                if type(step.get('completed')) is not int or step['completed'] != len(finished):
                    raise ValueError('Temporary placement must not increase target layer progress')
            elif phase != 'scene':
                raise ValueError('Unknown physical scene event')
        else:
            raise ValueError('Unknown physical recipe step kind')
        expected = end
    if carrying or attached or release_stage or grasps != clears or grasps < 1:
        raise ValueError('Physical recipe ends without a complete place/release sequence')
    if (recipe['transport_moves'] != grasps or recipe['buffer_moves'] != temporary_places
            or len(finished) != recipe['moves'] or recipe['layers'] != len(finished)+recipe['foundation_layers']):
        raise ValueError('Physical recipe block/layer counts differ')
    if incremental_after is not None:
        if (towers[recipe['target_cell']] != incremental_after['target_blocks']
                or sorted(cell for cell, tower in towers.items() if cell != recipe['target_cell'] and tower)
                != incremental_after['remaining_sources']):
            raise ValueError('Executed incremental layout differs from its declared next state')
    return steps


def trajectory_duration(trajectory):
    if not trajectory.joint_trajectory.points:
        raise ValueError('Empty physical trajectory')
    stamp = trajectory.joint_trajectory.points[-1].time_from_start
    return stamp.sec+stamp.nanosec*1e-9


class PhysicalStackTask(AirTask):
    def recipe_policy(self):
        return RECIPE_POLICY

    def motion_execution_profile(self):
        """Dispatch the sealed physical-stack thresholds, separately from recovery."""
        return arm_execution_profile()

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
        self.sensor_observed, self.sensor_observed_received = {}, -math.inf
        self.sensor_rejection = None
        sensor_qos = copy.copy(qos_profile_sensor_data)
        sensor_qos.depth = 1  # Only the newest feedback can satisfy freshness.
        self.sensor_subscription = self.create_subscription(
            DynamicJointState, '/gripper/dynamic_joint_states', self.sensor_feedback, sensor_qos)
        self.carrying = False
        self.holding_reference = None
        self.verified_holding_context = None
        self.release_stage = 0
        self.execution_started = False
        self.active_recipe = None
        self.grasp_attempt = None
        self.scene_request_pending = False
        self.stop_reason = None
        self.monitor_clients = {}
        self.motion_clients = {}
        self.replay_queries = None
        self.prevalidated_replay = False
        self.execution_artifacts = None
        self.stationary_since = None
        self.stationary_observed = -math.inf
        self.library = self.path.parent/'physical_path_library'
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

    def joints(self, message):
        super().joints(message)
        now = time.monotonic()
        quiet = self.quiet_feedback(now)
        previous = getattr(self, 'stationary_observed', -math.inf)
        since = getattr(self, 'stationary_since', None)
        self.stationary_since = (now if since is None or now-previous >= .5 else since) if quiet else None
        self.stationary_observed = now

    def quiet_feedback(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.fresh() and self.sensor_fresh()
                and all(name in self.velocities and now-self.velocities[name][1] < .5
                        and abs(self.velocities[name][0]) < .01 for name in JOINTS))

    def sensor_feedback(self, message):
        if len(message.joint_names) != len(message.interface_values) or HAND not in message.joint_names:
            return
        item = message.interface_values[message.joint_names.index(HAND)]
        if len(item.interface_names) != len(item.values) or len(set(item.interface_names)) != len(item.interface_names):
            return
        record = dict(zip(item.interface_names, item.values))
        self.sensor_observed, self.sensor_observed_received = record, time.monotonic()
        required = ('position', 'raw_position', 'grip_state', 'force_percent', 'communication_age', 'sample_sequence')
        if any(name not in record or not math.isfinite(record[name]) for name in required):
            self.sensor_rejection = 'missing_or_nonfinite_fields'
            return
        q, raw, grip, force, age, sequence = [record[name] for name in required]
        if (not 0 <= q <= .93 or not 0 <= raw <= 1000 or raw != int(raw)
                or grip not in (0, 1, 2, 3) or not 20 <= force <= 100 or not 0 <= age <= .5
                or sequence != int(sequence)
                or abs(q-.93*(1000-raw)/1000) > .0015):
            self.sensor_rejection = 'invalid_fields_or_reported_age'
            return
        if sequence <= self.sensor_sequence:
            self.sensor_rejection = 'sequence_not_advanced'
            return
        self.sensor, self.sensor_sequence = record, int(sequence)
        self.sensor_received = time.monotonic()-age
        self.sensor_rejection = None

    def sensor_stale_details(self):
        """Record both device age and callback gaps without changing the limit."""
        now = time.monotonic()
        def age(stamp):
            value = now-stamp
            return round(value, 6) if math.isfinite(value) else None
        observed = getattr(self, 'sensor_observed', {})
        return dict(sample_sequence=getattr(self, 'sensor_sequence', -1), limit_seconds=.5,
                    accepted_sample_age_seconds=age(getattr(self, 'sensor_received', -math.inf)),
                    callback_gap_seconds=age(getattr(self, 'sensor_observed_received', -math.inf)),
                    reported_communication_age=observed.get('communication_age'),
                    observed_sample_sequence=observed.get('sample_sequence'),
                    last_rejection=getattr(self, 'sensor_rejection', None))

    def sensor_fresh(self, after=None):
        return self.mock or (time.monotonic()-self.sensor_received < .5 and
                             (after is None or self.sensor_sequence > after))

    def holding_ok(self):
        if not self.carrying or self.release_stage:
            return True
        if not self.sensor_fresh():
            return False
        interval = self.holding_interval()
        return interval is not None and interval[0] <= self.values.get(HAND, math.inf) <= interval[1]

    def holding_interval(self):
        """Bound reviewed finger geometry without classifying object contact."""
        reference = self.holding_reference
        if reference is None or not math.isfinite(reference):
            return None
        return (reference-HAND_HOLD_TOLERANCE, HAND_TARGETS['grasp']+HAND_COMMAND_TOLERANCE)

    def first_release_target(self):
        interval = self.holding_interval()
        if interval is None:
            raise RuntimeError('놓기 전에 실제 파지 각도를 확인하지 못했습니다.')
        target = min(HAND_TARGETS['release_first'], interval[0]-.005)
        if target < HAND_TARGETS['release_clearance']:
            raise RuntimeError('현재 파지 각도에서는 검증된 0.60rad 범위 안의 열림을 보장할 수 없습니다.')
        return target

    def released_start_matches(self, step):
        target = getattr(self, 'last_first_release_target', None)
        return (self.release_stage in (1, 2) and step['phase'] in ('place', 'release_clearance')
                and target is not None and abs(self.values.get(HAND, math.inf)-target) <= .003)

    def holding_context_key(self, step, trajectory, scene_hash):
        interval = self.holding_interval()
        if interval is None:
            raise RuntimeError('Missing original measured finger geometry reference')
        return digest(json.dumps(dict(
            step=step['index'], scene=scene_hash, interval=interval,
            grasp_policy=GRASP_POLICY,
            arm_execution_profile=arm_execution_profile(),
            step_state={name: step.get(name) for name in
                        ('kind', 'phase', 'start_joints', 'end_joints', 'hand_target')},
            model=digest(self.model.encode()), semantic=digest(self.semantic.encode()),
            trajectory=message_to_ordereddict(trajectory)), sort_keys=True, allow_nan=False).encode())

    def holding_context_verified(self, step, trajectory, scene_hash):
        context = self.verified_holding_context
        return (context is not None and context['interval'] == self.holding_interval()
                and context['key'] == self.holding_context_key(step, trajectory, scene_hash))

    def verified_holding_ok(self, key):
        context = self.verified_holding_context
        return (context is not None and context['key'] == key
                and context['interval'] == self.holding_interval() and self.holding_ok())

    def approval_hand_ok(self, metadata):
        key = metadata.get('holding_context_key')
        if key is None:
            return super().approval_hand_ok(metadata)
        context = self.verified_holding_context
        return (self.carrying and not self.release_stage and context is not None
                and context['key'] == key and context['interval'] == self.holding_interval()
                and self.holding_ok())

    def observe_grasp_settle(self, before):
        """Observe fresh stationary finger geometry; retain its first q.

        Later finger movement is checked against the full reviewed interval.
        This observation never advances the original measured geometry anchor.
        """
        if self.mock:
            return
        began = time.monotonic()
        last_sequence, observations = self.sensor_sequence, []
        interval = self.holding_interval()
        while True:
            self.check_stop()
            rclpy.spin_once(self, timeout_sec=.02)
            now = time.monotonic()
            if (not self.fresh() or not self.sensor_fresh()
                    or interval is None or not interval[0] <= self.values.get(HAND, math.inf) <= interval[1]
                    or any(abs(self.values[name]-before[name]) > START_TOLERANCE for name in ARM)):
                stop_with_reason(self, 'HAND_GEOMETRY_UNSTABLE_BEFORE_LIFT')
                self.check_stop()
            if self.sensor_sequence != last_sequence:
                observations.append((now, self.values[HAND]))
                last_sequence = self.sensor_sequence
                observations = [item for item in observations if now-item[0] <= .5]
                if len(observations) >= 3 and observations[-1][0]-observations[0][0] >= .3:
                    print('STACK_GRASP_COMMAND_ASSUMED '+json.dumps(dict(
                        finger_reference=self.holding_reference, measured=self.values[HAND],
                        grasp_policy=GRASP_POLICY,
                        samples=len(observations), elapsed=round(now-began, 3))), flush=True)
                    return
            if now-began > 3.:
                stop_with_reason(self, 'HAND_GEOMETRY_SETTLE_TIMEOUT', elapsed=round(now-began, 3))
                self.check_stop()

    def measured_stop(self, seconds=4.):
        if getattr(self, 'prevalidated_replay', False):
            now = time.monotonic()
            since = getattr(self, 'stationary_since', None)
            if (since is not None and now-since >= .3
                    and now-self.stationary_observed < .5 and self.quiet_feedback(now)):
                return True
        began = None
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            now = time.monotonic()
            if getattr(self, 'prevalidated_replay', False):
                queries = getattr(self, 'replay_queries', None)
                if queries:
                    self.poll_flight_queries(queries)
                since = getattr(self, 'stationary_since', None)
                if (since is not None and now-since >= .3
                        and now-self.stationary_observed < .5 and self.quiet_feedback(now)):
                    return True
            quiet = self.fresh() and self.sensor_fresh() and all(name in self.velocities and now-self.velocities[name][1] < .5
                        and abs(self.velocities[name][0]) < .01 for name in JOINTS)
            began = now if quiet and began is None else began if quiet else None
            if began is not None and now-began >= .3:
                return True
        return False

    def acquire_stack_lock(self):
        if self.stack_lock is None:
            domain = os.environ.get('ROS_DOMAIN_ID', '0')
            descriptor = os.open(f'/tmp/ur5e_stack_{domain}.lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BaseException:
                os.close(descriptor)
                raise
            self.stack_lock = descriptor

    def release_stack_lock(self):
        if self.stack_lock is not None:
            descriptor, self.stack_lock = self.stack_lock, None
            os.close(descriptor)

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

    def cached_service_client(self, kind, service):
        """Reuse discovery, never response data, for sequential planning RPCs.

        This cache is separate from moving-monitor clients. Lazy creation also
        keeps inherited initialization safe if it begins using ``call`` later.
        Every request still has its own future and timeout/Stop handling.
        """
        if not hasattr(self, 'service_clients'):
            self.service_clients = {}
        key = (kind, service)
        if key not in self.service_clients:
            self.service_clients[key] = self.create_client(kind, service)
        return self.service_clients[key]

    def call(self, kind, service, request, seconds=10., cancel=True):
        client = self.cached_service_client(kind, service)
        reads = (GetParameters, ListHardwareComponents, ListControllers, GetPlanningScene)
        context = getattr(self, 'startup_read_stage', None)
        began = time.monotonic()

        def may_retry():
            # Only the initial, read-only planning preflight may tolerate one
            # lost reply. Motion monitors and scene transactions never enter it.
            return (getattr(self, 'startup_read_stage', None) == 'physical_plan_preflight'
                    and kind in reads and cancel
                    and not getattr(self, 'stopped', False)
                    and all(getattr(self, name, None) is None
                            for name in ('goal', 'goal_pending', 'child'))
                    and not getattr(self, 'scene_request_pending', False))

        max_attempts = 2 if may_retry() else 1

        def timeout(stage, attempt, cause):
            detail = dict(endpoint=service, stage=stage, context=context,
                          attempt=attempt, max_attempts=max_attempts,
                          elapsed=round(time.monotonic()-began, 3))
            print(('STACK_READ_RPC_TIMEOUT ' if kind in reads else 'STACK_RPC_TIMEOUT ')+
                  json.dumps(detail, sort_keys=True), flush=True)
            return TimeoutError(f'ROS service {service} {stage} timed out after '
                                f'{attempt} request attempt(s): {cause}')

        try:
            self.spin_until(client.service_is_ready, seconds, cancel=cancel)
        except TimeoutError as exc:
            # No request was sent. Repeating discovery would only hide an
            # unavailable service behind a second full discovery timeout.
            raise timeout('discovery', 0, exc) from exc
        for attempt in range(1, max_attempts+1):
            future = None
            try:
                future = client.call_async(request)
                try:
                    self.spin_until(future.done, seconds, cancel=cancel)
                except TimeoutError as exc:
                    error = timeout('response', attempt, exc)
                    if cancel:
                        self.check_stop()
                    if attempt == max_attempts or not may_retry():
                        raise error from exc
                    continue
                # A server-side exception or an invalid semantic reply is not
                # a lost response and must never trigger another request.
                return future.result()
            finally:
                # Remove an expired request before issuing its replacement.
                # Late replies keep their own sequence ID and cannot satisfy it.
                if future is not None:
                    client.remove_pending_request(future)

    def close_service_clients(self):
        for client in getattr(self, 'service_clients', {}).values():
            self.destroy_client(client)
        self.service_clients = {}

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
        reply = self.call(GetPlanningScene, '/get_planning_scene', request, cancel=cancel)
        if reply is None:
            raise RuntimeError('Unable to read the current scene for an exact geometry transaction')
        return reply.scene

    def apply_scene_message(self, message, cancel=True):
        if getattr(self, 'scene_request_pending', False):
            raise RuntimeError('An earlier planning-scene write has no acknowledgement; further writes/execution are blocked')
        snapshot = copy.deepcopy(message)
        snapshot.is_diff = snapshot.robot_state.is_diff = True
        snapshot.robot_state.joint_state.name = JOINTS
        snapshot.robot_state.joint_state.position = [self.values.get(name, 0.) for name in JOINTS]
        snapshot.robot_state.joint_state.velocity = []
        snapshot.robot_state.joint_state.effort = []
        client = self.cached_service_client(ApplyPlanningScene, '/apply_planning_scene')
        future = None
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
            # Unacknowledged scene mutations are never abandoned or reused.
            # The loop above waits through Stop; transport failure retains the
            # pending latch and blocks every later scene write/execution.
            if future is not None and future.done():
                client.remove_pending_request(future)

    def read_message(self, name, kind):
        pinned = getattr(self, 'execution_artifacts', None)
        if pinned is not None:
            if name not in pinned:
                raise RuntimeError('Physical replay references an unsealed artifact')
            return deserialize_message(pinned[name], kind)
        return deserialize_message(artifact_path(self.path.parent, name).read_bytes(), kind)

    @staticmethod
    def signal_planner(process, signum):
        """Signal only the session created by planner(start_new_session=True)."""
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass

    def finish_planner(self, process):
        """Bound exceptional cleanup; no robot-facing action clients are used."""
        for signum, seconds in ((signal.SIGINT, PLANNER_INTERRUPT_GRACE),
                                (signal.SIGTERM, PLANNER_TERMINATE_GRACE),
                                (signal.SIGKILL, PLANNER_KILL_GRACE)):
            if process.poll() is not None:
                return
            self.signal_planner(process, signum)
            try:
                process.wait(timeout=seconds)
                return
            except subprocess.TimeoutExpired:
                continue
        raise RuntimeError('Failed planner process did not exit after SIGKILL')

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
        process = self.child = subprocess.Popen(
            ['ros2', 'run', 'ur5e_a3_grid', 'stack_blocks', '--ros-args', '--params-file', str(params_file),
             '-r', '/ur5e_a3_stack/stop:=/ur5e_physical_stack/planner_stop'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        lines = queue.Queue()
        def read():
            for line in process.stdout:
                lines.put(line)
        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        stopping_at, signal_stage, problem = None, 0, ''
        try:
            while process.poll() is None or reader.is_alive() or not lines.empty():
                rclpy.spin_once(self, timeout_sec=.02)
                while not lines.empty():
                    line = lines.get_nowait()
                    if 'STACK_ERROR ' in line:
                        problem = line.split('STACK_ERROR ', 1)[1].strip()
                    print(line, end='', flush=True)
                now = time.monotonic()
                if stopping_at is None and (self.stopped or problem):
                    stopping_at = now
                if stopping_at is not None:
                    elapsed = now-stopping_at
                    stages = ((signal.SIGINT, 0.),
                              (signal.SIGTERM, PLANNER_INTERRUPT_GRACE),
                              (signal.SIGKILL, PLANNER_INTERRUPT_GRACE+PLANNER_TERMINATE_GRACE))
                    while signal_stage < len(stages) and elapsed >= stages[signal_stage][1]:
                        signum = stages[signal_stage][0]
                        self.signal_planner(process, signum)
                        print('STACK_PLANNER_STOPPING '+json.dumps(dict(
                            signal=signum.name, reason='operator_stop' if self.stopped else 'planner_error',
                            elapsed=round(elapsed, 3))), flush=True)
                        signal_stage += 1
                    if elapsed > PLANNER_INTERRUPT_GRACE+PLANNER_TERMINATE_GRACE+PLANNER_KILL_GRACE:
                        raise RuntimeError(problem or 'Stopped planner did not finish within its shutdown deadline')
            self.check_stop()
            if problem or process.returncode or not output.is_file():
                raise RuntimeError(problem or 'The complete physical recipe could not be planned')
        finally:
            try:
                self.finish_planner(process)
            finally:
                reader.join(timeout=.25)
                if not reader.is_alive():
                    process.stdout.close()
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

    def tracking_motion_samples(self, step, trajectory, actual_hand=None):
        """Retain the nominal path and the existing transfer wrist clearance sweep.

        The +/-0.010rad single-axis sweep is an extra geometry check, independent
        of the native 0.1rad abort threshold. It does not enumerate combinations
        of deviations of all six joints or certify their full tolerance region.
        """
        nominal = self.motion_samples(step, trajectory, actual_hand=actual_hand)
        if step['phase'] != 'transfer':
            return nominal
        limit = arm_execution_profile()['transfer_sweep_limit']
        count = max(1, math.ceil(2*limit/ARM_TRACKING_SWEEP_STEP))
        offsets = [-limit+2*limit*i/count for i in range(count+1)]
        if len(nominal)*len(offsets) > 1_000_000:
            raise RuntimeError('Physical transfer wrist clearance sweep exceeds the bounded validation workload')
        samples = []
        for stamp, values in nominal:
            for offset in offsets:
                q = list(values)
                q[5] += offset
                if not self.bounds[ARM[-1]][0] <= q[5] <= self.bounds[ARM[-1]][1]:
                    raise RuntimeError('Physical transfer wrist clearance sweep exceeds its joint limit')
                samples.append((stamp, q))
        return samples

    def validate_samples(self, samples):
        # A bounded window avoids overflowing Humble's service queue while
        # checking every controller-interpolated sample and every finger sweep.
        client = self.cached_service_client(GetStateValidity, '/check_state_validity')
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
                            client.remove_pending_request(future)
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
        return checked

    def certify_motion(self, worker, scene, step, trajectory, holding):
        """Validate the complete accepted finger range once, before playback."""
        fingers = replay.finger_shapes(step, holding, self.bounds[HAND])
        if step['kind'] == 'arm':
            path = self.tracking_motion_samples(step, trajectory)
        else:
            # Validate the source finger trajectory/endpoints as before; the
            # certificate expands its sweep to all accepted measured starts.
            self.motion_samples(step, trajectory)
            path = [(0., step['start_joints'])]
        if len(path)*len(fingers) > 1_000_000:
            raise RuntimeError('Prevalidated physical finger grid exceeds the bounded workload')
        samples = [(stamp, values[:6]+[hand]) for stamp, values in path for hand in fingers]
        try:
            checked = worker.validate(scene, samples)
        except RuntimeError as error:
            raise RuntimeError(f'충돌 검사 실패: step={step.get("index")} phase={step.get("phase")} '
                               f'label={step.get("label")}: {error}') from error
        if checked != len(samples):
            raise RuntimeError('Offline checker did not acknowledge the complete finger grid')
        step['collision_certificate'] = dict(
            policy=replay.POLICY, scene_sha256=step['scene_before_sha256'],
            trajectory_sha256=file_digest(artifact_path(self.path.parent, step['file'])),
            trajectory_content_sha256=self.trajectory_fingerprint(trajectory),
            context_sha256=replay.motion_context(step), holding=holding,
            finger_range=list(replay.finger_range(step, holding, self.bounds[HAND])), finger_step=replay.FINGER_STEP,
            checked_samples=checked)
        return checked

    @staticmethod
    def trajectory_fingerprint(trajectory):
        # CDR padding is not a canonical encoding; hashing a reserialization can
        # differ even for an unchanged message. Bind both sealed file bytes and
        # deterministic message content instead.
        return digest(json.dumps(message_to_ordereddict(trajectory), sort_keys=True, allow_nan=False).encode())

    def package_context(self, inputs, hardware):
        return dict(input_sha256=inputs, hardware_class=hardware,
                    model_sha256=digest(self.model.encode()), semantic_sha256=digest(self.semantic.encode()),
                    scene_hash_policy=SCENE_HASH_POLICY,
                    recipe_policy=getattr(self, 'recipe_policy', lambda: RECIPE_POLICY)(),
                    replay_policy=replay.POLICY, grasp_policy=GRASP_POLICY,
                    arm_execution_profile=arm_execution_profile(), finger_joint_bounds=list(self.bounds[HAND]))

    def seal_recipe(self, recipe, payloads, context, scene_hash, **extra):
        recipe['path_kind'] = 'physical_stack'
        recipe['replay_policy'] = replay.POLICY
        encoded = (json.dumps(recipe, sort_keys=True, allow_nan=False)+'\n').encode()
        metadata = dict(context, schema=1, path_kind='physical_stack', created_at=time.time(),
                        valid_for_seconds=REVIEW_TTL, recipe_sha256=digest(encoded),
                        artifact_sha256={name: digest(payload) for name, payload in payloads.items()},
                        scene_sha256=scene_hash, validated_samples=recipe['validated_samples'],
                        duration=recipe['duration'],
                        motion_steps=sum(s['kind'] != 'scene' for s in recipe['steps']), **extra)
        metadata['plan_id'] = digest(json.dumps(metadata, sort_keys=True).encode())[:16]
        replay.check_certificates(recipe, metadata)
        return metadata, encoded

    def install_session(self, recipe, payloads, context, scene_hash, **extra):
        recipe, payloads = replay.rename_artifacts(recipe, payloads, 'replay_'+uuid.uuid4().hex[:12]+'_')
        metadata, encoded = self.seal_recipe(recipe, payloads, context, scene_hash, **extra)
        # Publish manifests last. A partially copied package is never executable.
        self.meta_path.unlink(missing_ok=True)
        for name, payload in payloads.items():
            destination = self.path.parent/name
            if destination.exists() or destination.is_symlink() or destination.parent != self.path.parent:
                raise RuntimeError('Restored artifact destination already exists or escapes its directory')
            atomic_write(destination, payload)
        atomic_write(self.path, encoded)
        atomic_write(self.meta_path, (json.dumps(metadata, indent=2)+'\n').encode())
        return metadata, recipe

    def announce_plan(self, metadata, recipe):
        self.display.publish(self.recipe_display(recipe, recipe['steps']))
        print('STACK_PHYSICAL_PLAN '+json.dumps(metadata, sort_keys=True), flush=True)
        print('STACK_STATUS phase=PLAN_READY moved=0 total='+str(recipe['moves'])+' target='+str(recipe['target_cell'])+' layers='+str(recipe['foundation_layers']), flush=True)

    def restore_saved_plan(self, inputs, hardware):
        context = self.package_context(inputs, hardware)
        folder = self.library/replay.context_key(context)
        if not folder.is_dir():
            print('STACK_REPLAY_CACHE_MISS reason=no_prepared_path', flush=True)
            return False
        live_hash = scene_fingerprint(self.scene())
        candidates = sorted((p for p in folder.iterdir() if not p.name.startswith('.') and p.is_dir()),
                            key=lambda p: p.stat().st_mtime, reverse=True)
        for candidate in candidates[:32]:
            self.check_stop()
            try:
                metadata, recipe, payloads = replay.read_package(candidate)
                validate_recipe(recipe)
                replay.check_certificates(recipe, metadata)
                if any(metadata.get(key) != value for key, value in context.items()):
                    raise ValueError('Prepared path configuration differs')
                if metadata['scene_sha256'] != live_hash:
                    raise ValueError('Initial collision scene differs')
                if (not self.fresh() or not self.sensor_fresh()
                        or any(abs(self.values[name]-q) > START_TOLERANCE
                               for name, q in zip(ARM, recipe['initial_joints']))
                        or not replay.initial_hand_matches(recipe, self.values[HAND], self.bounds[HAND])):
                    raise ValueError('Initial measured robot state differs')
            except (ValueError, KeyError, TypeError, OSError) as error:
                print('STACK_REPLAY_CACHE_SKIPPED '+json.dumps(dict(path=str(candidate), reason=str(error))), flush=True)
                continue
            # The archive may be old. Renew only the review session after its
            # exact context/scene/start match; no trajectory/geometry is changed.
            fresh_meta, fresh_recipe = self.install_session(recipe, payloads, context, live_hash,
                source_plan_id=metadata['plan_id'], source_package=str(candidate), collision_rechecks=0)
            print('STACK_REPLAY_CACHE_HIT '+json.dumps(dict(path=str(candidate),
                  source_plan_id=metadata['plan_id'], collision_rechecks=0)), flush=True)
            self.announce_plan(fresh_meta, fresh_recipe)
            return True
        print('STACK_REPLAY_CACHE_MISS reason=no_matching_scene_or_start', flush=True)
        return False

    def prepare_saved_plan(self):
        """Prepare a current-policy path offline; never contact hardware."""
        metadata, recipe = self.load_recipe(preparing=True)
        initial_hash = metadata['scene_sha256']
        payloads = {name: artifact_path(self.path.parent, name).read_bytes()
                    for name in metadata['artifact_sha256']}
        began, samples = time.monotonic(), 0
        with CollisionWorker({'robot_description': self.model, 'robot_description_semantic': self.semantic},
                             self.path.parent,
                             lambda: (self.check_stop(), rclpy.spin_once(self, timeout_sec=0.))) as worker:
            for step, holding in replay.holding_steps(recipe['steps']):
                self.check_stop()
                if step['kind'] == 'scene':
                    continue
                scene = deserialize_message(payloads[step['scene_before_file']], PlanningScene)
                if scene_fingerprint(scene) != step['scene_before_sha256']:
                    raise RuntimeError('Saved scene snapshot differs; generate a new physical plan')
                trajectory = deserialize_message(payloads[step['file']], RobotTrajectory)
                samples += self.certify_motion(worker, scene, step, trajectory, holding)
                print(f'STACK_REPLAY_PREPARING step={step["index"]} total={len(recipe["steps"])} samples={samples}', flush=True)
        recipe['validated_samples'] = samples
        recipe['replay_preparation_seconds'] = time.monotonic()-began
        context = self.package_context(metadata['input_sha256'], metadata['hardware_class'])
        updated, _ = self.seal_recipe(recipe, payloads, context, initial_hash,
                                     source_plan_id=metadata['plan_id'], offline_prepared=True)
        destination = replay.save_package(self.library, updated, recipe, payloads)
        # Preserve the old recipe/session byte for byte. GUI ④ can now load the
        # archive after checking the actual initial layout and measured start.
        print('STACK_REPLAY_SAVED '+json.dumps(dict(path=str(destination), samples=samples,
              seconds=round(time.monotonic()-began, 3), robot_commands=0)), flush=True)

    def validation_worker(self, bounded_contacts):
        return CollisionWorker(
            {'robot_description': self.model, 'robot_description_semantic': self.semantic,
             'bounded_surface_contacts': bounded_contacts},
            self.path.parent, lambda: (self.check_stop(), rclpy.spin_once(self, timeout_sec=0.)))

    def release_validation_worker(self, worker):
        worker.close()

    def plan(self):
        plan_began = time.monotonic()
        self.spin_until(self.fresh)
        self.startup_read_stage = 'physical_plan_preflight'
        try:
            self.check_model()
            kind = self.hardware(require_controller=True)
            self.gripper_runtime()
        finally:
            self.startup_read_stage = None
        previous_fault = read_held_fault()
        if previous_fault is not None and str(previous_fault.get('cause', '')).startswith('SCENE_RPC_PENDING:'):
            raise RuntimeError('이전 계획 장면 쓰기의 응답이 확인되지 않았습니다. 계획 서버 상태를 복구한 뒤 다시 시작하세요.')
        if not self.measured_stop() or not self.sensor_fresh() or (not self.mock and self.sensor.get('grip_state') != 1):
            raise RuntimeError('그리퍼가 비어 있고 정지한 센서 상태를 확인한 뒤 새 경로를 계획하세요. 잡고 있는 블록은 먼저 안전하게 내려놓으세요.')
        clear_held_fault()
        inputs = self.input_hashes()
        self.acquire_stack_lock()
        if getattr(self, 'library', None) is not None and self.restore_saved_plan(inputs, kind):
            return
        raw = self.path.with_name(self.path.name+'.raw.json')
        self.meta_path.unlink(missing_ok=True)
        # The C++ planner owns this same domain lock while generating a new
        # recipe. Do not make the parent conflict with its own child on a cache
        # miss. The outer air-task lock still excludes other physical sessions.
        self.release_stack_lock()
        self.planner(raw)
        # Reacquire before reading artifacts or mutating the validation scene.
        # A competing C++ task must make this fail, never bypass the lock.
        self.acquire_stack_lock()
        search_seconds = time.monotonic()-plan_began
        print(f'STACK_PLAN_TIMING stage=search seconds={search_seconds:.3f}', flush=True)
        recipe = json.loads(raw.read_text())
        recipe['arm_execution_profile'] = arm_execution_profile()
        recipe['grasp_policy'] = GRASP_POLICY
        steps = validate_recipe(recipe)
        initial = self.read_message(recipe['initial_scene_file'], PlanningScene)
        local_snapshots = recipe.get('routing_policy') == INCREMENTAL_ROUTING_POLICY
        if local_snapshots and scene_fingerprint(self.scene()) != scene_fingerprint(initial):
            raise RuntimeError('현재 장면이 한 블록 계획 시작 장면과 다릅니다.')
        hashes, samples_checked, duration = {}, 0, 0.
        worker = None
        validation_began = time.monotonic()
        try:
            worker = self.validation_worker(local_snapshots)
            for step, holding in replay.holding_steps(steps):
                self.check_stop()
                before = self.read_message(step['scene_before_file'], PlanningScene)
                after = self.read_message(step['scene_after_file'], PlanningScene)
                if local_snapshots:
                    applied_before = before
                else:
                    self.apply_snapshot(before)
                    applied_before = self.scene()
                step['scene_before_sha256'] = scene_fingerprint(applied_before)
                if step['kind'] != 'scene':
                    trajectory = self.read_message(step['file'], RobotTrajectory)
                    if step['kind'] == 'arm':
                        trajectory, metrics = self.normalize_arm(trajectory, step)
                        step['metrics'] = metrics
                        validated_name = step['file']+'.validated.cdr'
                        atomic_write(self.path.parent/validated_name, serialize_message(trajectory))
                        step['file'] = validated_name
                    samples_checked += self.certify_motion(worker, applied_before, step, trajectory, holding)
                    step['duration'] = trajectory_duration(trajectory)
                    duration += step['duration']
                if local_snapshots:
                    step['scene_after_sha256'] = scene_fingerprint(after)
                else:
                    self.apply_snapshot(after)
                    step['scene_after_sha256'] = scene_fingerprint(self.scene())
                if step['index'] % 10 == 0:
                    print(f'STACK_PHYSICAL_VALIDATING step={step["index"]} total={len(steps)} samples={samples_checked}', flush=True)
        finally:
            # Stop or validation failure cannot leave the live display in a
            # future attachment/tower scene. No robot commands are sent here.
            try:
                if worker is not None:
                    self.release_validation_worker(worker)
            finally:
                if not local_snapshots:
                    self.apply_snapshot(initial, cancel=False)
        validation_seconds = time.monotonic()-validation_began
        print(f'STACK_PLAN_TIMING stage=collision_validation backend=local_cpp seconds={validation_seconds:.3f} samples={samples_checked}', flush=True)
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
        recipe['planning_timing'] = dict(search_seconds=search_seconds, validation_seconds=validation_seconds,
                                        collision_backend='local_cpp', total_seconds=time.monotonic()-plan_began)
        payloads = {name: artifact_path(self.path.parent, name).read_bytes() for name in hashes}
        metadata, data = self.seal_recipe(recipe, payloads, self.package_context(inputs, kind), initial_fingerprint)
        destination = replay.save_package(self.library, metadata, recipe, payloads)
        atomic_write(self.path, data)
        atomic_write(self.meta_path, (json.dumps(metadata, indent=2)+'\n').encode())
        print('STACK_REPLAY_SAVED '+json.dumps(dict(path=str(destination), samples=samples_checked)), flush=True)
        self.announce_plan(metadata, recipe)

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

    def load_recipe(self, preparing=False):
        if not preparing and read_held_fault() is not None:
            raise RuntimeError('이전 작업의 잡힌 블록 상태를 복구한 뒤 새 경로를 계획하세요.')
        if not preparing and self.param('workspace_clear') is not True:
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
        policy = getattr(self, 'recipe_policy', lambda: RECIPE_POLICY)()
        accepted = PREPARABLE_POLICIES if preparing and policy == RECIPE_POLICY else (policy,)
        if metadata.get('recipe_policy') not in accepted:
            raise RuntimeError('임시 적재 우선순위·진입 높이·속도 정책이 바뀌었습니다. 이전 저장 경로는 보존하고 ④에서 다시 계획하세요.')
        if metadata.get('grasp_policy') != GRASP_POLICY:
            raise RuntimeError('잡기 명령 완료 기준이 바뀌었습니다. 실제 집기·쌓기 경로를 다시 계획하세요.')
        if metadata.get('arm_execution_profile') != arm_execution_profile():
            raise RuntimeError('팔 경로 추종·도착 허용 범위가 바뀌었습니다. 경로를 다시 계획하세요.')
        if preparing:
            self.check_files(metadata, allow_expired=True)
        else:
            self.check_files(metadata)
        recipe = json.loads(self.path.read_text())
        if recipe.get('grasp_policy') != GRASP_POLICY:
            raise RuntimeError('저장된 잡기 명령 완료 기준이 다릅니다. 경로를 다시 계획하세요.')
        if recipe.get('arm_execution_profile') != arm_execution_profile():
            raise RuntimeError('검증된 팔 실행 설정이 경로 파일과 일치하지 않습니다. 다시 계획하세요.')
        validate_recipe(recipe)
        if not preparing:
            if metadata.get('finger_joint_bounds') != list(self.bounds[HAND]):
                raise RuntimeError('Saved finger joint limits differ from this robot model')
            replay.check_certificates(recipe, metadata)
            pinned_meta, pinned_recipe, payloads = replay.read_package(self.path.parent, self.path.name)
            if pinned_meta != metadata or pinned_recipe != recipe:
                raise RuntimeError('Saved path changed while loading')
            self.execution_artifacts = payloads
        return metadata, recipe

    def check_files(self, metadata, allow_expired=False):
        if (metadata.get('arm_execution_profile') != arm_execution_profile()
                or metadata.get('grasp_policy') != GRASP_POLICY
                or metadata.get('input_sha256') != self.input_hashes()
                or metadata.get('model_sha256') != digest(self.model.encode())
                or metadata.get('semantic_sha256') != digest(self.semantic.encode())
                or metadata.get('recipe_sha256') != file_digest(self.path)
                or (not allow_expired and not 0 <= time.time()-metadata.get('created_at', 0) <= metadata.get('valid_for_seconds', 0))
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
        initial_open = (getattr(self, 'prevalidated_replay', False) and step['index'] == 1
                        and step['kind'] == 'hand' and step['phase'] == 'open'
                        and step['collision_certificate']['finger_range'][0] <= self.values[HAND]
                            <= step['collision_certificate']['finger_range'][1])
        if (not (self.carrying and not self.release_stage) and not initial_open
                and not PhysicalStackTask.released_start_matches(self, step)
                and abs(self.values[HAND]-step['start_joints'][-1]) > HAND_START_TOLERANCE):
            raise RuntimeError('그리퍼가 검토한 집기·쌓기 시작 위치에서 바뀌었습니다.')
        if not self.holding_ok():
            raise RuntimeError('그리퍼 실제 형상이 검토한 충돌 검사 범위와 다릅니다.')
        if step['phase'] == 'release_first':
            PhysicalStackTask.first_release_target(self)
        self.check_stop()

    def replay_checkpoint(self, metadata, step, expected_scene):
        """Check measured boundaries/context without resampling the saved path."""
        self.check_stop()
        rclpy.spin_once(self, timeout_sec=0.)
        if (self.input_hashes() != metadata['input_sha256']
                or file_digest(self.path) != metadata['recipe_sha256']
                or not 0 <= time.time()-metadata['created_at'] <= REVIEW_TTL):
            raise RuntimeError('저장 경로의 배치·파일·검토 시간이 바뀌었습니다.')
        if expected_scene != step['scene_before_sha256']:
            raise RuntimeError('Saved physical scene sequence is discontinuous')
        self.prepare_replay_monitors(metadata, expected_scene)
        if not self.fresh() or not self.sensor_fresh() or not self.measured_stop():
            raise RuntimeError('팔·그리퍼의 최신 피드백과 정지를 확인하지 못했습니다.')
        if any(abs(self.values.get(name, math.inf)-q) > START_TOLERANCE
               for name, q in zip(ARM, step['start_joints'])):
            raise RuntimeError('팔이 저장 경로의 시작 자세와 다릅니다.')
        initial_open = (step['index'] == 1 and step['kind'] == 'hand' and step['phase'] == 'open'
                        and step['collision_certificate']['finger_range'][0] <= self.values[HAND]
                            <= step['collision_certificate']['finger_range'][1])
        if (not (self.carrying and not self.release_stage) and not initial_open
                and not PhysicalStackTask.released_start_matches(self, step)
                and abs(self.values.get(HAND, math.inf)-step['start_joints'][-1]) > HAND_START_TOLERANCE):
            raise RuntimeError('그리퍼가 저장 경로의 시작 자세와 다릅니다.')
        if not self.holding_ok():
            raise RuntimeError('그리퍼 형상이 검토 범위를 벗어났습니다.')
        if step['phase'] == 'release_first':
            PhysicalStackTask.first_release_target(self)
        self.check_stop()

    def bind_certified_motion(self, step, trajectory, expected_scene):
        """Bind measured hand geometry to an already checked certificate; no RPC."""
        cert = step['collision_certificate']
        if (cert['scene_sha256'] != expected_scene or cert['context_sha256'] != replay.motion_context(step)
                or self.trajectory_fingerprint(trajectory) != cert['trajectory_content_sha256']):
            raise RuntimeError('Saved motion no longer matches its collision certificate')
        low, high = cert['finger_range']
        if self.carrying and not self.release_stage:
            interval = self.holding_interval()
            if not self.holding_ok() or interval is None:
                raise RuntimeError('실제 파지 각도·센서 상태가 안정적이지 않습니다. 팔은 정지합니다.')
            if not low <= interval[0] <= interval[1] <= high:
                if self.holding_context_verified(step, trajectory, expected_scene):
                    return
                measured = self.values.get(HAND)
                active = getattr(self, 'active_recipe', None) or {}
                print('STACK_FINGER_EXTENSION_REQUIRED '+json.dumps(dict(
                    step=step['index'], label=step.get('label'), layer=active.get('layers'),
                    source_cells=active.get('source_cells'), measured=measured,
                    holding_reference=getattr(self, 'holding_reference', None),
                    required_range=interval, certified_range=[low, high],
                    hold_tolerance=HAND_HOLD_TOLERANCE), sort_keys=True), flush=True)
                PhysicalStackTask.extend_measured_fingers(self, step, trajectory, expected_scene)
                return
            self.verified_holding_context = dict(
                key=self.holding_context_key(step, trajectory, expected_scene), interval=interval,
                scene=expected_scene, step=step['index'], samples=cert['checked_samples'], source='saved_certificate')
        else:
            self.verified_holding_context = None
            margin = replay.EMPTY_MOTION_TOLERANCE if step['kind'] == 'arm' else 0.
            actual = self.values[HAND]
            required_low = max(self.bounds[HAND][0], actual-margin)
            required_high = min(self.bounds[HAND][1], actual+margin)
            if not low-1e-12 <= required_low <= required_high <= high+1e-12:
                raise RuntimeError('실측 그리퍼가 저장 경로의 사전 검증 범위 밖입니다.')

    def extend_measured_fingers(self, step, trajectory, expected_scene):
        """Check only the missing finger grid, keeping the sealed arm path."""
        if self.holding_context_verified(step, trajectory, expected_scene):
            return
        began = time.monotonic()
        PhysicalStackTask.first_release_target(self)  # Reject an unusable grasp before lifting.
        interval = self.holding_interval()
        before = [self.values[name] for name in JOINTS]
        key = self.holding_context_key(step, trajectory, expected_scene)
        cert_low, cert_high = step['collision_certificate']['finger_range']
        low, high = interval
        if not self.bounds[HAND][0] <= low <= high <= self.bounds[HAND][1]:
            raise RuntimeError('실측 파지 검사 범위가 그리퍼 관절 한계를 벗어납니다.')
        fingers = []
        if low < cert_low:
            count = max(1, math.ceil((cert_low-low)/replay.FINGER_STEP))
            fingers.extend(low+(cert_low-low)*i/count for i in range(count))
        if high > cert_high:
            count = max(1, math.ceil((high-cert_high)/replay.FINGER_STEP))
            fingers.extend(cert_high+(high-cert_high)*i/count for i in range(1,count+1))
        if not fingers:
            raise RuntimeError('추가 파지 형상 표본을 생성하지 못했습니다.')
        path = (self.tracking_motion_samples(step, trajectory) if step['kind']=='arm'
                else [(0.,step['start_joints'])])
        if len(path)*len(fingers) > 1_000_000:
            raise RuntimeError('추가 파지 형상 검사의 표본 수가 한계를 넘었습니다.')
        scene = self.read_message(step['scene_before_file'], PlanningScene)
        if scene_fingerprint(scene) != expected_scene:
            raise RuntimeError('추가 파지 검사 장면이 저장 경로와 다릅니다.')
        def stationary():
            self.check_stop()
            if (not self.fresh() or not self.sensor_fresh() or not self.holding_ok()
                    or self.holding_interval() != interval
                    or any(abs(self.values[name]-q) > START_TOLERANCE for name,q in zip(ARM,before))):
                raise RuntimeError('추가 파지 형상 검사 중 팔·손가락 상태가 바뀌었습니다.')
        stationary()
        active = getattr(self,'active_recipe',None) or {}
        worker = self.validation_worker(active.get('routing_policy') == INCREMENTAL_ROUTING_POLICY)
        original_poll = worker.poll
        def poll():
            original_poll()
            queries = getattr(self,'replay_queries',None)
            if queries:self.poll_flight_queries(queries)
            stationary()
        worker.poll = poll
        try:
            samples = [(stamp,q[:6]+[finger]) for stamp,q in path for finger in fingers]
            checked = worker.validate(scene,samples)
            if checked != len(samples):
                raise RuntimeError('추가 파지 형상 검사가 모든 표본을 확인하지 못했습니다.')
            stationary()
            validation_seconds = time.monotonic()-began
            self.verified_holding_context = dict(key=key,interval=interval,scene=expected_scene,
                step=step['index'],samples=checked,validation_seconds=validation_seconds,
                source='local_cpp_finger_extension')
            print('STACK_FINGER_EXTENSION_VALIDATED '+json.dumps(dict(step=step['index'],
                measured=self.values[HAND],required_range=interval,previous_range=[cert_low,cert_high],
                added_fingers=fingers,samples=checked,seconds=round(validation_seconds,3))),flush=True)
        finally:
            worker.poll = original_poll
            self.release_validation_worker(worker)

    def live_arm_validation(self, step, trajectory, expected_scene):
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('Live scene changed before arm validation')
        if self.carrying and not self.release_stage:
            self.validate_holding_envelope(step, trajectory, expected_scene)
        else:
            self.verified_holding_context = None
            self.validate_samples(self.tracking_motion_samples(step, trajectory, actual_hand=self.values[HAND]))
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('Live scene changed during arm validation')

    def validate_holding_envelope(self, step, trajectory, expected_scene):
        if not self.holding_ok():
            raise RuntimeError('Fresh reviewed finger geometry is required for collision validation')
        key = self.holding_context_key(step, trajectory, expected_scene)
        if self.holding_context_verified(step, trajectory, expected_scene):
            return
        interval = self.holding_interval()
        low, high = interval
        if step['kind'] == 'hand':
            target = (PhysicalStackTask.first_release_target(self) if step['phase']=='release_first'
                      else step['hand_target'])
            low, high = min(low, target-.003), max(high, target+.003)
        count = max(1, math.ceil((high-low)/HAND_ENVELOPE_STEP))
        fingers = sorted(set([low+(high-low)*i/count for i in range(count+1)] +
                             [self.values[HAND], self.holding_reference, HAND_TARGETS['grasp']]))
        if any(not self.bounds[HAND][0] <= q <= self.bounds[HAND][1] for q in fingers):
            raise RuntimeError('Physical holding envelope exceeds the gripper joint limit')
        if step['kind'] == 'arm':
            arm_samples = self.tracking_motion_samples(step, trajectory, actual_hand=fingers[0])
            if len(arm_samples)*len(fingers) > 1_000_000:
                raise RuntimeError('Physical holding envelope exceeds the bounded validation workload')
            samples = [(stamp, values[:6]+[hand]) for stamp, values in arm_samples for hand in fingers]
        else:
            arm_samples = [(0., step['start_joints'][:6])]
            samples = [(0., step['start_joints'][:6]+[hand]) for hand in fingers]
        checked = self.validate_samples(samples)
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('Live scene changed during physical holding-envelope validation')
        self.verified_holding_context = dict(key=key, interval=interval, scene=expected_scene,
                                            step=step['index'], samples=checked)
        print('STACK_HOLDING_ENVELOPE_VALIDATED '+json.dumps(dict(
            step=step['index'], lower=interval[0], upper=interval[1],
            finger_shapes=len(fingers), arm_samples=len(arm_samples), checked=checked)), flush=True)

    def live_hand_validation(self, step, trajectory, expected_scene):
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('Live scene changed before finger validation')
        if self.carrying and not self.release_stage:
            self.validate_holding_envelope(step, trajectory, expected_scene)
        else:
            self.verified_holding_context = None
            actual_step = copy.deepcopy(step)
            actual_step['start_joints'][-1] = self.values[HAND]
            actual_trajectory = copy.deepcopy(trajectory)
            actual_trajectory.joint_trajectory.points[0].positions = [self.values[HAND]]
            self.validate_samples(self.motion_samples(actual_step, actual_trajectory))
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('Live scene changed during finger validation')

    def approve_continuous_recipe(self, metadata, recipe, steps, expected_scene):
        approval_metadata = dict(metadata, gripper_position=self.values[HAND])
        self.await_approval(lambda: self.recipe_display(recipe, steps), approval_metadata, None, 1, 1,
                            f'실제 집기·쌓기 전체 경로 · 블록 {recipe["moves"]}개 · 임시 적재 {recipe["buffer_moves"]}회',
                            context=lambda *_: self.replay_checkpoint(metadata, steps[0], expected_scene))

    def execute(self):
        began = time.monotonic()
        self.acquire_stack_lock()
        metadata, recipe = self.load_recipe()
        self.prevalidated_replay = True
        self.active_recipe = recipe
        steps = recipe['steps']
        expected_scene = metadata['scene_sha256']
        self.spin_until(self.fresh)
        self.guard(metadata, steps[0], expected_scene)
        self.prepare_replay_monitors(metadata, expected_scene)
        print('STACK_REPLAY_READY '+json.dumps(dict(plan_id=metadata['plan_id'],
              collision_rechecks=0, startup_seconds=round(time.monotonic()-began, 3))), flush=True)
        motions = [step for step in steps if step['kind'] != 'scene']
        if self.execution_mode == 'continuous':
            approval = getattr(self, 'approve_continuous_recipe', None)
            if approval is None:
                PhysicalStackTask.approve_continuous_recipe(self, metadata, recipe, steps, expected_scene)
            else:
                approval(metadata, recipe, steps, expected_scene)
        motion_index, completed = 0, 0
        for step in steps:
            self.replay_checkpoint(metadata, step, expected_scene)
            if expected_scene != step['scene_before_sha256']:
                raise RuntimeError('Physical recipe scene sequence is discontinuous')
            if step['kind'] == 'scene':
                if step['phase'] == 'attach' and not self.holding_ok():
                    raise RuntimeError('그리퍼 형상의 충돌 검사 범위를 확인한 뒤 팔을 들어 올리세요.')
                self.verified_holding_context = None
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
                    phase = 'BUFFERED' if step['temporary'] else 'PLACED'
                    print(f'STACK_TRANSFER_COMPLETED block={step["block_id"]} destination={step["destination_cell"]} temporary={int(step["temporary"])} moved={completed}', flush=True)
                    print(f'STACK_STATUS phase={phase} moved={completed} total={recipe["moves"]} target={recipe["target_cell"]} layers={completed+recipe["foundation_layers"]}', flush=True)
                continue
            motion_index += 1
            trajectory = self.read_message(step['file'], RobotTrajectory)
            if self.execution_mode == 'step':
                self.bind_certified_motion(step, trajectory, expected_scene)
                approval_metadata = dict(metadata, gripper_position=self.values[HAND])
                if self.carrying and not self.release_stage:
                    if not self.holding_context_verified(step, trajectory, expected_scene):
                        raise RuntimeError('Pending physical motion lacks its validated holding interval')
                    approval_metadata['holding_context_key'] = self.verified_holding_context['key']
                self.await_approval(lambda: self.recipe_display(recipe, [step]), approval_metadata, None, motion_index, len(motions), step['label'],
                                    context=lambda *_: self.replay_checkpoint(metadata, step, expected_scene))
            self.bind_certified_motion(step, trajectory, expected_scene)
            self.replay_checkpoint(metadata, step, expected_scene)
            self.run_motion(step, trajectory, metadata, expected_scene,
                            motion_index if self.execution_mode == 'step' else 1,
                            len(motions) if self.execution_mode == 'step' else 1)
            if step['phase'] == 'grasp':
                self.carrying = True
                # run_motion fixed the finger-shape anchor after the completed
                # command, before the stationary geometry observation.
            elif step['phase'] == 'release_first':
                self.release_stage = 1
            elif step['phase'] == 'release_clearance':
                self.release_stage = 0
                self.holding_reference = None
                self.verified_holding_context = None
            expected_scene = step['scene_after_sha256']
        if self.execution_mode == 'continuous':
            self.phase('STACK_STEP_COMPLETED', index=1, total=1, label='실제 집기·쌓기 전체 경로')
        print(f'STACK_STATUS phase=DONE moved={recipe["moves"]} total={recipe["moves"]} target={recipe["target_cell"]} layers={recipe["layers"]}', flush=True)

    def run_motion(self, step, trajectory, metadata, scene_hash, index, total):
        preparation_began = time.monotonic()
        fast = getattr(self, 'prevalidated_replay', False)
        extension = getattr(self, 'verified_holding_context', None) or {}
        has_extension = (fast and extension.get('source') == 'local_cpp_finger_extension'
                         and extension.get('step') == step['index'])
        additional_finger_samples = extension['samples'] if has_extension else 0
        additional_finger_validation_seconds = extension['validation_seconds'] if has_extension else 0.
        hand = step['kind'] == 'hand'
        action = GripperCommand if hand else FollowJointTrajectory
        endpoint = (('' if self.mock else '/gripper')+'/dh_ag95_gripper_controller/gripper_cmd'
                    if hand and fast else self.gripper_runtime() if hand else f'/{CONTROLLER}/follow_joint_trajectory')
        if fast:
            key = (action, endpoint)
            if key not in self.motion_clients:
                self.motion_clients[key] = ActionClient(self, action, endpoint)
            client = self.motion_clients[key]
        else:
            client = ActionClient(self, action, endpoint)
        self.spin_until(client.server_is_ready)
        if fast:
            self.replay_checkpoint(metadata, step, scene_hash)
        else:
            self.guard(metadata, step, scene_hash)
        goal = action.Goal()
        if hand:
            hand_target = (PhysicalStackTask.first_release_target(self) if step['phase']=='release_first'
                           else float(step['hand_target']))
            low, high = step['collision_certificate']['finger_range'] if fast else (0.,.93)
            if not low <= hand_target <= high:
                raise RuntimeError('실측 각도에 맞춘 열림 목표가 검증된 형상 범위를 벗어납니다.')
            goal.command.position = hand_target
            goal.command.max_effort = 0.
        else:
            profile = (getattr(self, 'motion_execution_profile', None) or arm_execution_profile)()
            goal.trajectory = trajectory.joint_trajectory
            goal.path_tolerance = [JointTolerance(name=name, position=tolerance) for name, tolerance in
                                   zip(ARM, profile['phase_path_position'][step['phase']])]
            goal.goal_tolerance = [JointTolerance(name=name, position=profile['goal_position'],
                                                 velocity=profile['goal_velocity']) for name in ARM]
            goal.goal_time_tolerance = Duration(sec=profile['goal_time_seconds'])
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
        held_key = None
        last_arm_feedback = {}
        def arm_feedback(message):
            feedback = message.feedback
            names = list(feedback.joint_names)
            if len(names) != len(ARM) or set(names) != set(ARM):
                return
            record = {}
            for field in ('desired', 'actual', 'error'):
                values = list(getattr(feedback, field).positions)
                if len(values) != len(names) or not all(math.isfinite(value) for value in values):
                    return
                record[field] = {name: values[names.index(name)] for name in ARM}
            last_arm_feedback.clear()
            last_arm_feedback.update(record, received=time.monotonic())
        try:
            if getattr(self, 'carrying', False) and not self.release_stage:
                if not self.holding_context_verified(step, trajectory, scene_hash):
                    raise RuntimeError('Physical motion has no matching validated holding envelope')
                held_key = self.verified_holding_context['key']
            queries = self.replay_queries if fast else self.flight_queries(metadata, scene_hash)
            self.prewarm_flight_queries(queries)
            if not self.measured_stop(seconds=.6):
                stop_with_reason(self, 'STANDSTILL_UNCONFIRMED_AFTER_MONITOR_PREWARM')
            # Warm-up spins callbacks, so bind dispatch to the same stationary
            # arm/finger state which was reviewed before discovery began.
            if (any(abs(self.values.get(name, math.inf)-before[name]) > START_TOLERANCE for name in ARM)
                    or (held_key is None and abs(self.values.get(HAND, math.inf)-before[HAND]) > .003)):
                stop_with_reason(self, 'START_STATE_CHANGED_DURING_MONITOR_PREWARM')
            if (not self.holding_ok() or (held_key is not None and
                    (not self.verified_holding_ok(held_key)
                     or not self.holding_context_verified(step, trajectory, scene_hash)))):
                stop_with_reason(self, 'HAND_GEOMETRY_UNVERIFIED', stage='monitor_prewarm')
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
                send_options = dict(goal_uuid=goal_id)
                if not hand:
                    send_options['feedback_callback'] = arm_feedback
                self.goal_pending = client.send_goal_async(goal, **send_options)
                while not self.goal_pending.done():
                    rclpy.spin_once(self, timeout_sec=.02)
                    self.poll_flight_queries(queries)
                    if not self.fresh():
                        stop_with_reason(self, 'JOINT_FEEDBACK_STALE', stage='goal_acknowledgement')
                    elif not self.sensor_fresh():
                        stop_with_reason(self, 'GRIPPER_SENSOR_STALE', stage='goal_acknowledgement')
                    elif not hand and held_key is not None and not self.verified_holding_ok(held_key):
                        stop_with_reason(self, 'HAND_GEOMETRY_UNVERIFIED', stage='goal_acknowledgement',
                                         actual=self.values.get(HAND), interval=self.holding_interval(),
                                         grip_state=self.sensor.get('grip_state'))
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
                                     **PhysicalStackTask.sensor_stale_details(self))
                elif now-began > timeout:
                    stop_with_reason(self, 'MOTION_TIMEOUT', service=endpoint,
                                     elapsed=round(now-began, 3), limit=timeout)
                if hand:
                    if any(abs(self.values.get(name, math.inf)-before[name]) > START_TOLERANCE for name in ARM):
                        stop_with_reason(self, 'ARM_MOVED_DURING_HAND_COMMAND')
                elif held_key is None and abs(self.values.get(HAND, math.inf)-before[HAND]) > .003:
                    stop_with_reason(self, 'HAND_MOVED_DURING_ARM_COMMAND',
                                     expected=before[HAND], actual=self.values.get(HAND))
                elif (not self.holding_ok() or (held_key is not None and not self.verified_holding_ok(held_key))):
                    stop_with_reason(self, 'HAND_GEOMETRY_OUTSIDE_REVIEW', stage='motion',
                                     actual=self.values.get(HAND), interval=self.holding_interval(),
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
            terminal_at = time.monotonic()
            quiet = self.measured_stop()
            arm_result = None
            if not hand:
                feedback = {key: value for key, value in last_arm_feedback.items() if key != 'received'} or None
                if feedback is not None:
                    feedback['age_seconds'] = round(time.monotonic()-last_arm_feedback['received'], 3)
                arm_result = arm_result_record(step, result, self.values, quiet, feedback,
                                               execution_profile=profile)
                print('STACK_ARM_RESULT '+json.dumps(arm_result, sort_keys=True), flush=True)
            if quiet and result.status in (4, 5, 6):
                self.goal = self.goal_pending = None
                if self.stopped or hand or arm_result['classification'] == 'SUCCESS':
                    status = 'STACK_STOPPED_CONFIRMED' if self.stopped else 'STACK_MOTION_SETTLED'
                    print(status+' physical motion terminal and arm/gripper standstill', flush=True)
            if not hand:
                if held_key is not None and not self.verified_holding_ok(held_key):
                    stop_with_reason(self, 'HAND_GEOMETRY_OUTSIDE_REVIEW', stage='motion_terminal',
                                     actual=self.values.get(HAND), interval=self.holding_interval(),
                                     grip_state=self.sensor.get('grip_state'))
                elif held_key is None and abs(self.values.get(HAND, math.inf)-before[HAND]) > .003:
                    stop_with_reason(self, 'HAND_MOVED_DURING_ARM_COMMAND', stage='motion_terminal',
                                     expected=before[HAND], actual=self.values.get(HAND))
            elif any(abs(self.values.get(name, math.inf)-before[name]) > START_TOLERANCE for name in ARM):
                stop_with_reason(self, 'ARM_MOVED_DURING_HAND_COMMAND', stage='motion_terminal')
            if self.stopped or not quiet or not self.fresh():
                reason = getattr(self, 'stop_reason', None)
                raise RuntimeError('Actual pick/place motion stopped; current scene preserved'+
                                   ('; '+json.dumps(reason, sort_keys=True) if reason else '')+
                                   ('; '+json.dumps(arm_result, sort_keys=True) if arm_result is not None else ''))
            if hand:
                action_result = getattr(result, 'result', None)
                hand_result = dict(
                    phase=step['phase'], target=hand_target, nominal_target=step['hand_target'], start=before[HAND],
                    measured=self.values[HAND], position_error=self.values[HAND]-hand_target,
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
                    completed = (result.status == 4 and getattr(action_result, 'reached_goal', False)
                                 and not getattr(action_result, 'stalled', False)
                                 and abs(self.values[HAND]-step['hand_target']) <= HAND_COMMAND_TOLERANCE)
                    stalled = result.status == 6 and getattr(action_result, 'stalled', False)
                    if not (completed or stalled):
                        raise RuntimeError(f'{HAND_TARGETS["grasp"]:.2f} 잡기 명령 완료를 확인하지 못했습니다. '+
                                           json.dumps(hand_result, sort_keys=True))
                    self.holding_reference = self.values[HAND]
                    self.verified_holding_context = None
                    self.observe_grasp_settle(before)
                elif (result.status != 4 or abs(self.values[HAND]-hand_target) > .003
                      or result.result.stalled or not result.result.reached_goal):
                    raise RuntimeError('검토한 그리퍼 준비·열림 위치를 확인하지 못했습니다. '+
                                       json.dumps(hand_result, sort_keys=True))
                if step['phase']=='release_first':
                    self.last_first_release_target = hand_target
            elif arm_result['classification'] != 'SUCCESS':
                messages = {
                    'ARM_PATH_TOLERANCE_VIOLATED': '이동 중 팔 경로 추종 허용 오차를 벗어나 제어기가 중단했습니다.',
                    'ARM_GOAL_TOLERANCE_VIOLATED': '팔 도착 위치·속도 허용 조건을 만족하지 못했습니다.',
                    'ARM_ENDPOINT_MISMATCH': '실측 팔 관절값이 검토한 도착 목표와 다릅니다.'}
                raise RuntimeError(messages.get(arm_result['classification'], '팔 동작 결과를 확인하지 못했습니다.')+
                                   ' '+json.dumps(arm_result, sort_keys=True))
            if self.execution_mode == 'step':
                self.phase('STACK_STEP_COMPLETED', index=index, total=total, label=step['label'])
            if fast:
                print('STACK_REPLAY_STEP_TIMING '+json.dumps(dict(step=step['index'], phase=step['phase'],
                      prepare_seconds=round(began-preparation_began, 3),
                      action_seconds=round(terminal_at-began, 3),
                      settle_seconds=round(time.monotonic()-terminal_at, 3),
                      # One recheck is one additional grid-validation pass.
                      collision_rechecks=int(has_extension),
                      additional_finger_samples=additional_finger_samples,
                      additional_finger_validation_seconds=round(additional_finger_validation_seconds, 3))), flush=True)
        finally:
            if not fast and 'queries' in locals():
                self.destroy_flight_queries(queries)
            self.cancel_remaining_goal()
            if self.stopped and step['phase'] == 'grasp':
                self.preserve_contact_after_stop()
            if not fast:
                client.destroy()

    def prepare_replay_monitors(self, metadata, scene_hash):
        queries = self.replay_queries
        if queries is None:
            queries = self.replay_queries = self.flight_queries(metadata, scene_hash)
            self.replay_monitor_scene = scene_hash
        elif self.replay_monitor_scene != scene_hash:
            scene_query = next(q for q in queries if q['service'] == '/get_planning_scene')
            self.destroy_flight_queries([scene_query])
            scene_query.update(
                valid=lambda reply: scene_fingerprint(reply.scene) == scene_hash,
                describe=lambda reply: dict(expected_sha256=scene_hash, actual_sha256=scene_fingerprint(reply.scene)),
                sent=-math.inf, verified=False, last_ok=-math.inf)
            self.replay_monitor_scene = scene_hash
        self.prewarm_flight_queries(queries)

    def flight_queries(self, metadata, scene_hash):
        """Fresh predicates/futures per motion, using clients warmed before goals."""
        expected = 'mock_components/GenericSystem' if self.mock else 'ur_robot_driver/URPositionHardwareInterface'
        grip_expected = 'mock_components/GenericSystem' if self.mock else 'dh_gripper_driver/DHGripperHardwareInterface'
        grip_manager = '/controller_manager' if self.mock else '/gripper/controller_manager'
        request_scene = GetPlanningScene.Request()
        request_scene.components.components = 1023
        queries = []
        def active_hardware(reply, plugin, joints):
            return any(c.class_type == plugin and c.state.id == 3 and all(
                name+'/position' in {interface.name for interface in c.command_interfaces if interface.is_claimed}
                for name in joints) for c in reply.component)
        arm_joints = ARM if getattr(self, 'prevalidated_replay', False) else []
        hand_joints = [HAND] if getattr(self, 'prevalidated_replay', False) else []
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
            lambda reply: active_hardware(reply, expected, arm_joints),
            lambda reply: dict(hardware=[dict(plugin=c.class_type, state=c.state.id) for c in reply.component]))
        add(ListHardwareComponents, grip_manager+'/list_hardware_components', ListHardwareComponents.Request(), 2.,
            lambda reply: active_hardware(reply, grip_expected, hand_joints),
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
        if getattr(self, 'prevalidated_replay', False):
            request = GetParameters.Request()
            request.names = ['allow_trajectory_execution']
            add(GetParameters, '/move_group/get_parameters', request, 2.,
                lambda reply: bool(reply.values and reply.values[0].bool_value))
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
                stop_with_reason(self, 'GRIPPER_SENSOR_STALE', stage='monitor_prewarm',
                                 **PhysicalStackTask.sensor_stale_details(self))
            self.poll_flight_queries(queries, warming=True)
            self.check_stop()
            if all(query['verified'] and now-query['last_ok'] <= 2. and
                   (query['future'] is None or now-query['sent'] <= 2.) for query in queries):
                # Remove any in-flight warm-up reads before a new motion/event
                # can change its expected scene. Late replies cannot satisfy a
                # new future: rclpy matches the service request sequence ID.
                if not getattr(self, 'prevalidated_replay', False):
                    self.destroy_flight_queries(queries)
                    for query in queries:
                        query['sent'] = -math.inf
                if time.monotonic()-began >= .05 or not getattr(self, 'prevalidated_replay', False):
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
        if getattr(self, 'replay_queries', None):
            self.destroy_flight_queries(self.replay_queries)
            self.replay_queries = None
        for client in getattr(self, 'monitor_clients', {}).values():
            self.destroy_client(client)
        self.monitor_clients = {}
        for client in getattr(self, 'motion_clients', {}).values():
            client.destroy()
        self.motion_clients = {}

    def preserve_contact_after_stop(self):
        """Do not erase a physically held cube when Stop arrives near attachment."""
        attempt = self.grasp_attempt
        if (self.mock or attempt is None or self.active_recipe is None
                or not self.sensor_fresh(after=attempt['sensor_sequence'])
                or not self.fresh()
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
        elif self.mode == 'physical_prepare':
            self.prepare_saved_plan()
        else:
            raise ValueError('Physical stack mode must be physical_plan, physical_prepare or physical_run')


def main(task_class=PhysicalStackTask):
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdigit():
        raise ValueError('ROS_DOMAIN_ID must be numeric')
    lock = os.open(f'/tmp/ur5e_air_task_{domain}.lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
    fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    task = task_class()
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
        task.release_stack_lock()
        task.close_monitor_clients()
        task.close_service_clients()
        task.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(lock)
    raise SystemExit(code)
