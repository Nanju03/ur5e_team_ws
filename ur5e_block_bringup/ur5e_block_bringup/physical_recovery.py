"""Reviewed arm recovery, preserving fingers and collision geometry by default.

The optional legacy empty-gripper mode also opens the hand and resets the scene.
Preserving recovery accepts any initial grip-state code and sends arm goals only.
"""
import copy
import fcntl
import json
import math
import os
import re
import signal
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.serialization import serialize_message
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectoryPoint

from .air_task import ARM, HAND, START_TOLERANCE, atomic_write, digest, file_digest
from .air_trajectory import trajectory_metrics
from .physical_held import HOME_PRESERVE_POLICY, EMPTY_UNLOCK_POLICY, clear_held_fault, read_held_fault, write_held_fault
from .physical_stack import (JOINTS, HAND_TARGETS, PhysicalStackTask, artifact_path,
                             arm_execution_profile, scene_fingerprint, stop_with_reason, trajectory_duration)

RECOVERY_POLICY = 'physical_empty_recovery_v1'
RECOVERY_TTL = 600.
PREFIX = 'ur5e_stack/'


def recovery_policy(task):
    return HOME_PRESERVE_POLICY if getattr(task, 'preserve_gripper', False) else RECOVERY_POLICY


def home_joints(semantic, bounds):
    """Use exactly the configured six-axis named state; never a guessed pose."""
    states = ET.fromstring(semantic).findall("group_state[@group='ur5e_arm'][@name='home']")
    if len(states) != 1:
        raise ValueError('복구에 사용할 ur5e_arm/home 자세가 하나 있어야 합니다.')
    values = {}
    for joint in states[0].findall('joint'):
        name, raw = joint.get('name'), joint.get('value', '').split()
        if name in values or len(raw) != 1:
            raise ValueError('복구 home 관절 설정이 올바르지 않습니다.')
        values[name] = float(raw[0])
    if set(values) != set(ARM) or any(not math.isfinite(values[name]) or not
            bounds[name][0] <= values[name] <= bounds[name][1] for name in ARM):
        raise ValueError('복구 home 자세는 관절 한계 안의 팔 관절 6개여야 합니다.')
    return [values[name] for name in ARM]


def without_owned_scene(scene):
    """Remove only task objects and their ACM rows; retain foreign geometry."""
    result = copy.deepcopy(scene)
    result.world.collision_objects = [o for o in result.world.collision_objects if not o.id.startswith(PREFIX)]
    result.robot_state.attached_collision_objects = [o for o in result.robot_state.attached_collision_objects
                                                    if not o.object.id.startswith(PREFIX)]
    result.object_colors = [o for o in result.object_colors if not o.id.startswith(PREFIX)]
    matrix = result.allowed_collision_matrix
    size = len(matrix.entry_names)
    if len(matrix.entry_values) != size or any(len(row.enabled) != size for row in matrix.entry_values):
        raise ValueError('Malformed live collision matrix; recovery will not overwrite it')
    keep = [i for i, name in enumerate(matrix.entry_names) if not name.startswith(PREFIX)]
    matrix.entry_names = [matrix.entry_names[i] for i in keep]
    rows = [copy.deepcopy(matrix.entry_values[i]) for i in keep]
    for row in rows:
        row.enabled = [row.enabled[i] for i in keep]
    matrix.entry_values = rows
    if len(matrix.default_entry_names) != len(matrix.default_entry_values):
        raise ValueError('Malformed live default collision matrix')
    defaults = [(name, enabled) for name, enabled in zip(matrix.default_entry_names, matrix.default_entry_values)
                if not name.startswith(PREFIX)]
    matrix.default_entry_names = [name for name, _ in defaults]
    matrix.default_entry_values = [enabled for _, enabled in defaults]
    return result


def transform_pose(transform):
    """Convert an FK matrix to a normalized quaternion, including half turns."""
    m = transform[:3, :3]
    trace = float(m.trace())
    if trace > 0.:
        scale = math.sqrt(trace+1.)*2.
        q = [(m[2, 1]-m[1, 2])/scale, (m[0, 2]-m[2, 0])/scale,
             (m[1, 0]-m[0, 1])/scale, scale/4.]
    else:
        index = max(range(3), key=lambda i: m[i, i])
        nxt, last = (index+1) % 3, (index+2) % 3
        scale = math.sqrt(1.+m[index, index]-m[nxt, nxt]-m[last, last])*2.
        q = [0., 0., 0., 0.]
        q[index], q[nxt], q[last], q[3] = (scale/4., (m[nxt, index]+m[index, nxt])/scale,
            (m[last, index]+m[index, last])/scale, (m[last, nxt]-m[nxt, last])/scale)
    norm = math.sqrt(sum(float(v)**2 for v in q))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in transform[:3, 3]]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [float(v/norm) for v in q]
    return pose


def canonical_recovery_path(source):
    """Keep planner interior timing, with independently checked stopped ends."""
    names = list(source.joint_trajectory.joint_names)
    if (len(names) != 6 or set(names) != set(ARM) or source.multi_dof_joint_trajectory.points
            or source.multi_dof_joint_trajectory.joint_names):
        raise ValueError('Recovery planner must return exactly the six arm joints')
    indices = [names.index(name) for name in ARM]
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(ARM)
    previous = None
    for source_point in source.joint_trajectory.points:
        if any(len(getattr(source_point, field)) != 6 for field in ('positions', 'velocities', 'accelerations')):
            raise ValueError('Recovery planner returned incomplete positions/velocities/accelerations')
        positions = [float(source_point.positions[i]) for i in indices]
        velocities = [float(source_point.velocities[i]) for i in indices]
        accelerations = [float(source_point.accelerations[i]) for i in indices]
        if not all(math.isfinite(value) for values in (positions, velocities, accelerations) for value in values):
            raise ValueError('Recovery planner returned nonfinite positions/velocities/accelerations')
        if previous is not None:
            delta = max(abs(a-b) for a, b in zip(previous, positions))
            if delta > .35:
                raise ValueError('Recovery planner returned a joint jump larger than 0.35rad')
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities, point.accelerations = velocities, accelerations
        point.time_from_start = copy.deepcopy(source_point.time_from_start)
        result.joint_trajectory.points.append(point)
        previous = positions
    if len(result.joint_trajectory.points) < 2:
        raise ValueError('Recovery planner returned no complete arm movement')
    for point in (result.joint_trajectory.points[0], result.joint_trajectory.points[-1]):
        point.velocities, point.accelerations = [0.]*6, [0.]*6
    # This checks finite full vectors, strictly increasing normalized times,
    # canonical joint order, and extrema of every quintic interpolation. The
    # caller then rescales against .05 phase limits and collision-checks the
    # exact resulting polynomial, including the adjusted boundary derivatives.
    trajectory_metrics(result)
    return result


class PhysicalRecoveryTask(PhysicalStackTask):
    def motion_execution_profile(self):
        """Recovery retains its strict tracking threshold after stacking changes."""
        return arm_execution_profile(path_tolerance=.005)

    def __init__(self):
        super().__init__()
        self.path_kind = 'physical_recovery'
        self.recovery_metadata = None
        self.recovery_steps = []
        self.recovery_fault = None
        self.recovery_finished = False
        self.preserve_gripper = (self.has_parameter('recovery_preserve_gripper')
                                 and self.param('recovery_preserve_gripper') is True)
        self.recovery_hand_position = None
        self.recovery_initial_grip_state = None

    def require_empty(self, stationary=True):
        """Check recovery readiness; only the legacy mode requires code 1."""
        self.check_stop()
        if getattr(self, 'preserve_gripper', False):
            if self.param('workspace_clear') is not True:
                raise RuntimeError('초기 자세 복귀 전에 작업 공간과 현재 그리퍼·부착 물체를 확인하세요.')
            if not self.fresh() or not self.sensor_fresh():
                raise RuntimeError('초기 자세 복귀에 사용할 최신 관절·그리퍼 센서를 확인하지 못했습니다.')
            target = getattr(self, 'recovery_hand_position', None)
            if target is not None and abs(self.values.get(HAND, math.inf)-target) > .003:
                raise RuntimeError('검토한 복귀 경로의 그리퍼 자세가 바뀌었습니다.')
            if stationary and not self.measured_stop():
                raise RuntimeError('복귀 전에 팔과 그리퍼의 정지를 확인하지 못했습니다.')
            self.check_stop()
            return
        if (self.param('workspace_clear') is not True
                or not self.has_parameter('recovery_empty_confirmed')
                or self.param('recovery_empty_confirmed') is not True):
            raise RuntimeError('복구 전에 블록을 안전하게 분리하고 빈 그리퍼와 작업 공간을 확인하세요.')
        if (not self.fresh() or not self.sensor_fresh()
                or (not self.mock and self.sensor.get('grip_state') != 1)):
            raise RuntimeError('복구에는 최신 빈 그리퍼 상태(1)가 필요합니다. 잡힘(2)·이탈(3) 상태로 이동하지 않습니다.')
        if stationary and not self.measured_stop():
            raise RuntimeError('복구 전에 팔과 그리퍼의 정지를 확인하지 못했습니다.')
        self.check_stop()

    def holding_ok(self):
        if getattr(self, 'preserve_gripper', False):
            target = getattr(self, 'recovery_hand_position', None)
            return self.sensor_fresh() and (target is None or abs(self.values.get(HAND, math.inf)-target) <= .003)
        # The inherited arm executor invokes this throughout movement. A drop
        # indication is never accepted as proof of an empty recovery gripper.
        return self.sensor_fresh() and (self.mock or self.sensor.get('grip_state') == 1)

    def sensor_feedback(self, message):
        super().sensor_feedback(message)
        if (not getattr(self, 'preserve_gripper', False)
                and getattr(self, 'goal_pending', None) is not None and not self.mock
                and self.sensor_fresh() and self.sensor.get('grip_state') in (2, 3)):
            stop_with_reason(self, 'RECOVERY_GRIPPER_NOT_EMPTY', grip_state=self.sensor.get('grip_state'))
        elif (getattr(self, 'preserve_gripper', False) and getattr(self, 'goal_pending', None) is not None
                and self.sensor_fresh() and getattr(self, 'recovery_initial_grip_state', None) == 2
                and self.sensor.get('grip_state') == 3):
            stop_with_reason(self, 'RECOVERY_PAYLOAD_DROPPED', grip_state=3)

    def approval_hand_ok(self, metadata):
        return (self.holding_ok() and self.fresh()
                and abs(self.values.get(HAND, math.inf)-metadata['gripper_position']) <= .003)

    def check_files(self, metadata):
        if (metadata is not self.recovery_metadata
                or metadata.get('policy') != recovery_policy(self)
                or metadata.get('input_sha256') != self.input_hashes()
                or metadata.get('model_sha256') != digest(self.model.encode())
                or metadata.get('semantic_sha256') != digest(self.semantic.encode())
                or metadata.get('recipe_sha256') != file_digest(self.path)
                or not 0 <= time.time()-metadata['created_at'] <= RECOVERY_TTL
                or read_held_fault() != self.recovery_fault):
            raise RuntimeError('복구 경로·배치·모델·중단 기록이 바뀌었습니다. 복구 경로를 다시 확인하세요.')
        for name, expected in metadata['artifact_sha256'].items():
            if file_digest(artifact_path(self.path.parent, name)) != expected:
                raise RuntimeError('검토한 복구 경로 파일이 바뀌었습니다.')

    def guard(self, metadata, step, expected_scene):
        self.check_stop()
        self.check_files(metadata)
        self.check_model()
        if self.hardware(require_controller=True) != metadata['hardware_class']:
            raise RuntimeError('복구 경로를 만든 하드웨어와 현재 하드웨어가 다릅니다.')
        self.gripper_runtime()
        request = GetParameters.Request()
        request.names = ['allow_trajectory_execution']
        answer = self.call(GetParameters, '/move_group/get_parameters', request)
        if not answer or not answer.values or not answer.values[0].bool_value:
            raise RuntimeError('복구 전에 allow_execution:=true로 연결하세요.')
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('복구 경로를 검토한 뒤 주변 충돌 장면이 바뀌었습니다.')
        self.require_empty()
        if (any(abs(self.values[name]-q) > START_TOLERANCE for name, q in zip(ARM, step['start_joints']))
                or abs(self.values[HAND]-step['start_joints'][-1]) > .003):
            raise RuntimeError('복구 경로의 시작 자세가 바뀌었습니다. 현재 자세에서 다시 계획하세요.')

    def state_at(self, scene, positions):
        state = copy.deepcopy(scene.robot_state)
        state.is_diff = False
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        values.update(zip(JOINTS, positions))
        state.joint_state.name = list(values)
        state.joint_state.position = [float(values[name]) for name in values]
        state.joint_state.velocity = []
        state.joint_state.effort = []
        state.joint_state.header.stamp.sec = state.joint_state.header.stamp.nanosec = 0
        return state

    def arm_step(self, trajectory, start, label):
        trajectory = canonical_recovery_path(trajectory)
        endpoint = list(trajectory.joint_trajectory.points[-1].positions)+[start[-1]]
        if max(abs(a-b) for a, b in zip(trajectory.joint_trajectory.points[0].positions, start[:6])) > 1e-7:
            raise ValueError('Recovery planner changed its measured start state')
        step = dict(index=len(self.recovery_steps)+1, kind='arm', phase='approach', label=label,
                    start_joints=list(start), end_joints=endpoint, velocity_scaling=.05, acceleration_scaling=.05)
        trajectory, step['metrics'] = self.normalize_arm(trajectory, step)
        self.validate_samples(self.motion_samples(step, trajectory))
        return step, trajectory

    def validate_vertical(self, step, trajectory, start_tcp, target):
        previous_z = float(start_tcp[2, 3])
        for _, joints in self.motion_samples(step, trajectory):
            pose = self.kinematics.tcp(dict(zip(JOINTS, joints)))
            z = float(pose[2, 3])
            if (math.dist(pose[:2, 3], start_tcp[:2, 3]) > .002
                    or z < previous_z-.0001 or z > float(target[2, 3])+.001
                    or max(abs(float(v)) for v in (pose[:3, :3]-start_tcp[:3, :3]).flat) > .002):
                raise RuntimeError('복구 상승 경로가 수직 이동·현재 방향 유지 조건을 만족하지 않습니다.')
            previous_z = z

    def plan_recovery(self):
        # /joint_states and the sensor-backed dynamic state are independent
        # subscriptions. The first can be ready before the first sensor sample
        # has been dispatched, even while the real gripper is empty and idle.
        # Wait for both at startup. Preserving mode freezes the measured hand;
        # the legacy mode additionally requires the empty state code.
        try:
            self.spin_until(lambda: self.fresh() and self.sensor_fresh(), seconds=5.)
        except TimeoutError as error:
            details = dict(joint_feedback_fresh=self.fresh(), **self.sensor_stale_details())
            print('STACK_RECOVERY_FEEDBACK_TIMEOUT '+json.dumps(details, sort_keys=True), flush=True)
            raise RuntimeError('복구용 관절·그리퍼 센서 수신을 5초 동안 확인하지 못했습니다. '
                               '/joint_states와 /gripper/dynamic_joint_states 연결을 확인하세요.') from error
        self.require_empty()
        self.check_model()
        hardware = self.hardware(require_controller=True)
        self.gripper_runtime()
        token = self.param('preparation_id') if self.has_parameter('preparation_id') else ''
        if not isinstance(token, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', token):
            raise ValueError('복구 승인에 사용할 preparation_id가 필요합니다.')
        fault = read_held_fault()
        if fault is not None:
            cause = str(fault.get('cause', ''))
            if (cause.startswith('SCENE_RPC_PENDING:') or 'Unreadable' in cause
                    or fault.get('model_sha256') != digest(self.model.encode())):
                raise RuntimeError('장면 응답 또는 중단 기록의 모델 상태를 먼저 복구해야 합니다. 잠금을 지우지 않습니다.')
        self.recovery_fault = copy.deepcopy(fault)
        inputs = self.input_hashes()
        initial = self.scene()
        expected_scene = scene_fingerprint(initial)
        start = [self.values[name] for name in JOINTS]
        self.recovery_hand_position = start[-1]
        self.recovery_initial_grip_state = self.sensor.get('grip_state') if getattr(self, 'preserve_gripper', False) else None
        initial_start = list(start)
        home = home_joints(self.semantic, self.bounds)
        self.recovery_home = home
        current = self.kinematics.tcp(dict(zip(JOINTS, start)))
        goal = self.kinematics.tcp(dict(zip(ARM, home)))
        obstacle_top = float(self.param('obstacle_surface_z'))+float(self.param('obstacle_size_xyz')[2])
        safe_z = max(float(current[2, 3]), float(goal[2, 3]), obstacle_top+.10)
        planned = []
        if safe_z-float(current[2, 3]) > .001:
            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            request.start_state = self.state_at(initial, start)
            request.group_name, request.link_name = 'ur5e_arm', 'grasp_link'
            target = current.copy()
            target[2, 3] = safe_z
            request.waypoints = [transform_pose(target)]
            request.max_step, request.jump_threshold = .005, 5.
            request.revolute_jump_threshold = .35
            request.avoid_collisions = True
            request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = .05
            reply = self.call(GetCartesianPath, '/compute_cartesian_path', request, seconds=20.)
            if (reply is None or reply.error_code.val != 1 or not math.isfinite(reply.fraction)
                    or reply.fraction < 1.-1e-9):
                raise RuntimeError('현재 자세에서 방향을 유지한 수직 복구 경로를 만들 수 없습니다. 자동으로 우회하지 않습니다.')
            step, trajectory = self.arm_step(reply.solution, start, '현재 그리퍼 모양을 유지하며 수직 상승')
            self.validate_vertical(step, trajectory, current, target)
            fk_end = self.kinematics.tcp(dict(zip(JOINTS, step['end_joints'])))
            if (math.dist(fk_end[:3, 3], target[:3, 3]) > .001
                    or max(abs(float(v)) for v in (fk_end[:3, :3]-target[:3, :3]).flat) > .002):
                raise RuntimeError('수직 복구 경로가 검토한 높이·방향에 도착하지 않습니다.')
            planned.append((step, trajectory))
            self.recovery_steps.append(step)
            start = list(step['end_joints'])
        if max(abs(a-b) for a, b in zip(start[:6], home)) > START_TOLERANCE:
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.start_state = self.state_at(initial, start)
            motion.group_name, motion.pipeline_id = 'ur5e_arm', 'ompl'
            motion.allowed_planning_time, motion.num_planning_attempts = 10., 1
            motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = .05
            constraints = Constraints()
            constraints.joint_constraints = [JointConstraint(joint_name=name, position=float(q),
                tolerance_above=.0001, tolerance_below=.0001, weight=1.) for name, q in zip(ARM, home)]
            motion.goal_constraints = [constraints]
            answer = self.call(GetMotionPlan, '/plan_kinematic_path', request, seconds=20.)
            reply = answer.motion_plan_response if answer is not None else None
            if reply is None or reply.error_code.val != 1:
                raise RuntimeError('현재 충돌 장면을 유지한 초기 자세 복귀 경로를 만들 수 없습니다.')
            step, trajectory = self.arm_step(reply.trajectory, start, '현재 그리퍼 자세를 유지하며 초기 자세로 복귀')
            if max(abs(a-b) for a, b in zip(step['end_joints'][:6], home)) > START_TOLERANCE:
                raise RuntimeError('복구 경로가 지정된 초기 자세에 도착하지 않습니다.')
            planned.append((step, trajectory))
            self.recovery_steps.append(step)
            start = list(step['end_joints'])
        if not getattr(self, 'preserve_gripper', False):
            hand_path = RobotTrajectory()
            hand_path.joint_trajectory.joint_names = [HAND]
            for index, q in enumerate((start[-1], HAND_TARGETS['open'])):
                point = JointTrajectoryPoint()
                point.positions, point.velocities, point.accelerations = [float(q)], [0.], [0.]
                point.time_from_start.sec = index*max(1, math.ceil(abs(start[-1]-HAND_TARGETS['open'])/.05))
                hand_path.joint_trajectory.points.append(point)
            hand_step = dict(index=len(self.recovery_steps)+1, kind='hand', phase='open',
                             label='초기 자세에서 그리퍼 0.4rad로 준비', start_joints=list(start),
                             end_joints=start[:6]+[HAND_TARGETS['open']], hand_target=HAND_TARGETS['open'])
            self.validate_samples(self.motion_samples(hand_step, hand_path))
            planned.append((hand_step, hand_path))
            self.recovery_steps.append(hand_step)
        self.require_empty()
        if (self.input_hashes() != inputs or scene_fingerprint(self.scene()) != expected_scene
                or read_held_fault() != self.recovery_fault
                or any(abs(self.values[name]-q) > (START_TOLERANCE if name in ARM else .003)
                       for name, q in zip(JOINTS, initial_start))):
            raise RuntimeError('복구 계획 중 입력 또는 충돌 장면이 바뀌었습니다.')
        artifacts = {}
        for index, (step, trajectory) in enumerate(planned, 1):
            name = self.path.name+f'.{index:02d}.cdr'
            atomic_write(self.path.parent/name, serialize_message(trajectory))
            artifacts[name] = file_digest(self.path.parent/name)
            step['file'], step['duration'] = name, trajectory_duration(trajectory)
        manifest = dict(policy=recovery_policy(self), plan_id=token, steps=self.recovery_steps,
                        initial_scene=expected_scene, home=home, artifact_sha256=artifacts)
        data = (json.dumps(manifest, sort_keys=True, allow_nan=False)+'\n').encode()
        atomic_write(self.path, data)
        self.recovery_metadata = dict(policy=recovery_policy(self), plan_id=token, created_at=time.time(),
            hardware_class=hardware, input_sha256=inputs, model_sha256=digest(self.model.encode()),
            semantic_sha256=digest(self.semantic.encode()), recipe_sha256=digest(data),
            artifact_sha256=artifacts, scene_sha256=expected_scene, fault=self.recovery_fault,
            gripper_position=initial_start[-1])
        self.phase('STACK_RECOVERY_PLAN', plan_id=token, motions=len(planned),
                   gripper_position=initial_start[-1], scene_preserved=True,
                   hand_target=initial_start[-1] if getattr(self, 'preserve_gripper', False) else HAND_TARGETS['open'])
        return planned

    def complete_recovery(self, metadata):
        self.require_empty()
        self.check_files(metadata)
        if getattr(self, 'preserve_gripper', False):
            if (any(abs(self.values[name]-q) > START_TOLERANCE for name,q in zip(ARM,self.recovery_home))
                    or abs(self.values[HAND]-metadata['gripper_position']) > .003):
                raise RuntimeError('팔 초기 자세와 유지한 그리퍼 자세를 확인하지 못했습니다.')
            live = self.scene()
            if scene_fingerprint(live) != metadata['scene_sha256']:
                raise RuntimeError('초기 자세 복귀 중 충돌 장면이 바뀌었습니다.')
            owned_attached = any(item.object.id.startswith(PREFIX) for item in live.robot_state.attached_collision_objects)
            # Home alone is not proof of releasing an object. Only fresh empty
            # feedback plus no task attachment can clear an old stopped fault.
            if (self.mock or self.sensor.get('grip_state') == 1) and not owned_attached:
                clear_held_fault()
            self.recovery_finished = True
            self.phase('STACK_RECOVERY_DONE', plan_id=metadata['plan_id'], policy=HOME_PRESERVE_POLICY,
                       home_reached=True, gripper_preserved=True, gripper_position=metadata['gripper_position'],
                       scene_preserved=True, held_fault_retained=read_held_fault() is not None)
            print('STACK_STATUS phase=DONE moved=0 total=0 target=0 layers=0', flush=True)
            return
        if (any(abs(self.values[name]-q) > START_TOLERANCE for name, q in zip(ARM, self.recovery_home))
                or abs(self.values[HAND]-HAND_TARGETS['open']) > .003):
            raise RuntimeError('팔 초기 자세와 그리퍼 0.4rad를 확인한 뒤에만 초기화할 수 있습니다.')
        live = self.scene()
        if scene_fingerprint(live) != metadata['scene_sha256']:
            raise RuntimeError('복구가 끝나기 전에 주변 충돌 장면이 바뀌었습니다.')
        desired = without_owned_scene(live)
        self.apply_snapshot(desired)
        if scene_fingerprint(self.scene()) != scene_fingerprint(desired):
            raise RuntimeError('복구 장면 초기화 응답을 확인하지 못했습니다. 중단 잠금을 유지합니다.')
        self.require_empty()
        if (any(abs(self.values[name]-q) > START_TOLERANCE for name, q in zip(ARM, self.recovery_home))
                or abs(self.values[HAND]-HAND_TARGETS['open']) > .003):
            raise RuntimeError('장면 초기화 중 팔·그리퍼 초기 자세가 바뀌었습니다. 중단 잠금을 유지합니다.')
        clear_held_fault()
        self.recovery_finished = True
        self.phase('STACK_RECOVERY_DONE', plan_id=metadata['plan_id'], home_reached=True,
                   gripper_empty=True, hand_target=HAND_TARGETS['open'], scene_cleared=True)
        print('STACK_STATUS phase=DONE moved=0 total=0 target=0 layers=0', flush=True)

    def require_empty_unlock(self):
        """This metadata-only operation never accepts a retained payload."""
        self.check_stop()
        if (not self.has_parameter('recovery_empty_confirmed')
                or self.param('recovery_empty_confirmed') is not True):
            raise RuntimeError('중단 잠금 해제 전에 실제 빈 그리퍼를 확인하세요.')
        if (not self.fresh() or not self.sensor_fresh() or self.sensor.get('grip_state') != 1
                or abs(self.values.get(HAND, math.inf)-self.sensor.get('position', -math.inf)) > .003):
            raise RuntimeError('잠금 해제에는 최신 빈 그리퍼 센서 상태(1)와 일치하는 관절 피드백이 필요합니다.')
        if not self.measured_stop():
            raise RuntimeError('잠금 해제 전에 팔·그리퍼 정지를 확인하지 못했습니다.')
        self.check_stop()

    def acknowledge_empty(self):
        """Clear stale evidence after read-only checks; no action or scene writes."""
        token = self.param('preparation_id') if self.has_parameter('preparation_id') else ''
        if not isinstance(token,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',token):
            raise ValueError('빈 그리퍼 확인 기록의 preparation_id가 필요합니다.')
        self.spin_until(lambda: self.fresh() and self.sensor_fresh(),seconds=5.)
        self.require_empty_unlock()
        self.check_model()
        fault = read_held_fault()
        if fault is not None and (str(fault.get('cause','')).startswith('SCENE_RPC_PENDING:')
                or 'Unreadable' in str(fault.get('cause',''))
                or fault.get('model_sha256') != digest(self.model.encode())):
            raise RuntimeError('장면 응답·중단 기록 모델을 확인하지 못했습니다. 중단 잠금을 유지합니다.')
        live = self.scene()
        if any(a.object.id.startswith(PREFIX) for a in live.robot_state.attached_collision_objects):
            raise RuntimeError('RViz에 부착 블록이 남아 있습니다. 실제 물체와 부착 장면을 확인한 뒤 다시 시도하세요.')
        before = scene_fingerprint(live)
        start = [self.values[name] for name in JOINTS]
        self.require_empty_unlock()
        if (scene_fingerprint(self.scene()) != before or read_held_fault() != fault
                or any(abs(self.values[name]-q) > .003 for name,q in zip(JOINTS,start))):
            raise RuntimeError('빈 그리퍼 확인 중 장면·관절·중단 기록이 바뀌었습니다. 잠금을 유지합니다.')
        self.require_empty_unlock()
        record = dict(policy=EMPTY_UNLOCK_POLICY,plan_id=token,recorded_at=time.time(),previous_fault=fault,
                      scene_sha256=before,joints=start,sensor=dict(self.sensor),
                      arm_motion=False,gripper_motion=False,scene_preserved=True)
        atomic_write(self.path.parent/('empty_unlock_'+token+'.json'),
                     (json.dumps(record,sort_keys=True,indent=2)+'\n').encode())
        self.check_stop()
        clear_held_fault()
        self.recovery_finished = True
        self.phase('STACK_EMPTY_UNLOCK_DONE',policy=EMPTY_UNLOCK_POLICY,plan_id=token,
                   gripper_empty=True,fault_cleared=True,arm_motion=False,gripper_motion=False,scene_preserved=True)

    def run(self):
        if self.mode == 'physical_unlock':
            self.acquire_stack_lock()
            self.acknowledge_empty()
            return
        if self.mode != 'physical_recover':
            raise ValueError('Recovery node only accepts physical_recover or physical_unlock')
        self.acquire_stack_lock()
        planned = self.plan_recovery()
        metadata, scene = self.recovery_metadata, self.recovery_metadata['scene_sha256']
        if not planned:
            self.complete_recovery(metadata)
            return
        first = planned[0][0]
        self.guard(metadata, first, scene)
        if self.execution_mode == 'continuous':
            display = self.recipe_display({}, self.recovery_steps)
            self.await_approval(display, dict(metadata, gripper_position=first['start_joints'][-1]),
                display.trajectory[0], 1, 1, '안전 확인 후 초기 자세 복구 전체 경로',
                context=lambda *_: self.guard(metadata, first, scene))
            self.guard(metadata, first, scene)
        for index, (step, trajectory) in enumerate(planned, 1):
            self.guard(metadata, step, scene)
            samples = self.motion_samples(step, trajectory)
            self.validate_samples(samples)
            if self.execution_mode == 'step':
                display = self.recipe_display({}, [step])
                self.await_approval(display, dict(metadata, gripper_position=step['start_joints'][-1]),
                    display.trajectory[0], index, len(planned), step['label'],
                    context=lambda *_: self.guard(metadata, step, scene))
                self.guard(metadata, step, scene)
                self.validate_samples(samples)
            self.run_motion(step, trajectory, metadata, scene,
                            index if self.execution_mode == 'step' else 1,
                            len(planned) if self.execution_mode == 'step' else 1)
            self.require_empty()
        if self.execution_mode == 'continuous':
            self.phase('STACK_STEP_COMPLETED', index=1, total=1, label='안전 확인 후 초기 자세 복구 전체 경로')
        self.complete_recovery(metadata)


def main():
    """Use the same process/domain locks and exact-goal shutdown as stacking."""
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdigit():
        raise ValueError('ROS_DOMAIN_ID must be numeric')
    lock = os.open(f'/tmp/ur5e_air_task_{domain}.lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
    fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    task = PhysicalRecoveryTask()
    signal.signal(signal.SIGINT, lambda *_: stop_with_reason(task, 'PROCESS_SIGNAL', signal='SIGINT'))
    signal.signal(signal.SIGTERM, lambda *_: stop_with_reason(task, 'PROCESS_SIGNAL', signal='SIGTERM'))
    code = 0
    try:
        task.run()
    except Exception as error:
        print('STACK_ERROR '+str(error), flush=True)
        code = 1
        # Existing evidence, especially uncertain scene writes, is never
        # replaced by a less specific generic recovery error.
        if task.mode != 'physical_unlock' and read_held_fault() is None:
            write_held_fault('RECOVERY_FAILED: '+str(error), digest(task.model.encode()))
    finally:
        task.cancel_remaining_goal()
        if code and task.goal_pending is None and task.measured_stop():
            print('STACK_STOPPED_CONFIRMED', flush=True)
        if task.stack_lock is not None:
            os.close(task.stack_lock)
        task.close_monitor_clients()
        task.close_service_clients()
        task.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(lock)
    raise SystemExit(code)
