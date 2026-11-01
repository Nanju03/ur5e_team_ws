"""Plan, validate, and replay the reviewed empty-hand arm path.

The original stacking executor remains restricted to mock hardware. Arm replay
holds the measured gripper shape. The separate alignment_prepare mode sends
only a reviewed gripper-to-0.65 goal while the arm remains at rest.
"""
from collections import deque
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID as GoalUUID
from moveit_msgs.msg import DisplayTrajectory, PlanningScene, CollisionObject, AttachedCollisionObject, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
import yaml

from .air_scene import RobotKinematics, cartesian_metrics, scene_fingerprint
from .air_trajectory import prepare_trajectory, sample_trajectory, trajectory_metrics, split_trajectory
from .plate_geometry import ALIGNMENT_GRIPPER_POSITION, ALIGNMENT_POLICY

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HAND = 'left_outer_knuckle_joint'
CONTROLLER = 'scaled_joint_trajectory_controller'
GRIPPER_CONTROLLER = 'dh_ag95_gripper_controller'
ALIGNMENT_HAND = ALIGNMENT_GRIPPER_POSITION
MAX_FILE_BYTES = 100 * 1024 * 1024
START_TOLERANCE = .001


def digest(data):
    return hashlib.sha256(data).hexdigest()


def file_digest(path):
    return digest(Path(path).expanduser().read_bytes())


def frozen_hand_tolerance(metadata):
    # Alignment and physical pick/place bind the actual measured hand tightly
    # during approval pauses. Older high-clearance air paths retain their limit.
    return .003 if metadata.get('path_kind') in ('plate_alignment', 'physical_stack') else .015


def review_ttl(metadata):
    """Give only the sealed eight-stop alignment time for manual adjustment."""
    if (metadata.get('path_kind') == 'plate_alignment'
            and metadata.get('alignment_policy') == ALIGNMENT_POLICY):
        if metadata.get('valid_for_seconds') != 1800:
            raise RuntimeError('X/Y 맞춤 경로의 유효 시간 정보가 올바르지 않습니다. 다시 계획하세요.')
        return 1800
    return 600


def alignment_rotation(quaternion):
    """Validate a sealed unit quaternion and return its rotation matrix."""
    if (not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                for v in quaternion)):
        raise RuntimeError('X/Y 맞춤 방향 정보가 올바르지 않습니다. 다시 계획하세요.')
    x, y, z, w = quaternion
    if abs(x*x+y*y+z*z+w*w-1.) > 1e-6:
        raise RuntimeError('X/Y 맞춤 방향 정보가 단위 회전이 아닙니다. 다시 계획하세요.')
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def alignment_hold_label(stop):
    return f'모서리 홈 {stop["corner"]} · {stop["pocket_cell"]}번 · {stop["axis"]}방향 맞춤 위치에서 대기'


def alignment_retract_label(stop):
    return f'모서리 홈 {stop["corner"]} · {stop["pocket_cell"]}번 · {stop["axis"]}방향에서 수직 상승'


def alignment_turn_label(stop):
    axis = 'Y' if stop['axis'] == 'X' else 'X'
    return f'모서리 홈 {stop["corner"]} · {stop["pocket_cell"]}번 · 안전 높이에서 {axis}방향으로 회전'


def alignment_pause_boundaries(metadata, segments):
    """Validate all eight sealed XY holds; old single-direction plans fail."""
    if metadata.get('path_kind') != 'plate_alignment':
        return []
    stops = metadata.get('alignment_stops')
    labels, points = metadata.get('segment_labels'), metadata.get('scan_cells')
    clearance = metadata.get('alignment_rotation_clearance')
    if (metadata.get('alignment_policy') != ALIGNMENT_POLICY
            or not isinstance(stops, list) or len(stops) != 8
            or not isinstance(labels, list) or len(labels) != len(segments)
            or not isinstance(points, list) or len(points) != 4
            or isinstance(clearance, bool) or not isinstance(clearance, (int, float))
            or not math.isfinite(clearance) or not .03 <= clearance <= .10):
        raise RuntimeError('X/Y 모서리 맞춤의 필수 정지 정보가 없습니다. 새 경로를 계획하세요.')
    cells, previous, base = [], 0, None
    for index, stop in enumerate(stops):
        corner, axis = 'ABCD'[index//2], 'XY'[index % 2]
        point = points[index//2]
        if not isinstance(stop, dict) or not isinstance(point, dict):
            raise RuntimeError('X/Y 모서리 맞춤 정지 기록이 올바르지 않습니다.')
        after, cell = stop.get('after_segment'), point.get('pocket_cell')
        if (type(after) is not int or not previous < after <= len(segments)
                or stop.get('corner') != corner or stop.get('axis') != axis
                or stop.get('yaw_deg') != (0 if axis == 'X' else 90)
                or type(stop.get('yaw_deg')) not in (int, float)
                or type(cell) is not int or cell < 1 or point.get('cell') != index//2+1
                or type(stop.get('pocket_cell')) is not int or stop['pocket_cell'] != cell
                or labels[after-1] != alignment_hold_label(stop)):
            raise RuntimeError('X/Y 모서리 맞춤 정지 순서 또는 홈이 바뀌었습니다. 다시 계획하세요.')
        for coordinate in ('x', 'y', 'z'):
            values = (stop.get(coordinate), point.get(coordinate))
            if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                    for v in values) or abs(values[0]-values[1]) > 1e-8):
                raise RuntimeError('X/Y 맞춤 정지 위치가 모서리 홈 중심과 다릅니다.')
        rotation = alignment_rotation(stop.get('orientation_xyzw'))
        if base is None:
            base = rotation
        expected = base if axis == 'X' else (tuple(-v for v in base[1]), base[0], base[2])
        if max(abs(rotation[r][c]-expected[r][c]) for r in range(3) for c in range(3)) > 1e-6:
            raise RuntimeError('X/Y 맞춤 자세가 동일한 기준 방향과 90도 회전을 사용하지 않습니다.')
        arrival = segments[after-1].joint_trajectory.points[-1]
        boundary = [arrival]
        if index < 7:
            if (after+1 >= len(segments) or labels[after] != alignment_retract_label(stop)
                    or labels[after+1] != alignment_turn_label(stop)):
                raise RuntimeError('X/Y 맞춤 뒤 수직 상승과 안전 높이 회전 구간이 없습니다.')
            boundary.append(segments[after].joint_trajectory.points[0])
        if any(len(p.velocities) != 6 or len(p.accelerations) != 6 or
               any(not math.isfinite(v) or abs(v) > 1e-8 for v in p.velocities+p.accelerations)
               for p in boundary):
            raise RuntimeError('X/Y 맞춤 정지 경계의 속도와 가속도가 0이 아닙니다.')
        if axis == 'X':
            cells.append(cell)
        previous = after
    if len(set(cells)) != 4 or previous != len(segments):
        raise RuntimeError('X/Y 맞춤은 네 모서리를 방문하고 마지막 Y 위치에서 끝나야 합니다.')
    return stops


def atomic_write(path, data):
    temporary = path.with_name(path.name + f'.tmp.{os.getpid()}')
    temporary.write_bytes(data)
    temporary.replace(path)


class AirTask(Node):
    def __init__(self):
        super().__init__('ur5e_air_task', automatically_declare_parameters_from_overrides=True)
        self.stopped = False
        self.child = None
        self.goal = None
        self.goal_pending = None
        self.values, self.velocities, self.received = {}, {}, {}
        self.sub = self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.display = self.create_publisher(DisplayTrajectory, '/display_planned_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path = Path(self.param('trajectory_file')).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.meta_path = self.path.with_name(self.path.name + '.meta.json')
        self.model = self.param('robot_description')
        self.semantic = self.param('robot_description_semantic')
        self.mock = bool(self.param('allow_mock_hardware'))
        self.mode = self.param('stack_mode')
        self.execution_mode = self.param('execution_mode') if self.has_parameter('execution_mode') else 'step'
        if self.execution_mode not in ('step', 'continuous'):
            raise ValueError('execution_mode must be step or continuous')
        self.plate_alignment = self.has_parameter('plate_alignment_test') and self.param('plate_alignment_test') is True
        self.xy_scan = (self.has_parameter('empty_xy_scan') and self.param('empty_xy_scan') is True) or self.plate_alignment
        self.path_kind = ('plate_alignment' if self.plate_alignment else
                          'obstacle_xy_scan' if self.xy_scan else 'empty_hand_stack_pattern')
        self.scan_floor = float(self.param('scan_min_gripper_z')) if self.xy_scan else None
        self.scan_points = []
        self.segment_labels = []
        self.alignment_stop_records = []
        self.kinematics = RobotKinematics(self.model)
        self.stop_service = self.create_service(Trigger, '/ur5e_a3_stack/stop', self.request_stop)
        self.result_future = None
        tree = ET.fromstring(self.model)
        self.bounds = {}
        for name in ARM+[HAND]:
            limit = tree.find(f"joint[@name='{name}']/limit")
            if limit is None:
                raise ValueError(f'Missing URDF bounds for {name}')
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError(f'Invalid URDF bounds for {name}')
            self.bounds[name] = (lower, upper)

    def param(self, name):
        return self.get_parameter(name).value

    def joints(self, message):
        if len(message.name) != len(message.position):
            return
        now = time.monotonic()
        for index, name in enumerate(message.name):
            value = message.position[index]
            if math.isfinite(value):
                self.values[name] = value
                self.received[name] = now
            if len(message.velocity) == len(message.name) and math.isfinite(message.velocity[index]):
                self.velocities[name] = (message.velocity[index], now)

    def fresh(self):
        now = time.monotonic()
        return all(name in self.values and now-self.received[name] < 1.0 for name in ARM+[HAND])

    def request_stop(self, _, reply):
        self.stopped = True
        reply.success = True
        reply.message = 'Stop requested; waiting for cancellation and measured standstill.'
        return reply

    def check_stop(self):
        if self.stopped:
            raise RuntimeError('Empty-hand task stopped before motion')

    def spin_until(self, predicate, seconds=10., cancel=True):
        deadline = time.monotonic()+seconds
        while not predicate():
            if cancel:
                self.check_stop()
            if time.monotonic() > deadline:
                raise TimeoutError('Timed out waiting for ROS feedback or a service')
            rclpy.spin_once(self, timeout_sec=.02)

    def call(self, kind, service, request, seconds=10.):
        client = self.create_client(kind, service)
        try:
            self.spin_until(client.service_is_ready, seconds)
            future = client.call_async(request)
            self.spin_until(future.done, seconds)
            return future.result()
        finally:
            self.destroy_client(client)

    def scene(self):
        request = GetPlanningScene.Request()
        request.components.components = 1023
        return self.call(GetPlanningScene, '/get_planning_scene', request).scene

    def check_model(self):
        for node, names, expected in [
            ('/robot_state_publisher', ['robot_description'], [self.model]),
            ('/move_group', ['robot_description', 'robot_description_semantic'], [self.model, self.semantic]),
        ]:
            request = GetParameters.Request()
            request.names = names
            response = self.call(GetParameters, node+'/get_parameters', request)
            if len(response.values) != len(expected) or any(
                    value.string_value != text for value, text in zip(response.values, expected)):
                raise RuntimeError('Robot model changed or differs from the reviewed physical setup')

    def hardware(self, require_controller=False):
        reply = self.call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                          ListHardwareComponents.Request())
        expected = 'mock_components/GenericSystem' if self.mock else 'ur_robot_driver/URPositionHardwareInterface'
        matches = [item for item in reply.component if item.class_type == expected and item.state.id == 3]
        if not matches:
            raise RuntimeError(f'Expected active {expected}; no motion sent')
        if require_controller:
            claimed = {interface.name for item in matches for interface in item.command_interfaces if interface.is_claimed}
            if not all(name+'/position' in claimed for name in ARM):
                raise RuntimeError('The real arm position interfaces are not claimed')
            reply = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
            if not any(item.name == CONTROLLER and item.state == 'active' for item in reply.controller):
                raise RuntimeError('The scaled arm trajectory controller is not active')
        return expected

    def input_hashes(self):
        hashes = {name: file_digest(self.param(name)) for name in
                  ('selection_file', 'calibration_file', 'physical_profile')}
        profile = Path.home()/'ur5e_ws/gripper_calibration.yaml'
        hashes['gripper_calibration'] = file_digest(profile) if profile.is_file() else None
        if self.has_parameter('plate_enabled') and self.param('plate_enabled') is True:
            mesh = Path(get_package_share_directory('ur5e_block_description'))/'meshes/cube_plate.STL'
            hashes['plate_mesh'] = file_digest(mesh)
        return hashes

    def remove_virtual_blocks(self):
        scene = self.scene()
        update = PlanningScene()
        update.is_diff = True
        update.robot_state.is_diff = True
        for item in scene.world.collision_objects:
            if item.id.startswith('ur5e_stack/block_'):
                remove = CollisionObject()
                remove.id, remove.operation = item.id, CollisionObject.REMOVE
                update.world.collision_objects.append(remove)
        for item in scene.robot_state.attached_collision_objects:
            if item.object.id.startswith('ur5e_stack/block_'):
                remove = AttachedCollisionObject()
                remove.object.id, remove.object.operation = item.object.id, CollisionObject.REMOVE
                update.robot_state.attached_collision_objects.append(remove)
        if update.world.collision_objects or update.robot_state.attached_collision_objects:
            request = ApplyPlanningScene.Request()
            request.scene = update
            if not self.call(ApplyPlanningScene, '/apply_planning_scene', request).success:
                raise RuntimeError('Unable to remove simulated blocks for empty-hand checking')

    def validate_path(self, trajectory, hand, expected_scene=None, scan_safe_after=None):
        self.check_model()
        before = scene_fingerprint(self.scene())
        if expected_scene is not None and before != expected_scene:
            raise RuntimeError('Planning scene changed; plan and review the path again')
        samples = sample_trajectory(trajectory, max_dt=.05, max_joint_step=.01)
        if self.xy_scan:
            if scan_safe_after is None or not math.isfinite(scan_safe_after) or scan_safe_after < 0:
                raise RuntimeError('Missing reviewed vertical-raise boundary for XY scan')
            initial_tcp = self.kinematics.tcp(dict(zip(ARM, samples[0][1])))
            for stamp, positions in samples:
                joints = dict(zip(ARM, positions))
                if stamp+1e-8 < scan_safe_after:
                    tcp = self.kinematics.tcp(joints)
                    if math.hypot(tcp[0, 3]-initial_tcp[0, 3], tcp[1, 3]-initial_tcp[1, 3]) > .003:
                        raise RuntimeError('The initial raise moves sideways before obstacle clearance')
                    if tcp[2, 3] < initial_tcp[2, 3]-.001:
                        raise RuntimeError('The initial clearance move descends instead of raising')
                elif self.kinematics.gripper_min_z(joints, hand) < self.scan_floor-1e-5:
                    raise RuntimeError(f'Gripper lower envelope is below the reviewed obstacle clearance at {stamp:.3f}s')
        client = self.create_client(GetStateValidity, '/check_state_validity')
        pending = deque()
        index, checked = 0, 0
        try:
            self.spin_until(client.service_is_ready)
            while index < len(samples) or pending:
                self.check_stop()
                # Stay below the service's default KEEP_LAST depth (10), so
                # bursts of fast replies cannot overflow the client queue.
                while index < len(samples) and len(pending) < 8:
                    request = GetStateValidity.Request()
                    q = list(samples[index][1])+[hand]
                    if any(not self.bounds[name][0]-1e-9 <= value <= self.bounds[name][1]+1e-9
                           for name, value in zip(ARM+[HAND], q)):
                        raise RuntimeError(f'Quintic path exceeds a URDF joint limit at sample {index}')
                    request.robot_state.is_diff = True
                    request.robot_state.joint_state.name = ARM+[HAND]
                    request.robot_state.joint_state.position = q
                    pending.append((index, time.monotonic(), client.call_async(request)))
                    index += 1
                rclpy.spin_once(self, timeout_sec=.005)
                for _ in range(len(pending)):
                    number, sent, future = pending.popleft()
                    if not future.done():
                        if time.monotonic()-sent > 10.:
                            raise TimeoutError('Collision validation timed out')
                        pending.append((number, sent, future))
                        continue
                    reply = future.result()
                    if reply is None or not reply.valid:
                        contacts = [] if reply is None else [
                            f'{c.contact_body_1}/{c.contact_body_2}' for c in reply.contacts[:3]]
                        raise RuntimeError(f'Empty-hand path collision or joint limit at sample {number}: {contacts}')
                    checked += 1
                if checked and checked % 500 == 0:
                    print(f'STACK_AIR_VALIDATING checked={checked} total={len(samples)}', flush=True)
        finally:
            for _, _, future in pending:
                client.remove_pending_request(future)
            self.destroy_client(client)
        if scene_fingerprint(self.scene()) != before:
            raise RuntimeError('Planning scene changed while validating; plan again')
        return before, samples

    def planner(self, output):
        params = {name: parameter.value for name, parameter in self.get_parameters_by_prefix('').items()}
        params.update(stack_mode='plan', empty_hand_test=True, display_trajectory_file=str(output))
        for name in ('trajectory_file', 'selection_file', 'calibration_file', 'physical_profile',
                     'workspace_clear', 'allow_mock_hardware', 'execution_mode'):
            params.pop(name, None)
        params = {name: value for name, value in params.items() if value != []}
        params_path = self.path.with_name(self.path.name+'.planner.yaml')
        params_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        self.child = subprocess.Popen(
            ['ros2', 'run', 'ur5e_a3_grid', 'stack_blocks', '--ros-args', '--params-file', str(params_path),
             '-r', '/ur5e_a3_stack/stop:=/ur5e_air_task/planner_stop'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        lines = queue.Queue()
        def read():
            for line in self.child.stdout:
                lines.put(line)
        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        interrupted = False
        planner_error = ''
        try:
            while self.child.poll() is None or reader.is_alive() or not lines.empty():
                rclpy.spin_once(self, timeout_sec=.02)
                if self.stopped and self.child.poll() is None and not interrupted:
                    self.child.send_signal(signal.SIGINT)
                    interrupted = True
                while not lines.empty():
                    line = lines.get_nowait()
                    if 'STACK_ERROR ' in line:
                        planner_error = line.split('STACK_ERROR ', 1)[1].strip()
                    if 'STACK_SCAN_POINT ' in line:
                        match = re.search(r'cell=(\d+).*?x=([-+\d.eE]+).*?y=([-+\d.eE]+).*?z=([-+\d.eE]+)', line)
                        if match:
                            point = {'cell':int(match[1]),'x':float(match[2]),'y':float(match[3]),'z':float(match[4])}
                            pocket = re.search(r'pocket_cell=(\d+)', line)
                            if pocket:
                                point['pocket_cell'] = int(pocket[1])
                            self.scan_points.append(point)
                    if 'STACK_PATH_SEGMENT ' in line:
                        record = json.loads(line.split('STACK_PATH_SEGMENT ', 1)[1])
                        if record['index'] != len(self.segment_labels)+1 or not isinstance(record['label'], str):
                            raise RuntimeError('Planner segment labels are out of order')
                        self.segment_labels.append(record['label'])
                    if 'STACK_ALIGNMENT_STOP ' in line:
                        record = json.loads(line.split('STACK_ALIGNMENT_STOP ', 1)[1])
                        if not isinstance(record, dict) or len(self.alignment_stop_records) >= 8:
                            raise RuntimeError('Planner returned invalid or duplicate X/Y alignment stops')
                        self.alignment_stop_records.append(record)
                    print(line, end='', flush=True)
            if self.child.returncode != 0:
                if 'XY scan initial vertical lift' in planner_error:
                    raise RuntimeError('현재 자세에서 첫 수직 상승 경로를 만들 수 없습니다. 팔 자세와 주변 장애물을 확인하고 다시 계획하세요. ('+planner_error+')')
                raise RuntimeError(planner_error or 'The empty-hand path planner did not complete')
            self.check_stop()
        finally:
            if self.child.poll() is None:
                self.child.send_signal(signal.SIGINT)
                self.child.wait(timeout=10.)
            self.child = None
            params_path.unlink(missing_ok=True)

    @staticmethod
    def stretch(trajectory, scale):
        for point in trajectory.joint_trajectory.points:
            old = point.time_from_start.sec + point.time_from_start.nanosec*1e-9
            nanos = round(old*scale*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanos, 10**9)
            point.velocities = [value/scale for value in point.velocities]
            point.accelerations = [value/(scale*scale) for value in point.accelerations]

    def plan(self):
        self.spin_until(self.fresh)
        if self.plate_alignment and abs(self.values[HAND]-ALIGNMENT_HAND) > .003:
            raise RuntimeError(f'플레이트 맞춤 준비에서 그리퍼를 {ALIGNMENT_HAND:g}rad로 맞춘 뒤 경로를 다시 계획하세요.')
        kind = self.hardware()
        self.check_model()
        inputs = self.input_hashes()
        raw = self.path.with_name(self.path.name+'.unvalidated.cdr')
        raw.unlink(missing_ok=True)
        self.meta_path.unlink(missing_ok=True)
        self.planner(raw)
        if not raw.is_file() or raw.stat().st_size > MAX_FILE_BYTES:
            raise RuntimeError('Planner did not write a bounded trajectory file')
        display = deserialize_message(raw.read_bytes(), DisplayTrajectory)
        start = dict(zip(display.trajectory_start.joint_state.name, display.trajectory_start.joint_state.position))
        if HAND not in start or not math.isfinite(start[HAND]):
            raise RuntimeError('The planned fixed gripper position is missing')
        trajectory, metrics = prepare_trajectory(display)
        joint_scale = metrics['time_scale']
        tcp_scale = 1.0
        samples = sample_trajectory(trajectory)
        cartesian = cartesian_metrics(samples, self.model, ARM)
        if cartesian['max_tcp_speed'] > .035:
            tcp_scale = cartesian['max_tcp_speed']/.035*1.05
            self.stretch(trajectory, tcp_scale)
            samples = sample_trajectory(trajectory)
            cartesian = cartesian_metrics(samples, self.model, ARM)
        metrics.update(trajectory_metrics(trajectory))
        metrics.update(cartesian)
        metrics['additional_tcp_time_scale'] = tcp_scale
        segments = split_trajectory(trajectory, metrics['segment_end_indices'])
        if self.segment_labels and len(self.segment_labels) != len(segments):
            raise RuntimeError('Planner labels do not match the saved arm segments')
        labels = self.segment_labels or [f'팔 이동 구간 {i+1}' for i in range(len(segments))]
        if self.plate_alignment:
            ids = self.param('plate_alignment_cell_ids') if self.has_parameter('plate_alignment_cell_ids') else []
            if not isinstance(ids, (list, tuple)) or len(ids) != 4:
                raise RuntimeError('모서리 홈 기준이 없습니다. 맞춤 경로를 다시 계획하세요.')
            translations = {
                'Plate alignment initial vertical lift preserving current orientation': '현재 방향을 유지하며 수직 상승',
                'XY scan turn gripper down at fixed TCP': '높이를 유지하며 그리퍼 방향 맞추기',
            }
            for corner, cell in zip('ABCD', ids):
                prefix = f'Plate pocket {corner} [cell {cell}]'
                for axis in 'XY':
                    stop = dict(corner=corner, pocket_cell=cell, axis=axis)
                    translations[f'{prefix} {axis} alignment hold'] = alignment_hold_label(stop)
                    translations[f'{prefix} {axis} safe-height approach'] = f'모서리 홈 {corner} · {cell}번 · {axis}방향 위 안전 높이로 접근'
                    translations[f'{prefix} {axis} vertical retract'] = alignment_retract_label(stop)
                    target = 'Y' if axis == 'X' else 'X'
                    translations[f'{prefix} turn to {target} at safe height'] = alignment_turn_label(stop)
            labels = [translations.get(label, label) for label in labels]
        if self.xy_scan and not self.plate_alignment and len(segments) == len(self.scan_points)+2:
            labels = ['현재 방향을 유지하며 수직 상승', '높이를 유지하며 그리퍼 방향 맞추기'] + [
                f'{point["cell"]}번 칸 XY 이동' for point in self.scan_points]
        safe_after = None
        if self.xy_scan:
            expected_count = 4 if self.plate_alignment else int(self.param('rows'))*int(self.param('columns'))
            if len(self.scan_points) != expected_count or {p['cell'] for p in self.scan_points} != set(range(1,expected_count+1)):
                raise RuntimeError('The alignment/scan did not plan every required reference point exactly once')
            first_end = display.trajectory[0].joint_trajectory.points[-1].time_from_start
            safe_after = (first_end.sec+first_end.nanosec*1e-9)*joint_scale*tcp_scale
        if metrics['max_velocity'] > .10001 or metrics['max_acceleration'] > .10001 or metrics['max_tcp_speed'] > .04:
            raise RuntimeError('Retimed path exceeds the empty-hand speed/acceleration limits')
        self.remove_virtual_blocks()
        fingerprint, samples = self.validate_path(trajectory, start[HAND], scan_safe_after=safe_after)
        if inputs != self.input_hashes():
            raise RuntimeError('Layout or physical setup changed while planning')
        display.trajectory = [trajectory]
        data = serialize_message(display)
        metadata = {
            'schema': 2, 'created_at': time.time(), 'trajectory_sha256': digest(data),
            'input_sha256': inputs, 'model_sha256': digest(self.model.encode()),
            'semantic_sha256': digest(self.semantic.encode()), 'scene_sha256': fingerprint,
            'hardware_class': kind, 'gripper_position': start[HAND], 'metrics': metrics,
            'validated_samples': len(samples),
            'path_kind': self.path_kind,
            'scan_min_gripper_z': self.scan_floor,
            'scan_safe_after': safe_after,
            'scan_cells': self.scan_points,
            'segment_labels': labels,
        }
        if self.plate_alignment:
            metadata.update(alignment_policy=ALIGNMENT_POLICY, valid_for_seconds=1800,
                            alignment_stops=copy.deepcopy(self.alignment_stop_records),
                            alignment_rotation_clearance=float(self.param('plate_alignment_clearance')))
            self.validate_alignment_geometry(metadata, segments)
        metadata['plan_id'] = digest(json.dumps(metadata, sort_keys=True).encode())[:16]
        atomic_write(self.path, data)
        atomic_write(self.meta_path, (json.dumps(metadata, indent=2)+'\n').encode())
        raw.unlink(missing_ok=True)
        self.display.publish(display)
        print('STACK_AIR_PLAN '+json.dumps(metadata, sort_keys=True), flush=True)
        print('STACK_STATUS phase=PLAN_READY moved=0 total=0 target=0 layers=0', flush=True)
        until = time.monotonic()+1.
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.02)

    def measured_stop(self, seconds=4.):
        began = None
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
            now = time.monotonic()
            names = ARM+[HAND] if getattr(self, 'mode', None) == 'alignment_prepare' else ARM
            quiet = all(name in self.velocities and now-self.velocities[name][1] < .5
                        and abs(self.velocities[name][0]) < .01 for name in names)
            began = now if quiet and began is None else began if quiet else None
            if began is not None and now-began >= .3:
                return True
        return False

    def validate_alignment_geometry(self, metadata, segments):
        """Bind every physical hold and every high rotation to the saved path."""
        stops = alignment_pause_boundaries(metadata, segments)
        if not stops:
            return
        ids = self.param('plate_alignment_cell_ids')
        configured = self.param('plate_alignment_points_xyz')
        clearance = self.param('plate_alignment_clearance')
        if (not isinstance(ids, (list, tuple)) or len(ids) != 4
                or not isinstance(configured, (list, tuple)) or len(configured) != 12
                or metadata['alignment_rotation_clearance'] != clearance
                or [stop['pocket_cell'] for stop in stops[::2]] != list(ids)):
            raise RuntimeError('X/Y 맞춤 홈 또는 안전 높이 설정이 바뀌었습니다. 다시 계획하세요.')
        hand = metadata['gripper_position']
        floor = metadata['scan_min_gripper_z']
        cache = {}
        def fk(positions):
            key = tuple(positions)
            if key not in cache:
                joints = dict(zip(ARM, positions))
                cache[key] = (self.kinematics.tcp(joints), joints)
            return cache[key]
        def rotation_error(tcp, rotation):
            return max(abs(float(tcp[row, column])-rotation[row][column])
                       for row in range(3) for column in range(3))
        for index, stop in enumerate(stops):
            if any(abs(stop[axis]-configured[(index//2)*3+column]) > 1e-8
                   for column, axis in enumerate(('x', 'y'))):
                raise RuntimeError('X/Y 맞춤 정지점이 설정한 모서리 홈과 다릅니다.')
            point = segments[stop['after_segment']-1].joint_trajectory.points[-1]
            tcp, joints = fk(point.positions)
            rotation = alignment_rotation(stop['orientation_xyzw'])
            minimum = self.kinematics.gripper_min_z(joints, hand)
            if (math.dist([float(tcp[i, 3]) for i in range(3)],
                          [stop[a] for a in ('x', 'y', 'z')]) > .001
                    or rotation_error(tcp, rotation) > .003
                    or not floor-1e-5 <= minimum <= floor+.002):
                raise RuntimeError('X/Y 맞춤 정지점의 실제 위치·방향·끝 높이가 검토한 홈과 다릅니다.')
        first_rotation = alignment_rotation(stops[0]['orientation_xyzw'])
        expected_rotation = first_rotation
        retracts = {stop['after_segment']: stop for stop in stops[:-1]}
        turns = {stop['after_segment']+1: stop for stop in stops[:-1]}
        for index in range(stops[0]['after_segment'], len(segments)):
            segment = segments[index]
            samples = sample_trajectory(segment, max_dt=.05, max_joint_step=.01)
            start_tcp, _ = fk(samples[0][1])
            previous_z = float(start_tcp[2, 3])
            for _, positions in samples:
                tcp, joints = fk(positions)
                if index in turns:
                    if (math.dist([float(tcp[i, 3]) for i in range(3)],
                                  [float(start_tcp[i, 3]) for i in range(3)]) > .001
                            or self.kinematics.gripper_min_z(joints, hand) < floor+clearance-1e-5):
                        raise RuntimeError('X/Y 방향 전환은 수직 상승 후 안전 높이의 고정 위치에서만 가능합니다.')
                elif rotation_error(tcp, expected_rotation) > .003:
                    raise RuntimeError('X/Y 맞춤의 낮은 위치 또는 이동 중에 그리퍼 방향이 바뀝니다.')
                if index in retracts:
                    if (math.hypot(float(tcp[0, 3]-start_tcp[0, 3]),
                                   float(tcp[1, 3]-start_tcp[1, 3])) > .001
                            or float(tcp[2, 3]) < previous_z-.0001):
                        raise RuntimeError('X/Y 맞춤 뒤 같은 방향으로 수직 상승해야 합니다.')
                    previous_z = max(previous_z, float(tcp[2, 3]))
            if index in turns:
                stop = turns[index]
                expected_rotation = (tuple(-v for v in first_rotation[1]), first_rotation[0], first_rotation[2]) \
                    if stop['axis'] == 'X' else first_rotation
                end_tcp, _ = fk(samples[-1][1])
                if rotation_error(end_tcp, expected_rotation) > .003:
                    raise RuntimeError('안전 높이의 X/Y 방향 전환이 지정된 90도 자세에 도착하지 않습니다.')

    def load_reviewed_path(self):
        if self.param('workspace_clear') is not True:
            raise RuntimeError('Confirm the clear workspace and ready emergency stop in the GUI before motion')
        if not self.path.is_file() or self.path.stat().st_size > MAX_FILE_BYTES:
            raise RuntimeError('No reviewed path file; plan first')
        metadata = json.loads(self.meta_path.read_text())
        if not isinstance(metadata, dict):
            raise RuntimeError('Invalid reviewed path metadata')
        unsigned = {name: value for name, value in metadata.items() if name != 'plan_id'}
        if metadata.get('plan_id') != digest(json.dumps(unsigned, sort_keys=True).encode())[:16]:
            raise RuntimeError('Reviewed metadata changed; plan again')
        if metadata.get('path_kind') != self.path_kind:
            raise RuntimeError('Reviewed path kind differs from the selected GUI mode')
        if self.xy_scan and metadata.get('scan_min_gripper_z') != self.scan_floor:
            raise RuntimeError('Reviewed obstacle clearance differs from the selected GUI settings')
        hand = metadata.get('gripper_position')
        if isinstance(hand, bool) or not isinstance(hand, (int, float)) or not math.isfinite(hand):
            raise RuntimeError('Invalid reviewed gripper position')
        if not self.bounds[HAND][0] <= hand <= self.bounds[HAND][1]:
            raise RuntimeError('Reviewed gripper position exceeds its joint limits')
        if self.plate_alignment and abs(hand-ALIGNMENT_HAND) > .003:
            raise RuntimeError(f'맞춤 그리퍼 값이 {ALIGNMENT_HAND:g}rad로 바뀌었습니다. 모서리 경로를 다시 계획하세요.')
        data = self.path.read_bytes()
        if (metadata.get('schema') != 2 or metadata.get('trajectory_sha256') != digest(data)
                or metadata.get('input_sha256') != self.input_hashes()
                or metadata.get('model_sha256') != digest(self.model.encode())
                or metadata.get('semantic_sha256') != digest(self.semantic.encode())
                or not 0 <= time.time()-metadata.get('created_at', 0) <= review_ttl(metadata)):
            raise RuntimeError('Reviewed path is stale or its inputs changed; plan again')
        display = deserialize_message(data, DisplayTrajectory)
        start_state = dict(zip(display.trajectory_start.joint_state.name, display.trajectory_start.joint_state.position))
        if HAND not in start_state or abs(start_state[HAND]-hand) > 1e-9 or not math.isfinite(start_state[HAND]):
            raise RuntimeError('Reviewed gripper shape differs from the sealed trajectory')
        if len(display.trajectory) != 1:
            raise RuntimeError('Reviewed file must contain one validated arm trajectory')
        trajectory = display.trajectory[0]
        metrics = trajectory_metrics(trajectory)
        if metrics['max_velocity'] > .10001 or metrics['max_acceleration'] > .10001:
            raise RuntimeError('Reviewed trajectory timing no longer meets limits')
        if self.plate_alignment:
            segments = split_trajectory(trajectory, metadata.get('metrics', {}).get('segment_end_indices'))
            self.validate_alignment_geometry(metadata, segments)
        return metadata, display, trajectory

    @staticmethod
    def phase(name, **record):
        print(name+' '+json.dumps(record, ensure_ascii=False, sort_keys=True), flush=True)

    @staticmethod
    def preview_digest(display):
        # CDR alignment padding is not a semantic part of a ROS message and
        # may differ after transport. Hash the actual displayed fields.
        return digest(json.dumps(message_to_ordereddict(display), sort_keys=True,
                                 allow_nan=False, separators=(',', ':')).encode())

    def approval_hand_ok(self, metadata):
        """The default approval gate keeps its reviewed finger shape frozen."""
        return abs(self.values.get(HAND, math.inf)-metadata['gripper_position']) <= frozen_hand_tolerance(metadata)

    def execution_context(self, metadata, trajectory):
        """Recheck mutable state on both sides of every operator approval."""
        self.check_stop()
        self.check_model()
        if self.hardware(require_controller=True) != metadata['hardware_class']:
            raise RuntimeError('Hardware differs from the hardware used for planning')
        request = GetParameters.Request()
        request.names = ['allow_trajectory_execution']
        reply = self.call(GetParameters, '/move_group/get_parameters', request)
        if not reply.values or not reply.values[0].bool_value:
            raise RuntimeError('Start the real robot with allow_execution:=true before replay')
        if (self.input_hashes() != metadata['input_sha256']
                or file_digest(self.path) != metadata['trajectory_sha256']
                or not 0 <= time.time()-metadata['created_at'] <= review_ttl(metadata)
                or scene_fingerprint(self.scene()) != metadata['scene_sha256']):
            raise RuntimeError('Reviewed inputs, age or environment changed; plan and review again')
        if not self.measured_stop():
            raise RuntimeError('The arm is not measured at rest before the reviewed path')
        first = trajectory.joint_trajectory.points[0].positions
        if (not self.fresh() or abs(self.values[HAND]-metadata['gripper_position']) > frozen_hand_tolerance(metadata)
                or any(abs(self.values[name]-value) > START_TOLERANCE for name, value in zip(ARM, first))):
            raise RuntimeError('Robot or gripper moved away from the reviewed segment start; plan again')
        if self.xy_scan:
            expected_tcp = self.kinematics.tcp(dict(zip(ARM, first)))
            measured_tcp = self.kinematics.tcp(self.values)
            if math.dist(measured_tcp[:3, 3], expected_tcp[:3, 3]) > .003:
                raise RuntimeError('TCP moved more than 3 mm from the reviewed segment start; plan again')
        self.check_stop()

    @staticmethod
    def segment_display(template, trajectory):
        display = copy.deepcopy(template)
        display.trajectory = [copy.deepcopy(trajectory)]
        positions = dict(zip(ARM, trajectory.joint_trajectory.points[0].positions))
        state = display.trajectory_start.joint_state
        state.position = [positions.get(name, position) for name, position in zip(state.name, state.position)]
        return display

    def await_approval(self, display, metadata, trajectory, index, total, label, context=None):
        """Only this displayed path's single-use service can release its gate."""
        request_id = uuid.uuid4().hex
        service_name = '/ur5e_a3_stack/approve_next_'+request_id
        gate = {'approved': False, 'active': True}

        def approve(_, response):
            if self.stopped or not gate['active'] or gate['approved']:
                response.success = False
                response.message = 'This path approval is no longer pending'
            else:
                gate['approved'] = True
                response.success = True
                response.message = 'Approved only the currently displayed path '+request_id
            return response

        service = self.create_service(Trigger, service_name, approve)
        try:
            self.display.publish(display)
            duration = trajectory.joint_trajectory.points[-1].time_from_start
            self.phase('STACK_STEP_READY', plan_id=metadata['plan_id'], request_id=request_id,
                       index=index, total=total, label=label,
                       duration=duration.sec+duration.nanosec*1e-9,
                       preview_sha256=self.preview_digest(display), approve_service=service_name)
            published_at = checked_at = time.monotonic()
            first = trajectory.joint_trajectory.points[0].positions
            hand_check = getattr(self, 'approval_hand_ok', None) or (lambda meta: AirTask.approval_hand_ok(self, meta))
            while not gate['approved']:
                rclpy.spin_once(self, timeout_sec=.02)
                self.check_stop()
                if (not self.fresh() or not hand_check(metadata)
                        or any(abs(self.values.get(name, math.inf)-value) > START_TOLERANCE
                               for name, value in zip(ARM, first))):
                    raise RuntimeError('Robot or gripper changed while waiting for path approval')
                now = time.monotonic()
                if now-checked_at > 2.:
                    (context or self.execution_context)(metadata, trajectory)
                    checked_at = time.monotonic()
                if now-published_at > 1.:
                    self.display.publish(display)
                    published_at = now
            self.check_stop()
            self.phase('STACK_STEP_APPROVED', request_id=request_id, index=index, total=total)
        finally:
            gate['active'] = False
            self.destroy_service(service)

    def gripper_runtime(self):
        """Only the existing active position controller may receive this goal."""
        manager = '/controller_manager' if self.mock else '/gripper/controller_manager'
        plugin = 'mock_components/GenericSystem' if self.mock else 'dh_gripper_driver/DHGripperHardwareInterface'
        components = self.call(ListHardwareComponents, manager+'/list_hardware_components',
                               ListHardwareComponents.Request()).component
        matches = [item for item in components if item.class_type == plugin and item.state.id == 3]
        if not any(any(interface.name == HAND+'/position' and interface.is_claimed
                       for interface in item.command_interfaces) for item in matches):
            raise RuntimeError('활성 그리퍼 위치 제어기와 하드웨어를 먼저 확인하세요.')
        controllers = self.call(ListControllers, manager+'/list_controllers', ListControllers.Request()).controller
        if not any(item.name == GRIPPER_CONTROLLER and item.state == 'active' for item in controllers):
            raise RuntimeError('그리퍼 위치 제어기가 active 상태가 아닙니다.')
        prefix = '' if self.mock else '/gripper'
        return prefix+'/'+GRIPPER_CONTROLLER+'/gripper_cmd'

    def alignment_prepare_context(self, metadata, trajectory):
        """This preparation never reuses or relaxes a sealed arm-path guard."""
        self.check_stop()
        if self.mode != 'alignment_prepare' or self.param('workspace_clear') is not True:
            raise RuntimeError('그리퍼 준비 동작 전에 작업 공간 확인이 필요합니다.')
        self.check_model()
        if self.hardware(require_controller=True) != metadata['hardware_class']:
            raise RuntimeError('Hardware changed after the gripper preview')
        self.gripper_runtime()
        request = GetParameters.Request()
        request.names = ['allow_trajectory_execution']
        reply = self.call(GetParameters, '/move_group/get_parameters', request)
        if not reply.values or not reply.values[0].bool_value:
            raise RuntimeError('그리퍼 준비 동작 전에 allow_execution:=true로 연결하세요.')
        if (self.input_hashes() != metadata['input_sha256']
                or not 0 <= time.time()-metadata['created_at'] <= 600
                or scene_fingerprint(self.scene()) != metadata['scene_sha256']):
            raise RuntimeError('맞춤 설정이나 주변 장면이 바뀌었습니다. 다시 준비하세요.')
        if not self.measured_stop() or not self.fresh():
            raise RuntimeError('팔과 그리퍼의 최신 피드백 및 정지를 확인할 수 없습니다.')
        first = trajectory.joint_trajectory.points[0].positions
        if (any(abs(self.values[name]-value) > START_TOLERANCE for name, value in zip(ARM, first))
                or abs(self.values[HAND]-metadata['gripper_position']) > .003):
            raise RuntimeError('팔 또는 그리퍼가 미리보기 시작 자세에서 바뀌었습니다. 다시 준비하세요.')
        self.check_stop()

    def validate_gripper_sweep(self, arm, start, target, expected_scene):
        """Check every 0.01rad of the finger motion with the six arm joints held."""
        if not self.bounds[HAND][0] <= target <= self.bounds[HAND][1]:
            raise RuntimeError('Alignment gripper target is outside its URDF bounds')
        steps = max(1, math.ceil(abs(target-start)/.01))
        for index in range(steps+1):
            self.check_stop()
            q = arm+[start+(target-start)*index/steps]
            if any(not self.bounds[name][0] <= value <= self.bounds[name][1] for name, value in zip(ARM+[HAND], q)):
                raise RuntimeError('Gripper preparation is outside a joint limit')
            request = GetStateValidity.Request()
            request.robot_state.is_diff = True
            request.robot_state.joint_state.name = ARM+[HAND]
            request.robot_state.joint_state.position = q
            reply = self.call(GetStateValidity, '/check_state_validity', request)
            if reply is None or not reply.valid:
                raise RuntimeError(f'현재 팔 자세에서 그리퍼 {ALIGNMENT_HAND:g}rad 준비 경로에 충돌이 있습니다.')
        if scene_fingerprint(self.scene()) != expected_scene:
            raise RuntimeError('그리퍼 준비 검사 중 주변 장면이 바뀌었습니다.')

    @staticmethod
    def alignment_prepare_preview(arm, start, target):
        """Display the finger change together with an explicitly stationary arm."""
        display = DisplayTrajectory()
        display.trajectory_start.joint_state.name = ARM+[HAND]
        display.trajectory_start.joint_state.position = list(arm)+[start]
        hold, fingers = RobotTrajectory(), RobotTrajectory()
        hold.joint_trajectory.joint_names = ARM
        fingers.joint_trajectory.joint_names = ARM+[HAND]
        duration = max(1., abs(target-start)/.05)
        for seconds, hand in ((0., start), (duration, target)):
            for trajectory, positions in ((hold, arm), (fingers, list(arm)+[hand])):
                point = JointTrajectoryPoint()
                point.positions = list(positions)
                point.velocities = [0.]*len(positions)
                point.accelerations = [0.]*len(positions)
                nanos = round(seconds*1e9)
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanos, 10**9)
                trajectory.joint_trajectory.points.append(point)
        display.trajectory = [fingers]
        return display, hold

    def prepare_alignment_gripper(self):
        if self.param('workspace_clear') is not True:
            raise RuntimeError('빈 그리퍼와 작업 공간을 확인한 뒤 맞춤 준비를 실행하세요.')
        self.spin_until(self.fresh)
        if not self.measured_stop():
            raise RuntimeError('팔과 그리퍼가 정지한 뒤 다시 준비하세요.')
        arm, start = [self.values[name] for name in ARM], self.values[HAND]
        display, hold = self.alignment_prepare_preview(arm, start, ALIGNMENT_HAND)
        metadata = dict(created_at=time.time(), hardware_class=self.hardware(require_controller=True),
                        input_sha256=self.input_hashes(), scene_sha256=scene_fingerprint(self.scene()),
                        gripper_position=start, gripper_target=ALIGNMENT_HAND)
        preparation_id = self.param('preparation_id')
        if not isinstance(preparation_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', preparation_id):
            raise RuntimeError('Invalid preparation_id for gripper approval')
        metadata['plan_id'] = preparation_id
        self.alignment_prepare_context(metadata, hold)
        self.validate_gripper_sweep(arm, start, ALIGNMENT_HAND, metadata['scene_sha256'])
        label = f'플레이트 맞춤 준비 · 그리퍼 {ALIGNMENT_HAND:g}rad'
        self.await_approval(display, metadata, hold, 1, 1, label, context=self.alignment_prepare_context)
        self.alignment_prepare_context(metadata, hold)
        endpoint = self.gripper_runtime()
        client = ActionClient(self, GripperCommand, endpoint)
        self.spin_until(client.server_is_ready)
        self.alignment_prepare_context(metadata, hold)
        goal = GripperCommand.Goal()
        goal.command.position = ALIGNMENT_HAND
        goal.command.max_effort = 0.  # Keep the driver's existing force setting.
        self.check_stop()
        self.result_future = None
        self.phase('STACK_STEP_EXECUTING', index=1, total=1, label=label)
        goal_id = GoalUUID(uuid=list(uuid.uuid4().bytes))
        began = time.monotonic()
        try:
            try:
                self.goal_pending = client.send_goal_async(goal, goal_uuid=goal_id)
                while not self.goal_pending.done():
                    rclpy.spin_once(self, timeout_sec=.02)
                    if time.monotonic()-began > 15.:
                        self.stopped = True
                self.goal = self.goal_pending.result()
                if self.goal is None:
                    raise RuntimeError('No gripper goal acknowledgement')
            except Exception:
                # A local send/acceptance error does not prove that the remote
                # goal was rejected. Its UUID remains known even without a
                # ClientGoalHandle, so cancel and recover that exact result.
                self.stopped = True
                self.recover_gripper_goal(endpoint, goal_id)
                raise RuntimeError('그리퍼 목표 수락 통신 오류 후 해당 목표의 정지를 확인했습니다. 다시 준비하세요.')
            if not self.goal.accepted:
                self.goal = self.goal_pending = None
                raise RuntimeError('그리퍼 제어기가 맞춤 준비 목표를 거절했습니다.')
            self.result_future = self.goal.get_result_async()
            cancelled_at, checked_at = -math.inf, time.monotonic()
            while not self.result_future.done():
                rclpy.spin_once(self, timeout_sec=.02)
                now = time.monotonic()
                if (not self.fresh() or any(abs(self.values.get(name, math.inf)-q) > START_TOLERANCE
                                          for name, q in zip(ARM, arm))
                        or not min(start, ALIGNMENT_HAND)-.015 <= self.values.get(HAND, math.inf) <= max(start, ALIGNMENT_HAND)+.015
                        or now-began > 15.):
                    self.stopped = True
                if now-checked_at > .5:
                    try:
                        self.gripper_runtime()
                        if (self.input_hashes() != metadata['input_sha256']
                                or scene_fingerprint(self.scene()) != metadata['scene_sha256']):
                            self.stopped = True
                    except Exception:
                        self.stopped = True
                    checked_at = time.monotonic()
                if self.stopped and now-cancelled_at > .5:
                    self.goal.cancel_goal_async()
                    cancelled_at = now
            result = self.result_future.result()
            quiet = self.measured_stop()
            if quiet and result.status in (4, 5, 6):
                self.goal = self.goal_pending = None
                print('STACK_STOPPED_CONFIRMED gripper goal terminal and measured standstill', flush=True)
            if (self.stopped or result.status != 4 or not result.result.reached_goal or result.result.stalled
                    or not quiet or not self.fresh() or abs(self.values[HAND]-ALIGNMENT_HAND) > .003
                    or any(abs(self.values[name]-q) > START_TOLERANCE for name, q in zip(ARM, arm))):
                raise RuntimeError(f'그리퍼 {ALIGNMENT_HAND:g}rad 도달과 팔 정지를 확인하지 못했습니다. 정지 상태를 확인하세요.')
            self.phase('STACK_STEP_COMPLETED', index=1, total=1, label=label)
            print('STACK_STATUS phase=DONE moved=0 total=0 target=0 layers=0', flush=True)
        finally:
            self.cancel_remaining_goal()
            client.destroy()

    def recover_gripper_goal(self, endpoint, goal_id):
        """Never abandon an ambiguously accepted preparation goal.

        Both service requests name only the UUID chosen before sending. An
        unknown goal, a cancel acknowledgement or a local transport error is
        not terminal evidence; keep retrying until a result and fresh quiet
        feedback prove the gripper and arm have stopped.
        """
        self.stopped = True
        result_kind = GripperCommand.Impl.GetResultService
        cancel_client = self.create_client(CancelGoal, endpoint+'/_action/cancel_goal')
        result_client = self.create_client(result_kind, endpoint+'/_action/get_result')
        cancel_future = result_future = None
        cancel_at = result_at = announced_at = -math.inf
        terminal = False
        try:
            while True:
                now = time.monotonic()
                if cancel_future is not None and cancel_future.done():
                    cancel_future = None
                if not terminal and now-cancel_at > 2. and cancel_client.service_is_ready():
                    if cancel_future is not None:
                        cancel_client.remove_pending_request(cancel_future)
                    request = CancelGoal.Request()
                    request.goal_info.goal_id = goal_id
                    try:
                        cancel_future = cancel_client.call_async(request)
                    except Exception:
                        cancel_future = None
                    cancel_at = now
                if result_future is not None and result_future.done():
                    try:
                        response = result_future.result()
                        terminal = response is not None and response.status in (4, 5, 6)
                    except Exception:
                        terminal = False
                    result_future = None
                if not terminal and (result_future is None or now-result_at > 10.) and result_client.service_is_ready():
                    if result_future is not None:
                        result_client.remove_pending_request(result_future)
                    request = result_kind.Request()
                    request.goal_id = goal_id
                    try:
                        result_future = result_client.call_async(request)
                    except Exception:
                        result_future = None
                    result_at = now
                if terminal and self.measured_stop(seconds=.6) and self.fresh():
                    self.goal = self.goal_pending = self.result_future = None
                    print('STACK_STOPPED_CONFIRMED exact gripper UUID terminal and measured standstill', flush=True)
                    return
                if now-announced_at > 5.:
                    print('STACK_STOP_PENDING recovering exact gripper goal UUID; waiting for terminal result and standstill', flush=True)
                    announced_at = now
                rclpy.spin_once(self, timeout_sec=.05)
        finally:
            for client, future in ((cancel_client, cancel_future), (result_client, result_future)):
                if future is not None:
                    client.remove_pending_request(future)
                self.destroy_client(client)

    def execute(self):
        metadata, display, trajectory = self.load_reviewed_path()
        segments = split_trajectory(trajectory, metadata['metrics'].get('segment_end_indices'))
        labels = metadata.get('segment_labels')
        if not isinstance(labels, list) or len(labels) != len(segments) or any(
                not isinstance(label, str) or not label or len(label) > 200 for label in labels):
            raise RuntimeError('Reviewed segment labels are missing; plan again')
        labels = list(labels)
        stops = alignment_pause_boundaries(metadata, segments)
        self.spin_until(self.fresh)
        self.execution_context(metadata, trajectory)
        self.validate_path(trajectory, metadata['gripper_position'], metadata['scene_sha256'],
                           scan_safe_after=metadata.get('scan_safe_after'))
        if self.execution_mode == 'continuous':
            if stops:
                ends = metadata['metrics']['segment_end_indices']
                paths = split_trajectory(trajectory, [ends[stop['after_segment']-1] for stop in stops])
                labels = [f'{alignment_hold_label(stop)} · 도착 후 판 조정을 위해 정지' for stop in stops]
                paused = {index: stop for index, stop in enumerate(stops, 1)}
                for index in range(1, len(labels)):
                    labels[index] = '판 조정 후 손을 빼고 OK로 계속 · '+labels[index]
            else:
                paths = [trajectory]
                labels = [f'전체 경로 연속 실행 · {len(segments)}개 구간']
                paused = {}
        else:
            paths = segments
            paused = {stop['after_segment']: stop for stop in stops}
            for stop in stops[:-1]:
                labels[stop['after_segment']] = '판 조정 후 손을 빼고 OK로 계속 · '+labels[stop['after_segment']]
        for index, (path, label) in enumerate(zip(paths, labels), 1):
            self.execution_context(metadata, path)
            preview = self.segment_display(display, path)
            self.await_approval(preview, metadata, path, index, len(paths), label)
            # Waiting is not execution authorization for changed state. Recheck
            # again after OK, then send only this exact approved subtrajectory.
            self.execution_context(metadata, path)
            self.execute_arm(path, metadata, index, len(paths), label)
            if index in paused:
                stop = paused[index]
                print('STACK_ALIGNMENT_PAUSED '+json.dumps(dict(
                    corner=stop['corner'], axis=stop['axis'], pocket_cell=stop['pocket_cell'],
                    stop_index=stops.index(stop)+1, total=8), sort_keys=True), flush=True)
        print('STACK_STATUS phase=DONE moved=0 total=0 target=0 layers=0', flush=True)

    def execute_arm(self, trajectory, metadata, index, total, label):
        client = ActionClient(self, FollowJointTrajectory, f'/{CONTROLLER}/follow_joint_trajectory')
        self.spin_until(client.server_is_ready)
        self.execution_context(metadata, trajectory)
        # Send the reviewed six-joint path to the scaled arm controller. Keeping
        # its accepted goal handle also makes Stop cancel that exact arm goal.
        # MoveIt still supplies planning, collision checks, display and the
        # explicit allow_execution gate above.
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        # The driver's general-purpose defaults allow much larger tracking
        # errors. This slow clearance trial requires a close match to the
        # reviewed path; the controller aborts if an arm joint deviates.
        goal.path_tolerance = [JointTolerance(name=name, position=.005) for name in ARM]
        goal.goal_tolerance = [JointTolerance(name=name, position=.005) for name in ARM]
        goal.goal_time_tolerance = Duration(sec=5)
        self.check_stop()
        self.result_future = None
        self.phase('STACK_STEP_EXECUTING', index=index, total=total, label=label)
        self.goal_pending = client.send_goal_async(goal)
        # A stop during goal acceptance must still cancel the eventual accepted goal.
        acceptance_started = time.monotonic()
        while not self.goal_pending.done():
            rclpy.spin_once(self, timeout_sec=.02)
            if time.monotonic()-acceptance_started > 15. and not self.stopped:
                self.stopped = True
                print('STACK_STOP_PENDING waiting for goal acceptance before cancellation', flush=True)
        self.goal = self.goal_pending.result()
        if not self.goal.accepted:
            if self.measured_stop():
                print('STACK_STOPPED_CONFIRMED no arm goal was accepted', flush=True)
            self.goal = None
            self.goal_pending = None
            raise RuntimeError('Arm controller rejected the reviewed path')
        result = self.goal.get_result_async()
        self.result_future = result
        cancel_future = None
        print(f'STACK_AIR_EXECUTING plan_id={metadata["plan_id"]}', flush=True)
        controller_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        check_future, checked_at = None, time.monotonic()
        scene_future, scene_checked_at = None, time.monotonic()
        try:
            while not result.done():
                rclpy.spin_once(self, timeout_sec=.02)
                now = time.monotonic()
                if not self.fresh() or abs(self.values.get(HAND, math.inf)-metadata['gripper_position']) > frozen_hand_tolerance(metadata):
                    self.stopped = True
                if check_future is None and now-checked_at > .5:
                    if not controller_client.service_is_ready():
                        self.stopped = True
                    else:
                        check_future = controller_client.call_async(ListControllers.Request())
                    checked_at = now
                if check_future is not None:
                    if check_future.done():
                        reply = check_future.result()
                        if reply is None or not any(c.name == CONTROLLER and c.state == 'active' for c in reply.controller):
                            self.stopped = True
                        check_future = None
                    elif now-checked_at > 2.:
                        self.stopped = True
                if scene_future is None and now-scene_checked_at > 1.:
                    scene_request = GetPlanningScene.Request()
                    scene_request.components.components = 1023
                    scene_future = scene_client.call_async(scene_request)
                    scene_checked_at = now
                if scene_future is not None:
                    if scene_future.done():
                        response = scene_future.result()
                        if response is None or scene_fingerprint(response.scene) != metadata['scene_sha256']:
                            self.stopped = True
                        scene_future = None
                    elif now-scene_checked_at > 2.:
                        self.stopped = True
                if self.stopped and cancel_future is None:
                    cancel_future = self.goal.cancel_goal_async()
                    print('STACK_STOP_PENDING cancelling reviewed arm path', flush=True)
            reply = result.result()
            quiet = self.measured_stop()
            if quiet and reply.status in (4, 5, 6):
                self.goal = None
                self.goal_pending = None
                # Controller aborts and external cancellations also need this
                # acknowledgement so the GUI can finish its stop checks.
                print('STACK_STOPPED_CONFIRMED arm goal terminal and measured standstill', flush=True)
            if self.stopped:
                if quiet and reply.status in (4, 5, 6):
                    raise RuntimeError('Reviewed empty-hand path stopped')
                raise RuntimeError('Stop is not confirmed; use the physical stop and inspect robot status')
            if reply.status != 4 or reply.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL or not quiet:
                raise RuntimeError('Arm execution failed or measured standstill was not confirmed')
            last = trajectory.joint_trajectory.points[-1].positions
            if any(abs(self.values[name]-value) > .02 for name, value in zip(ARM, last)):
                raise RuntimeError('Measured final arm pose differs from the reviewed path endpoint')
            self.phase('STACK_STEP_COMPLETED', index=index, total=total, label=label)
        finally:
            self.cancel_remaining_goal()
            self.destroy_client(controller_client)
            self.destroy_client(scene_client)
            client.destroy()

    def cancel_remaining_goal(self):
        """Never exit while a previously accepted remote goal can keep moving."""
        if self.goal is None:
            return
        self.stopped = True
        cancelled_at, announced_at = -math.inf, -math.inf
        terminal = False
        while True:
            now = time.monotonic()
            if not terminal and now-cancelled_at > 2.:
                try:
                    self.goal.cancel_goal_async()
                except Exception:
                    pass  # Keep retrying; a local client error is not a stop acknowledgement.
                cancelled_at = now
            if self.result_future is None:
                try:
                    self.result_future = self.goal.get_result_async()
                except Exception:
                    pass
            if self.result_future is not None and self.result_future.done():
                try:
                    response = self.result_future.result()
                    terminal = response is not None and response.status in (4, 5, 6)
                except Exception:
                    terminal = False
                if not terminal:
                    self.result_future = None
            if terminal and self.measured_stop(seconds=.6):
                print('STACK_STOPPED_CONFIRMED', flush=True)
                self.goal = None
                return
            if now-announced_at > 5.:
                print('STACK_STOP_PENDING waiting for remote terminal status and measured standstill', flush=True)
                announced_at = now
            rclpy.spin_once(self, timeout_sec=.05)

    def run(self):
        if self.mode == 'dry_plan':
            self.plan()
        elif self.mode == 'dry_run':
            self.execute()
        elif self.mode == 'alignment_prepare':
            self.prepare_alignment_gripper()
        else:
            raise ValueError('Air task supports dry_plan, dry_run or alignment_prepare only')


def main():
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdigit():
        raise ValueError('ROS_DOMAIN_ID must be numeric')
    fd = os.open(f'/tmp/ur5e_air_task_{domain}.lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
    fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    task = AirTask()
    signal.signal(signal.SIGINT, lambda *_: setattr(task, 'stopped', True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(task, 'stopped', True))
    code = 0
    try:
        task.run()
    except Exception as error:
        print('STACK_ERROR '+str(error), flush=True)
        code = 1
    finally:
        task.cancel_remaining_goal()
        if code and task.goal_pending is None and task.measured_stop():
            print('STACK_STOPPED_CONFIRMED', flush=True)
        task.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(fd)
    raise SystemExit(code)
