"""Exercise recovery with real MoveIt services on domain 104 GenericSystem only."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, DisplayTrajectory, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetCartesianPath
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from ur5e_block_bringup.air_task import AirTask, ARM, HAND, digest
from ur5e_block_bringup.air_scene import RobotKinematics
from ur5e_block_bringup.model import load_moveit_settings
from ur5e_block_bringup.physical_held import held_fault_path, write_held_fault
from ur5e_block_bringup.physical_stack import scene_fingerprint, normalized_collision_data
from ur5e_block_bringup.physical_recovery import transform_pose

assert os.environ.get('ROS_DOMAIN_ID') == '104', 'Simulation probe is restricted to domain 104'
ROOT = Path('/home/ksn/ur5e_ws/validation')
BASE = ROOT.parent
TOKEN = 'recover_mock_unique'
LOG = ROOT/'recovery_runtime.log'
RESULT = ROOT/'recovery_runtime.result.json'
FAULT = held_fault_path('104')
jobs, streams, q, samples, previews = [], [], {}, [], []
node, active, seen, label = None, None, 0, ''


def process_row(pid):
    try:
        item = Path('/proc')/str(pid)
        stat = (item/'stat').read_text().split(') ', 1)[1].split()
        args = (item/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        env = (item/'environ').read_bytes().split(b'\0')
        domain = next((x.split(b'=', 1)[1].decode() for x in env if x.startswith(b'ROS_DOMAIN_ID=')), 'unset')
        return dict(pid=int(pid), ppid=int(stat[1]), start_ticks=int(stat[19]), domain=domain, args=args[:500])
    except (OSError, ValueError, IndexError):
        return None


def driver_rows():
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        row = process_row(entry.name)
        if row and row['domain'] == '104' and (Path(row['args'].split()[0]).name in {
                'ros2_control_node', 'move_group', 'robot_state_publisher'}
                or '/lib/ur5e_block_bringup/physical_recovery' in row['args']):
            result.append(row)
    return result


def start(command, log):
    assert os.environ['ROS_DOMAIN_ID'] == '104'
    stream = log.open('w')
    streams.append(stream)
    child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                             start_new_session=True, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
    jobs.append((child, process_row(child.pid)))
    return child


def content():
    return LOG.read_text() if LOG.exists() else ''


def records(marker):
    return [json.loads(line.split(marker+' ', 1)[1]) for line in content().splitlines()
            if marker+' ' in line]


def feedback(message):
    q.update(zip(message.name, message.position))
    if all(name in q for name in ARM+[HAND]):
        samples.append(dict(stamp=time.monotonic(), label=label, q=dict(q)))


def pump():
    global seen, label
    rclpy.spin_once(node, timeout_sec=.02)
    lines = content().splitlines()
    for line in lines[seen:]:
        if 'STACK_STEP_EXECUTING ' in line:
            label = json.loads(line.split('STACK_STEP_EXECUTING ', 1)[1])['label']
    seen = len(lines)


def until(predicate, seconds=30.):
    until_at = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() > until_at:
            raise TimeoutError(content()[-4000:])
        if active is not None and active.poll() is not None and 'STACK_RECOVERY_DONE ' not in content():
            raise RuntimeError(content()[-5000:])
        pump()


def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        pump()


def call(kind, endpoint, request, seconds=10.):
    client = node.create_client(kind, endpoint)
    try:
        if not client.wait_for_service(timeout_sec=seconds):
            raise TimeoutError(endpoint+' discovery')
        future = client.call_async(request)
        until(future.done, seconds)
        return future.result()
    finally:
        node.destroy_client(client)


def scene():
    request = GetPlanningScene.Request()
    request.components.components = 1023
    return call(GetPlanningScene, '/get_planning_scene', request).scene


def owned(scene_value):
    return ([obj.id for obj in scene_value.world.collision_objects if obj.id.startswith('ur5e_stack/')]+
            [obj.object.id for obj in scene_value.robot_state.attached_collision_objects
             if obj.object.id.startswith('ur5e_stack/')])


def stop_job(child, birth):
    current = process_row(child.pid)
    if child.poll() is not None or current is None or current['start_ticks'] != birth['start_ticks']:
        return
    assert current['domain'] == '104'
    for sig, seconds in ((signal.SIGINT, 8.), (signal.SIGTERM, 4.), (signal.SIGKILL, 2.)):
        try:
            os.killpg(child.pid, sig)
            child.wait(timeout=seconds)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    global node, active, label
    assert not driver_rows(), 'An existing simulation owns domain 104'
    assert not FAULT.exists(), 'An existing domain-104 fault must not be overwritten'
    LOG.write_text('')
    fixture = start(['ros2', 'launch', str(ROOT/'recovery_mock_system.launch.py')], ROOT/'recovery_fixture.log')
    rclpy.init()
    node = rclpy.create_node('recovery_mock_runtime_probe')
    node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
    node.create_subscription(DisplayTrajectory, '/display_planned_path', previews.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    hardware = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                    ListHardwareComponents.Request(), 30.).component
    assert hardware and all(c.class_type == 'mock_components/GenericSystem' for c in hardware)
    until(lambda: all(name in q for name in ARM+[HAND]), 15.)
    for _ in range(30):
        controllers = call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller
        if {'joint_state_broadcaster', 'scaled_joint_trajectory_controller', 'dh_ag95_gripper_controller'} <= {
                c.name for c in controllers if c.state == 'active'}:
            break
        spin(.25)
    else:
        raise RuntimeError('Mock controllers not active')
    initial_q = dict(q)
    assert abs(initial_q[HAND]) < 1e-12
    layout = start(['ros2', 'launch', 'ur5e_block_bringup', 'block_task.launch.py',
                    'stack_mode:=scene', 'selection_file:='+str(ROOT/'recovery_selection.yaml'),
                    'trajectory_file:='+str(ROOT/'recovery_unused.cdr'), 'allow_mock_hardware:=true'],
                   ROOT/'recovery_scene.log')
    until(lambda: layout.poll() is not None, 20.)
    assert layout.returncode == 0, (ROOT/'recovery_scene.log').read_text()[-2500:]
    initial_scene = scene()
    assert {'ur5e_stack/block_5', 'ur5e_stack/block_7'} <= set(owned(initial_scene))
    foreign = CollisionObject()
    foreign.header.frame_id, foreign.id, foreign.operation = 'world', 'foreign_recovery_fixture', CollisionObject.ADD
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.02, .02, .02])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z, pose.orientation.w = 1.5, 1.5, .9, 1.
    foreign.primitives, foreign.primitive_poses = [primitive], [pose]
    update = PlanningScene()
    update.is_diff = update.robot_state.is_diff = True
    update.world.collision_objects = [foreign]
    assert call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=update)).success
    before = scene()
    foreign_before = normalized_collision_data(next(obj for obj in before.world.collision_objects if obj.id == foreign.id))
    settings = load_moveit_settings(str(BASE/'robot_calibration/ur5e_20261031.yaml'), str(BASE/'physical_setup.yaml'))
    model = settings['robot_description']
    kinematics = RobotKinematics(model)
    home_state = ET.fromstring(settings['robot_description_semantic']).find("group_state[@group='ur5e_arm'][@name='home']")
    home = {item.get('name'): float(item.get('value')) for item in home_state.findall('joint')}
    # Retain a conservative attached shape from an interrupted pickup, even
    # though the mock recovery gripper is confirmed empty.
    ghost = AttachedCollisionObject()
    ghost.link_name = 'grasp_link'
    ghost.touch_links = ['left_finger_pad', 'right_finger_pad', 'left_finger', 'right_finger']
    ghost.object.id, ghost.object.header.frame_id = 'ur5e_stack/block_6', 'grasp_link'
    ghost.object.operation = CollisionObject.ADD
    ghost.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.025, .025, .025])]
    world_cube = np.eye(4)
    world_cube[:3, 3] = [.3703, .09, .8175]
    ghost.object.primitive_poses = [transform_pose(np.linalg.inv(kinematics.tcp(initial_q)) @ world_cube)]
    ghost_update = PlanningScene()
    ghost_update.is_diff = ghost_update.robot_state.is_diff = True
    ghost_update.robot_state.attached_collision_objects = [ghost]
    ghost_update.allowed_collision_matrix = copy.deepcopy(before.allowed_collision_matrix)
    matrix = ghost_update.allowed_collision_matrix
    ghost_index = matrix.entry_names.index(ghost.object.id)
    for name in ghost.touch_links+['workbench', 'ur5e_stack/plate']:
        if name in matrix.entry_names:
            other = matrix.entry_names.index(name)
            matrix.entry_values[ghost_index].enabled[other] = True
            matrix.entry_values[other].enabled[ghost_index] = True
    assert call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=ghost_update)).success
    before = scene()
    assert any(obj.object.id == ghost.object.id for obj in before.robot_state.attached_collision_objects)
    raw_request = GetCartesianPath.Request()
    raw_request.header.frame_id = 'world'
    raw_request.start_state = copy.deepcopy(before.robot_state)
    raw_request.start_state.is_diff = False
    raw_request.group_name, raw_request.link_name = 'ur5e_arm', 'grasp_link'
    raw_target = kinematics.tcp(initial_q).copy()
    raw_target[2, 3] = max(float(raw_target[2, 3]), float(kinematics.tcp(home)[2, 3]), 1.055)
    raw_request.waypoints = [transform_pose(raw_target)]
    raw_request.max_step, raw_request.jump_threshold = .005, 5.
    raw_request.revolute_jump_threshold = .35
    raw_request.avoid_collisions = True
    raw_request.max_velocity_scaling_factor = raw_request.max_acceleration_scaling_factor = .05
    raw = call(GetCartesianPath, '/compute_cartesian_path', raw_request, 20.)
    raw_points = raw.solution.joint_trajectory.points
    raw_timing = dict(fraction=raw.fraction, error_code=raw.error_code.val,
        names=list(raw.solution.joint_trajectory.joint_names), points=len(raw_points),
        times=[p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in raw_points],
        velocity_lengths=[len(p.velocities) for p in raw_points],
        acceleration_lengths=[len(p.accelerations) for p in raw_points],
        max_velocity=max((abs(v) for p in raw_points for v in p.velocities), default=0.),
        max_acceleration=max((abs(v) for p in raw_points for v in p.accelerations), default=0.))
    (ROOT/'recovery_raw_cartesian_timing.json').write_text(json.dumps(raw_timing, indent=2)+'\n')
    print(json.dumps(dict(raw_cartesian_timing=str(ROOT/'recovery_raw_cartesian_timing.json'),
                          raw_duration=raw_timing['times'][-1], points=len(raw_points))), flush=True)
    write_held_fault('Simulation stopped while holding a block', digest(model.encode()), '104')
    active = start(['ros2', 'launch', 'ur5e_block_bringup', 'block_task.launch.py',
                    'stack_mode:=physical_recover', 'selection_file:='+str(ROOT/'recovery_selection.yaml'),
                    'trajectory_file:='+str(ROOT/'recovery_mock.recipe.json'),
                    'recipe_manifest_file:='+str(ROOT/'recovery_mock.recipe.json'),
                    'allow_mock_hardware:=true', 'workspace_clear:=true',
                    'recovery_empty_confirmed:=true', 'preparation_id:='+TOKEN, 'execution_mode:=step'], LOG)
    until(lambda: records('STACK_STEP_READY') and previews, 60.)
    first = records('STACK_STEP_READY')[0]
    assert first['plan_id'] == TOKEN
    assert FAULT.exists() and scene_fingerprint(scene()) == scene_fingerprint(before)
    until(lambda: any(AirTask.preview_digest(p) == first['preview_sha256'] for p in previews), 8.)
    spin(1.)
    assert max(abs(q[n]-initial_q[n]) for n in ARM+[HAND]) < 1e-9
    assert not records('STACK_STEP_EXECUTING')
    approvals, boundaries = [], []
    for index in range(1, first['total']+1):
        until(lambda: any(r['index'] == index for r in records('STACK_STEP_READY')), 60.)
        ready = next(r for r in records('STACK_STEP_READY') if r['index'] == index)
        assert ready['plan_id'] == TOKEN and ready['request_id'] not in approvals
        until(lambda: any(AirTask.preview_digest(p) == ready['preview_sha256'] for p in previews), 8.)
        assert FAULT.exists() and owned(scene()), 'Scene and held latch must remain until complete recovery'
        assert any(obj.object.id == ghost.object.id for obj in scene().robot_state.attached_collision_objects)
        if index == first['total']:
            assert max(abs(q[n]-home[n]) for n in ARM) <= .001
            assert abs(q[HAND]) <= .003
        before_approve = dict(q)
        spin(.35)
        assert max(abs(q[n]-before_approve[n]) for n in ARM+[HAND]) < 1e-9
        label = ready['label']
        assert call(Trigger, ready['approve_service'], Trigger.Request()).success
        approvals.append(ready['request_id'])
        until(lambda: any(r['index'] == index for r in records('STACK_STEP_COMPLETED')),
              min(180., max(60., ready['duration']+25.)))
        boundaries.append(dict(index=index, label=ready['label'], measured=dict(q)))
        print(json.dumps(dict(stage='motion_complete', index=index, total=first['total'], label=ready['label'])), flush=True)
    until(lambda: 'STACK_RECOVERY_DONE ' in content(), 20.)
    until(lambda: active.poll() is not None, 10.)
    assert active.returncode == 0 and 'STACK_ERROR ' not in content(), content()[-5000:]
    after = scene()
    assert not owned(after) and not FAULT.exists()
    foreign_after = normalized_collision_data(next(obj for obj in after.world.collision_objects if obj.id == foreign.id))
    assert foreign_before == foreign_after
    assert max(abs(q[n]-home[n]) for n in ARM) <= .001 and abs(q[HAND]-.4) <= .003
    arm_samples = [sample for sample in samples if sample['label'] and '0.4rad' not in sample['label']]
    assert arm_samples and max(abs(sample['q'][HAND]) for sample in arm_samples) <= .003
    first_label = first['label']
    lift_samples = [sample for sample in samples if sample['label'] == first_label]
    poses = [kinematics.tcp(sample['q']) for sample in lift_samples]
    assert poses and all(float(b[2, 3]-a[2, 3]) >= -.0001 for a, b in zip(poses, poses[1:]))
    start_tcp = kinematics.tcp(initial_q)
    assert max(abs(float(p[a, 3]-start_tcp[a, 3])) for p in poses for a in (0, 1)) <= .001
    report = dict(result='passed', domain=104, hardware='mock_components/GenericSystem',
                  actual_domain_touched=False, actual_robot_goals_sent=False,
                  initial_measured=initial_q, final_measured=dict(q), home=home,
                  movement_before_approval=False, approvals=len(approvals), boundaries=boundaries,
                  frozen_hand_during_arm=True, hand_open_only_after_home=True,
                  vertical_lift_monotonic=True, scene_and_fault_preserved_until_final=True,
                  attached_ghost_preserved_until_final=True,
                  owned_objects_after=owned(after), held_fault_after=FAULT.exists(),
                  foreign_object_preserved=True, arm_results=records('STACK_ARM_RESULT'),
                  finished=records('STACK_RECOVERY_DONE'), feedback_samples=len(samples))
    RESULT.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(result='passed', report=str(RESULT), approvals=len(approvals))), flush=True)


if __name__ == '__main__':
    error = None
    try:
        main()
    except BaseException as exc:
        error = exc
        RESULT.write_text(json.dumps(dict(result='failed', error=str(exc), traceback=traceback.format_exc(),
                                         domain=104, actual_domain_touched=False), indent=2)+'\n')
        traceback.print_exc()
    finally:
        owned_before = driver_rows()
        for child, birth in reversed(jobs):
            stop_job(child, birth)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for stream in streams:
            stream.close()
        if FAULT.exists() and any(child for child, _ in jobs):
            FAULT.unlink()
        remaining = driver_rows()
        cleanup = dict(domain=104, owned_launches=[birth for _, birth in jobs],
                       children_before_stop=owned_before, all_exited=not remaining,
                       remaining=remaining, actual_domain_touched=False)
        (ROOT/'recovery_runtime_cleanup.json').write_text(json.dumps(cleanup, indent=2)+'\n')
        print(json.dumps(dict(cleanup_all_exited=not remaining, remaining=remaining)), flush=True)
    if error is not None:
        raise SystemExit(1)
