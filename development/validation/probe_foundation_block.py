"""Foundation stacking integration probe: refuses every domain except mock 100."""
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import yaml

assert os.environ.get('ROS_DOMAIN_ID') == '100', 'Only isolated mock domain 100 is allowed'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
BASE = Path('/home/ksn/ur5e_ws/validation')
LOG = BASE/'foundation_block_run.log'
LAYOUT = BASE/'foundation_block_layout.yaml'
ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HAND = 'left_outer_knuckle_joint'
rclpy.init()
node = rclpy.create_node('foundation_block_mock_probe')
q, samples = {}, []


def feedback(message):
    q.update(zip(message.name, message.position))
    if all(joint in q for joint in ARM+[HAND]):
        samples.append(dict(q))


subscription = node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)


def contents():
    return LOG.read_text() if LOG.exists() else ''


def until(predicate, seconds=60.):
    deadline = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(contents()[-3500:])
        rclpy.spin_once(node, timeout_sec=.02)


def spin(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)


def call(kind, service, request, timeout=10.):
    client = node.create_client(kind, service)
    assert client.wait_for_service(timeout_sec=timeout), service
    try:
        future = client.call_async(request)
        until(future.done, timeout)
        return future.result()
    finally:
        node.destroy_client(client)


def assert_mock():
    components = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                      ListHardwareComponents.Request()).component
    assert components and all(c.class_type == 'mock_components/GenericSystem' and c.state.id == 3
                              for c in components), 'Physical or inactive hardware is forbidden'
    covered = {i.name for c in components for i in c.command_interfaces if i.is_claimed}
    assert {joint+'/position' for joint in ARM+[HAND]} <= covered
    return components


def scene():
    request = GetPlanningScene.Request()
    request.components.components = 1023
    return call(GetPlanningScene, '/get_planning_scene', request).scene


def pose_tuple(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def cube_position(obj):
    assert len(obj.primitives) == 1 and len(obj.primitive_poses) == 1
    # MoveIt may normalize the object origin while serializing the same shape.
    # The cube's primitive offset is either world-positioned under an identity
    # object pose or zero under the object origin, so their translations add.
    p = obj.primitive_poses[0].position
    origin = obj.pose.position
    return [origin.x+p.x, origin.y+p.y, origin.z+p.z]


def pose_digest(obj):
    data = serialize_message(obj.pose)+b''.join(serialize_message(p) for p in obj.primitive_poses)
    return hashlib.sha256(data).hexdigest()


def ready_records():
    return [json.loads(line.split('STACK_STEP_READY ', 1)[1]) for line in contents().splitlines()
            if 'STACK_STEP_READY ' in line]


components = assert_mock()
until(lambda: all(j in q for j in ARM+[HAND]))
assert abs(q[HAND]-.4) < .001, q[HAND]
params = yaml.safe_load((BASE/'corrected_plate_three_layers_layout.yaml').read_text())['/**']['ros__parameters']
params.update(target_cell=11, empty_cells=sorted(set(range(1,25))-{4,11,13}),
              # A saved pre-correction layout must migrate at launch while
              # preserving the operator-selected destination pocket.
              obstacle_cells=[], target_has_foundation=False, plate_task_revision=2,
              plate_alignment_test=False, empty_xy_scan=False, velocity_scaling=.6,
              acceleration_scaling=.6)
LAYOUT.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
command = ['ros2', 'launch', 'ur5e_a3_grid', 'stack_blocks.launch.py',
           'stack_mode:=run', 'execution_mode:=continuous', 'riser_height:=0.016',
           f'selection_file:={LAYOUT}']
stream = LOG.open('w')
task = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                        start_new_session=True)
try:
    until(lambda: ready_records() or 'STACK_ERROR' in contents(), 120.)
    assert 'STACK_ERROR' not in contents(), contents()[-4000:]
    first = ready_records()[0]
    assert first['index'] == first['total'] == 1, first
    plan = re.findall(r'STACK_PLANNING source=(\d+) target=(\d+) layer=(\d+)/(\d+)', contents())
    assert {int(item[0]) for item in plan} == {4,13}, plan
    assert [int(item[2]) for item in plan] == [2,3], plan
    assert all(int(item[1]) == 11 and int(item[3]) == 3 for item in plan)
    assert 'STACK_PREFLIGHT_PASSED moves=2 layers=3' in contents()
    initial_scene = scene()
    initial_cubes = {obj.id:obj for obj in initial_scene.world.collision_objects
                     if obj.id.startswith('ur5e_stack/block_')}
    assert set(initial_cubes) == {'ur5e_stack/block_4','ur5e_stack/block_11','ur5e_stack/block_13'}
    target = initial_cubes['ur5e_stack/block_11']
    initial_target_xyz = cube_position(target)
    assert max(abs(a-b) for a,b in zip(initial_target_xyz,[.4297,.03,.8175])) < 1e-9, initial_target_xyz
    assert list(target.primitives[0].dimensions) == [.025]*3
    original_target_pose = pose_digest(target)
    held = dict(q)
    spin(1.)
    assert max(abs(q[j]-held[j]) for j in ARM+[HAND]) < 1e-9
    assert 'STACK_STEP_EXECUTING' not in contents()
    assert_mock()
    assert call(Trigger, first['approve_service'], Trigger.Request()).success
    until(lambda: 'STACK_STATUS phase=DONE' in contents() or 'STACK_ERROR' in contents(), 180.)
    assert 'STACK_ERROR' not in contents(), contents()[-4000:]
    spin(.3)
    final_scene = scene()
    final_cubes = {obj.id:obj for obj in final_scene.world.collision_objects
                   if obj.id.startswith('ur5e_stack/block_')}
    assert set(final_cubes) == set(initial_cubes)
    positions = {name:cube_position(obj) for name,obj in final_cubes.items()}
    assert pose_digest(final_cubes['ur5e_stack/block_11']) == original_target_pose
    assert sorted(round(xyz[2],7) for xyz in positions.values()) == [.8175,.8425,.8675], positions
    assert all(abs(xyz[0]-.4297) < 1e-8 and abs(xyz[1]-.03) < 1e-8 for xyz in positions.values())
    assert not final_scene.robot_state.attached_collision_objects
    assert len(ready_records()) == 1
    assert 'STACK_ALIGNMENT_PAUSED' not in contents()
    task.wait(timeout=10.)
    assert task.returncode == 0
    report = dict(domain=100, hardware=[c.class_type for c in components],
                  target_cell=11, foundation_block_initial_xyz_m=initial_target_xyz,
                  foundation_present_before_motion=True, foundation_pose_unchanged=True,
                  old_empty_target_layout_migrated_to_foundation=True,
                  source_cells=sorted(int(item[0]) for item in plan), carried_layers=[2,3],
                  final_layers=3, final_top_world_z_m=.880, final_cube_positions_m=positions,
                  no_attached_objects=True, held_before_OK=True, continuous_OK_count=1,
                  feedback_samples=len(samples), real_robot_goals_sent=False, result='passed')
    (BASE/'foundation_block_runtime.result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
finally:
    if task.poll() is None:
        try:
            call(Trigger, '/ur5e_a3_stack/stop', Trigger.Request(), timeout=3.)
        except Exception:
            pass
        os.killpg(task.pid, signal.SIGINT)
        task.wait(timeout=15.)
    stream.close()
    node.destroy_node()
    rclpy.shutdown()
