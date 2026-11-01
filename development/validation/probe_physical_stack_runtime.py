"""Exercise the reviewed physical executor on GenericSystem domain 103 ONLY."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from ur5e_block_bringup.air_task import AirTask, ARM, HAND
from ur5e_block_bringup.physical_stack import HAND_HOLD_TOLERANCE

assert os.environ.get('ROS_DOMAIN_ID') == '103', 'This probe cannot address a real-robot domain'
ROOT = Path('/home/ksn/ur5e_ws/validation')
LOG = Path(os.environ.get('PHYSICAL_STACK_RUNTIME_LOG', str(ROOT/'physical_stack_continuous_runtime.log')))
RESULT = Path(os.environ.get('PHYSICAL_STACK_RUNTIME_RESULT', str(ROOT/'physical_stack_continuous_runtime.result.json')))
RECIPE = Path(os.environ.get('PHYSICAL_STACK_RUNTIME_RECIPE', str(ROOT/'physical_stack_one.recipe.json')))
SELECTION = ROOT/'physical_stack_one_block.yaml'
# A repeated probe must not ingest EXECUTING events from its previous run
# while performing the initial read-only hardware and scene checks.
LOG.write_text('')
rclpy.init()
node = rclpy.create_node('physical_stack_mock_runtime_probe')
q, samples, previews, motions = {}, [], [], []
current_label = ''
seen_lines = 0
child = None


def feedback(message):
    q.update(zip(message.name, message.position))
    if all(name in q for name in ARM+[HAND]):
        samples.append((time.monotonic(), current_label, {name:q[name] for name in ARM+[HAND]}))


subscription = node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
preview_subscription = node.create_subscription(DisplayTrajectory, '/display_planned_path', previews.append,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))


def content():
    return LOG.read_text() if LOG.exists() else ''


def records(marker):
    return [json.loads(line.split(marker+' ', 1)[1]) for line in content().splitlines() if marker+' ' in line]


def pump():
    global seen_lines, current_label
    rclpy.spin_once(node, timeout_sec=.02)
    lines = content().splitlines()
    for line in lines[seen_lines:]:
        if 'STACK_STEP_EXECUTING ' in line:
            event = json.loads(line.split('STACK_STEP_EXECUTING ', 1)[1])
            current_label = event['label']
            motions.append(dict(label=current_label, q=copy.deepcopy(q), stamp=time.monotonic()))
    seen_lines = len(lines)


def until(predicate, seconds=30.):
    deadline = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(content()[-3000:])
        if child is not None and child.poll() is not None and 'phase=DONE' not in content():
            raise RuntimeError(content()[-5000:])
        pump()


def spin(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        pump()


def call(kind, service, request, seconds=10.):
    client = node.create_client(kind, service)
    assert client.wait_for_service(timeout_sec=seconds), service
    future = client.call_async(request)
    until(future.done, seconds)
    result = future.result()
    node.destroy_client(client)
    return result


def scene():
    request = GetPlanningScene.Request()
    request.components.components = 1023
    return call(GetPlanningScene, '/get_planning_scene', request).scene


def object_pose(obj):
    return json.dumps(dict(pose=message_to_ordereddict(obj.pose),
                           primitive_poses=[message_to_ordereddict(p) for p in obj.primitive_poses]), sort_keys=True)


def center(obj):
    assert abs(obj.pose.orientation.x)+abs(obj.pose.orientation.y) < 1e-7
    # These blocks are fixed axis-aligned cubes. Full scene normalization may
    # move translation from primitive pose into CollisionObject.pose.
    return [getattr(obj.pose.position, a)+getattr(obj.primitive_poses[0].position, a) for a in 'xyz']


components = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                  ListHardwareComponents.Request()).component
assert components and all(item.class_type == 'mock_components/GenericSystem' and item.state.id == 3 for item in components)
claimed = {interface.name for item in components for interface in item.command_interfaces if interface.is_claimed}
assert {name+'/position' for name in ARM+[HAND]} <= claimed
until(lambda: all(name in q for name in ARM+[HAND]))
initial_q = dict(q)
assert abs(q[HAND]-.4) < .001
before = scene()
foundation_before = next(obj for obj in before.world.collision_objects if obj.id == 'ur5e_stack/block_11')
foundation_pose = object_pose(foundation_before)
assert abs(center(foundation_before)[2]-.8175) < 1e-8
command = ['ros2', 'launch', 'ur5e_block_bringup', 'block_task.launch.py', 'stack_mode:=physical_run',
           'selection_file:='+str(SELECTION),
           'trajectory_file:='+str(RECIPE),
           'recipe_manifest_file:='+str(RECIPE),
           'allow_mock_hardware:=true', 'workspace_clear:=true', 'execution_mode:=continuous']
with LOG.open('w') as stream:
    child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
                             env={**os.environ, 'PYTHONUNBUFFERED':'1'})
try:
    until(lambda: records('STACK_STEP_READY') and previews, 40.)
    first = records('STACK_STEP_READY')[0]
    assert first['index'] == first['total'] == 1
    until(lambda: any(AirTask.preview_digest(display) == first['preview_sha256'] for display in previews), 8.)
    spin(1.)
    assert max(abs(q[name]-initial_q[name]) for name in ARM+[HAND]) < 1e-9
    assert not motions
    assert call(Trigger, first['approve_service'], Trigger.Request()).success
    until(lambda: 'phase=DONE' in content() or 'STACK_ERROR ' in content(), 300.)
    assert 'STACK_ERROR ' not in content(), content()[-5000:]
    until(lambda: child.poll() is not None, 15.)
    assert child.returncode == 0
    spin(.5)
    after = scene()
    blocks = {obj.id: obj for obj in after.world.collision_objects if obj.id.startswith('ur5e_stack/block_')}
    assert set(blocks) == {'ur5e_stack/block_11', 'ur5e_stack/block_4'}
    assert object_pose(blocks['ur5e_stack/block_11']) == foundation_pose
    foundation, placed = center(blocks['ur5e_stack/block_11']), center(blocks['ur5e_stack/block_4'])
    assert abs(foundation[2]-.8175) < 1e-8 and abs(placed[2]-.8425) < .001
    assert max(abs(foundation[i]-placed[i]) for i in (0,1)) < .001
    assert not after.robot_state.attached_collision_objects
    labels = [motion['label'] for motion in motions]
    grasp = next(i for i,label in enumerate(labels) if label.startswith('Close on block'))
    first_release = next(i for i,label in enumerate(labels) if 'first opening' in label)
    clearance = next(i for i,label in enumerate(labels) if 'clearance opening before retreat' in label)
    retreat = next(i for i,label in enumerate(labels) if label.startswith('Retreat from layer'))
    assert grasp < first_release < clearance < retreat
    assert clearance == first_release+1 and retreat == clearance+1
    release_samples = [state for _,label,state in samples
                       if 'first opening' in label or 'clearance opening before retreat' in label]
    held_arm = motions[first_release]['q']
    max_release_arm_delta = max(abs(state[name]-held_arm[name]) for state in release_samples for name in ARM)
    assert max_release_arm_delta < .001
    assert any(abs(state[HAND]-.73) < .001 for _,_,state in samples)
    assert any(abs(state[HAND]-.70) < .001 for state in release_samples)
    assert any(abs(state[HAND]-.60) < .001 for state in release_samples)
    assert abs(motions[retreat]['q'][HAND]-.60) < .001
    assert abs(q[HAND]-.60) < .001
    monitor_warmups = records('STACK_MONITORS_READY')
    assert len(monitor_warmups) == len(motions)
    assert all(item['services'] == 7 for item in monitor_warmups)
    assert 'STACK_STOP_REASON ' not in content()
    envelopes = records('STACK_HOLDING_ENVELOPE_VALIDATED')
    if os.environ.get('PHYSICAL_STACK_RUNTIME_REQUIRE_ENVELOPE') == '1':
        assert len(envelopes) == 4
        assert len({item['step'] for item in envelopes}) == 4
        assert all(abs(item['lower']-(.73-HAND_HOLD_TOLERANCE)) < 1e-9 and abs(item['upper']-.733) < 1e-9
                   and item['finger_shapes'] >= 3 and item['arm_samples'] >= 1
                   and item['checked'] == item['finger_shapes']*item['arm_samples'] for item in envelopes)
    arm_results = records('STACK_ARM_RESULT')
    if os.environ.get('PHYSICAL_STACK_RUNTIME_REQUIRE_TRACKING') == '1':
        assert len(arm_results) == 8
        for item in arm_results:
            assert item['classification'] == 'SUCCESS' and item['quiet']
            assert item['goal_position_tolerance'] == item['endpoint_position_tolerance'] == .001
            assert item['goal_velocity_tolerance'] == .01 and item['goal_time_seconds'] == 5
            assert item['path_position_tolerance'] == {
                name:(.01 if item['phase']=='transfer' and name==ARM[-1] else .005) for name in ARM}
            assert max(abs(joint['error']) for joint in item['joints'].values()) <= .001
    report = dict(domain=103, hardware='mock_components/GenericSystem', held_before_OK=True,
                  preview_hash_verified=True, continuous_approvals=1, result='passed',
                  gripper_sequence=[.73,.70,.60], maximum_release_arm_delta_rad=max_release_arm_delta,
                  release_clearance_before_retreat=True, final_gripper_q=q[HAND],
                  foundation_pose_preserved=True, block_centers_world_m={key:center(obj) for key,obj in blocks.items()},
                  attached_objects=0, feedback_samples=len(samples),
                  motion_labels=labels, monitor_warmups=len(monitor_warmups),
                  monitored_services_per_motion=7, monitor_stop_reasons=[],
                  holding_envelope_validations=envelopes,
                  arm_tracking_results=arm_results,
                  actual_robot_goals_sent=False)
    RESULT.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))
except Exception as error:
    if child.poll() is None:
        try:
            call(Trigger, '/ur5e_a3_stack/stop', Trigger.Request(), 5.)
        except Exception:
            os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=30.)
        except subprocess.TimeoutExpired:
            pass
    raise
finally:
    node.destroy_node()
    rclpy.shutdown()
