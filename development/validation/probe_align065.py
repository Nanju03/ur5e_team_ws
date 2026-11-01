"""Mandatory first-pocket pause integration probe: GenericSystem domain 101 only."""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.msg import DisplayTrajectory
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from ur5e_block_bringup.air_scene import RobotKinematics


parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['continuous', 'stop', 'step'])
parser.add_argument('log')
args = parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID') == '101', 'Only isolated mock domain 101 is allowed'
rclpy.init()
node = rclpy.create_node('align065_mock_probe')
ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HAND = 'left_outer_knuckle_joint'
q, samples, previews = {}, [], []
log = Path(args.log)


def feedback(message):
    q.update(zip(message.name, message.position))
    if all(joint in q for joint in ARM+[HAND]):
        samples.append(dict(q))


subscription = node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
preview_subscription = node.create_subscription(
    DisplayTrajectory, '/display_planned_path', previews.append,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))


def contents():
    return log.read_text() if log.exists() else ''


def records(marker):
    return [json.loads(line.split(marker+' ', 1)[1]) for line in contents().splitlines()
            if marker+' ' in line]


def until(predicate, seconds=60.):
    deadline = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(contents()[-2500:])
        rclpy.spin_once(node, timeout_sec=.02)


def spin(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)


def call(kind, service, request):
    client = node.create_client(kind, service)
    assert client.wait_for_service(timeout_sec=8.), service
    future = client.call_async(request)
    until(future.done, 8.)
    result = future.result()
    node.destroy_client(client)
    return result


def same_pose(reference, tolerance=1e-9):
    return max(abs(q[joint]-reference[joint]) for joint in ARM+[HAND]) < tolerance


components = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                  ListHardwareComponents.Request()).component
assert components and all(component.class_type == 'mock_components/GenericSystem'
                          and component.state.id == 3 for component in components)
covered = {interface.name for component in components for interface in component.command_interfaces
           if interface.is_claimed}
assert {joint+'/position' for joint in ARM+[HAND]} <= covered
until(lambda: all(joint in q for joint in ARM+[HAND]))
request = GetParameters.Request()
request.names = ['robot_description']
description = call(GetParameters, '/robot_state_publisher/get_parameters', request).values[0].string_value
kinematics = RobotKinematics(description)
until(lambda: records('STACK_STEP_READY') and previews)
first = records('STACK_STEP_READY')[0]
initial = dict(q)
assert abs(initial[HAND]-.4) < .001, 'The default open value must remain 0.4'
spin(1.)
assert same_pose(initial), 'Motion happened before the first explicit approval'
assert not records('STACK_STEP_EXECUTING')
assert first['index'] == 1
if args.mode != 'step':
    assert first['total'] == 2, first
assert call(Trigger, first['approve_service'], Trigger.Request()).success

if args.mode == 'step':
    # Every preliminary segment still needs a unique OK. Stop approving when
    # the first A hold completes and the mandatory adjustment pause is reached.
    handled = 1
    while 'STACK_ALIGNMENT_PAUSED' not in contents():
        until(lambda: len(records('STACK_STEP_READY')) > handled or 'STACK_ALIGNMENT_PAUSED' in contents())
        if 'STACK_ALIGNMENT_PAUSED' in contents():
            break
        ready = records('STACK_STEP_READY')[handled]
        assert ready['index'] == handled+1 and ready['total'] == first['total']
        assert call(Trigger, ready['approve_service'], Trigger.Request()).success
        handled += 1
    until(lambda: len(records('STACK_STEP_READY')) > handled)
    paused_ready = records('STACK_STEP_READY')[handled]
else:
    until(lambda: len(records('STACK_STEP_READY')) == 2)
    paused_ready = records('STACK_STEP_READY')[1]
    assert paused_ready['index'] == paused_ready['total'] == 2
assert 'STACK_ALIGNMENT_PAUSED' in contents(), 'First A pause marker was not emitted'
assert 'OK' in paused_ready['label'], paused_ready
spin(.1)
held = dict(q)
tcp_a = kinematics.tcp({joint:q[joint] for joint in ARM})[:3, 3].tolist()
assert abs(tcp_a[0]-.3109) < .001 and abs(tcp_a[1]-.15) < .001, tcp_a
assert abs(q[HAND]-.65) < .001
minimum_a = kinematics.gripper_min_z({joint:q[joint] for joint in ARM}, q[HAND])
assert minimum_a >= .820-1e-5
executions_before = len(records('STACK_STEP_EXECUTING'))
spin(2.)
assert same_pose(held), 'Robot left first A pocket without explicit resume'
assert len(records('STACK_STEP_EXECUTING')) == executions_before
assert 'STACK_STATUS phase=DONE' not in contents()
old = node.create_client(Trigger, first['approve_service'])
assert not old.wait_for_service(timeout_sec=.4), 'An expired first approval remained callable'
node.destroy_client(old)
result = dict(domain=101, hardware=[component.class_type for component in components],
              mode=args.mode, default_gripper_position=initial[HAND], alignment_target=.65, held_before_first_OK=True, first_pocket_cell=1,
              first_pocket_tcp_world_m=tcp_a, first_pocket_gripper_position=q[HAND],
              first_pocket_min_gripper_z_m=minimum_a, held_at_A_for_seconds=2.,
              explicit_resume_required=True, no_automatic_B_motion=True,
              expired_first_approval_unavailable=True)
if args.mode == 'continuous':
    assert call(Trigger, paused_ready['approve_service'], Trigger.Request()).success
    until(lambda: 'STACK_STATUS phase=DONE' in contents(), 120.)
    spin(.3)
    assert len(records('STACK_STEP_READY')) == 2
    assert [record['index'] for record in records('STACK_STEP_EXECUTING')] == [1, 2]
    assert [record['index'] for record in records('STACK_STEP_COMPLETED')] == [1, 2]
    tcp_d = kinematics.tcp({joint:q[joint] for joint in ARM})[:3, 3].tolist()
    assert abs(tcp_d[0]-.3109) < .001 and abs(tcp_d[1]+.15) < .001, tcp_d
    assert abs(q[HAND]-.65) < .001
    result.update(resumed_with_fresh_OK=True, completed='DONE', final_pocket_cell=21,
                  final_tcp_world_m=tcp_d, final_gripper_position=q[HAND],
                  final_gripper_min_z_m=kinematics.gripper_min_z({joint:q[joint] for joint in ARM}, q[HAND]))
else:
    assert call(Trigger, '/ur5e_a3_stack/stop', Trigger.Request()).success
    until(lambda: 'STACK_ERROR' in contents())
    spin(1.)
    assert same_pose(held), 'Pose changed after stopping at A'
    assert 'STACK_STATUS phase=DONE' not in contents()
    assert len(records('STACK_STEP_EXECUTING')) == executions_before
    result.update(stop_while_adjusting_confirmed=True, no_resume_after_Stop=True)
result.update(result='passed', feedback_samples=len(samples), real_robot_goals_sent=False)
log.with_suffix('.result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
