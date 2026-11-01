"""Real handoff checks: subscriptions and read-only services only."""
import hashlib
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from ur5e_block_bringup.air_scene import RobotKinematics
from ur5e_block_bringup.model import load_moveit_settings

assert os.environ.get('ROS_DOMAIN_ID') == '0'
ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HAND = 'left_outer_knuckle_joint'
values, received, velocities, running = {}, {}, {}, []
rclpy.init()
node = rclpy.create_node('actual_robot_readonly_check')


def state(message):
    now = time.monotonic()
    if len(message.name) != len(message.position):
        return
    for index, name in enumerate(message.name):
        if name in ARM+[HAND] and math.isfinite(message.position[index]):
            values[name], received[name] = message.position[index], now
            if len(message.velocity) == len(message.name):
                velocities[name] = message.velocity[index]


def call(kind, service, request):
    client = node.create_client(kind, service)
    try:
        assert client.wait_for_service(timeout_sec=8.), service+' unavailable'
        future = client.call_async(request)
        deadline = time.monotonic()+10.
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.03)
        assert future.done(), service+' timed out'
        return future.result()
    finally:
        node.destroy_client(client)


subscription = node.create_subscription(JointState, '/joint_states', state, qos_profile_sensor_data)
program = node.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                   lambda message: running.append(message.data), 10)
deadline = time.monotonic()+6.
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.03)
assert all(name in received for name in ARM+[HAND]), 'Partial joint feedback'
report = {'domain': 0, 'agent_sent_task_goal': False, 'controllers': {}, 'hardware': {}, 'models': {}}
for manager in ('/controller_manager', '/gripper/controller_manager'):
    components = call(ListHardwareComponents, manager+'/list_hardware_components', ListHardwareComponents.Request()).component
    report['hardware'][manager] = [{'name': item.name, 'plugin': item.class_type, 'state': item.state.id}
                                   for item in components]
    controllers = call(ListControllers, manager+'/list_controllers', ListControllers.Request()).controller
    report['controllers'][manager] = {item.name: item.state for item in controllers}
assert any(item['plugin'] == 'ur_robot_driver/URPositionHardwareInterface' and item['state'] == 3
           for item in report['hardware']['/controller_manager'])
assert any(item['plugin'] == 'dh_gripper_driver/DHGripperHardwareInterface' and item['state'] == 3
           for item in report['hardware']['/gripper/controller_manager'])
assert report['controllers']['/controller_manager']['scaled_joint_trajectory_controller'] == 'active'
assert report['controllers']['/gripper/controller_manager']['dh_ag95_gripper_controller'] == 'active'
desired = load_moveit_settings('/home/ksn/ur5e_ws/robot_calibration/ur5e_20261031.yaml',
                               '/home/ksn/ur5e_ws/physical_setup.yaml')
for owner, names in [('/robot_state_publisher', ['robot_description']),
                     ('/move_group', ['robot_description', 'robot_description_semantic', 'allow_trajectory_execution'])]:
    request = GetParameters.Request()
    request.names = names
    reply = call(GetParameters, owner+'/get_parameters', request)
    report['models'][owner] = {}
    for name, value in zip(names, reply.values):
        if name == 'allow_trajectory_execution':
            report['allow_execution'] = value.bool_value
            assert value.bool_value
        else:
            expected = desired[name]
            matches = value.string_value == expected
            report['models'][owner][name] = dict(matches=matches,
                sha256=hashlib.sha256(value.string_value.encode()).hexdigest())
            assert matches, owner+' '+name+' differs'
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=.02)
report['joint_values_rad'] = {name: values[name] for name in ARM+[HAND]}
report['joint_feedback_age_seconds'] = {name: time.monotonic()-received[name] for name in ARM+[HAND]}
assert max(report['joint_feedback_age_seconds'].values()) < 1.
report['joint_velocity_rad_s'] = {name: velocities.get(name) for name in ARM+[HAND]}
tcp = RobotKinematics(desired['robot_description']).tcp({name: values[name] for name in ARM})
report['measured_tcp_world_m'] = [float(tcp[index, 3]) for index in range(3)]
report['robot_program_running'] = running[-1] if running else None
report['result'] = 'passed'
Path('/home/ksn/ur5e_ws/validation/actual_robot_ready.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({'result': report['result'], 'all_seven_joints_fresh': True,
                  'real_arm_active': True, 'real_gripper_active': True,
                  'model_matches': True, 'allow_execution': report['allow_execution'],
                  'robot_program_running': report['robot_program_running'],
                  'tcp_world_m': report['measured_tcp_world_m'], 'gripper_q': values[HAND]}))
node.destroy_node()
rclpy.shutdown()
