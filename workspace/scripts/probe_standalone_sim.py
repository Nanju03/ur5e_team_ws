"""Smoke test: refuses any non-GenericSystem controller or non-sim domain."""
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListHardwareComponents
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

assert os.environ.get('ROS_DOMAIN_ID') == '42'
rclpy.init()
node = rclpy.create_node('standalone_sim_smoke_test')
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
values, samples = {}, []


def feedback(message):
    values.update(zip(message.name, message.position))
    if joint_names[0] in values:
        samples.append(values[joint_names[0]])


def wait(future, timeout=15.):
    deadline = time.monotonic()+timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    assert future.done(), 'Timed out'
    return future.result()


subscription = node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
hardware = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
assert hardware.wait_for_service(timeout_sec=10.)
components = wait(hardware.call_async(ListHardwareComponents.Request())).component
assert components and all(c.class_type == 'mock_components/GenericSystem' and c.state.id == 3 for c in components)
deadline = time.monotonic()+5.
while not all(j in values for j in joint_names) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.05)
assert all(j in values for j in joint_names)
start = [values[j] for j in joint_names]
client = ActionClient(node, FollowJointTrajectory, '/ur5e_arm_controller/follow_joint_trajectory')
assert client.wait_for_server(timeout_sec=10.)
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = joint_names
for seconds, offset in [(2, .02), (4, 0.)]:
    point = JointTrajectoryPoint()
    point.positions = list(start)
    point.positions[0] += offset
    point.velocities = [0.]*6
    point.time_from_start.sec = seconds
    goal.trajectory.points.append(point)
handle = wait(client.send_goal_async(goal))
assert handle.accepted
result = wait(handle.get_result_async())
assert result.status == 4 and result.result.error_code == 0
for _ in range(5):
    rclpy.spin_once(node, timeout_sec=.05)
maximum_delta = max(abs(q-start[0]) for q in samples)
return_error = max(abs(values[j]-q) for j, q in zip(joint_names, start))
assert maximum_delta > .015 and return_error < .001
report = dict(domain=42, hardware=[c.class_type for c in components], samples=len(samples),
              maximum_joint_delta_rad=maximum_delta, return_error_rad=return_error, result='passed')
(Path(__file__).resolve().parents[1]/'log/sim_entry_motion_result.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report))
node.destroy_node()
rclpy.shutdown()
