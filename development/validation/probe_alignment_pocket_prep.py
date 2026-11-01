"""Mock-only reviewed preparation proof; rejects any other domain/hardware."""
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from controller_manager_msgs.srv import ListHardwareComponents
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from moveit_msgs.msg import DisplayTrajectory
from ur5e_block_bringup.air_task import ARM, HAND, AirTask

assert os.environ.get('ROS_DOMAIN_ID') == '97'
rclpy.init()
node = rclpy.create_node('mock_alignment_prep_probe')
q, previews = {}, []
node.create_subscription(JointState, '/joint_states', lambda message: q.update(zip(message.name, message.position)), qos_profile_sensor_data)
node.create_subscription(DisplayTrajectory, '/display_planned_path', previews.append,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))


def spin(seconds):
    until = time.monotonic()+seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.02)


def request(kind, name, data):
    client = node.create_client(kind, name)
    assert client.wait_for_service(timeout_sec=10.)
    future = client.call_async(data)
    until = time.monotonic()+10.
    while not future.done() and time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.02)
    assert future.done()
    response = future.result()
    node.destroy_client(client)
    return response


components = request(ListHardwareComponents, '/controller_manager/list_hardware_components', ListHardwareComponents.Request()).component
assert components and all(component.class_type == 'mock_components/GenericSystem' for component in components)
spin(1.)
assert all(name in q for name in ARM+[HAND]) and previews
before = dict(q)
assert abs(q[HAND]-.4) < .001
log_path = Path('/home/ksn/ur5e_ws/validation/alignment_pocket_prep.log')
ready = next(json.loads(line.split('STACK_STEP_READY ', 1)[1]) for line in log_path.read_text().splitlines() if 'STACK_STEP_READY ' in line)
assert ready['plan_id'] == 'prep_pockets_mock_97'
assert AirTask.preview_digest(previews[-1]) == ready['preview_sha256']
assert list(previews[-1].trajectory[0].joint_trajectory.points[-1].positions)[-1] == .7
spin(1.)
assert max(abs(q[name]-before[name]) for name in ARM+[HAND]) < .001, 'Moved before approval'
assert request(Trigger, ready['approve_service'], Trigger.Request()).success
until = time.monotonic()+15.
while 'STACK_STATUS phase=DONE' not in log_path.read_text() and time.monotonic() < until:
    rclpy.spin_once(node, timeout_sec=.02)
spin(.4)
log = log_path.read_text()
assert 'STACK_STATUS phase=DONE' in log and 'STACK_STOPPED_CONFIRMED' in log, log
assert abs(q[HAND]-.7) < .003
arm_delta = max(abs(q[name]-before[name]) for name in ARM)
assert arm_delta < .001
report = dict(domain=97, hardware='mock_components/GenericSystem', held_before_approval=True,
              preview_sha256_checked=True, gripper_before=before[HAND], gripper_after=q[HAND],
              maximum_arm_delta_rad=arm_delta, terminal_and_hand_standstill_confirmed=True)
Path('/home/ksn/ur5e_ws/validation/alignment_pocket_prep_result.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report))
node.destroy_node()
rclpy.shutdown()
