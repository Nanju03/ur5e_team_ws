"""Isolated GenericSystem approval regression. Never use on a hardware domain."""
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


assert os.environ.get('ROS_DOMAIN_ID') == '93', 'Fixture-only domain required'
mode = sys.argv[1]
assert mode in ('step', 'continuous')
base = Path('/home/ksn/ur5e_ws/validation')
log = base/f'sim_{mode}_approval.log'
rclpy.init()
node = rclpy.create_node('sim_approval_regression')
values = {}
names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'left_outer_knuckle_joint']


def feedback(message):
    values.update(zip(message.name, message.position))


subscription = node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)


def until(predicate, seconds=30):
    deadline = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(f'Approval test timeout: {log.read_text()[-800:]}')
        rclpy.spin_once(node, timeout_sec=.02)


def call(kind, service, request):
    client = node.create_client(kind, service)
    assert client.wait_for_service(timeout_sec=3)
    future = client.call_async(request)
    until(future.done, 3)
    reply = future.result()
    node.destroy_client(client)
    return reply


def ready_records():
    return [json.loads(line.split('STACK_STEP_READY ', 1)[1])
            for line in log.read_text().splitlines() if 'STACK_STEP_READY ' in line]


def hold(seconds):
    before = dict(values)
    largest = 0.
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        largest = max(largest, max(abs(values[name]-before[name]) for name in names))
    assert largest < 1e-8, (mode, 'Moved while awaiting approval', largest)
    return largest


hardware = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                ListHardwareComponents.Request())
assert hardware.component
assert all(item.class_type == 'mock_components/GenericSystem' for item in hardware.component)
assert all(item.state.id == 3 for item in hardware.component)
claimed = {interface.name for item in hardware.component for interface in item.command_interfaces if interface.is_claimed}
assert {name+'/position' for name in names} <= claimed
until(lambda: set(names) <= values.keys())
until(lambda: bool(ready_records()), 45)
ready = ready_records()[0]
assert ready['index'] == 1 and ready['total'] >= 1
before_approval = dict(values)
waiting_delta = hold(1.25)
assert 'STACK_STEP_EXECUTING' not in log.read_text()
client = node.create_client(Trigger, ready['approve_service'])
assert client.wait_for_service(timeout_sec=3)
first = client.call_async(Trigger.Request())
duplicate = client.call_async(Trigger.Request())
until(first.done, 3)
assert first.result().success
until(lambda: 'STACK_STEP_COMPLETED' in log.read_text() or
      'STACK_STEP_EXECUTING' in log.read_text(), 8)

if mode == 'step':
    until(lambda: len(ready_records()) >= 2, 15)
    second = ready_records()[1]
    assert second['index'] == 2 and second['request_id'] != ready['request_id']
    assert second['approve_service'] != ready['approve_service']
    assert 'STACK_STEP_COMPLETED' in log.read_text()
    assert log.read_text().count('STACK_STEP_EXECUTING') == 1
    after_first = dict(values)
    assert abs(after_first[names[-1]]-before_approval[names[-1]]) > .05
    second_wait_delta = hold(1.25)
    assert log.read_text().count('STACK_STEP_EXECUTING') == 1
else:
    assert ready['total'] == 1
    until(lambda: log.read_text().count('STACK_EXECUTING ') >= 2, 15)
    assert len(ready_records()) == 1
    second_wait_delta = None

assert not duplicate.done() or not duplicate.result().success, 'A preview was approved twice'
node.destroy_client(client)
assert call(Trigger, '/ur5e_a3_stack/stop', Trigger.Request()).success
until(lambda: 'STACK_ERROR' in log.read_text(), 8)
# The simulation backend reports cancellation; allow the controller stop to
# settle, then independently check joint positions remain stationary.
settle = time.monotonic()+.5
while time.monotonic() < settle:
    rclpy.spin_once(node, timeout_sec=.02)
stopped_delta = hold(1.)
result = dict(mode=mode, domain=93, hardware='mock_components/GenericSystem',
              first_ready=ready, waiting_max_joint_delta=waiting_delta,
              second_wait_max_joint_delta=second_wait_delta,
              duplicate_accepted=False, stop_confirmed_max_joint_delta=stopped_delta,
              ready_count=len(ready_records()),
              physical_motion_sent=False)
(base/f'sim_{mode}_approval_result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
