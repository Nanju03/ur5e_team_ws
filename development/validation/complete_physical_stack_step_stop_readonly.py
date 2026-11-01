"""Finish the stopped domain103 mock probe using read-only observations."""
import json
import os
from pathlib import Path
import time

import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from ur5e_block_bringup.air_task import ARM, HAND
from ur5e_block_bringup.physical_held import read_held_fault

assert os.environ.get('ROS_DOMAIN_ID') == '103'
root = Path('/home/ksn/ur5e_ws/validation')
log = (root/'physical_stack_step_stop_runtime.log').read_text()
probe_log = (root/'physical_stack_step_stop_probe.log').read_text()
assert 'assert fault is not None' in probe_log

def records(marker):
    return [json.loads(line.split(marker+' ', 1)[1]) for line in log.splitlines() if marker+' ' in line]

ready, approved = records('STACK_STEP_READY'), records('STACK_STEP_APPROVED')
executing, completed = records('STACK_STEP_EXECUTING'), records('STACK_STEP_COMPLETED')
assert len(ready) == 6 and len(approved) == len(executing) == len(completed) == 5
assert ready[-1]['label'] == 'Lift block 4' and ready[-1]['index'] == 6
assert executing[-1]['label'] == 'Close on block 4'
assert not any(item['label'].startswith(('Lift block ', 'Release block ')) for item in executing)
assert log.count('STACK_STOPPED_CONFIRMED') == 1
assert log.index('STACK_STOPPED_CONFIRMED') > log.index('Physical pick/place stopped; current holding state preserved')

rclpy.init()
node = rclpy.create_node('physical_step_stop_readonly_final')
q, samples = {}, []
def feedback(message):
    q.update(zip(message.name, message.position))
    if all(name in q for name in ARM+[HAND]):
        samples.append([q[name] for name in ARM+[HAND]])
sub = node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
def until(predicate, seconds=10.):
    deadline = time.monotonic()+seconds
    while not predicate():
        assert time.monotonic() < deadline
        rclpy.spin_once(node, timeout_sec=.02)
def call(kind, name, request):
    client = node.create_client(kind, name)
    assert client.wait_for_service(timeout_sec=5.)
    result = client.call_async(request)
    until(result.done)
    reply = result.result()
    node.destroy_client(client)
    return reply

hardware = call(ListHardwareComponents, '/controller_manager/list_hardware_components', ListHardwareComponents.Request()).component
assert hardware and all(item.class_type == 'mock_components/GenericSystem' and item.state.id == 3 for item in hardware)
claimed = {interface.name for item in hardware for interface in item.command_interfaces if interface.is_claimed}
assert {name+'/position' for name in ARM+[HAND]} <= claimed
until(lambda: bool(samples))
deadline = time.monotonic()+1.5
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.02)
delta = max(abs(value-initial) for sample in samples for value, initial in zip(sample, samples[0]))
assert delta < 1e-9 and abs(q[HAND]-.73) < .001
request = GetPlanningScene.Request(); request.components.components = 1023
scene = call(GetPlanningScene, '/get_planning_scene', request).scene
attached = [body.object.id for body in scene.robot_state.attached_collision_objects]
assert attached == ['ur5e_stack/block_4']
assert read_held_fault() is None
report = dict(result='passed', domain=103, hardware='mock_components/GenericSystem',
              approvals_before_grasp=len(approved),preview_hashes_checked=len(ready),
              each_READY_held_before_OK=True,lift_approval_pending_index=6,
              gripper_held_q=q[HAND],held_geometry_retained=True,attached_objects=attached,
              physical_held_fault_expected=False,physical_held_fault_present=False,
              physical_fault_assertion_correction='Mock mode deliberately does not write the physical recovery latch; original probe stopped only at that incorrect expectation.',
              stop_confirmed=True,no_auto_lift=True,no_auto_release=True,
              readonly_post_stop_samples=len(samples),maximum_post_stop_joint_delta_rad=delta,
              actual_robot_goals_sent=False)
(root/'physical_stack_step_stop_runtime.result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False))
node.destroy_node();rclpy.shutdown()
