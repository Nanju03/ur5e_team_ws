"""Stop a reviewed mock grasp before its first lift; never address domain 0."""
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
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from ur5e_block_bringup.air_task import AirTask, ARM, HAND
from ur5e_block_bringup.physical_held import read_held_fault

assert os.environ.get('ROS_DOMAIN_ID') == '103'
ROOT = Path('/home/ksn/ur5e_ws/validation')
LOG = ROOT/'physical_stack_step_stop_runtime.log'
rclpy.init()
node = rclpy.create_node('physical_step_stop_mock_probe')
q, samples, previews = {}, [], []
child = None


def feedback(message):
    q.update(zip(message.name, message.position))
    if all(name in q for name in ARM+[HAND]):
        samples.append((time.monotonic(), {name:q[name] for name in ARM+[HAND]}))


subscription = node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
preview_sub = node.create_subscription(DisplayTrajectory, '/display_planned_path', previews.append,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))


def text():
    return LOG.read_text() if LOG.exists() else ''


def records(marker):
    return [json.loads(line.split(marker+' ', 1)[1]) for line in text().splitlines() if marker+' ' in line]


def until(predicate, seconds=60.):
    deadline = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(text()[-4000:])
        rclpy.spin_once(node, timeout_sec=.02)


def spin(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)


def call(kind, service, request):
    client = node.create_client(kind, service)
    assert client.wait_for_service(timeout_sec=8.), service
    future = client.call_async(request)
    until(future.done, 10.)
    reply = future.result()
    node.destroy_client(client)
    return reply


hardware = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                ListHardwareComponents.Request()).component
assert hardware and all(item.class_type == 'mock_components/GenericSystem' and item.state.id == 3 for item in hardware)
claimed = {i.name for item in hardware for i in item.command_interfaces if i.is_claimed}
assert {name+'/position' for name in ARM+[HAND]} <= claimed
assert read_held_fault() is None
until(lambda: all(name in q for name in ARM+[HAND]))
initial = dict(q)
command = ['ros2','launch','ur5e_block_bringup','block_task.launch.py','stack_mode:=physical_run',
           'selection_file:=/home/ksn/ur5e_ws/validation/physical_stack_one_block.yaml',
           'trajectory_file:=/home/ksn/ur5e_ws/validation/physical_stack_step.recipe.json',
           'recipe_manifest_file:=/home/ksn/ur5e_ws/validation/physical_stack_step.recipe.json',
           'allow_mock_hardware:=true','workspace_clear:=true','execution_mode:=step']
with LOG.open('w') as stream:
    child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
                             env={**os.environ,'PYTHONUNBUFFERED':'1'})
approved, verified = [], 0
try:
    while True:
        until(lambda: len(records('STACK_STEP_READY')) > len(approved) or child.poll() is not None, 90.)
        assert child.poll() is None, text()[-5000:]
        ready = records('STACK_STEP_READY')[len(approved)]
        assert ready['index'] == len(approved)+1
        until(lambda: any(AirTask.preview_digest(display) == ready['preview_sha256'] for display in previews), 8.)
        verified += 1
        before = dict(q)
        spin(.5)
        assert max(abs(q[name]-before[name]) for name in ARM+[HAND]) < 1e-9
        if ready['label'].startswith('Lift block '):
            lift = ready
            break
        assert call(Trigger, ready['approve_service'], Trigger.Request()).success
        approved.append(ready['request_id'])
    assert abs(q[HAND]-.73) < .001
    held = dict(q)
    pause_start = time.monotonic()
    spin(1.)
    assert max(abs(q[name]-held[name]) for name in ARM+[HAND]) < 1e-9
    executing = records('STACK_STEP_EXECUTING')
    assert not any(item['label'].startswith('Lift block ') or item['label'].startswith('Release block ') for item in executing)
    request = GetPlanningScene.Request(); request.components.components = 1023
    held_scene = call(GetPlanningScene, '/get_planning_scene', request).scene
    assert [body.object.id for body in held_scene.robot_state.attached_collision_objects] == ['ur5e_stack/block_4']
    stop_offset = len(text())
    assert call(Trigger, '/ur5e_a3_stack/stop', Trigger.Request()).success
    until(lambda: child.poll() is not None, 30.)
    assert 'STACK_STOPPED_CONFIRMED' in text()[stop_offset:]
    spin(1.)
    assert max(abs(q[name]-held[name]) for name in ARM+[HAND]) < .001
    fault = read_held_fault()
    # Mock runs deliberately do not create the physical recovery latch.
    assert fault is None
    final_scene = call(GetPlanningScene, '/get_planning_scene', request).scene
    assert [body.object.id for body in final_scene.robot_state.attached_collision_objects] == ['ur5e_stack/block_4']
    assert not any(item['label'].startswith('Lift block ') or item['label'].startswith('Release block ')
                   for item in records('STACK_STEP_EXECUTING'))
    report = dict(result='passed',domain=103,hardware='mock_components/GenericSystem',
                  approvals_before_grasp=len(approved),preview_hashes_checked=verified,
                  each_READY_held_before_OK=True,lift_approval_pending_index=lift['index'],
                  gripper_held_q=q[HAND],held_geometry_retained=True,
                  physical_held_fault_expected=False,physical_held_fault_present=False,
                  stop_confirmed=True,no_auto_lift=True,no_auto_release=True,
                  fault=fault,feedback_samples=len(samples),actual_robot_goals_sent=False)
    (ROOT/'physical_stack_step_stop_runtime.result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))
except Exception:
    if child.poll() is None:
        try:
            call(Trigger,'/ur5e_a3_stack/stop',Trigger.Request())
        except Exception:
            os.killpg(child.pid,signal.SIGINT)
        try:
            child.wait(timeout=30.)
        except subprocess.TimeoutExpired:
            pass
    raise
finally:
    node.destroy_node();rclpy.shutdown()
