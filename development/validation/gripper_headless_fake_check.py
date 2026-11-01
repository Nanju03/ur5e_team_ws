"""End-to-end launch/services check on domain 96 with only a fake AG95."""
import ast
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import GripperCommand
from control_msgs.msg import DynamicJointState, InterfaceValue
from controller_manager_msgs.msg import ControllerState, HardwareComponentState, HardwareInterface
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from lifecycle_msgs.msg import State
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger, SetBool
from ur5e_block_bringup.gripper_calibration import ARM, HAND

assert os.environ.get('ROS_DOMAIN_ID') == '96'
BASE = Path('/home/ksn/ur5e_ws/validation')
GRIPPER_ONLY = True
# Reuse the independent fake motor/protocol from the prior integration probe.
tree = ast.parse((BASE/'gripper_calibration_fake_check.py').read_text())
fake_class = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FakeAG95')
exec(compile(ast.Module(body=[fake_class],type_ignores=[]),'<fake AG95>', 'exec'))

rclpy.init(args=[])
fake = FakeAG95()
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(fake)
thread = threading.Thread(target=executor.spin,daemon=True)
thread.start()
client = rclpy.create_node('headless_calibration_probe')
services = {}
states = []
subscription = client.create_subscription(String,'/gripper_calibration/state',lambda m:states.append(json.loads(m.data)),10)
process = stream = None


def call(name, data=None):
    kind = SetBool if name=='ready' else Trigger
    if name not in services:
        services[name]=client.create_client(kind,'/gripper_calibration/'+name)
    c=services[name]
    assert c.wait_for_service(timeout_sec=8),name
    req=kind.Request()
    if name=='ready': req.data=data
    future=c.call_async(req)
    rclpy.spin_until_future_complete(client,future,timeout_sec=4)
    assert future.done(),name
    return future.result()


def status():
    reply=call('status')
    assert reply.success
    return json.loads(reply.message)


def wait(condition, timeout=25):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        s=status()
        if condition(s): return s
        time.sleep(.04)
    raise AssertionError(status())


def launch(name, **overrides):
    global process,stream
    args=dict(start_driver='false',allow_mock_hardware='true',use_mock_hardware='true',
              open_q='0.60',max_q='0.76',step_q='0.02',settle_s='0.3',execution_mode='continuous')
    args.update(overrides)
    stream=(BASE/name).open('w')
    process=subprocess.Popen(['ros2','launch','ur5e_block_bringup','gripper_calibration.launch.py']+
        [f'{k}:={v}' for k,v in args.items()],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)


def close_launch():
    global process,stream
    if process and process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        process.wait(timeout=12)
    if stream: stream.close()
    process=stream=None
    services.clear()
    # Drop graph endpoints of the previous service node before a new run.
    end=time.monotonic()+.5
    while time.monotonic()<end:rclpy.spin_once(client,timeout_sec=.02)


try:
    launch('gripper_headless_monitor.log')
    wait(lambda s:s.get('ready'))
    assert fake.goals==0 and not status()['workspace_clear']
    denied=call('baseline')
    assert not denied.success and fake.goals==0
    assert call('ready',True).success
    assert fake.goals==0
    assert call('baseline').success
    wait(lambda s:s.get('baseline_passed') and not s.get('busy'))
    for i in range(3):
        fake.loaded=True
        wait(lambda s:s.get('ready'))
        assert call('grasp').success
        wait(lambda s:s.get('can_record'))
        assert call('success').success
        wait(lambda s:s.get('ready'))
        assert call('open').success
        wait(lambda s:not s.get('busy') and s.get('phase')=='IDLE')
        print('headless verified grasp/release',i+1,flush=True)
    candidate=status()['recommendation']
    assert candidate and candidate['trials']==3
    assert not call('apply').success  # Fake evidence cannot become a real profile.
    fake.loaded=False
    fake.speed_step=.0005
    wait(lambda s:s.get('ready'))
    assert call('baseline').success
    wait(lambda s:s.get('busy') and abs(fake.target-fake.q)>.005)
    assert call('ready',False).success
    wait(lambda s:s.get('stop_confirmed'))
    assert not call('baseline').success
    assert fake.cancels>=1 and states
    close_launch()
    print('headless terminal services and graceful shutdown passed',flush=True)

    fake.speed_step=.015
    count=fake.goals
    launch('gripper_headless_startup.log',startup_operation='baseline',workspace_clear='true')
    wait(lambda s:s.get('baseline_passed') and not s.get('busy'))
    assert status()['startup_pending'] is None
    finished_count=fake.goals
    time.sleep(.6)
    assert fake.goals==finished_count and fake.goals>count
    close_launch()

    # The owner must refuse to initialize a second driver when this fake
    # gripper manager already exists, even in an explicitly mocked launch.
    count=fake.goals
    launch('gripper_headless_existing_driver.log',start_driver='true')
    wait(lambda s:bool(s.get('driver_error')))
    assert 'start_driver:=false' in status()['driver_error'] and fake.goals==count
    close_launch()

    # Preserve and check the independently added step-preview interface too.
    launch('gripper_headless_step.log',execution_mode='step',workspace_clear='true')
    wait(lambda s:s.get('ready'))
    count=fake.goals
    assert call('baseline').success
    first=wait(lambda s:bool(s.get('pending_approval')) and bool(s.get('approval_service')))
    assert fake.goals==count
    path=first['approval_service']
    approval=client.create_client(Trigger,path)
    assert approval.wait_for_service(timeout_sec=3)
    f=approval.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(client,f,timeout_sec=3)
    assert f.done() and f.result().success
    second=wait(lambda s:bool(s.get('pending_approval')) and s.get('approval_service')!=path)
    assert fake.goals==count+1
    assert call('stop').success
    wait(lambda s:s.get('stop_confirmed'))
    assert fake.goals==count+1 and not status()['pending_approval']
    close_launch()
    result=dict(domain=96,fake_hardware=True,gui_used=False,real_commands_sent=False,
        monitor_sends_no_goals=True,permission_service_required=True,grasp_release_trials=3,
        recommendation=candidate,mock_profile_apply_rejected=True,
        ready_revocation_cancels=True,shutdown_completed=True,startup_once=True,
        existing_driver_not_reinitialized=True,state_topic_received=True,
        step_preview_sends_no_goal_until_approved=True,one_approval_one_goal=True)
    (BASE/'gripper_headless_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
finally:
    fake.freeze=False
    if process and process.poll() is None:
        try: call('stop')
        except Exception: pass
        close_launch()
    executor.shutdown()
    fake.destroy_node()
    client.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=2)
