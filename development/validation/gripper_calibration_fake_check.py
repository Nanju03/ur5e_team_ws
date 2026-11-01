"""Explicit integration probe: fake sensor and gripper action, no devices.

Run only in ROS_DOMAIN_ID=93. The calibration backend requires GenericSystem
in this test and is forbidden from applying its fake profile to real hardware.
"""
import json
import os
import sys
from pathlib import Path
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
from std_msgs.msg import Float64MultiArray
from ur5e_block_bringup.gripper_calibration import CalibrationBackend, ARM, HAND
from ur5e_block_bringup.model import load_moveit_settings

GRIPPER_ONLY = '--gripper-only' in sys.argv
DOMAIN = 95 if GRIPPER_ONLY else 93
assert os.environ.get('ROS_DOMAIN_ID') == str(DOMAIN)
BASE = Path('/home/ksn/ur5e_ws/validation')
CAL = '/home/ksn/ur5e_ws/robot_calibration/ur5e_20261031.yaml'
PHYSICAL = '/home/ksn/ur5e_ws/physical_setup.yaml'


class FakeAG95(Node):
    def __init__(self):
        super().__init__('fake_ag95_only' if GRIPPER_ONLY else 'robot_state_publisher')
        if not GRIPPER_ONLY:
            self.declare_parameter('robot_description',load_moveit_settings(CAL,PHYSICAL)['robot_description'])
        self.q = self.target = .5
        self.force, self.sequence = 20, 0
        self.loaded = self.freeze = self.arm_moving = False
        self.publish_arm = not GRIPPER_ONLY
        self.speed_step = .015
        self.grip, self.goals, self.cancels = 1, 0, 0
        self.group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(DynamicJointState,'/gripper/dynamic_joint_states',10)
        self.armpub = self.create_publisher(JointState,'/joint_states',10)
        self.create_subscription(Float64MultiArray,'/gripper/force_forward_controller/commands',self.set_force,10)
        self.create_service(ListControllers,'/gripper/controller_manager/list_controllers',self.controllers)
        self.create_service(ListHardwareComponents,'/gripper/controller_manager/list_hardware_components',self.hardware)
        self.server = ActionServer(self,GripperCommand,'/gripper/dh_ag95_gripper_controller/gripper_cmd',
                                  execute_callback=self.execute,cancel_callback=self.cancel,callback_group=self.group)
        self.create_timer(.02,self.sensor)

    def controllers(self, _, response):
        response.controller = [ControllerState(name=n,state='active') for n in
                               ('dh_ag95_gripper_controller','force_forward_controller')]
        return response

    def hardware(self, _, response):
        response.component = [HardwareComponentState(name='fake',class_type='mock_components/GenericSystem',
            state=State(id=3,label='active'),command_interfaces=[
                HardwareInterface(name=HAND+'/position',is_available=True,is_claimed=True)])]
        return response

    def set_force(self, msg):
        if len(msg.data) == 1:
            self.force = round(20+80*msg.data[0])

    def sensor(self):
        desired = min(self.target,.71796) if self.loaded else self.target
        self.q += max(-self.speed_step,min(self.speed_step,desired-self.q))
        self.grip = 2 if self.loaded and self.target > .71796 and abs(self.q-.71796)<.001 else (
                    1 if abs(self.q-self.target)<.001 else 0)
        if self.publish_arm:
            self.armpub.publish(JointState(name=ARM,position=[0.]*6,
                                          velocity=[.02 if self.arm_moving else 0.]*6))
        if self.freeze:
            return
        self.sequence += 1
        raw = round(1000*(1-self.q/.93))
        q = .93*(1000-raw)/1000
        self.pub.publish(DynamicJointState(joint_names=[HAND],interface_values=[InterfaceValue(
            interface_names=['position','velocity','raw_position','grip_state','force_percent','communication_age','sample_sequence'],
            values=[q,0.,float(raw),float(self.grip),float(self.force),0.,float(self.sequence)])]))

    def cancel(self, _):
        self.cancels += 1
        self.target = self.q
        return CancelResponse.ACCEPT

    def execute(self, goal):
        self.goals += 1
        self.target = goal.request.command.position
        until = time.monotonic()+4
        while time.monotonic()<until:
            if goal.is_cancel_requested:
                self.target = self.q
                goal.canceled()
                return GripperCommand.Result(position=self.q)
            if self.loaded and self.target > .71796 and self.grip == 2 and abs(self.q-.71796)<.001:
                time.sleep(.05)
                goal.abort()
                return GripperCommand.Result(position=self.q,stalled=True,reached_goal=False)
            if abs(self.q-self.target)<.001:
                goal.succeed()
                return GripperCommand.Result(position=self.q,stalled=False,reached_goal=True)
            time.sleep(.01)
        goal.abort()
        return GripperCommand.Result(position=self.q,stalled=True)


def wait(backend, condition, timeout=20):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        status = backend.poll()
        if condition(status):
            return status
        time.sleep(.02)
    raise AssertionError(backend.poll())


rclpy.init(args=[])
fake = FakeAG95()
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(fake)
thread = threading.Thread(target=executor.spin,daemon=True)
thread.start()
backend = CalibrationBackend(CAL,PHYSICAL,BASE/'fake_gripper_sessions',allow_mock=True,gripper_only=GRIPPER_ONLY)
try:
    backend.configure(open_q=.60,max_q=.76,step_q=.02,force_percent=20,settle_s=.3,width_mm=25)
    initial=wait(backend,lambda s:s['ready'])
    if GRIPPER_ONLY:
        assert initial['arm_required'] is False and initial['arm_stationary'] is None
        assert 'model' not in backend.queries and not backend.arm_values
    count = fake.goals
    try:
        backend.start('baseline',workspace_clear=False)
        raise AssertionError('Missing physical-readiness confirmation was accepted')
    except RuntimeError:
        assert fake.goals == count
    backend.start('baseline',workspace_clear=True)
    wait(backend,lambda s:not s['busy'])
    assert backend.baseline and backend.baseline['passed'],backend.poll()
    for index in range(3):
        fake.loaded = True
        wait(backend,lambda s:s['ready'])
        backend.start('grasp',workspace_clear=True)
        wait(backend,lambda s:s['can_record'])
        backend.record_result(True)
        backend.start('open',workspace_clear=True)
        wait(backend,lambda s:not s['busy'])
        assert backend.trials[-1]['released']
        print('verified simulated grasp/release',index+1,flush=True)
    result = backend.poll()['recommendation']
    assert result and result['trials']==3 and abs(result['commanded_q']-.72)<.001,result
    assert result['commanded_q'] == max(t['commanded_q'] for t in backend.trials)
    try:
        backend.apply_recommendation()
        raise AssertionError('Mock evidence was applied to real hardware profile')
    except RuntimeError:
        pass
    # A stalled sequence must cancel an accepted command and keep closure
    # pending until genuinely fresh physical samples return.
    fake.loaded = False
    fake.speed_step = .0005
    wait(backend,lambda s:s['ready'])
    backend.start('baseline',workspace_clear=True)
    wait(backend,lambda s:s['busy'] and backend.goal is not None and abs(fake.q-fake.target)>.005)
    fake.freeze = True
    wait(backend,lambda s:s['phase']=='CANCELLING')
    assert not backend.poll()['can_close']
    fake.freeze = False
    wait(backend,lambda s:s['stop_confirmed'])
    assert not backend.poll()['busy']
    assert fake.cancels >= 1
    if GRIPPER_ONLY:
        wait(backend,lambda s:s['ready'])
        backend.start('baseline',workspace_clear=True)
        wait(backend,lambda s:s['busy'] and backend.goal is not None and abs(fake.q-fake.target)>.005)
        fake.publish_arm=True
        wait(backend,lambda s:s['stop_confirmed'])
        assert backend.arm_detected and not backend.poll()['ready']
        fake.publish_arm=False
        try:
            backend.start('open',workspace_clear=True)
            raise AssertionError('Standalone mode bypassed the detected arm')
        except RuntimeError:
            pass
    summary=dict(fake_hardware=True,domain=DOMAIN,gripper_only=GRIPPER_ONLY,arm_goals=0,recorded_trials=3,
        recommendation=result,stale_sensor_stop_confirmed=True,
        no_arm_feedback_required=GRIPPER_ONLY,late_arm_presence_cancelled=GRIPPER_ONLY,
        mock_profile_application_rejected=True,csv_path=str(backend.csv_path))
    name='gripper_only_fake_result.json' if GRIPPER_ONLY else 'gripper_calibration_fake_result.json'
    (BASE/name).write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)
finally:
    fake.freeze = False
    backend.request_stop()
    wait(backend,lambda s:s['can_close'],timeout=10)
    assert backend.close()
    executor.shutdown()
    fake.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=2)
