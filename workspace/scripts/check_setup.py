"""Check this workspace on domain 42, with GenericSystem hardware only."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='Also check RViz/Tk windows using Xvfb')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Source env.bash first; this check requires localhost domain 42')
    root = Path(__file__).resolve().parents[1]
    report = {'timestamp': datetime.now().astimezone().isoformat(), 'domain': 42, 'passed': False}
    process = None
    rclpy.init(args=[])
    node = rclpy.create_node('ur5e_team_setup_check')
    joints = {}
    arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    hand = 'left_outer_knuckle_joint'
    subscription = node.create_subscription(
        JointState, '/joint_states', lambda message: joints.update(zip(message.name, message.position)),
        qos_profile_sensor_data)

    def wait(future, timeout=15):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS service or action timed out')
        return future.result()

    def call(kind, name, request, timeout=30):
        client = node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f'Service unavailable: {name}')
            return wait(client.call_async(request))
        finally:
            node.destroy_client(client)

    try:
        until = time.monotonic() + 2
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        existing = set(node.get_node_names()) - {node.get_name()}
        if existing:
            raise RuntimeError(f'Domain 42 is already in use; close its simulator first: {sorted(existing)}')
        with (root/'log/setup_simulation.log').open('w') as stream:
            command = [str(root/'start_block_sim.sh')]
            if not args.gui:
                command += ['use_gui:=false', 'use_rviz:=false']
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            components = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                              ListHardwareComponents.Request(), timeout=45).component
            if not components or any(c.class_type != 'mock_components/GenericSystem' or c.state.id != 3
                                     for c in components):
                raise RuntimeError('Expected active GenericSystem hardware only')
            report['hardware'] = [c.class_type for c in components]
            # Controller spawners can finish shortly after the hardware becomes active.
            until = time.monotonic() + 30
            expected = {'joint_state_broadcaster', 'ur5e_arm_controller', 'dh_ag95_gripper_controller'}
            while True:
                controllers = call(ListControllers, '/controller_manager/list_controllers',
                                   ListControllers.Request()).controller
                active = {c.name for c in controllers if c.state == 'active'}
                if expected <= active:
                    break
                if time.monotonic() > until:
                    raise RuntimeError(f'Controllers not active: {expected - active}')
                rclpy.spin_once(node, timeout_sec=.2)
            report['active_controllers'] = sorted(active)
            request = GetParameters.Request(names=['robot_description'])
            description = call(GetParameters, '/robot_state_publisher/get_parameters', request).values[0].string_value
            model = ET.fromstring(description)
            plugins = [item.findtext('hardware/plugin') for item in model.findall('ros2_control')]
            if not plugins or any(plugin != 'mock_components/GenericSystem' for plugin in plugins):
                raise RuntimeError(f'Unexpected robot-description hardware: {plugins}')
            until = time.monotonic() + 10
            while not all(joint in joints and math.isfinite(joints[joint]) for joint in arm + [hand]):
                if time.monotonic() > until:
                    raise TimeoutError('Missing arm or gripper joint feedback')
                rclpy.spin_once(node, timeout_sec=.1)
            report['joint_feedback'] = {joint: joints[joint] for joint in arm + [hand]}
            scene_request = GetPlanningScene.Request()
            scene_request.components.components = 1023
            scene = call(GetPlanningScene, '/get_planning_scene', scene_request).scene
            report['planning_scene_model'] = scene.robot_model_name
            plan_request = GetMotionPlan.Request()
            request = plan_request.motion_plan_request
            request.group_name = 'ur5e_arm'
            request.allowed_planning_time = 10.
            request.num_planning_attempts = 1
            request.max_velocity_scaling_factor = .1
            request.max_acceleration_scaling_factor = .1
            request.start_state.is_diff = True
            target = Constraints()
            for joint in arm:
                target.joint_constraints.append(JointConstraint(
                    joint_name=joint, position=joints[joint] + (.02 if joint == arm[0] else 0.),
                    tolerance_above=.001, tolerance_below=.001, weight=1.))
            request.goal_constraints = [target]
            plan = call(GetMotionPlan, '/plan_kinematic_path', plan_request).motion_plan_response
            if plan.error_code.val != 1 or len(plan.trajectory.joint_trajectory.points) < 2:
                raise RuntimeError(f'MoveIt planning failed: {plan.error_code.val}')
            report['moveit_plan_points'] = len(plan.trajectory.joint_trajectory.points)
            subprocess.run(['/usr/bin/python3', str(root/'scripts/probe_standalone_sim.py')],
                           check=True, timeout=40)
            report['arm_motion'] = json.loads((root/'log/sim_entry_motion_result.json').read_text())
            client = ActionClient(node, GripperCommand, '/dh_ag95_gripper_controller/gripper_cmd')
            try:
                if not client.wait_for_server(timeout_sec=10):
                    raise TimeoutError('Virtual gripper action unavailable')
                initial = joints[hand]
                for position in (initial + .02, initial):
                    goal = GripperCommand.Goal()
                    goal.command.position = position
                    handle = wait(client.send_goal_async(goal))
                    if not handle.accepted:
                        raise RuntimeError('Virtual gripper goal rejected')
                    result = wait(handle.get_result_async())
                    if result.status != 4 or not result.result.reached_goal:
                        raise RuntimeError('Virtual gripper goal did not finish')
                report['gripper_motion'] = 'passed'
            finally:
                client.destroy()
            if args.gui:
                windows = ''
                until = time.monotonic() + 20
                while time.monotonic() < until:
                    windows = subprocess.run(['xwininfo', '-root', '-tree'], check=True,
                                             text=True, capture_output=True).stdout
                    if 'RViz' in windows and 'UR5e' in windows:
                        break
                    rclpy.spin_once(node, timeout_sec=.2)
                (root/'log/setup_windows.txt').write_text(windows)
                if 'RViz' not in windows or 'UR5e' not in windows:
                    raise RuntimeError('RViz or task GUI window was not found')
                report['gui_windows'] = 'RViz + UR5e task GUI'
                for label, pattern in [('gui', r'^\s*(0x[0-9a-f]+) "UR5e'),
                                       ('rviz', r'^\s*(0x[0-9a-f]+) ".* - RViz"')]:
                    match = re.search(pattern, windows, re.MULTILINE)
                    if match:
                        subprocess.run(['import', '-window', match.group(1),
                                        str(root/f'log/setup_{label}.png')], check=True, timeout=10)
            startup_log = (root/'log/setup_simulation.log').read_text()
            if 'process has died' in startup_log or 'Traceback (most recent call last)' in startup_log:
                raise RuntimeError('A simulator process failed; see log/setup_simulation.log')
            if process.poll() is not None:
                raise RuntimeError(f'Simulation exited unexpectedly: {process.returncode}')
            report['runtime_passed'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        if process is not None:
            try:
                # Let ros2 launch signal each child once, avoiding duplicate SIGINTs.
                process.send_signal(signal.SIGINT)
                process.wait(timeout=15)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            report['launch_exit_code'] = process.returncode
            log = (root/'log/setup_simulation.log').read_text()
            report['shutdown_process_errors'] = [line for line in log.splitlines() if 'process has died' in line]
        node.destroy_node()
        rclpy.shutdown()
        report['shutdown_clean'] = process is not None and not report.get('shutdown_process_errors') and process.returncode == 0
        report['passed'] = report.get('runtime_passed', False) and report['shutdown_clean']
        (root/'log/setup_check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['passed']:
        raise RuntimeError('Simulation runtime or shutdown check failed; see log/setup_check.json')


if __name__ == '__main__':
    main()
