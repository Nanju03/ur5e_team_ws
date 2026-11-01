"""Corrected plate integration probe, isolated GenericSystem domain 98 only."""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from ur5e_block_bringup.air_scene import RobotKinematics

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['startup', 'scene', 'step', 'continuous'])
parser.add_argument('log')
args = parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID') == '98'
rclpy.init()
node = rclpy.create_node('corrected_plate_mock_probe')
arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
hand = 'left_outer_knuckle_joint'
q, samples, previews = {}, [], []
log = Path(args.log)


def feedback(message):
    q.update(zip(message.name, message.position))
    if all(j in q for j in arm+[hand]):
        samples.append(dict(q))


sub = node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
preview_sub = node.create_subscription(DisplayTrajectory, '/display_planned_path', previews.append,
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))


def text():
    return log.read_text() if log.exists() else ''


def records(marker):
    return [json.loads(line.split(marker+' ', 1)[1]) for line in text().splitlines()
            if marker+' ' in line]


def until(predicate, seconds=50.):
    end = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() > end:
            raise TimeoutError(text()[-2500:])
        rclpy.spin_once(node, timeout_sec=.02)


def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)


def call(kind, name, request):
    client = node.create_client(kind, name)
    assert client.wait_for_service(timeout_sec=8.), name
    future = client.call_async(request)
    until(future.done, 8.)
    response = future.result()
    node.destroy_client(client)
    return response


components = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                  ListHardwareComponents.Request()).component
assert components and all(c.class_type == 'mock_components/GenericSystem' and c.state.id == 3
                          for c in components)
covered = {interface.name for c in components for interface in c.command_interfaces if interface.is_claimed}
assert {j+'/position' for j in arm+[hand]} <= covered
until(lambda: all(j in q for j in arm+[hand]))
result = {'domain': 98, 'hardware': [c.class_type for c in components]}
if args.mode in ('startup', 'scene'):
    assert abs(q[hand]-.4) < 1e-9
    assert max(abs(q[j]-value) for j, value in zip(arm, [0., -1.5, 1.5, -1.5, -1.5, 0.])) < 1e-9
    result.update(default_gripper_position=q[hand], initial_arm={j:q[j] for j in arm})
    if args.mode == 'scene':
        import re
        until(lambda: 'STACK_STATUS phase=PLAN_READY' in text())
        layers = [tuple(map(int, match)) for match in
                  re.findall(r'STACK_PLANNING source=(\d+) target=(\d+) layer=(\d+)/3', text())]
        assert len(layers) == 3 and {v[0] for v in layers} == {4, 13, 21}
        assert [v[2] for v in layers] == [1, 2, 3] and all(v[1] == 11 for v in layers)
        request = GetPlanningScene.Request()
        request.components.components = 1023
        scene = call(GetPlanningScene, '/get_planning_scene', request).scene
        objects = {obj.id: obj for obj in scene.world.collision_objects}
        assert 'ur5e_stack/block_11' not in objects
        assert set(objects) == {'ur5e_stack/plate', *(f'ur5e_stack/block_{i}' for i in (4, 13, 21))}
        for i in (4, 13, 21):
            obj = objects[f'ur5e_stack/block_{i}']
            assert abs(obj.primitives[0].dimensions[2]-.025) < 1e-9
            assert abs(obj.pose.position.z+obj.primitive_poses[0].position.z-.8175) < 1e-9
        result.update(target_cell=11, sources=[v[0] for v in layers], layers=[v[2] for v in layers],
                      target_excluded_from_sources=True, target_pocket_initially_empty=True,
                      source_support_z_m=.805, default_open_position=.4,
                      preflight='STACK_PREFLIGHT_PASSED' in text())
else:
    until(lambda: records('STACK_STEP_READY') and previews)
    first = records('STACK_STEP_READY')[0]
    initial = dict(q)
    spin(1.)
    assert max(abs(q[j]-initial[j]) for j in arm+[hand]) < 1e-9
    assert not records('STACK_STEP_EXECUTING')
    assert first['duration'] > 0
    if args.mode == 'step':
        assert 'gripper position 0.7' in first['label']
        assert previews[-1].trajectory[-1].joint_trajectory.joint_names == [hand]
        assert len(previews[-1].trajectory[-1].joint_trajectory.points[-1].positions) == 1
        assert abs(previews[-1].trajectory[-1].joint_trajectory.points[-1].positions[0]-.7) < 1e-9
    assert call(Trigger, first['approve_service'], Trigger.Request()).success
    if args.mode == 'step':
        until(lambda: len(records('STACK_STEP_READY')) == 2)
        second = records('STACK_STEP_READY')[1]
        assert second['index'] == 2 and first['index'] == 1 and second['total'] == first['total']
        assert abs(q[hand]-.7) < .001
        assert max(abs(q[j]-initial[j]) for j in arm) < 1e-9
        held = dict(q)
        spin(1.)
        assert max(abs(q[j]-held[j]) for j in arm+[hand]) < 1e-9
        assert first['approve_service'] not in dict(node.get_service_names_and_types())
        assert call(Trigger, '/ur5e_a3_stack/stop', Trigger.Request()).success
        until(lambda: 'STACK_ERROR' in text())
        spin(.5)
        assert max(abs(q[j]-held[j]) for j in arm+[hand]) < 1e-9
        assert [v['index'] for v in records('STACK_STEP_EXECUTING')] == [1]
        result.update(held_before_OK=True, first_OK_moves_only_gripper=True,
                      gripper_position=q[hand], arm_unchanged=True,
                      next_segment_waits_for_OK=True, old_approval_removed=True,
                      stop_while_waiting_confirmed=True, segment_count=first['total'])
    else:
        assert first['index'] == first['total'] == 1
        until(lambda: 'STACK_STATUS phase=DONE' in text(), 240.)
        spin(.5)
        assert len(records('STACK_STEP_READY')) == 1
        assert len(records('STACK_STEP_EXECUTING')) == 1
        request = GetParameters.Request()
        request.names = ['robot_description']
        description = call(GetParameters, '/robot_state_publisher/get_parameters', request).values[0].string_value
        kinematics = RobotKinematics(description)
        tcp = kinematics.tcp({j:q[j] for j in arm})[:3, 3].tolist()
        minimum = kinematics.gripper_min_z({j:q[j] for j in arm}, q[hand])
        minimum_sampled = min(kinematics.gripper_min_z({j:s[j] for j in arm}, s[hand])
                              for s in samples[::5])
        assert abs(q[hand]-.7) < .001
        assert abs(tcp[0]-.3109) < .001 and abs(tcp[1]+.15) < .001
        assert abs(minimum-.820) < .0005 and minimum_sampled >= .8199
        result.update(held_before_OK=True, one_OK_full_path=True, completed='DONE',
                      final_pocket_cell=21, final_tcp_world_m=tcp,
                      final_gripper_position=q[hand], final_gripper_min_z_m=minimum,
                      sampled_gripper_min_z_m=minimum_sampled,
                      feedback_samples=len(samples))
result['result'] = 'passed'
log.with_suffix('.result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
