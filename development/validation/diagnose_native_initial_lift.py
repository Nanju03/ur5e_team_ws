"""Read-only IK/collision diagnosis; never creates an action client or publisher."""
import json
import os
from pathlib import Path

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from ur5e_block_bringup import air_scene
from ur5e_block_bringup.air_trajectory import ARM_JOINTS

assert os.environ.get('ROS_DOMAIN_ID') == '0'
base = Path('/home/ksn/ur5e_ws/validation')
snapshot = json.loads((base/'native_initial_lift_state.json').read_text())
kin = air_scene.RobotKinematics((base/'native_initial_lift_model.urdf').read_text())
joints = snapshot['joints']


def fk(link, positions):
    chain = []
    while link != 'world':
        parent, record = kin._parents[link]
        chain.append(record)
        link = parent
    result = np.eye(4)
    for name, kind, origin, axis in reversed(chain):
        result = result @ origin
        if kind != 'fixed':
            result = result @ air_scene._revolute_motion(axis, positions[name])
    return result


tip = fk('tool0', joints)
quaternion = Rotation.from_matrix(tip[:3, :3]).as_quat()
rclpy.init()
node = rclpy.create_node('readonly_native_lift_ik_diagnostic')
ik_client = node.create_client(GetPositionIK, '/compute_ik')
valid_client = node.create_client(GetStateValidity, '/check_state_validity')
assert ik_client.wait_for_service(timeout_sec=3)
assert valid_client.wait_for_service(timeout_sec=3)


def call(client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    assert future.done()
    return future.result()


results = []
for timeout in (.005, .12):
    for avoid in (False, True):
        seed = dict(joints)
        samples = []
        for dz in (.0005, .001, .0015, .002, .0025, .003, .0035, .004, .0045, .005):
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'ur5e_arm'
            request.ik_request.ik_link_name = 'tool0'
            request.ik_request.robot_state.is_diff = True
            request.ik_request.robot_state.joint_state.name = list(seed)
            request.ik_request.robot_state.joint_state.position = list(seed.values())
            request.ik_request.pose_stamped.header.frame_id = 'world'
            pose = request.ik_request.pose_stamped.pose
            pose.position.x, pose.position.y, pose.position.z = [float(value) for value in tip[:3, 3]]
            pose.position.z += dz
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [float(value) for value in quaternion]
            request.ik_request.avoid_collisions = avoid
            request.ik_request.timeout.nanosec = round(timeout*1e9)
            reply = call(ik_client, request)
            result = dict(dz=dz, error_code=reply.error_code.val)
            if reply.error_code.val == 1:
                solution = dict(zip(reply.solution.joint_state.name, reply.solution.joint_state.position))
                validity = GetStateValidity.Request()
                validity.robot_state = reply.solution
                checked = call(valid_client, validity)
                result.update(state_valid=checked.valid,
                              contacts=[{'a':c.contact_body_1,'b':c.contact_body_2,'depth':c.depth} for c in checked.contacts],
                              max_arm_step=max(abs(solution[name]-seed[name]) for name in ARM_JOINTS))
                seed.update({name: solution[name] for name in seed})
            samples.append(result)
        results.append(dict(timeout=timeout, avoid_collisions=avoid, samples=samples))
(base/'native_initial_lift_ik.json').write_text(json.dumps(results, indent=2)+'\n')
for item in results:
    print(json.dumps(dict(timeout=item['timeout'], avoid_collisions=item['avoid_collisions'],
                         result_codes=[sample['error_code'] for sample in item['samples']],
                         valid=[sample.get('state_valid') for sample in item['samples']],
                         contacts=[sample.get('contacts') for sample in item['samples']]), indent=2))
node.destroy_node()
rclpy.shutdown()
