"""Read current state/scene and collision-check a failed startup; no writes/goals."""
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from ur5e_block_bringup.physical_stack import JOINTS, scene_fingerprint


def main():
    rclpy.init()
    node = rclpy.create_node('diagnose_initial_hand_collision_readonly')
    values, received, clients = {}, {}, {}
    def feedback(msg):
        now = time.monotonic()
        for name, q in zip(msg.name, msg.position):
            if name in JOINTS:
                values[name], received[name] = float(q), now
    node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
    def call(kind, endpoint, req):
        if endpoint not in clients:
            clients[endpoint] = node.create_client(kind, endpoint)
        client = clients[endpoint]
        if not client.wait_for_service(timeout_sec=10.):
            raise TimeoutError(endpoint+' discovery')
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        try:
            if not future.done():
                raise TimeoutError(endpoint+' response')
            return future.result()
        finally:
            client.remove_pending_request(future)
    try:
        until = time.monotonic()+8.
        while (not all(name in received for name in JOINTS)
               or any(time.monotonic()-received.get(name, -100.) > 1. for name in JOINTS)):
            if time.monotonic() > until:
                raise TimeoutError('fresh seven joints')
            rclpy.spin_once(node, timeout_sec=.05)
        initial = [values[name] for name in JOINTS]
        scene_req = GetPlanningScene.Request()
        scene_req.components.components = 1023
        scene = call(GetPlanningScene, '/get_planning_scene', scene_req).scene
        def sweep(arm, positions):
            result = []
            for q in positions:
                req = GetStateValidity.Request()
                req.robot_state.is_diff = True
                req.robot_state.joint_state.name = JOINTS
                req.robot_state.joint_state.position = arm+[q]
                answer = call(GetStateValidity, '/check_state_validity', req)
                result.append(dict(hand=q, valid=answer.valid, contacts=[dict(
                    a=c.contact_body_1, b=c.contact_body_2, depth=c.depth,
                    xyz=[c.position.x, c.position.y, c.position.z]) for c in answer.contacts]))
            return result
        logged_arm = [-.120678727, -1.60306182, 2.40886504, -2.37550416, -1.56599456, -1.68091995]
        hand_samples = [i/100. for i in range(41)]
        logged_sweep = sweep(logged_arm, hand_samples)
        measured_sweep = sweep(initial[:6], [initial[6], .4])
        final_scene = call(GetPlanningScene, '/get_planning_scene', scene_req).scene
        result = dict(actual_goals_sent=False, scene_writes_sent=False,
                      measured_joints=dict(zip(JOINTS, initial)),
                      failed_plan='20261101_163246_physical_plan',
                      logged_start_sweep=logged_sweep, measured_sweep=measured_sweep,
                      scene_unchanged=scene_fingerprint(scene)==scene_fingerprint(final_scene),
                      objects=[dict(id=o.id, primitives=len(o.primitives), meshes=len(o.meshes))
                               for o in scene.world.collision_objects],
                      attached=[o.object.id for o in scene.robot_state.attached_collision_objects])
        target = Path('/home/ksn/ur5e_ws/validation/initial_hand_collision_readonly.json')
        target.write_text(json.dumps(result, indent=2)+'\n')
        invalid = [r for r in logged_sweep if not r['valid']]
        print(json.dumps(dict(report=str(target), measured_joints=result['measured_joints'],
                              logged_samples=len(logged_sweep), invalid_samples=len(invalid),
                              first_invalid=invalid[0] if invalid else None,
                              measured_sweep=measured_sweep, scene_unchanged=result['scene_unchanged']), indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
