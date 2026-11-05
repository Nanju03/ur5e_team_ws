#!/usr/bin/env python3
"""Create or inspect free Gazebo bodies; never attach/teleport carried cubes.

The SDF contact template is adapted from the preserved ur5_project_v2.
Only explicit scene initialization creates/deletes models. Run verifies them.
"""
import json
import fcntl
import math
import os
from pathlib import Path
import re
import time
import yaml

def layout_models(settings, mesh_uri):
    from ur5e_block_bringup.plate_geometry import plate_parameters, PROFILE
    if settings.get('geometry_profile') != PROFILE or settings.get('independent_stack_target', False):
        raise ValueError('Gazebo uses the current cube_plate_v1 pocket layout')
    data = dict(settings)
    data.update(plate_parameters(data))
    count = data['columns']*data['rows']
    target = data.get('target_cell', 0)
    obstacles, empty = set(data.get('obstacle_cells', [])), set(data.get('empty_cells', []))
    if (type(target) is not int or not 0 <= target <= count or
            any(type(i) is not int or not 1 <= i <= count for i in obstacles | empty) or
            obstacles & empty or target and target in obstacles | empty):
        raise ValueError('Invalid Gazebo cell roles')
    yaw = math.radians(data['plate_yaw_deg'])
    quaternion = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
    models = [dict(name='ur5e_stack_plate', role='plate', position=data['plate_origin_xyz'],
                   orientation_xyzw=quaternion, mesh_uri=mesh_uri, mesh_scale=.001)]
    for cell in range(1, count+1):
        if cell in empty:
            continue
        obstacle = cell in obstacles
        size = data['obstacle_size_xyz' if obstacle else 'block_size_xyz']
        if len(size) != 3 or any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in size):
            raise ValueError('Invalid Gazebo object dimensions')
        xy = data['cell_centers_xy'][2*(cell-1):2*cell]
        surface = data['obstacle_surface_z' if obstacle else 'source_surface_z']
        models.append(dict(name=f"ur5e_stack_{'obstacle' if obstacle else 'block'}_{cell}",
                           role='obstacle' if obstacle else 'foundation' if cell == target else 'BOARD',
                           size_xyz=size, position=[*xy, surface+size[2]/2], orientation_xyzw=quaternion))
    return models


def model_sdf(model, objects, contact_hz):
    if model["role"] == "plate":
        scale = model["mesh_scale"]
        geometry = f"<mesh><uri>{model['mesh_uri']}</uri><scale>{scale} {scale} {scale}</scale></mesh>"
        return f'''<sdf version="1.6">
  <model name="ur5e_stack_plate"><static>true</static>
    <link name="body">
      <collision name="collision"><geometry>{geometry}</geometry>
        <surface>
          <friction><ode><mu>{objects['friction_mu1']}</mu><mu2>{objects['friction_mu2']}</mu2></ode></friction>
          <bounce><restitution_coefficient>0</restitution_coefficient></bounce>
          <contact><ode><kp>{objects['contact_kp']}</kp><kd>{objects['contact_kd']}</kd>
            <max_vel>0.1</max_vel><min_depth>0.0001</min_depth>
          </ode></contact>
        </surface>
      </collision>
      <visual name="visual"><geometry>{geometry}</geometry>
        <material><ambient>0.65 0.65 0.7 1</ambient><diffuse>0.65 0.65 0.7 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>'''
    x, y, z = model["size_xyz"]
    obstacle = model["role"] == "obstacle"
    mass = objects["obstacle_mass_kg" if obstacle else "block_mass_kg"]
    mu1 = objects["friction_mu1"] if obstacle else objects.get("block_friction_mu1", objects["friction_mu1"])
    mu2 = objects["friction_mu2"] if obstacle else objects.get("block_friction_mu2", objects["friction_mu2"])
    color = "0.9 0.12 0.1 1" if obstacle else "1 0.65 0.1 1" if model["role"] == "foundation" else "0.15 0.55 0.85 1"
    # All names are generated from validated integer cells, never request XML.
    return f'''<sdf version="1.6">
  <model name="{model['name']}">
    <static>false</static><allow_auto_disable>false</allow_auto_disable>
    <link name="body">
      <gravity>true</gravity>
      <inertial><mass>{mass}</mass><inertia>
        <ixx>{mass * (y*y + z*z) / 12.}</ixx><iyy>{mass * (x*x + z*z) / 12.}</iyy><izz>{mass * (x*x + y*y) / 12.}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
      </inertia></inertial>
      <collision name="collision">
        <geometry><box><size>{x} {y} {z}</size></box></geometry>
        <surface>
          <friction><ode><mu>{mu1}</mu><mu2>{mu2}</mu2></ode></friction>
          <bounce><restitution_coefficient>0</restitution_coefficient></bounce>
          <contact><ode><kp>{objects['contact_kp']}</kp><kd>{objects['contact_kd']}</kd>
            <max_vel>0.1</max_vel><min_depth>0.0001</min_depth>
          </ode></contact>
        </surface>
      </collision>
      <visual name="visual"><geometry><box><size>{x} {y} {z}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
      <sensor name="contacts" type="contact">
        <always_on>true</always_on><update_rate>{contact_hz}</update_rate>
        <contact><collision>collision</collision></contact>
        <plugin name="{model['name']}_contacts" filename="libgazebo_ros_bumper.so">
          <ros><namespace>/ur5e_a3_stack</namespace><remapping>bumper_states:=gazebo_contacts</remapping></ros>
          <frame_name>world</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>'''


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '44' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        print('STACK_ERROR Gazebo layout requires localhost ROS domain 44', flush=True)
        return 1
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import DeleteEntity, GetEntityState, SpawnEntity
    from ament_index_python.packages import get_package_share_directory
    rclpy.init()
    node = rclpy.create_node('ur5e_gazebo_layout', automatically_declare_parameters_from_overrides=True)
    get = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    spawn = node.create_client(SpawnEntity, '/spawn_entity')
    delete = node.create_client(DeleteEntity, '/delete_entity')
    names = set()
    sub = node.create_subscription(ModelStates, '/gazebo/model_states',
        lambda msg: names.update(msg.name), qos_profile_sensor_data)

    def call(client, request):
        if not client.wait_for_service(timeout_sec=15.):
            raise RuntimeError('Gazebo service unavailable: '+client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.)
        if not future.done() or future.result() is None:
            raise RuntimeError('Gazebo service did not acknowledge the operation: '+client.srv_name)
        return future.result()

    code = 0
    lock = None
    try:
        lock = os.open('/tmp/ur5e_stack_44.lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('A stacking task is running; stop it before changing the Gazebo scene')
        operation = node.get_parameter('operation').value
        if operation not in ('configure', 'verify', 'reset'):
            raise ValueError('Invalid Gazebo layout operation')
        data = json.loads(node.get_parameter('layout_json').value)
        share = Path(get_package_share_directory('ur5e_a3_grid'))
        physics = yaml.safe_load((share/'config/gazebo_physics.yaml').read_text())
        mesh = Path(get_package_share_directory('ur5e_block_description'))/'meshes/cube_plate.STL'
        models = layout_models(data, mesh.as_uri())
        robot = call(get, GetEntityState.Request(name='ur5e_stack_robot', reference_frame='world'))
        if not robot.success:
            raise RuntimeError('The Gazebo robot is not present')
        deadline = time.monotonic()+10.
        while not names and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not names:
            raise RuntimeError('Gazebo state stream is unavailable; resume the world first')
        if operation == 'verify':
            expected_names = {model['name'] for model in models}
            owned = {name for name in names if re.fullmatch(r'ur5e_stack_(plate|block_\d+|obstacle_\d+)', name)}
            if owned != expected_names:
                raise RuntimeError('Gazebo layout differs; apply the selected layout with button ③ first')
            for model in models:
                state = call(get, GetEntityState.Request(name=model['name'], reference_frame='world'))
                p = state.state.pose.position
                error = math.dist([p.x, p.y, p.z], model['position'])
                if not state.success or not math.isfinite(error) or error > .006:
                    raise RuntimeError('Gazebo object moved from its initial layout: '+model['name']+'; reset with button ③')
            print('STACK_GAZEBO_LAYOUT_VERIFIED '+str(len(models)), flush=True)
        else:
            for name in sorted(names):
                if re.fullmatch(r'ur5e_stack_(plate|block_\d+|obstacle_\d+)', name):
                    reply = call(delete, DeleteEntity.Request(name=name))
                    if not reply.success:
                        raise RuntimeError('Gazebo refused to remove '+name)
            if operation == 'configure':
                for model in models:
                    request = SpawnEntity.Request()
                    request.name, request.reference_frame = model['name'], 'world'
                    request.xml = model_sdf(model, physics['objects'], physics['logging']['contact_hz'])
                    p = model['position']
                    request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = p
                    if model['role'] != 'plate':
                        request.initial_pose.position.z += .001
                    q = model['orientation_xyzw']
                    (request.initial_pose.orientation.x, request.initial_pose.orientation.y,
                     request.initial_pose.orientation.z, request.initial_pose.orientation.w) = q
                    reply = call(spawn, request)
                    if not reply.success:
                        raise RuntimeError('Gazebo refused to spawn '+model['name']+': '+reply.status_message)
                print('STACK_GAZEBO_LAYOUT_CREATED '+str(len(models)), flush=True)
    except Exception as error:
        print('STACK_ERROR '+str(error), flush=True)
        code = 1
    finally:
        if lock is not None:
            os.close(lock)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())

