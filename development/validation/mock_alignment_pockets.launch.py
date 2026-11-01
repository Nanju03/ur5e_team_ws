"""Validation fixture only: no physical drivers or device/network connections."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ur5e_block_bringup.model import load_moveit_settings


def generate_launch_description():
    model = load_moveit_settings('/home/ksn/ur5e_ws/robot_calibration/ur5e_20261031.yaml',
                                 '/home/ksn/ur5e_ws/physical_setup.yaml')
    model.update(yaml.safe_load(Path('/home/ksn/ur5e_ws/src/ur5e_ros2/ur5e_block_bringup/config/real_controllers.yaml').read_text()))
    model.update(allow_trajectory_execution=True, publish_robot_description_semantic=True,
                 publish_planning_scene=True, publish_geometry_updates=True,
                 publish_state_updates=True, publish_transforms_updates=True)
    robot = ET.fromstring(model['robot_description'])
    control = ET.SubElement(robot, 'ros2_control', name='air_mock', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
    arm = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    initial = [0., -1.5, 1.5, -1.5, -1.5, 0., .4]
    for name, position in zip(arm+['left_outer_knuckle_joint'], initial):
        joint = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(joint, 'command_interface', name='position')
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(position)
        ET.SubElement(joint, 'state_interface', name='velocity')
    config = {
        '/**/controller_manager': {'ros__parameters': {
            'update_rate': 100,
            'joint_state_broadcaster': {'type':'joint_state_broadcaster/JointStateBroadcaster'},
            'scaled_joint_trajectory_controller': {'type':'joint_trajectory_controller/JointTrajectoryController'},
            'dh_ag95_gripper_controller': {'type':'position_controllers/GripperActionController'},
        }},
        '/**/scaled_joint_trajectory_controller': {'ros__parameters': {
            'joints':arm, 'command_interfaces':['position'], 'state_interfaces':['position','velocity'],
            'constraints':{'goal_time':5.0, 'stopped_velocity_tolerance':.01},
        }},
    }
    config['/**/dh_ag95_gripper_controller'] = {'ros__parameters': {'joint':'left_outer_knuckle_joint','goal_tolerance':.001,'stall_timeout':1.0}}
    config_path = Path('/home/ksn/ur5e_ws/validation/mock_alignment_pockets_controllers.yaml')
    config_path.write_text(yaml.safe_dump(config))
    assert [h.text for h in robot.findall('ros2_control/hardware/plugin')] == ['mock_components/GenericSystem']
    return LaunchDescription([
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description':ET.tostring(robot,encoding='unicode')},str(config_path)], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster','scaled_joint_trajectory_controller','dh_ag95_gripper_controller']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description':model['robot_description']}]),
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group', parameters=[model],output='screen'),
    ])
