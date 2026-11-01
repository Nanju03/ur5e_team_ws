"""Approval validation only: nominal model and GenericSystem, domain 93."""
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    source = Path('/home/ksn/ur5e_ws/src/ur5e_ros2')
    spec = importlib.util.spec_from_file_location('mock_sim_stack_model', source/'ur5e_a3_grid/launch/stack_model.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    settings = module.make_model().to_dict()
    settings.update(allow_trajectory_execution=True, publish_robot_description_semantic=True,
                    publish_planning_scene=True, publish_geometry_updates=True,
                    publish_state_updates=True, publish_transforms_updates=True)
    robot = ET.fromstring(settings['robot_description'])
    controls = robot.findall('ros2_control')
    assert len(controls) == 1
    assert controls[0].find('hardware/plugin').text == 'mock_components/GenericSystem'
    initial = dict(shoulder_pan_joint=0., shoulder_lift_joint=-1.5, elbow_joint=1.5,
                   wrist_1_joint=-1.5, wrist_2_joint=-1.5, wrist_3_joint=0.,
                   left_outer_knuckle_joint=.4)
    for joint in controls[0].findall('joint'):
        joint.find("state_interface[@name='position']/param[@name='initial_value']").text = str(initial[joint.get('name')])
    return LaunchDescription([
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': ET.tostring(robot, encoding='unicode')},
                         str(source/'ur5e_ag95_moveit_config/config/ros2_controllers.yaml')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'ur5e_arm_controller', 'dh_ag95_gripper_controller']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description':settings['robot_description']}]),
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group',
             parameters=[settings], output='screen'),
    ])
