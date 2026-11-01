"""Reuse the installed UR and DH drivers with the user's combined MoveIt model."""
from pathlib import Path
import hashlib

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ur5e_block_bringup.model import load_moveit_settings
from ur5e_block_bringup.gripper_calibration import load_calibration_profile
from ur5e_block_bringup.driver_launch_guard import guard_hardware_launch


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    guard_hardware_launch(start_arm=value('start_arm') == 'true',
                          start_gripper=value('start_gripper') == 'true')
    calibration = str(Path(value('calibration_file')).expanduser().resolve())
    settings = load_moveit_settings(calibration, value('physical_profile'))
    grip = (load_calibration_profile(model_sha256=hashlib.sha256(settings['robot_description'].encode()).hexdigest())
            if value('use_gripper_calibration') == 'true' else None)
    force_percent = str(grip['force_percent'] if grip else 20)
    share = Path(get_package_share_directory('ur5e_block_bringup'))
    settings.update(yaml.safe_load((share/'config/real_controllers.yaml').read_text()))
    settings.update({
        'allow_trajectory_execution': value('allow_execution') == 'true',
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'use_sim_time': False,
    })
    if value('allow_execution') != 'true':
        settings['disable_capabilities'] = 'move_group/MoveGroupExecuteTrajectoryAction'
    driver = Path(get_package_share_directory('ur_robot_driver'))/'launch/ur_control.launch.py'
    gripper = Path(get_package_share_directory('dh_gripper_driver'))/'launch/dh_ag95_control.launch.py'
    rviz = share/'config/plate_view.rviz'
    return [
        LogInfo(msg='Existing UR5e/AG95 drivers + your combined block model; no stacking task is started.'),
        GroupAction([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver)), launch_arguments={
                'ur_type': 'ur5e', 'robot_ip': value('robot_ip'),
                'kinematics_params_file': calibration,
                'headless_mode': value('headless_mode'),
                'activate_joint_controller': value('activate_arm_controller'),
                'initial_joint_controller': 'scaled_joint_trajectory_controller',
                'use_fake_hardware': 'false', 'launch_rviz': 'false',
                'launch_robot_state_publisher': 'false',
            }.items()),
        ], scoped=True, condition=IfCondition(LaunchConfiguration('start_arm'))),
        GroupAction([
            LogInfo(msg='AG95 driver initializes the gripper; its fingers may move.'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gripper)), launch_arguments={
                'namespace': 'gripper', 'com_port': value('gripper_device'),
                'gui': 'false', 'launch_robot_state_publisher': 'false',
                'use_mock_hardware': 'false', 'joint_states_topic': '/joint_states',
                'controller_mode': 'action',
                'force_percent': force_percent,
            }.items()),
        ], scoped=True, condition=IfCondition(LaunchConfiguration('start_gripper'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': settings['robot_description']}],
             output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group',
             parameters=[settings], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(rviz)],
             parameters=[settings], condition=IfCondition(LaunchConfiguration('use_rviz')), output='log'),
    ]


def generate_launch_description():
    calibration = str(Path.home()/'ur5e_ws/robot_calibration/ur5e_20261031.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.2'),
        DeclareLaunchArgument('calibration_file', default_value=calibration),
        DeclareLaunchArgument('physical_profile', default_value=str(Path.home()/'ur5e_ws/physical_setup.yaml')),
        DeclareLaunchArgument('gripper_device', default_value='/dev/robot/dh_ag95_gripper'),
        DeclareLaunchArgument('start_arm', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('start_gripper', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('headless_mode', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('activate_arm_controller', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('allow_execution', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('use_gripper_calibration', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false']),
        OpaqueFunction(function=start),
    ])
