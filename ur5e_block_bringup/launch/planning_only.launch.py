"""Refresh the combined planning model while reusing running hardware drivers.

The two driver-start arguments are deliberately fixed to false rather than
exposed by this entrypoint. This launch creates no hardware interface and
performs no gripper initialization or task execution.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    real_launch = Path(get_package_share_directory('ur5e_block_bringup'))/'launch/real_robot.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=str(Path.home()/'ur5e_ws/robot_calibration/ur5e_20261031.yaml')),
        DeclareLaunchArgument('physical_profile', default_value=str(Path.home()/'ur5e_ws/physical_setup.yaml')),
        DeclareLaunchArgument('allow_execution', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('use_gripper_calibration', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false']),
        LogInfo(msg='Planning model only: reuse running arm/gripper drivers; no driver initialization or task is started.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(real_launch)), launch_arguments={
            'calibration_file': LaunchConfiguration('calibration_file'),
            'physical_profile': LaunchConfiguration('physical_profile'),
            'allow_execution': LaunchConfiguration('allow_execution'),
            'use_gripper_calibration': LaunchConfiguration('use_gripper_calibration'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'start_arm': 'false',
            'start_gripper': 'false',
        }.items()),
    ])
