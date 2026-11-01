"""Open RViz for the existing combined planning model without launching servers."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ur5e_block_bringup.model import load_moveit_settings


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    settings = load_moveit_settings(value('calibration_file'), value('physical_profile'))
    share = Path(get_package_share_directory('ur5e_block_bringup'))
    settings.update(yaml.safe_load((share/'config/real_controllers.yaml').read_text()))
    settings['use_sim_time'] = False
    rviz = share/'config/plate_view.rviz'
    return [Node(package='rviz2', executable='rviz2', name='rviz2',
                 arguments=['-d', str(rviz)], parameters=[settings], output='log')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=str(Path.home()/'ur5e_ws/robot_calibration/ur5e_20261031.yaml')),
        DeclareLaunchArgument('physical_profile', default_value=str(Path.home()/'ur5e_ws/physical_setup.yaml')),
        OpaqueFunction(function=start),
    ])
