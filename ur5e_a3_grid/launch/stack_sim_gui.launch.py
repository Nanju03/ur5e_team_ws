"""Standalone robot-free simulator and task GUI on an isolated ROS domain."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("ur5e_a3_grid"))
    return LaunchDescription([
        DeclareLaunchArgument("riser_height", default_value="0.016"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/stack_sim.launch.py")),
            launch_arguments={"riser_height": LaunchConfiguration("riser_height"),
                              "use_rviz": LaunchConfiguration("use_rviz")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/stack_gui.launch.py")),
            launch_arguments={"backend": "simulation", "domain_id": "42",
                              "simulation_riser_height": LaunchConfiguration("riser_height")}.items(),
            condition=IfCondition(LaunchConfiguration("use_gui"))),
    ])
