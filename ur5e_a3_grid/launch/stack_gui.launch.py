"""Click A3 cells to select obstacles and the tower foundation."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("domain_id", default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0")),
        DeclareLaunchArgument("backend", default_value="simulation", choices=["simulation", "real"]),
        DeclareLaunchArgument("simulation_riser_height", default_value="0.0"),
        DeclareLaunchArgument("calibration_file", default_value=str(Path.home()/"ur5e_ws/robot_calibration/ur5e_20261031.yaml")),
        DeclareLaunchArgument("physical_profile", default_value=str(Path.home()/"ur5e_ws/physical_setup.yaml")),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        Node(package="ur5e_a3_grid", executable="stack_gui", name="ur5e_a3_stack_gui", output="screen",
             parameters=[{"backend": LaunchConfiguration("backend"),
                          "simulation_riser_height": ParameterValue(
                              LaunchConfiguration("simulation_riser_height"), value_type=float),
                          "calibration_file": LaunchConfiguration("calibration_file"),
                          "physical_profile": LaunchConfiguration("physical_profile")}]),
    ])
