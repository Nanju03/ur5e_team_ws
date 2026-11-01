"""Connect to an existing MoveIt; this launch starts no drivers or controllers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def start_client(context):
    model = MoveItConfigsBuilder(
        LaunchConfiguration("robot_name").perform(context),
        package_name=LaunchConfiguration("moveit_config_package").perform(context),
    ).to_moveit_configs()
    overrides = {}
    for name, value_type in (
        ("execute", bool),
        ("operation", str),
        ("cell_id", int),
        ("keep_alive", bool),
        ("coordinates_file", str),
    ):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    return [Node(
        package="ur5e_a3_grid",
        executable="a3_grid",
        name="ur5e_a3_grid",
        output="screen",
        parameters=[
            model.robot_description,
            model.robot_description_semantic,
            model.robot_description_kinematics,
            model.joint_limits,
            LaunchConfiguration("config_file"),
            overrides,
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
    )]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory("ur5e_a3_grid")) / "config/a3_grid.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("moveit_config_package", default_value="ur5e_ag95_moveit_config"),
        DeclareLaunchArgument("robot_name", default_value="ur5e_on_box"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("execute", default_value="", description="true sends MoveIt execution requests"),
        DeclareLaunchArgument("operation", default_value="", description="scan, cell, home, preview, pick_cycle, or pick_cell"),
        DeclareLaunchArgument("cell_id", default_value="", description="1..24 for cell/pick_cell"),
        DeclareLaunchArgument("keep_alive", default_value=""),
        DeclareLaunchArgument("coordinates_file", default_value=""),
        OpaqueFunction(function=start_client),
    ])
