"""Stacking backend. A selected layout is loaded after the common model/grid settings."""
from pathlib import Path
import importlib.util
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    share = Path(get_package_share_directory("ur5e_a3_grid"))
    spec = importlib.util.spec_from_file_location("ur5e_stack_model", share / "launch/stack_model.py")
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    model = helper.make_model(
        float(LaunchConfiguration("riser_height").perform(context)),
        LaunchConfiguration("moveit_config_package").perform(context),
        LaunchConfiguration("robot_name").perform(context),
    )
    parameters = [
        model.robot_description, model.robot_description_semantic,
        model.robot_description_kinematics, model.joint_limits, model.planning_pipelines,
    ]
    settings = {}
    for filename in (share / "config/a3_grid.yaml", share / "config/stack_blocks.yaml"):
        settings.update(yaml.safe_load(filename.read_text())["/**"]["ros__parameters"])
    selection = LaunchConfiguration("selection_file").perform(context)
    if selection:
        settings.update(yaml.safe_load(Path(selection).read_text())["/**"]["ros__parameters"])
    if settings.get("plate_enabled", False):
        from ur5e_block_bringup.plate_geometry import plate_parameters, PROFILE
        if selection and settings.get("geometry_profile") != PROFILE:
            raise ValueError("플레이트 배치를 새 GUI에서 저장하고 다시 계획하세요.")
        settings.update(plate_parameters(settings))
    if LaunchConfiguration("stack_mode").perform(context) not in ("plan", "run"):
        settings["plate_alignment_test"] = False
        settings["empty_xy_scan"] = False
    # Humble cannot infer an integer-array type from YAML []. Let the C++ client
    # declare its typed empty-array default when no squares have that role.
    parameters.append({key: value for key, value in settings.items() if value != []})
    parameters.append({
        "stack_mode": LaunchConfiguration("stack_mode"),
        "execution_mode": LaunchConfiguration("execution_mode"),
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    })
    return [Node(package="ur5e_a3_grid", executable="stack_blocks", name="ur5e_a3_stack",
                 parameters=parameters, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stack_mode", default_value="scene"),
        DeclareLaunchArgument("execution_mode", default_value="step", choices=["step", "continuous"]),
        DeclareLaunchArgument("selection_file", default_value=""),
        DeclareLaunchArgument("moveit_config_package", default_value="ur5e_ag95_moveit_config"),
        DeclareLaunchArgument("robot_name", default_value="ur5e_on_box"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("riser_height", default_value="0.0"),
        OpaqueFunction(function=start),
    ])
