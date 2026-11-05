"""Stacking backend. A selected layout is loaded after the common model/grid settings."""
from pathlib import Path
import importlib.util
import yaml
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
        simulation_backend=LaunchConfiguration("simulation_backend").perform(context),
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
    settings.pop('gripper_pregrasp_position', None)  # Retired narrow-entry policy.
    if settings.get("plate_enabled", False):
        from ur5e_block_bringup.plate_geometry import plate_parameters, PROFILE
        if selection and settings.get("geometry_profile") != PROFILE:
            raise ValueError("플레이트 배치를 새 GUI에서 저장하고 다시 계획하세요.")
        settings.update(plate_parameters(settings))
    simulator = LaunchConfiguration("simulation_backend").perform(context)
    if simulator == "gazebo":
        physics = yaml.safe_load((share/'config/gazebo_physics.yaml').read_text())
        settings.update(physics['task'])
        settings['gazebo_gripper_max_effort'] = physics['gripper']['max_effort_nm']
    settings['simulation_backend'] = simulator
    mode = LaunchConfiguration('stack_mode').perform(context)
    if simulator == 'gazebo' and mode == 'run':
        from ur5e_block_bringup.path_archive import prepare_execution_archive
        archive_settings = {**settings, 'stack_mode': mode, 'use_sim_time': True,
                            'execution_mode': LaunchConfiguration('execution_mode').perform(context)}
        folder = prepare_execution_archive(archive_settings, model.to_dict(), share,
            root=LaunchConfiguration('path_archive_root').perform(context) or None)
        settings['execution_archive_directory'] = str(folder)
    if LaunchConfiguration("stack_mode").perform(context) not in ("plan", "run"):
        settings["plate_alignment_test"] = False
        settings["empty_xy_scan"] = False
    # Humble cannot infer an integer-array type from YAML []. Let the C++ client
    # declare its typed empty-array default when no squares have that role.
    parameters.append({key: value for key, value in settings.items() if value != []})
    parameters.append({
        "stack_mode": LaunchConfiguration("stack_mode"),
        "execution_mode": LaunchConfiguration("execution_mode"),
        "use_sim_time": True if simulator == 'gazebo' else ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    })
    task = Node(package="ur5e_a3_grid", executable="stack_blocks", name="ur5e_a3_stack",
                parameters=parameters, output="screen")
    mode = LaunchConfiguration('stack_mode').perform(context)
    if simulator == 'gazebo' and mode in ('scene', 'run', 'reset'):
        operation = {'scene': 'configure', 'run': 'verify', 'reset': 'reset'}[mode]
        layout = Node(package='ur5e_a3_grid', executable='gazebo_layout',
                      parameters=[{'layout_json': json.dumps(settings), 'operation': operation}], output='screen')
        def ready(event, _):
            return [task] if event.returncode == 0 else [LogInfo(msg='STACK_ERROR Gazebo layout preparation failed'), EmitEvent(event=Shutdown(reason='Gazebo layout preparation failed'))]
        return [RegisterEventHandler(OnProcessExit(target_action=layout, on_exit=ready)), layout]
    return [task]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stack_mode", default_value="scene"),
        DeclareLaunchArgument("execution_mode", default_value="step", choices=["step", "continuous"]),
        DeclareLaunchArgument("selection_file", default_value=""),
        DeclareLaunchArgument("moveit_config_package", default_value="ur5e_ag95_moveit_config"),
        DeclareLaunchArgument("robot_name", default_value="ur5e_on_box"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("simulation_backend", default_value="mock", choices=['mock', 'gazebo']),
        DeclareLaunchArgument("riser_height", default_value="0.0"),
        DeclareLaunchArgument("path_archive_root", default_value=""),
        OpaqueFunction(function=start),
    ])
