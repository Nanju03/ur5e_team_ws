"""Standalone GenericSystem/RViz simulation with an optional robot riser."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launches import generate_move_group_launch


def start(context):
    share = Path(get_package_share_directory("ur5e_a3_grid"))
    spec = importlib.util.spec_from_file_location("ur5e_stack_model", share / "launch/stack_model.py")
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    model = helper.make_model(float(LaunchConfiguration("riser_height").perform(context)))
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             parameters=[model.robot_description], output="screen"),
        *generate_move_group_launch(model).entities,
        Node(package="controller_manager", executable="ros2_control_node", parameters=[model.robot_description,
             str(model.package_path / "config/ros2_controllers.yaml")], output="screen"),
    ]
    for controller in ("joint_state_broadcaster", "ur5e_arm_controller", "dh_ag95_gripper_controller"):
        actions.append(Node(package="controller_manager", executable="spawner", arguments=[controller], output="screen"))
    actions.append(Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config/stack.rviz")],
                        parameters=[model.robot_description, model.robot_description_semantic,
                                    model.robot_description_kinematics, model.joint_limits, model.planning_pipelines],
                        condition=IfCondition(LaunchConfiguration("use_rviz")), output="log"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("riser_height", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=start),
    ])
