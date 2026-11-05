"""Gazebo Classic contact simulation, isolated from the real robot domain."""
import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.launches import generate_move_group_launch


def start(context):
    if os.environ.get('ROS_DOMAIN_ID') != '44' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('Use start_block_gazebo.sh: Gazebo requires localhost ROS domain 44')
    share = Path(get_package_share_directory('ur5e_a3_grid'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    spec = importlib.util.spec_from_file_location('ur5e_stack_model', share/'launch/stack_model.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    height = float(LaunchConfiguration('riser_height').perform(context))
    model = helper.make_model(height, simulation_backend='gazebo')
    robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'ur5e_stack_robot', '-topic', 'robot_description', '-timeout', '120'], output='screen')
    controllers = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', 'ur5e_arm_controller', 'dh_ag95_gripper_controller',
                                  '--controller-manager-timeout', '120'], parameters=[{'use_sim_time': True}], output='screen')

    def spawned(event, _):
        return [controllers] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Gazebo robot spawn failed'))]

    def activated(event, _):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='Gazebo controllers failed'))]
        return [GroupAction([
            SetParameter(name='use_sim_time', value=True),
            *generate_move_group_launch(model).entities,
            Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/stack.rviz')],
                 parameters=[model.robot_description, model.robot_description_semantic,
                             model.robot_description_kinematics, model.joint_limits, model.planning_pipelines,
                             {'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('use_rviz')), output='log'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/stack_gui.launch.py')),
                launch_arguments={'backend': 'simulation', 'simulation_backend': 'gazebo', 'domain_id': '44',
                                  'simulation_riser_height': str(height), 'use_sim_time': 'true'}.items(),
                condition=IfCondition(LaunchConfiguration('use_gui'))),
        ])]

    return [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gzserver.launch.py')),
            launch_arguments={'world': str(share/'worlds/stack.world'), 'server_required': 'true',
                              'pause': 'false', 'verbose': LaunchConfiguration('verbose')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gzclient.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             parameters=[model.robot_description, {'use_sim_time': True}], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=robot, on_exit=spawned)),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=activated)),
        robot,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('riser_height', default_value='0.016'),
        DeclareLaunchArgument('verbose', default_value='false'),
        OpaqueFunction(function=start),
    ])
