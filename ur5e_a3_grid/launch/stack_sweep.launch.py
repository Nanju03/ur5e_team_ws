"""Plan every single-obstacle / tower-cell combination in an isolated mock world."""
import fcntl
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
import yaml

_locks = []


def boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError('Use true or false, got '+value)
    return value.lower() == 'true'


def unused_domain(domain):
    """Planning changes the remote scene, so never reuse another demo's domain."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    lock = open(f'/tmp/ur5e_stack_sweep_domain_{domain}.lock', 'a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError(f'Another sweep already owns ROS domain {domain}')
    _locks.append(lock)
    ctx = Context()
    rclpy.init(args=[], context=ctx, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    probe = rclpy.create_node('ur5e_sweep_domain_probe', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(probe)
    try:
        deadline = time.monotonic()+2.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
            names = set(probe.get_node_names())
            conflicts = names & {'move_group', 'controller_manager', 'robot_state_publisher',
                                 'ur5e_a3_stack', 'ur5e_a3_stack_gui', 'ur5e_stack_sweep'}
            services = dict(probe.get_service_names_and_types())
            if conflicts or '/get_planning_scene' in services or '/controller_manager/list_controllers' in services:
                raise RuntimeError(f'ROS domain {domain} is already in use. Choose a different domain_id; existing nodes were not changed.')
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown(context=ctx)


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    domain = int(value('domain_id'))
    if not 0 <= domain <= 232:
        raise ValueError('domain_id must be in 0..232')
    dry_run = boolean(value('dry_run'))
    use_rviz = boolean(value('use_rviz'))
    share = Path(get_package_share_directory('ur5e_a3_grid'))
    model = MoveItConfigsBuilder('ur5e_on_box', package_name='ur5e_ag95_moveit_config').to_moveit_configs()
    settings = {}
    for name in ('a3_grid.yaml', 'stack_blocks.yaml'):
        settings.update(yaml.safe_load((share/'config'/name).read_text())['/**']['ros__parameters'])
    if value('config_file'):
        settings.update(yaml.safe_load(Path(value('config_file')).expanduser().read_text())['/**']['ros__parameters'])
    if (settings.get('rows'), settings.get('columns')) != (6, 4):
        raise ValueError('This sweep checks the current 4-column x 6-row board')
    settings.update(stack_mode='plan', target_cell=0, obstacle_cells=[], empty_cells=[])
    settings['obstacle_size_xyz'] = [float(settings['obstacle_size_xyz'][0]), float(settings['obstacle_size_xyz'][1]), float(value('obstacle_height'))]
    settings['placement_mode'] = value('placement_mode')
    if settings.get('robot_riser_height', 0.) != 0.:
        raise ValueError('This sweep uses the original zero-riser robot model')
    if settings['placement_mode'] not in ('side', 'vertical') or not math.isfinite(settings['obstacle_size_xyz'][2]) or settings['obstacle_size_xyz'][2] <= 0:
        raise ValueError('Invalid placement_mode / obstacle_height')
    backend = {}
    for item in (model.robot_description, model.robot_description_semantic, model.robot_description_kinematics,
                 model.joint_limits, model.planning_pipelines):
        backend.update(item)
    backend.update(settings)

    semantic = ET.fromstring(model.robot_description_semantic['robot_description_semantic'])
    home_tag = semantic.find("group_state[@name='home'][@group='ur5e_arm']")
    if home_tag is None:
        raise ValueError('The named home pose is missing')
    home = {joint.get('name'): float(joint.get('value')) for joint in home_tag.findall('joint')}
    home['left_outer_knuckle_joint'] = 0.
    controller = ET.fromstring(model.robot_description['robot_description'])
    hardware = controller.findall('./ros2_control/hardware/plugin')
    if not hardware or any(plugin.text.strip() != 'mock_components/GenericSystem' for plugin in hardware):
        raise ValueError('Sweep fixture requires mock_components/GenericSystem')
    for joint in controller.findall('./ros2_control/joint'):
        initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        if initial is not None:
            initial.text = str(home[joint.get('name')])

    arguments = []
    for name in ('output_dir', 'obstacles', 'targets', 'max_cases', 'attempts', 'communication_retries', 'case_timeout',
                 'resume', 'retry_failed', 'pause_seconds', 'review', 'keep_alive', 'dry_run'):
        arguments.extend(['--'+name.replace('_', '-'), value(name)])
    runner = Node(package='ur5e_a3_grid', executable='stack_sweep', name='ur5e_stack_sweep',
                  arguments=arguments, output='screen', sigterm_timeout='20',
                  parameters=[{'backend_json': json.dumps(backend), 'home_json': json.dumps(home)}])
    actions = []
    if not dry_run:
        unused_domain(domain)
        actions.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
                 parameters=[model.robot_description], output='log'),
            *generate_move_group_launch(model).entities,
            # Only mock initial joint values change. RSP/MoveIt keep the original
            # description, geometry and limits so backend model guards still match.
            Node(package='controller_manager', executable='ros2_control_node',
                 parameters=[{'robot_description': ET.tostring(controller, encoding='unicode')},
                             str(model.package_path/'config/ros2_controllers.yaml')], output='log'),
        ])
        for name in ('joint_state_broadcaster', 'ur5e_arm_controller', 'dh_ag95_gripper_controller'):
            actions.append(Node(package='controller_manager', executable='spawner', arguments=[name], output='log'))
        if use_rviz:
            # Keep the normal MotionPlanning layout; batch previews must replace
            # the previous case immediately rather than wait for its animation.
            configuration = yaml.safe_load((model.package_path/'config/moveit.rviz').read_text())
            for display in configuration['Visualization Manager']['Displays']:
                if display.get('Class') == 'moveit_rviz_plugin/MotionPlanning':
                    display.setdefault('Planned Path', {}).update({'Interrupt Display': True, 'Loop Animation': False})
            output = Path(value('output_dir')).expanduser().resolve()
            output.mkdir(parents=True, exist_ok=True)
            rviz_file = output/f'sweep_domain_{domain}.rviz'
            rviz_file.write_text(yaml.safe_dump(configuration))
            actions.append(Node(package='rviz2', executable='rviz2',
                                arguments=['-d', str(rviz_file)],
                                parameters=[model.robot_description, model.robot_description_semantic,
                                            model.robot_description_kinematics, model.joint_limits], output='log'))
    actions.extend([
        RegisterEventHandler(OnProcessExit(target_action=runner,
                             on_exit=[EmitEvent(event=Shutdown(reason='Stack sweep runner finished'))])),
        runner,
    ])
    return actions


def generate_launch_description():
    defaults = {
        'domain_id': ('77', 'Unused ROS domain, separate from the manual demo'),
        'output_dir': ('/workspaces/cobot_ws/log/stack_sweep', 'Results, logs, saved robot paths and resume state'),
        'obstacles': ('all', 'Obstacle cells: all, 10, or 1-4,10'),
        'targets': ('all', 'Tower cells: all, 11, or 1-4,11'),
        'max_cases': ('0', 'Maximum new cases this run; 0 checks the whole selected set'),
        'attempts': ('1', 'Planning attempts per case this invocation'),
        'communication_retries': ('2', 'Additional whole-case retries after transient service timeouts; 0..5'),
        'case_timeout': ('180', 'Timeout in seconds per attempt'),
        'resume': ('true', 'Continue a compatible existing result folder'),
        'retry_failed': ('false', 'On resume, retry non-passing cases too'),
        'pause_seconds': ('0', 'Pause between cases, useful with RViz'),
        'use_rviz': ('false', 'Show the normal demo MotionPlanning RViz'),
        'keep_alive': ('false', 'Keep the last preview visible after the sweep'),
        'review': ('', 'Show a saved passing robot path, e.g. 10,11, without replanning'),
        'dry_run': ('false', 'List all selected pairs without starting MoveIt or planning'),
        'obstacle_height': ('0.15', 'Obstacle height in metres'),
        'placement_mode': ('side', 'side or vertical'),
        'config_file': ('', 'Optional common A3/stack parameter overlay'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in defaults.items()],
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        OpaqueFunction(function=start),
    ])
