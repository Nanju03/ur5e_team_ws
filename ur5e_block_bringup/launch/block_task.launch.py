"""Use the existing block planner against the same calibrated real robot model."""
from pathlib import Path
import hashlib
import os
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ur5e_block_bringup.model import load_moveit_settings
from ur5e_block_bringup.gripper_calibration import load_calibration_profile
from ur5e_block_bringup.plate_geometry import plate_parameters, PROFILE
from ur5e_block_bringup.physical_motion import motion_parameters, incremental_motion_parameters


def physical_held_path():
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdigit():
        raise ValueError('ROS_DOMAIN_ID must be numeric')
    return Path('/tmp')/f'ur5e_physical_held_{domain}.json'


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('stack_mode')
    physical = mode in ('physical_plan', 'physical_prepare', 'physical_run', 'physical_recover', 'physical_unlock')
    recovery = mode in ('physical_recover', 'physical_unlock')
    incremental = physical and not recovery and LaunchConfiguration('incremental_stack', default='true').perform(context) == 'true'
    held = physical_held_path()
    if not physical and (held.exists() or held.is_symlink()):
        raise ValueError('집기 중단 후 그리퍼에 블록이 남아 있을 수 있어 일반 경로·장면 초기화를 잠갔습니다. '
                         '그리퍼가 비어 있으면 GUI의 빈 그리퍼 확인 · 중단 잠금 해제를 사용하세요. '
                         '팔 복귀가 필요하면 안전 확인 후 초기 자세 복구를 사용하세요.')
    settings = load_moveit_settings(value('calibration_file'), value('physical_profile'))
    share = Path(get_package_share_directory('ur5e_a3_grid'))
    for filename in ('a3_grid.yaml', 'stack_blocks.yaml'):
        settings.update(yaml.safe_load((share/'config'/filename).read_text())['/**']['ros__parameters'])
    settings['velocity_scaling'] = 0.05
    selection = None
    if value('selection_file'):
        selection = yaml.safe_load(Path(value('selection_file')).expanduser().read_text())['/**']['ros__parameters']
        settings.update(selection)
    settings.pop('gripper_pregrasp_position', None)  # Ignore saved narrow-entry settings.
    if settings.get('plate_enabled', False):
        if selection is not None and selection.get('geometry_profile') != PROFILE:
            raise ValueError('플레이트 좌표 형식이 바뀌었습니다. 새 GUI에서 배치를 저장하고 다시 계획하세요.')
        settings.update(plate_parameters(settings))
    grip = load_calibration_profile(model_sha256=hashlib.sha256(settings['robot_description'].encode()).hexdigest(),
                                    block_width_mm=settings['block_size_xyz'][0]*1000)
    if grip and not physical:
        settings.update(gripper_open_position=grip['gripper_open_position'],
                        gripper_grasp_position=grip['gripper_grasp_position'])
    # The reviewed physical executor is separate from the legacy C++ run mode,
    # whose GenericSystem-only execution guard remains in place.
    if physical:
        if recovery and not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', value('preparation_id')):
            raise ValueError('A unique preparation_id is required for reviewed physical recovery')
        settings.update(
            stack_mode=mode, physical_stack_task=True, empty_hand_test=False,
            empty_xy_scan=False, plate_alignment_test=False,
            gripper_open_position=.4, gripper_grasp_position=.73,
            staged_release_enabled=True, release_first_position=.70,
            release_clearance_position=.60,
            **motion_parameters(),
            **incremental_motion_parameters(),
            side_entry_clearance=.010,
            vertical_placement_layers=19, placement_approach_clearance=.12,
            incremental_stack=incremental, post_place_retreat_clearance=.050,
            incremental_pickup_approach_clearance=.080,
            incremental_transfer_clearance=.050,
            trajectory_file=value('trajectory_file'),
            recipe_manifest_file=value('recipe_manifest_file') or value('trajectory_file'),
            selection_file=value('selection_file'), calibration_file=value('calibration_file'),
            physical_profile=value('physical_profile'),
            workspace_clear=value('workspace_clear') == 'true',
            execution_mode=value('execution_mode'),
            allow_mock_hardware=value('allow_mock_hardware') == 'true')
        if recovery:
            settings.update(recovery_empty_confirmed=value('recovery_empty_confirmed') == 'true',
                            recovery_preserve_gripper=LaunchConfiguration('recovery_preserve_gripper', default='false').perform(context) == 'true',
                            preparation_id=value('preparation_id'))
        settings = {key: val for key, val in settings.items() if val != []}
        executable = 'physical_recovery' if recovery else 'physical_incremental' if incremental else 'physical_stack'
        return [Node(package='ur5e_block_bringup', executable=executable,
                     parameters=[settings], output='screen')]
    if mode not in ('dry_plan', 'dry_run', 'alignment_prepare'):
        settings['empty_xy_scan'] = False
        settings['plate_alignment_test'] = False
    settings.update(stack_mode=mode, acceleration_scaling=0.05)
    if mode in ('dry_plan', 'dry_run', 'alignment_prepare'):
        # Scene display and legacy planning also load physical selections.
        # Only empty-hand trials use this conservative speed limit.
        if not 0 < float(settings.get('velocity_scaling', 0.05)) <= 0.05:
            raise ValueError('Physical empty-hand trial speed must be in (0, 0.05]')
        clearance = float(settings.get('empty_hand_clearance', 0.05))
        if not 0.03 <= clearance <= 0.10:
            raise ValueError('Empty-hand path clearance must be between 30 and 100 mm')
        # Only the logical pickup/placement plane rises. The real workbench
        # collision geometry remains at the measured, modelled height.
        alignment = mode == 'alignment_prepare' or bool(settings.get('plate_alignment_test', False))
        if alignment and not settings.get('plate_enabled', False):
            raise ValueError('Plate alignment requires the measured cube plate geometry')
        settings['plate_alignment_test'] = alignment
        settings['empty_xy_scan'] = bool(settings.get('empty_xy_scan', False)) or alignment
        if settings['empty_xy_scan']:
            if alignment:
                # A selected stacking pocket has a lower floor than the plate
                # rim. Alignment clearance is always measured from the rim.
                settings['scan_min_gripper_z'] = settings['plate_bottom_z'] + .010 + settings['plate_alignment_tip_clearance']
            else:
                settings['scan_min_gripper_z'] = settings.get('obstacle_surface_z', settings['table_surface_z']) + settings['obstacle_size_xyz'][2] + clearance
        else:
            if settings.get('plate_enabled', False):
                settings['source_surface_z'] += clearance
                settings['stack_surface_z'] += clearance
            else:
                settings['obstacle_surface_z'] = settings['table_surface_z']
                settings['table_surface_z'] += clearance
        settings['empty_hand_clearance'] = clearance
        settings.update(
            trajectory_file=value('trajectory_file'), selection_file=value('selection_file'),
            calibration_file=value('calibration_file'), physical_profile=value('physical_profile'),
            workspace_clear=value('workspace_clear') == 'true',
            execution_mode=value('execution_mode'),
            allow_mock_hardware=value('allow_mock_hardware') == 'true')
        if mode == 'alignment_prepare':
            preparation_id = value('preparation_id')
            if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', preparation_id):
                raise ValueError('A unique preparation_id is required for reviewed gripper preparation')
            settings['preparation_id'] = preparation_id
        settings = {key: val for key, val in settings.items() if val != []}
        return [Node(package='ur5e_block_bringup', executable='air_task',
                     parameters=[settings], output='screen')]
    settings = {key: val for key, val in settings.items() if val != []}
    return [Node(package='ur5e_a3_grid', executable='stack_blocks', name='ur5e_a3_stack',
                 parameters=[settings], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=str(Path.home()/'ur5e_ws/robot_calibration/ur5e_20261031.yaml')),
        DeclareLaunchArgument('physical_profile', default_value=str(Path.home()/'ur5e_ws/physical_setup.yaml')),
        DeclareLaunchArgument('stack_mode', default_value='scene', choices=['scene', 'plan', 'reach', 'reset', 'dry_plan', 'dry_run', 'alignment_prepare', 'physical_plan', 'physical_prepare', 'physical_run', 'physical_recover', 'physical_unlock']),
        DeclareLaunchArgument('selection_file', default_value=''),
        DeclareLaunchArgument('trajectory_file', default_value=str(Path.home()/'.ros/ur5e_a3_stack/air_plan_domain0.cdr')),
        DeclareLaunchArgument('recipe_manifest_file', default_value=''),
        DeclareLaunchArgument('workspace_clear', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('recovery_empty_confirmed', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('recovery_preserve_gripper', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('allow_mock_hardware', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('execution_mode', default_value='step', choices=['step','continuous']),
        DeclareLaunchArgument('incremental_stack', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('preparation_id', default_value=''),
        OpaqueFunction(function=start),
    ])
