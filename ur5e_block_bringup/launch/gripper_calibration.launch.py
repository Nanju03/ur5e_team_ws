"""Launch headless AG95 calibration with a driver owned by the service node."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ur5e_block_bringup.gripper_calibration_data import validate_settings
from ur5e_block_bringup.model import load_moveit_settings


OPERATIONS = ('monitor', 'baseline', 'grasp', 'open', 'step_close')


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)

    def boolean(name):
        raw = value(name)
        if raw not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return raw == 'true'

    start_driver = boolean('start_driver')
    gripper_only = boolean('gripper_only')
    workspace_clear = boolean('workspace_clear')
    use_mock = boolean('use_mock_hardware')
    allow_mock = boolean('allow_mock_hardware')
    operation = value('startup_operation')
    execution_mode = value('execution_mode')
    if execution_mode not in ('step', 'continuous'):
        raise ValueError('execution_mode must be step or continuous')
    if operation not in OPERATIONS:
        raise ValueError('startup_operation must be monitor, baseline, grasp, open or step_close')
    if operation != 'monitor' and not workspace_clear:
        raise ValueError('실제 초기 보정 동작을 시작하려면 고정·받침·손 제외를 확인하고 workspace_clear:=true로 지정하세요.')
    if start_driver and use_mock != allow_mock:
        raise ValueError('시험용 driver 시작은 use_mock_hardware와 allow_mock_hardware를 모두 true로 지정하세요.')

    settings = validate_settings(**{name: float(value(name)) for name in
                                   ('open_q', 'max_q', 'step_q', 'force_percent', 'settle_s', 'width_mm')})
    paths = {}
    for name in ('calibration_file', 'physical_profile'):
        path = Path(value(name)).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'{name} 파일을 찾을 수 없습니다: {path}')
        paths[name] = str(path.resolve())
    # Validate the same model before the driver can initialize any hardware.
    load_moveit_settings(paths['calibration_file'], paths['physical_profile'])
    device = str(Path(value('com_port')).expanduser())
    if not value('com_port').strip() or not Path(device).is_absolute():
        raise ValueError('com_port는 /dev/robot/dh_ag95_gripper 같은 절대 경로로 지정하세요.')

    actions = [LogInfo(msg='AG95 그리퍼 보정 서비스 · GUI와 팔 노드는 시작하지 않습니다.')]
    if start_driver:
        actions.append(LogInfo(msg=('시험용 GenericSystem driver를 보정 노드의 자식 프로세스로 시작합니다. 실제 장치는 열지 않습니다.'
                                    if use_mock else
                                    '보정 노드가 AG95 driver를 시작하면 초기화 과정에서 손가락이 움직일 수 있습니다.')))
    else:
        actions.append(LogInfo(msg='이미 실행 중인 AG95 driver를 사용합니다. 새 driver는 시작하지 않습니다.'))
    actions.append(LogInfo(msg='종료 시 보정 노드가 그리퍼 정지를 확인한 뒤 자신이 시작한 driver를 종료합니다.'))
    actions.append(Node(package='ur5e_block_bringup', executable='gripper_calibration_node',
        name='gripper_calibration', output='screen', sigterm_timeout='60', sigkill_timeout='60',
        parameters=[dict(settings, **paths, start_driver=start_driver, use_mock_hardware=use_mock,
                         com_port=device, gripper_only=gripper_only, workspace_clear=workspace_clear,
                         allow_mock_hardware=allow_mock, startup_operation=operation,
                         execution_mode=execution_mode)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('com_port', default_value='/dev/robot/dh_ag95_gripper'),
        DeclareLaunchArgument('start_driver', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gripper_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('calibration_file', default_value=str(Path.home()/'ur5e_ws/robot_calibration/ur5e_20261031.yaml')),
        DeclareLaunchArgument('physical_profile', default_value=str(Path.home()/'ur5e_ws/physical_setup.yaml')),
        DeclareLaunchArgument('open_q', default_value='0.50'),
        DeclareLaunchArgument('max_q', default_value='0.78'),
        DeclareLaunchArgument('step_q', default_value='0.01'),
        DeclareLaunchArgument('force_percent', default_value='20'),
        DeclareLaunchArgument('settle_s', default_value='0.4'),
        DeclareLaunchArgument('width_mm', default_value='25'),
        DeclareLaunchArgument('workspace_clear', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('startup_operation', default_value='monitor', choices=list(OPERATIONS)),
        DeclareLaunchArgument('execution_mode', default_value='step', choices=['step', 'continuous']),
        DeclareLaunchArgument('use_mock_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('allow_mock_hardware', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
