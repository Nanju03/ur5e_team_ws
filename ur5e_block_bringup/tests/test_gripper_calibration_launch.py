"""Validate launch wiring with fake launch actions; no ROS process is started."""
import ast
import math
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE/'launch/gripper_calibration.launch.py'


class Action:
    instances = []
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs
        self.instances.append(self)


class DeclareLaunchArgument(Action): pass
class GroupAction(Action): pass
class IncludeLaunchDescription(Action): pass
class LogInfo(Action): pass
class Node(Action): pass
class OpaqueFunction(Action): pass
class SetLaunchConfiguration(Action): pass
class PythonLaunchDescriptionSource(Action): pass
class LaunchDescription(Action): pass


class LaunchConfiguration:
    def __init__(self, name): self.name = name
    def perform(self, context): return context[self.name]


@pytest.fixture
def launch(tmp_path):
    Action.instances = []
    namespace = dict(Path=Path, math=math, LaunchConfiguration=LaunchConfiguration,
                     LaunchDescription=LaunchDescription, DeclareLaunchArgument=DeclareLaunchArgument,
                     GroupAction=GroupAction, IncludeLaunchDescription=IncludeLaunchDescription,
                     LogInfo=LogInfo, Node=Node, OpaqueFunction=OpaqueFunction,
                     SetLaunchConfiguration=SetLaunchConfiguration,
                     PythonLaunchDescriptionSource=PythonLaunchDescriptionSource)
    # Reuse the exact production validator, without importing its geometry/ROS
    # dependencies. No validation or launch behavior is copied into this test.
    data_tree = ast.parse((PACKAGE/'ur5e_block_bringup/gripper_calibration_data.py').read_text())
    validator = next(n for n in data_tree.body if isinstance(n, ast.FunctionDef) and n.name == 'validate_settings')
    tree = ast.parse(LAUNCH.read_text())
    bodies = [n for n in tree.body if isinstance(n, (ast.Assign, ast.FunctionDef))]
    exec(compile(ast.Module(body=[validator]+bodies, type_ignores=[]), '<actual calibration launch>', 'exec'), namespace)
    namespace['model_calls'] = []
    namespace['load_moveit_settings'] = lambda *args: namespace['model_calls'].append(args)
    driver_share = tmp_path/'dh_driver'
    (driver_share/'launch').mkdir(parents=True)
    (driver_share/'launch/dh_ag95_control.launch.py').write_text('# Fake path only; never imported or executed.\n')
    namespace['get_package_share_directory'] = lambda package: str(driver_share)
    declarations = namespace['generate_launch_description']().args[0]
    defaults = {action.args[0]: action.kwargs['default_value'] for action in declarations
                if isinstance(action, DeclareLaunchArgument)}
    for name in ('calibration_file', 'physical_profile'):
        path = tmp_path/(name+'.yaml'); path.write_text('# model validation is isolated below\n')
        defaults[name] = str(path)
    Action.instances = []
    return namespace, defaults


def node(actions):
    values = [item for item in actions if isinstance(item, Node)]
    assert len(values) == 1
    return values[0]


def test_default_launch_starts_only_driver_owning_headless_node(launch):
    module, context = launch
    actions = module['start'](context)
    backend = node(actions)
    assert backend.kwargs['package'] == 'ur5e_block_bringup'
    assert backend.kwargs['executable'] == 'gripper_calibration_node'
    assert backend.kwargs['name'] == 'gripper_calibration'
    params = backend.kwargs['parameters'][0]
    assert params['gripper_only'] is True and params['workspace_clear'] is False
    assert params['startup_operation'] == 'monitor' and params['allow_mock_hardware'] is False
    assert params['execution_mode'] == 'step'
    assert params['open_q'] == .5 and params['max_q'] == .78 and params['step_q'] == .01
    assert params['force_percent'] == 20 and type(params['force_percent']) is int
    assert params['settle_s'] == .4 and params['width_mm'] == 25
    assert params['start_driver'] is True and params['use_mock_hardware'] is False
    assert params['com_port'] == '/dev/robot/dh_ag95_gripper'
    assert not any(isinstance(item, (GroupAction, IncludeLaunchDescription)) for item in Action.instances)
    assert module['model_calls'] == [(context['calibration_file'], context['physical_profile'])]
    assert all(item.kwargs.get('package') not in ('ur_robot_driver', 'rviz2', 'robot_state_publisher')
               for item in Action.instances if isinstance(item, Node))


def test_continuous_launch_preserves_explicit_execution_choice(launch):
    module,context=launch;context['execution_mode']='continuous'
    assert node(module['start'](context)).kwargs['parameters'][0]['execution_mode']=='continuous'


def test_launch_rejects_unknown_execution_choice(launch):
    module,context=launch;context['execution_mode']='automatic'
    with pytest.raises(ValueError,match='execution_mode'):
        module['start'](context)


def test_driver_initialization_and_shutdown_behavior_are_disclosed(launch):
    module, context = launch; actions = module['start'](context)
    messages = [item.kwargs['msg'] for item in actions if isinstance(item, LogInfo)]
    assert any('초기화' in message and '움직일' in message for message in messages)
    assert any('정지를 확인한 뒤' in message and 'driver' in message for message in messages)
    assert node(actions).kwargs['sigterm_timeout'] == '60'
    assert node(actions).kwargs['sigkill_timeout'] == '60'


def test_reuse_existing_driver_adds_no_second_hardware_process(launch):
    module, context = launch; context['start_driver'] = 'false'
    actions = module['start'](context)
    assert not any(isinstance(item, GroupAction) for item in actions)
    assert len([item for item in Action.instances if isinstance(item, IncludeLaunchDescription)]) == 0
    assert node(actions).kwargs['parameters'][0]['startup_operation'] == 'monitor'
    assert node(actions).kwargs['parameters'][0]['start_driver'] is False


@pytest.mark.parametrize('operation', ['baseline', 'grasp', 'open', 'step_close'])
def test_startup_motion_requires_explicit_workspace_confirmation(launch, operation):
    module, context = launch; context['startup_operation'] = operation
    with pytest.raises(ValueError, match='workspace_clear'): module['start'](context)
    assert not any(isinstance(item, (Node, IncludeLaunchDescription)) for item in Action.instances)
    context['workspace_clear'] = 'true'
    assert node(module['start'](context)).kwargs['parameters'][0]['startup_operation'] == operation


@pytest.mark.parametrize('field,value', [
    ('force_percent','20.5'), ('force_percent','nan'), ('force_percent','101'),
    ('max_q','inf'), ('step_q','0'), ('settle_s','0.1'), ('width_mm','-25'),
    ('start_driver','yes'), ('gripper_only','1'), ('workspace_clear','TRUE'),
    ('startup_operation','automatic'), ('com_port','ttyUSB0'),
])
def test_invalid_arguments_fail_before_creating_any_process(launch, field, value):
    module, context = launch; context[field] = value
    with pytest.raises(ValueError): module['start'](context)
    assert not any(isinstance(item, (Node, IncludeLaunchDescription)) for item in Action.instances)


@pytest.mark.parametrize('field', ['calibration_file', 'physical_profile'])
def test_missing_file_fails_before_driver_initialization(launch, field):
    module, context = launch; context[field] += '.missing'
    with pytest.raises(FileNotFoundError): module['start'](context)
    assert not any(isinstance(item, (Node, IncludeLaunchDescription)) for item in Action.instances)


def test_invalid_model_fails_before_driver_initialization(launch):
    module, context = launch
    def rejected_model(*args): raise ValueError('invalid calibration YAML or physical profile')
    module['load_moveit_settings'] = rejected_model
    with pytest.raises(ValueError): module['start'](context)
    assert not any(isinstance(item, (Node, IncludeLaunchDescription)) for item in Action.instances)


@pytest.mark.parametrize('field', ['allow_mock_hardware', 'use_mock_hardware'])
def test_one_mock_flag_cannot_accidentally_start_a_real_driver(launch, field):
    module, context = launch; context[field] = 'true'
    with pytest.raises(ValueError): module['start'](context)
    assert not any(isinstance(item, (Node, IncludeLaunchDescription)) for item in Action.instances)


def test_explicit_mock_pair_is_forwarded_to_driver_and_backend(launch):
    module, context = launch
    context.update(use_mock_hardware='true', allow_mock_hardware='true')
    actions = module['start'](context)
    assert node(actions).kwargs['parameters'][0]['allow_mock_hardware'] is True
    assert node(actions).kwargs['parameters'][0]['use_mock_hardware'] is True


def test_existing_mock_driver_only_needs_backend_mock_permission(launch):
    module, context = launch
    context.update(start_driver='false', allow_mock_hardware='true')
    actions = module['start'](context)
    assert node(actions).kwargs['parameters'][0]['allow_mock_hardware'] is True
    assert not any(isinstance(item, GroupAction) for item in actions)
