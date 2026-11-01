"""Inspect actual launch wiring with inert actions; no node/process is started."""
import ast
import hashlib
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class DeclareLaunchArgument(Action): pass
class GroupAction(Action): pass
class IncludeLaunchDescription(Action): pass
class LogInfo(Action): pass
class Node(Action): pass
class OpaqueFunction(Action): pass
class PythonLaunchDescriptionSource(Action): pass
class LaunchDescription(Action): pass


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class IfCondition:
    def __init__(self, expression):
        self.expression = expression

    def evaluate(self, context):
        value = resolve(self.expression, context)
        assert value in ('true', 'false')
        return value == 'true'


def resolve(value, context):
    return value.perform(context) if isinstance(value, LaunchConfiguration) else value


def read_launch(name):
    namespace = dict(Path=Path, yaml=yaml, hashlib=hashlib,
                     LaunchConfiguration=LaunchConfiguration, IfCondition=IfCondition,
                     LaunchDescription=LaunchDescription, DeclareLaunchArgument=DeclareLaunchArgument,
                     GroupAction=GroupAction, IncludeLaunchDescription=IncludeLaunchDescription,
                     LogInfo=LogInfo, Node=Node, OpaqueFunction=OpaqueFunction,
                     PythonLaunchDescriptionSource=PythonLaunchDescriptionSource)
    namespace['get_package_share_directory'] = lambda package: str(PACKAGE.parent/package)
    namespace['guard_calls'] = []
    namespace['guard_hardware_launch'] = lambda **kwargs: namespace['guard_calls'].append(kwargs)
    # Evaluate production functions using inert constructor replacements. In
    # particular, IncludeLaunchDescription never imports a real driver launch.
    tree = ast.parse((PACKAGE/'launch'/name).read_text())
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)]
    exec(compile(ast.Module(body=functions, type_ignores=[]), name, 'exec'), namespace)
    namespace['model_calls'], namespace['gripper_profile_calls'] = [], []

    def model(*args):
        namespace['model_calls'].append(args)
        return {'robot_description': '<robot name="inert_fixture"/>'}

    def profile(**kwargs):
        namespace['gripper_profile_calls'].append(kwargs)
        return None

    namespace['load_moveit_settings'] = model
    namespace['load_calibration_profile'] = profile
    return namespace


def defaults(actions):
    return {item.args[0]: item.kwargs['default_value'] for item in actions
            if isinstance(item, DeclareLaunchArgument)}


def selected(actions, context):
    result = []
    for item in actions:
        condition = item.kwargs.get('condition')
        if condition is not None and not condition.evaluate(context):
            continue
        if isinstance(item, GroupAction):
            result.extend(selected(item.args[0], context))
        else:
            result.append(item)
    return result


@pytest.fixture
def real_launch():
    module = read_launch('real_robot.launch.py')
    actions = module['generate_launch_description']().args[0]
    return module, defaults(actions)


def driver_sources(actions):
    return [Path(item.args[0].args[0]).name for item in actions
            if isinstance(item, IncludeLaunchDescription)]


def test_existing_real_entrypoint_keeps_both_driver_defaults_enabled(real_launch):
    module, context = real_launch
    assert context['start_arm'] == context['start_gripper'] == 'true'
    assert context['allow_execution'] == 'false'
    actions = selected(module['start'](context), context)
    assert driver_sources(actions) == ['ur_control.launch.py', 'dh_ag95_control.launch.py']
    assert module['guard_calls'] == [dict(start_arm=True, start_gripper=True)]


@pytest.mark.parametrize('arm,gripper', [('true', 'true'), ('false', 'true'), ('true', 'false')])
def test_duplicate_guard_runs_before_any_model_profile_or_child_creation(real_launch, arm, gripper):
    module, context = real_launch
    context.update(start_arm=arm, start_gripper=gripper)
    def duplicate(**kwargs):
        assert kwargs == dict(start_arm=arm == 'true', start_gripper=gripper == 'true')
        raise RuntimeError('Existing hardware connection')
    module['guard_hardware_launch'] = duplicate
    with pytest.raises(RuntimeError, match='Existing hardware connection'):
        module['start'](context)
    assert module['model_calls'] == module['gripper_profile_calls'] == []


@pytest.mark.parametrize('arm,gripper,expected', [
    ('false', 'false', []),
    ('false', 'true', ['dh_ag95_control.launch.py']),
    ('true', 'false', ['ur_control.launch.py']),
])
def test_driver_gates_are_independent_and_do_not_gate_the_planning_model(
        real_launch, arm, gripper, expected):
    module, context = real_launch
    context.update(start_arm=arm, start_gripper=gripper)
    actions = selected(module['start'](context), context)
    assert driver_sources(actions) == expected
    nodes = [item for item in actions if isinstance(item, Node)]
    assert {item.kwargs['executable'] for item in nodes} == {
        'robot_state_publisher', 'move_group', 'rviz2'}
    rviz = next(item for item in nodes if item.kwargs['executable'] == 'rviz2')
    assert Path(rviz.kwargs['arguments'][1]) == PACKAGE/'config/plate_view.rviz'
    assert not any(item.kwargs['executable'] in ('air_task', 'stack_blocks', 'gripper_calibration_node')
                   for item in nodes)


def test_disabled_gripper_group_does_not_emit_initialization_notice(real_launch):
    module, context = real_launch
    context.update(start_arm='false', start_gripper='false')
    messages = [item.kwargs['msg'] for item in selected(module['start'](context), context)
                if isinstance(item, LogInfo)]
    assert all('initializes the gripper' not in message for message in messages)


def test_no_driver_mode_still_shares_the_exact_model_and_execution_gate(real_launch):
    module, context = real_launch
    context.update(start_arm='false', start_gripper='false', use_rviz='false', allow_execution='true')
    actions = selected(module['start'](context), context)
    nodes = {item.kwargs['executable']: item for item in actions if isinstance(item, Node)}
    assert set(nodes) == {'robot_state_publisher', 'move_group'}
    rsp = nodes['robot_state_publisher'].kwargs['parameters'][0]
    moveit = nodes['move_group'].kwargs['parameters'][0]
    assert rsp['robot_description'] == moveit['robot_description'] == '<robot name="inert_fixture"/>'
    assert moveit['allow_trajectory_execution'] is True
    assert not driver_sources(actions)


@pytest.mark.parametrize('execution,rviz', [('false', 'true'), ('true', 'false')])
def test_planning_only_entrypoint_forces_no_hardware_even_with_outer_true_values(
        real_launch, execution, rviz):
    real, real_context = real_launch
    wrapper = read_launch('planning_only.launch.py')
    actions = wrapper['generate_launch_description']().args[0]
    context = defaults(actions)
    assert context['allow_execution'] == 'false'
    assert not {'start_arm', 'start_gripper'} & context.keys()
    context.update(start_arm='true', start_gripper='true', allow_execution=execution, use_rviz=rviz)
    include = [item for item in actions if isinstance(item, IncludeLaunchDescription)]
    assert len(include) == 1
    assert Path(include[0].args[0].args[0]).name == 'real_robot.launch.py'
    forwarded = {name: resolve(value, context)
                 for name, value in dict(include[0].kwargs['launch_arguments']).items()}
    assert forwarded['start_arm'] == forwarded['start_gripper'] == 'false'
    assert forwarded['calibration_file'] == context['calibration_file']
    assert forwarded['physical_profile'] == context['physical_profile']
    nested_context = {**real_context, **forwarded}
    nested = selected(real['start'](nested_context), nested_context)
    assert not driver_sources(nested)
    nodes = [item for item in nested if isinstance(item, Node)]
    assert {item.kwargs['executable'] for item in nodes} == (
        {'robot_state_publisher', 'move_group', 'rviz2'} if rviz == 'true'
        else {'robot_state_publisher', 'move_group'})
    moveit = next(item for item in nodes if item.kwargs['executable'] == 'move_group')
    assert moveit.kwargs['parameters'][0]['allow_trajectory_execution'] == (execution == 'true')


def test_planning_only_can_skip_stale_gripper_profile_without_changing_hardware(real_launch):
    module, context = real_launch
    context.update(start_arm='false', start_gripper='false', use_gripper_calibration='false')
    actions = selected(module['start'](context), context)
    assert module['gripper_profile_calls'] == []
    assert not driver_sources(actions)


def test_view_only_creates_rviz_without_any_server_or_driver_action():
    module = read_launch('view_only.launch.py')
    declarations = module['generate_launch_description']().args[0]
    context = defaults(declarations)
    assert set(context) == {'calibration_file', 'physical_profile'}
    context.update(start_arm='true', start_gripper='true')
    description, semantic = '<robot name="same_model"/>', '<robot name="same_semantic"/>'
    calls = []
    def actual_model(*args):
        calls.append(args)
        return {'robot_description': description, 'robot_description_semantic': semantic}
    module['load_moveit_settings'] = actual_model
    actions = module['start'](context)
    assert len(actions) == 1 and isinstance(actions[0], Node)
    rviz = actions[0]
    assert rviz.kwargs['package'] == rviz.kwargs['executable'] == 'rviz2'
    assert rviz.kwargs['arguments'][0] == '-d'
    assert Path(rviz.kwargs['arguments'][1]) == PACKAGE/'config/plate_view.rviz'
    params = rviz.kwargs['parameters'][0]
    assert params['robot_description'] == description
    assert params['robot_description_semantic'] == semantic
    assert params['use_sim_time'] is False
    assert params['moveit_simple_controller_manager']['controller_names'] == [
        'scaled_joint_trajectory_controller', 'gripper/dh_ag95_gripper_controller']
    assert calls == [(context['calibration_file'], context['physical_profile'])]
    assert not driver_sources(actions)
    assert module['gripper_profile_calls'] == []


def test_view_only_propagates_model_validation_failure_before_node_creation():
    module = read_launch('view_only.launch.py')
    context = defaults(module['generate_launch_description']().args[0])
    def invalid_model(*args):
        raise ValueError('profile does not match the source model')
    module['load_moveit_settings'] = invalid_model
    with pytest.raises(ValueError, match='profile does not match'):
        module['start'](context)


def test_plate_viewer_exposes_only_scene_path_and_camera_controls():
    config = yaml.safe_load((PACKAGE/'config/plate_view.rviz').read_text())
    assert {panel['Class'] for panel in config['Panels']} == {
        'rviz_common/Displays', 'rviz_common/Views'}
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'world'
    displays = {display['Class']: display for display in manager['Displays']}
    assert set(displays) == {'rviz_default_plugins/Grid', 'moveit_rviz_plugin/PlanningScene',
                             'moveit_rviz_plugin/Trajectory', 'rviz_default_plugins/MarkerArray'}
    # Removing MotionPlanning prevents its Plan/Execute panel and interactive
    # end-effector controls from bypassing the project's explicit OK workflow.
    assert {tool['Class'] for tool in manager['Tools']} == {
        'rviz_default_plugins/MoveCamera', 'rviz_default_plugins/Select'}
    scene = displays['moveit_rviz_plugin/PlanningScene']
    assert scene['Robot Description'] == 'robot_description'
    assert scene['Planning Scene Topic'] == '/monitored_planning_scene'
    assert scene['Scene Geometry'] == {'Show Scene Geometry': True, 'Scene Alpha': 1.0}
    assert scene['Scene Robot'] == {'Show Robot Visual': True, 'Show Robot Collision': False,
                                    'Robot Alpha': 1.0}
    path = displays['moveit_rviz_plugin/Trajectory']
    assert 'Planned Path' not in path  # Standalone Trajectory properties are direct children.
    assert path['Robot Description'] == 'robot_description'
    assert path['Trajectory Topic'] == '/display_planned_path'
    assert path['State Display Time'] == '0.05 s'
    assert path['Robot Alpha'] == .45
    assert path['Loop Animation'] is path['Show Trail'] is path['Interrupt Display'] is False
    assert displays['rviz_default_plugins/MarkerArray']['Topic']['Value'] == '/ur5e_a3_stack/markers'
    view = manager['Views']['Current']
    assert view['Class'] == 'rviz_default_plugins/Orbit' and view['Distance'] == 1.8
    assert view['Focal Point'] == {'X': .3, 'Y': 0, 'Z': 1.0}
    assert view['Pitch'] == .65 and view['Yaw'] == -.8
    assert config['Window Geometry'] == {'Height': 900, 'Width': 1200}


def test_package_installs_the_read_only_rviz_config(monkeypatch):
    import runpy
    import sys
    import types
    installed = {}
    setuptools = types.ModuleType('setuptools')
    setuptools.find_packages = lambda: []
    setuptools.setup = lambda **kwargs: installed.update(kwargs)
    monkeypatch.setitem(sys.modules, 'setuptools', setuptools)
    monkeypatch.chdir(PACKAGE)
    runpy.run_path(str(PACKAGE/'setup.py'))
    config_files = dict(installed['data_files'])['share/ur5e_block_bringup/config']
    assert 'config/plate_view.rviz' in config_files
