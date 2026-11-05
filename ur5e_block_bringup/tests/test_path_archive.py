"""Archive data contracts with real ROS serialization but no ROS node or motion."""
import json
import importlib.util
from pathlib import Path
import zipfile

import pytest
from rclpy.serialization import serialize_message, deserialize_message
from moveit_msgs.msg import PlanningScene, RobotState, RobotTrajectory, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

from ur5e_block_bringup.path_archive import prepare_execution_archive, seal_successful_archive, archive_from_log, sha256


GRID = Path(__file__).resolve().parents[2]/'ur5e_a3_grid'


def prepare(tmp_path, **changes):
    settings = dict(simulation_backend='gazebo', target_cell=14, obstacle_cells=[11],
                    empty_cells=[], stack_mode='run', execution_mode='continuous', use_sim_time=True,
                    vertical_placement_layers=19, placement_approach_clearance=.12)
    settings.update(changes)
    return prepare_execution_archive(settings, {'robot_description': '<robot name="archive_test_only"/>',
         'robot_description_semantic': '<robot name="archive_test_only"/>'}, GRID, root=tmp_path)


def trajectory(value):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ['left_outer_knuckle_joint']
    for stamp, q in ((0, 0.), (1, value)):
        point = JointTrajectoryPoint(positions=[q], velocities=[0.], accelerations=[0.])
        point.time_from_start.sec = stamp
        result.joint_trajectory.points.append(point)
    return result


@pytest.fixture
def recorded(tmp_path):
    folder = prepare(tmp_path)
    (folder/'initial.cdr').write_bytes(serialize_message(PlanningScene()))
    (folder/'initial_state.cdr').write_bytes(serialize_message(RobotState()))
    (folder/'preview.cdr').write_bytes(serialize_message(DisplayTrajectory()))
    steps = []
    journal = []
    for index, kind in ((1, 'hand'), (2, 'scene'), (3, 'hand')):
        step = dict(index=index, kind=kind, file=f'planned_{index}.cdr',
                    scene_before_file=f'before_{index}.cdr', scene_after_file=f'after_{index}.cdr')
        steps.append(step)
        message = PlanningScene() if kind == 'scene' else trajectory(.1)
        (folder/step['file']).write_bytes(serialize_message(message))
        for name in (step['scene_before_file'], step['scene_after_file']):
            (folder/name).write_bytes(serialize_message(PlanningScene()))
        if kind != 'scene':
            sent = dict(event='dispatched', index=index, file=f'command_{index}.cdr', start_state_file=f'start_{index}.cdr')
            done = dict(event='completed', index=index, end_state_file=f'end_{index}.cdr')
            # Actual commands differ from the nominal plan (e.g. contact-adjusted fingers).
            (folder/sent['file']).write_bytes(serialize_message(trajectory(.2)))
            (folder/sent['start_state_file']).write_bytes(serialize_message(RobotState()))
            (folder/done['end_state_file']).write_bytes(serialize_message(RobotState()))
            journal.extend([sent, done])
    (folder/'commands.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in journal))
    recipe = dict(target_cell=14, layers=2, moves=1, steps=steps,
                  initial_scene_file='initial.cdr', initial_state_file='initial_state.cdr')
    (folder/'plan.recipe.json').write_text(json.dumps(recipe))
    result = dict(status='succeeded', target_cell=14, completed_moves=1, layers=2,
                  completed_commands=2, plan_id='synthetic_data_contract_only')
    (folder/'execution_result.json').write_text(json.dumps(result))
    return folder


def modify(folder, name, change):
    value = json.loads((folder/name).read_text())
    change(value)
    (folder/name).write_text(json.dumps(value))


def test_each_run_gets_a_separate_directory_with_effective_settings_and_model_hashes(tmp_path):
    first, second = prepare(tmp_path), prepare(tmp_path)
    assert first != second and first.parent.name == 'obstacle_11_target_14'
    context = json.loads((first/'context.json').read_text())
    assert context['model_sha256'] == sha256(first/'robot.urdf')
    p = yaml.safe_load((first/'execution_parameters.yaml').read_text())['/**']['ros__parameters']
    assert p['stack_mode'] == 'run' and p['use_sim_time'] and p['vertical_placement_layers'] == 19


def test_zip_preserves_exact_dispatched_commands_and_all_scene_artifacts(recorded):
    bundle = seal_successful_archive(recorded)
    with zipfile.ZipFile(bundle) as zipped:
        assert zipped.testzip() is None
        manifest = json.loads(zipped.read('archive.json'))
        assert manifest['status'] == 'succeeded' and manifest['backend'] == 'gazebo'
        assert manifest['command_count'] == 2 and not manifest['replay_implemented']
        for name in manifest['sha256']:
            assert zipped.read(name) == (recorded/name).read_bytes()
        sent = deserialize_message(zipped.read('command_1.cdr'), RobotTrajectory)
        planned = deserialize_message(zipped.read('planned_1.cdr'), RobotTrajectory)
        assert list(sent.joint_trajectory.points[-1].positions) == [.2]
        assert list(planned.joint_trajectory.points[-1].positions) == [.1]


@pytest.mark.parametrize('status', ['planning', 'planned', 'running', 'failed'])
def test_incomplete_runs_never_become_successful_paths(recorded, status):
    modify(recorded, 'execution_result.json', lambda value: value.update(status=status))
    with pytest.raises(ValueError, match='incomplete'):
        seal_successful_archive(recorded)
    assert not (recorded/'successful_path.zip').exists()


@pytest.mark.parametrize('fault', ['missing_completion', 'duplicate', 'wrong_order', 'missing_artifact',
                                  'wrong_target', 'wrong_layers', 'modified_model', 'modified_config'])
def test_damaged_or_inconsistent_recording_is_rejected(recorded, fault):
    if fault in ('missing_completion', 'duplicate', 'wrong_order'):
        rows = (recorded/'commands.jsonl').read_text().splitlines()
        if fault == 'missing_completion': rows.pop()
        elif fault == 'duplicate': rows.append(rows[-1])
        else: rows[0], rows[1] = rows[1], rows[0]
        (recorded/'commands.jsonl').write_text('\n'.join(rows)+'\n')
    elif fault == 'missing_artifact': (recorded/'command_3.cdr').unlink()
    elif fault == 'wrong_target': modify(recorded, 'execution_result.json', lambda value: value.update(target_cell=10))
    elif fault == 'wrong_layers': modify(recorded, 'execution_result.json', lambda value: value.update(layers=23))
    elif fault == 'modified_model': (recorded/'robot.urdf').write_text('changed')
    else: (recorded/'config/gazebo_physics.yaml').write_text('changed')
    with pytest.raises((ValueError, FileNotFoundError)):
        seal_successful_archive(recorded)
    assert not (recorded/'successful_path.zip').exists()


@pytest.mark.parametrize('fault', ['traversal', 'symlink'])
def test_artifacts_cannot_read_outside_the_archive(recorded, fault):
    outside = recorded.parent/'outside.cdr'
    outside.write_bytes(b'not an archive artifact')
    if fault == 'traversal':
        modify(recorded, 'plan.recipe.json', lambda value: value['steps'][0].update(file='../outside.cdr'))
    else:
        (recorded/'command_1.cdr').unlink()
        (recorded/'command_1.cdr').symlink_to(outside)
    with pytest.raises(ValueError):
        seal_successful_archive(recorded)


def test_resealing_is_idempotent_and_modified_success_is_never_overwritten(recorded):
    bundle = seal_successful_archive(recorded)
    original = bundle.read_bytes()
    assert seal_successful_archive(recorded).read_bytes() == original
    (recorded/'command_1.cdr').write_bytes(b'changed')
    with pytest.raises(ValueError, match='refusing to overwrite'):
        seal_successful_archive(recorded)
    assert bundle.read_bytes() == original


def test_old_success_log_without_recorded_trajectories_is_not_fabricated(tmp_path):
    log = tmp_path/'old.log'
    log.write_text('STACK_SUCCESS moved=22 layers=23 target=14\n')
    assert archive_from_log(log) is None


def test_gui_worker_packages_only_the_backend_confirmed_directory(recorded):
    log = recorded.parent/'gui.log'
    log.write_text('STACK_EXECUTION_ARCHIVE '+json.dumps({'directory': str(recorded)})+'\n'
                   'STACK_SUCCESS moved=1 layers=2 target=14\n')
    bundle = archive_from_log(log)
    assert bundle.is_file()
    with zipfile.ZipFile(bundle) as zipped:
        assert zipped.read('execution.log') == log.read_bytes()


def test_error_after_success_prevents_successful_packaging(recorded):
    log = recorded.parent/'gui.log'
    log.write_text('STACK_EXECUTION_ARCHIVE '+json.dumps({'directory': str(recorded)})+'\n'
                   'STACK_SUCCESS moved=1 layers=2 target=14\nSTACK_ERROR save failed\n')
    assert archive_from_log(log) is None


@pytest.mark.parametrize('changes', [dict(simulation_backend='mock'), dict(target_cell=11), dict(obstacle_cells=[True])])
def test_invalid_backend_or_layout_cannot_start_a_recording(tmp_path, changes):
    with pytest.raises(ValueError):
        prepare(tmp_path, **changes)


def test_actual_launch_builds_a_recording_with_the_effective_gazebo_model_without_starting_nodes(tmp_path):
    from launch import LaunchContext
    source = GRID/'launch/stack_blocks.launch.py'
    spec = importlib.util.spec_from_file_location('archive_launch_probe', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    selection = tmp_path/'layout.yaml'
    selection.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'geometry_profile': 'cube_plate_v1', 'target_cell': 14, 'obstacle_cells': [11]}}}))
    context = LaunchContext()
    context.launch_configurations.update(stack_mode='run', execution_mode='continuous',
        selection_file=str(selection), simulation_backend='gazebo', riser_height='0.016',
        moveit_config_package='ur5e_ag95_moveit_config', robot_name='ur5e_on_box',
        use_sim_time='false', path_archive_root=str(tmp_path/'archives'))
    actions = module.start(context)  # Creates launch descriptions; never runs them.
    assert len(actions) == 2
    folders = list((tmp_path/'archives/obstacle_11_target_14').iterdir())
    assert len(folders) == 1
    params = yaml.safe_load((folders[0]/'execution_parameters.yaml').read_text())['/**']['ros__parameters']
    assert params['stack_mode'] == 'run' and params['use_sim_time'] is True
    assert params['execution_mode'] == 'continuous' and params['gripper_grasp_position'] == .78
    assert 'gazebo_ros2_control/GazeboSystem' in params['robot_description']
    assert not (folders[0]/'execution_result.json').exists()
