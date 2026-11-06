"""Speculation must never bypass committed scene/start/context checks."""
import copy
import time
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
from moveit_msgs.msg import PlanningScene, RobotState
from rclpy.serialization import deserialize_message

from ur5e_block_bringup import physical_incremental as incremental
from ur5e_block_bringup import physical_stack as physical
from ur5e_block_bringup import physical_replay as replay
from ur5e_block_bringup.incremental_state import IncrementalFinished
from test_incremental_stack import layout, loop_task, one_block


def bind_cancel(task):
    task.cancel_prefetch = MethodType(incremental.IncrementalStackTask.cancel_prefetch, task)
    return task


def prediction_task(tmp_path, monkeypatch):
    recipe = one_block()
    recipe['steps'][-1].update(scene_after_sha256='final-scene')
    scene = PlanningScene()
    scene.robot_state.joint_state.name = list(physical.JOINTS)
    scene.robot_state.joint_state.position = [1.]*7  # deliberately stale snapshot state
    task = bind_cancel(SimpleNamespace(path=tmp_path/'current.recipe.json',
        prefetch_job=None, prefetch_enabled=True, execution_mode='continuous',
        read_message=Mock(return_value=scene),
        authorization_context=Mock(return_value={'model': 'same'}),
        get_parameters_by_prefix=Mock(return_value={'planner_workers': SimpleNamespace(value=18)}),
        bounds={name: [-6.28, 6.28] for name in physical.JOINTS},
        joint_velocity_limits={name: 3.14 for name in physical.ARM},
        joint_acceleration_limits={name: 1. for name in physical.ARM}))
    factory = Mock(side_effect=lambda directory, request: SimpleNamespace(
        directory=directory, request=request, close=Mock()))
    monkeypatch.setattr(incremental, 'PrefetchJob', factory)
    monkeypatch.setattr(incremental, 'scene_fingerprint', lambda value: 'final-scene')
    return task, recipe, scene, factory


def test_prediction_uses_sealed_endpoint_and_private_snapshot_without_mutating_live_state(tmp_path, monkeypatch):
    task, recipe, scene, factory = prediction_task(tmp_path, monkeypatch)
    original = copy.deepcopy(scene)
    incremental.IncrementalStackTask.on_recipe_execution_ready(task, {'plan_id': 'current'}, recipe)
    factory.assert_called_once()
    request = task.prefetch_job.request
    predicted = deserialize_message(Path(request['state_file']).read_bytes(), RobotState)
    assert list(predicted.joint_state.position) == recipe['steps'][-1]['end_joints']
    assert predicted.joint_state.position != scene.robot_state.joint_state.position
    assert list(predicted.joint_state.velocity) == [0.]*7
    assert not predicted.is_diff
    assert request['expected_state'] == recipe['incremental_state_after']
    assert request['expected_scene_sha256'] == 'final-scene'
    assert request['planner_parameters']['planner_workers'] == 2
    assert scene == original
    assert Path(request['scene_file']).parent != task.path.parent
    assert not task.path.exists()


@pytest.mark.parametrize('reason', ['step', 'disabled', 'last_block'])
def test_no_speculation_in_step_mode_disabled_mode_or_last_block(tmp_path, monkeypatch, reason):
    task, recipe, _, factory = prediction_task(tmp_path, monkeypatch)
    if reason == 'step':
        task.execution_mode = 'step'
    elif reason == 'disabled':
        task.prefetch_enabled = False
    else:
        recipe['incremental_state_after']['remaining_sources'] = []
    incremental.IncrementalStackTask.on_recipe_execution_ready(task, {}, recipe)
    factory.assert_not_called()
    task.read_message.assert_not_called()


def test_prediction_rejects_changed_final_snapshot_before_launching_worker(tmp_path, monkeypatch):
    task, recipe, _, factory = prediction_task(tmp_path, monkeypatch)
    monkeypatch.setattr(incremental, 'scene_fingerprint', lambda value: 'changed')
    with pytest.raises(RuntimeError, match='예측 장면'):
        incremental.IncrementalStackTask.on_recipe_execution_ready(task, {}, recipe)
    factory.assert_not_called()


def consumption_task(tmp_path, monkeypatch):
    before = layout()
    recipe = one_block(before)
    recipe.update(validated_samples=200, planning_timing={'total_seconds': 4.})
    context = dict(input_sha256={'selection': 'same'}, hardware_class='real', model='same')
    metadata = dict(context, scene_sha256='committed-scene', plan_id='next')
    request = dict(expected_state=copy.deepcopy(before), context=copy.deepcopy(context),
                   expected_scene_sha256='committed-scene')
    job = SimpleNamespace(request=request, directory=tmp_path/'offline', started=time.monotonic(),
        running=Mock(return_value=False), result=Mock(return_value={'status': 'ready'}), close=Mock())
    task = bind_cancel(SimpleNamespace(prefetch_job=job, carrying=False, release_stage=0,
        expected_incremental_state=copy.deepcopy(before), check_stop=Mock(),
        spin_until=Mock(), fresh=Mock(return_value=True), check_model=Mock(),
        hardware=Mock(return_value='real'), gripper_runtime=Mock(),
        input_hashes=Mock(return_value={'selection': 'same'}),
        package_context=lambda inputs, hardware: dict(context, input_sha256=inputs, hardware_class=hardware),
        scene=Mock(return_value='committed-scene'), measured_stop=Mock(return_value=True),
        sensor_fresh=Mock(return_value=True),
        values=dict(zip(physical.JOINTS, recipe['initial_joints'])),
        bounds={name: [-6.28, 6.28] for name in physical.JOINTS},
        joint_velocity_limits={name: 3.14 for name in physical.ARM},
        joint_acceleration_limits={name: 1. for name in physical.ARM},
        acquire_stack_lock=Mock(), library=tmp_path/'library',
        install_session=Mock(return_value=(metadata, recipe)), announce_plan=Mock()))
    request.update({key: copy.deepcopy(getattr(task, key)) for key in
        ('bounds', 'joint_velocity_limits', 'joint_acceleration_limits')})
    monkeypatch.setattr(incremental, 'scene_fingerprint', lambda scene: scene)
    readers = Mock(return_value=(metadata, recipe, {'artifact': b'sealed'}))
    certs = Mock()
    saver = Mock(return_value=task.library/'next')
    hand = Mock(return_value=True)
    monkeypatch.setattr(replay, 'read_package', readers)
    monkeypatch.setattr(replay, 'check_certificates', certs)
    monkeypatch.setattr(replay, 'save_package', saver)
    monkeypatch.setattr(replay, 'initial_hand_matches', hand)
    return task, job, recipe, metadata, certs, saver, hand


def test_consumption_installs_only_verified_committed_recipe_and_preserves_certificates(tmp_path, monkeypatch):
    task, job, recipe, metadata, certs, saver, hand = consumption_task(tmp_path, monkeypatch)
    assert incremental.IncrementalStackTask.consume_prefetch(task) is True
    certs.assert_called_once_with(recipe, metadata)
    task.hardware.assert_called_once_with(require_controller=True)
    task.acquire_stack_lock.assert_called_once()
    saver.assert_called_once()
    task.install_session.assert_called_once()
    assert task.install_session.call_args.kwargs['collision_rechecks'] == 0
    hand.assert_called_once_with(recipe, task.values[physical.HAND], task.bounds[physical.HAND])
    task.announce_plan.assert_called_once()
    job.close.assert_called_once()
    assert task.prefetch_job is None


@pytest.mark.parametrize('mismatch', ['uncommitted', 'carrying', 'release', 'model', 'inputs',
    'hardware', 'scene', 'moving', 'stale_sensor', 'arm_endpoint', 'hand_endpoint',
    'package_context', 'package_scene', 'package_layout', 'certificate',
    'velocity_limit', 'acceleration_limit', 'joint_bounds'])
def test_changed_commit_context_geometry_feedback_or_certificate_cannot_be_consumed(tmp_path, monkeypatch, mismatch):
    task, job, recipe, metadata, certs, saver, hand = consumption_task(tmp_path, monkeypatch)
    if mismatch == 'uncommitted': task.expected_incremental_state = None
    elif mismatch == 'carrying': task.carrying = True
    elif mismatch == 'release': task.release_stage = 1
    elif mismatch == 'model': task.check_model.side_effect = RuntimeError('model changed')
    elif mismatch == 'inputs': task.input_hashes.return_value = {'selection': 'changed'}
    elif mismatch == 'hardware': task.hardware.return_value = 'changed'
    elif mismatch == 'scene': task.scene.return_value = 'changed'
    elif mismatch == 'moving': task.measured_stop.return_value = False
    elif mismatch == 'stale_sensor': task.sensor_fresh.return_value = False
    elif mismatch == 'arm_endpoint': task.values[physical.ARM[0]] += .002
    elif mismatch == 'hand_endpoint': hand.return_value = False
    elif mismatch == 'package_context': metadata['model'] = 'changed'
    elif mismatch == 'package_scene': metadata['scene_sha256'] = 'changed'
    elif mismatch == 'package_layout': recipe['incremental_state_before']['target_blocks'] += [90]
    elif mismatch == 'certificate': certs.side_effect = ValueError('certificate changed')
    elif mismatch == 'velocity_limit': task.joint_velocity_limits[physical.ARM[0]] = 2.
    elif mismatch == 'acceleration_limit': task.joint_acceleration_limits[physical.ARM[0]] = 2.
    elif mismatch == 'joint_bounds': task.bounds[physical.ARM[0]] = [-3.14, 3.14]
    with pytest.raises((RuntimeError, ValueError)):
        incremental.IncrementalStackTask.consume_prefetch(task)
    saver.assert_not_called()
    task.install_session.assert_not_called()
    job.close.assert_called_once()
    assert task.prefetch_job is None


def test_failed_worker_falls_back_without_installing_a_partial_package(tmp_path, monkeypatch):
    task, job, _, _, certs, saver, _ = consumption_task(tmp_path, monkeypatch)
    job.result.side_effect = RuntimeError('offline planner failed')
    assert incremental.IncrementalStackTask.consume_prefetch(task) is False
    certs.assert_not_called()
    saver.assert_not_called()
    task.install_session.assert_not_called()
    job.close.assert_called_once()


def test_waiting_for_unfinished_worker_keeps_processing_stop_and_cancels_owned_job(tmp_path, monkeypatch):
    task, job, _, _, _, saver, _ = consumption_task(tmp_path, monkeypatch)
    job.running.return_value = True
    task.check_stop.side_effect = [None, RuntimeError('operator stop')]
    spin = Mock()
    monkeypatch.setattr(incremental.rclpy, 'spin_once', spin)
    with pytest.raises(RuntimeError, match='operator stop'):
        incremental.IncrementalStackTask.consume_prefetch(task)
    spin.assert_called_once_with(task, timeout_sec=.02)
    job.close.assert_called_once()
    job.result.assert_not_called()
    saver.assert_not_called()


def test_verified_no_feasible_next_pick_ends_session_without_installing_path(tmp_path, monkeypatch):
    task, job, _, _, _, saver, _ = consumption_task(tmp_path, monkeypatch)
    job.result.return_value = dict(status='finished', reason='no_feasible_direct_pick',
                                  state=copy.deepcopy(task.expected_incremental_state))
    with pytest.raises(IncrementalFinished) as result:
        incremental.IncrementalStackTask.consume_prefetch(task)
    assert result.value.reason == 'no_feasible_direct_pick'
    saver.assert_not_called()
    job.close.assert_called_once()


def test_worker_timeout_is_bounded_and_falls_back_only_after_cancel(tmp_path, monkeypatch):
    task, job, _, _, _, saver, _ = consumption_task(tmp_path, monkeypatch)
    job.running.return_value = True
    job.started = time.monotonic()-601.
    assert incremental.IncrementalStackTask.consume_prefetch(task) is False
    job.close.assert_called_once()
    job.result.assert_not_called()
    saver.assert_not_called()
    assert task.prefetch_job is None


def test_failed_current_action_discards_speculation_without_advancing_scene_or_consent(tmp_path, monkeypatch):
    task = loop_task(tmp_path)
    job = SimpleNamespace(close=Mock())
    task.prefetch_job = job
    consent = task.incremental_authorization = SimpleNamespace(advance=Mock())
    monkeypatch.setattr(physical.PhysicalStackTask, 'execute', Mock(side_effect=RuntimeError('arm aborted')))
    with pytest.raises(RuntimeError, match='arm aborted'):
        task.execute()
    assert task.expected_incremental_state is None
    assert not task.incremental_journal.exists()
    consent.advance.assert_not_called()
    job.close.assert_called_once()
    assert task.prefetch_job is None
