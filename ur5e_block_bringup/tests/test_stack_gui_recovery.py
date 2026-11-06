"""Recovery client gates and terminal reset; fake widgets/services only."""
import json
from pathlib import Path
import queue
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from test_stack_gui_execution_modes import make_gui, prepare_launch, ready, ui


def launch_gui(tmp_path):
    obj = make_gui()
    prepare_launch(obj, tmp_path)
    obj.scan_mode = False
    obj.target = 0  # Recovery must not require a stacking selection or old plan.
    obj.plan_valid = False
    obj.plan_matches_selection = Mock(side_effect=AssertionError('old plan must not be read'))
    obj.plan_manifest_file = tmp_path/'path.gui.json'
    obj.air_trajectory_file = obj.trajectory_file
    obj.physical_trajectory_file = tmp_path/'physical.recipe.json'
    obj.recovery_trajectory_file = tmp_path/'recovery.recipe.json'
    return obj


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_recovery_uses_a_new_review_identity_separate_path_and_selected_mode(monkeypatch, tmp_path, mode):
    obj = launch_gui(tmp_path)
    obj.execution_mode.set(obj.EXECUTION_LABELS[mode])
    old_data = obj.trajectory_file.read_bytes()
    old_metadata = obj.trajectory_file.with_name(obj.trajectory_file.name+'.meta.json')
    metadata_data = old_metadata.read_bytes()
    start = Mock(return_value=SimpleNamespace(stdout=[]))
    modal = Mock(return_value=True)
    monkeypatch.setattr(ui.subprocess, 'Popen', start)
    monkeypatch.setattr(ui.threading, 'Thread', Mock())
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('recovery')
    command = start.call_args.args[0]
    assert 'stack_mode:=physical_recover' in command
    assert 'workspace_clear:=true' in command and 'recovery_preserve_gripper:=true' in command
    assert 'recovery_empty_confirmed:=true' not in command
    assert f'execution_mode:={mode}' in command
    assert f'trajectory_file:={obj.recovery_trajectory_file}' in command
    assert f'recipe_manifest_file:={obj.recovery_trajectory_file}' in command
    assert f'preparation_id:={obj.job_plan_id}' in command and obj.job_plan_id.startswith('recover_')
    assert f'calibration_file:={obj.calibration_file}' in command
    assert f'physical_profile:={obj.physical_profile}' in command
    assert obj.trajectory_file.read_bytes() == old_data and old_metadata.read_bytes() == metadata_data
    obj.invalidate_plan.assert_not_called()
    obj.plan_matches_selection.assert_not_called()
    assert obj.node.clients == [] and obj.step_request is None
    assert '그리퍼를 열거나 초기화하지 않으며' in modal.call_args.args[1]
    assert '상단 OK' in modal.call_args.args[1]
    assert not obj.recovery_verified


def test_recovery_modal_cancel_never_launches_or_changes_old_plans(monkeypatch, tmp_path):
    obj = launch_gui(tmp_path)
    start = Mock()
    monkeypatch.setattr(ui.subprocess, 'Popen', start)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', Mock(return_value=False))
    obj.launch_task('recovery')
    start.assert_not_called()
    obj.invalidate_plan.assert_not_called()
    assert obj.node.clients == [] and obj.job is None


@pytest.mark.parametrize('block', ['job', 'stop_pending', 'close_requested', 'calibration_window'])
def test_recovery_cannot_overlap_an_active_task_or_pending_stop(monkeypatch, tmp_path, block):
    obj = launch_gui(tmp_path)
    setattr(obj, block, True)
    start, modal = Mock(), Mock()
    monkeypatch.setattr(ui.subprocess, 'Popen', start)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('recovery')
    start.assert_not_called()
    modal.assert_not_called()


def test_recovery_still_requires_fresh_controller_readiness_after_modal(monkeypatch, tmp_path):
    obj = launch_gui(tmp_path)
    obj.execution_ready = Mock(side_effect=[True, False])
    start = Mock()
    monkeypatch.setattr(ui.subprocess, 'Popen', start)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', Mock(return_value=True))
    obj.launch_task('recovery')
    start.assert_not_called()
    assert '연결 상태가 바뀌었습니다' in obj.status.get()


def test_recovery_uses_the_existing_single_use_ok_gate():
    obj = make_gui()
    obj.job_action = 'recovery'
    obj.job_plan_id = 'recover_current'
    payload = ready(total=1)
    payload['plan_id'] = obj.job_plan_id
    obj.receive_step_ready(payload)
    client = obj.step_client
    assert client.calls == []
    obj.approve_next_step()
    obj.approve_next_step()
    assert len(client.calls) == 1
    obj.process_step_event('STACK_STEP_EXECUTING '+json.dumps(dict(index=1, total=1, label='복귀')))
    assert obj.step_request is None and not obj.stop_backend_confirmed


def done_record(**changes):
    return dict(plan_id='recover_current', home_reached=True, gripper_empty=True,
                hand_target=.4, scene_cleared=True, **changes)


@pytest.mark.parametrize('field', ['plan_id', 'home_reached', 'gripper_empty', 'scene_cleared', 'hand_target'])
def test_incomplete_or_wrong_recovery_terminal_record_never_resets(field):
    obj = make_gui()
    obj.job_action, obj.job_plan_id = 'recovery', 'recover_current'
    payload = done_record()
    payload[field] = 'other' if field == 'plan_id' else .65 if field == 'hand_target' else False
    obj.receive_recovery_done(payload)
    assert not getattr(obj, 'recovery_verified', False)
    assert obj.job_failed


def poll_gui(tmp_path):
    obj = launch_gui(tmp_path)
    obj.job = SimpleNamespace(poll=lambda: 0, returncode=0)
    obj.job_action, obj.job_plan_id = 'recovery', 'recover_current'
    obj.job_succeeded, obj.job_failed = False, False
    obj.recovery_verified = False
    obj.pending_alignment_plan = False
    obj.stop_future = None
    obj.closing = False
    obj.events = queue.Queue()
    obj.root = SimpleNamespace(after=Mock())
    obj.refresh_connection = obj.refresh_step_approval = obj.refresh_stop = Mock()
    obj.finish_close = Mock()
    obj.launch_task = Mock()
    obj.log = SimpleNamespace(insert=Mock(), index=lambda *args: '1.0', delete=Mock(), see=Mock())
    return obj


def metadata_paths(obj):
    return [path.with_name(path.name+'.meta.json') for path in
            (obj.air_trajectory_file, obj.physical_trajectory_file)] + [
                obj.air_trajectory_file.with_suffix('.gui.json'),
                obj.physical_trajectory_file.with_suffix('.gui.json')]


def test_recovery_done_and_clean_exit_invalidate_both_plan_kinds_then_restore_scene(monkeypatch, tmp_path):
    obj = poll_gui(tmp_path)
    obj.obstacles, obj.empty, obj.target = {10}, {13, 14}, 11
    for path in metadata_paths(obj):
        path.write_text('old reviewed metadata')
    held = tmp_path/'ur5e_physical_held_0.json'
    held.write_text('backend owns this fault file')
    obj.step_seen_requests.add('old')
    obj.step_latest_index, obj.step_total = 3, 8
    obj.events.put(('job', 'STACK_RECOVERY_DONE '+json.dumps(done_record())))
    obj.events.put(('job', 'STACK_STATUS phase=DONE moved=0 total=0 target=0 layers=0'))
    obj.events.put(('finished', 'recovery.log'))
    monkeypatch.setattr(ui.rclpy, 'ok', lambda: False)
    obj.poll()
    assert all(not path.exists() for path in metadata_paths(obj))
    assert held.read_text() == 'backend owns this fault file'
    assert not obj.plan_valid and not obj.step_seen_requests
    assert obj.step_latest_index == 0 and obj.step_total is None and obj.job_plan_id is None
    assert (obj.obstacles, obj.empty, obj.target) == ({10}, {13, 14}, 11)
    assert obj.visible_layout is None
    callbacks = [call.args[1] for call in obj.root.after.call_args_list
                 if getattr(call.args[1], '__name__', '') == '<lambda>']
    assert len(callbacks) == 1
    obj.launch_task.assert_not_called()
    callbacks[0]()
    obj.launch_task.assert_called_once_with('scene')


@pytest.mark.parametrize('fault', [False,True])
def test_preserved_home_completion_keeps_the_tower_and_never_resets_the_scene(monkeypatch,tmp_path,fault):
    obj=poll_gui(tmp_path)
    obj.job_recovery_preserve=True
    obj.visible_layout={'target_cell':19,'obstacle_cells':[10]}
    original=obj.visible_layout.copy()
    payload=dict(plan_id=obj.job_plan_id,policy=ui.HOME_PRESERVE_POLICY,home_reached=True,
                 gripper_preserved=True,gripper_position=.73,scene_preserved=True,held_fault_retained=fault)
    obj.events.put(('job','STACK_RECOVERY_DONE '+json.dumps(payload)))
    obj.events.put(('finished','recovery.log'))
    monkeypatch.setattr(ui.rclpy,'ok',lambda:False)
    obj.poll()
    assert obj.recovery_verified
    assert obj.visible_layout==original
    assert obj.recovery_held_fault_retained==fault
    callbacks=[call.args[1] for call in obj.root.after.call_args_list
               if getattr(call.args[1],'__name__','')=='<lambda>']
    assert callbacks==[]
    obj.launch_task.assert_not_called()
    assert '그리퍼 각도와 탑·블록 장면을 유지' in obj.status.get()


@pytest.mark.parametrize('failure', ['error', 'exit_failure', 'missing_done', 'wrong_done', 'stop', 'closing'])
def test_failed_or_cancelled_recovery_preserves_old_metadata_and_never_restores_scene(monkeypatch, tmp_path, failure):
    obj = poll_gui(tmp_path)
    paths = metadata_paths(obj)
    for path in paths:
        path.write_text('keep until verified recovery')
    if failure == 'exit_failure':
        obj.job = SimpleNamespace(poll=lambda: 1, returncode=1)
    elif failure == 'stop':
        obj.user_stopped = obj.stop_pending = True
    elif failure == 'closing':
        obj.close_requested = True
    if failure != 'missing_done':
        record = done_record()
        if failure == 'wrong_done':
            record['plan_id'] = 'old_recovery'
        obj.events.put(('job', 'STACK_RECOVERY_DONE '+json.dumps(record)))
    obj.events.put(('job', 'STACK_STATUS phase=DONE moved=0 total=0 target=0 layers=0'))
    if failure == 'error':
        obj.events.put(('job', 'STACK_ERROR recovery rejected: gripper not empty'))
    obj.events.put(('finished', 'recovery.log'))
    monkeypatch.setattr(ui.rclpy, 'ok', lambda: False)
    obj.poll()
    assert all(path.read_text() == 'keep until verified recovery' for path in paths)
    assert obj.job is None
    callbacks = [call.args[1] for call in obj.root.after.call_args_list
                 if getattr(call.args[1], '__name__', '') == '<lambda>']
    assert callbacks == []
    obj.launch_task.assert_not_called()


def test_recovery_done_record_cannot_reset_a_different_task():
    obj = make_gui()
    obj.job_action = 'run'
    obj.receive_recovery_done(done_record())
    assert not getattr(obj, 'recovery_verified', False) and not obj.job_failed
