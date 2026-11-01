"""Approval callbacks with fake ROS clients and widgets; never start a node."""
from concurrent.futures import Future
import importlib.util
import json
import queue
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).resolve().parents[2] / 'ur5e_a3_grid/scripts/stack_gui.py'
SPEC = importlib.util.spec_from_file_location('stack_execution_ui', SOURCE)
ui = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ui)


class Var:
    def __init__(self, value=''):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class Widget:
    def __init__(self):
        self.options = {}

    def configure(self, **values):
        self.options.update(values)


class Client:
    def __init__(self, name):
        self.name, self.ready, self.calls = name, True, []
        self.future = Future()
        self.error = None

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        if self.error:
            raise self.error
        return self.future


class Node:
    def __init__(self):
        self.clients, self.destroyed = [], []

    def create_client(self, kind, name):
        result = Client(name)
        self.clients.append(result)
        return result

    def destroy_client(self, client):
        self.destroyed.append(client)


def make_gui(real=True):
    obj = ui.StackGui.__new__(ui.StackGui)
    obj.real_backend = real
    obj.simulation_riser_height = 0.0
    obj.node = Node()
    obj.execution_mode = Var(ui.StackGui.EXECUTION_LABELS['step'])
    obj.step_status, obj.status = Var(), Var()
    obj.step_request = obj.step_client = obj.step_future = obj.step_future_request_id = None
    obj.step_approval_locked = False
    obj.step_seen_requests = set()
    obj.step_latest_index, obj.step_total = 0, None
    obj.job_plan_id = 'abc123' if real else None
    obj.job, obj.job_action = object(), 'run'
    obj.job_failed = obj.job_succeeded = False
    obj.stop_pending = obj.user_stopped = obj.close_requested = False
    obj.stop_backend_confirmed = obj.stop_acknowledged = False
    obj.approve_button = Widget()
    obj.execution_ready = lambda: True
    return obj


def ready(index=1, total=2, **changes):
    token = f'token_{index}'
    return dict(plan_id='abc123', request_id=token, index=index, total=total,
                label=f'XY {index}번 칸', duration=3.5,
                approve_service='/ur5e_a3_stack/approve_next_' + token,
                preview_sha256='a' * 64, **changes)


def event(name, payload):
    return '[air_task-1] STACK_STEP_' + name + ' ' + json.dumps(payload) + '\n'


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_execution_mode_labels_map_without_planning_changes(mode):
    obj = make_gui()
    obj.execution_mode.set(obj.EXECUTION_LABELS[mode])
    assert obj.selected_execution_mode() == mode
    assert obj.node.clients == []


def test_unknown_mode_is_rejected():
    obj = make_gui()
    obj.execution_mode.set('unknown')
    with pytest.raises(ValueError):
        obj.selected_execution_mode()


def test_gripper_preparation_uses_the_same_unique_reviewed_approval_gate():
    obj = make_gui()
    obj.job_action = 'alignment_gripper'
    obj.receive_step_ready(ready(total=1))
    assert obj.step_request is not None
    assert obj.node.clients[0].calls == []
    obj.approve_next_step()
    assert len(obj.node.clients[0].calls) == 1
    obj.approve_next_step()
    assert len(obj.node.clients[0].calls) == 1


@pytest.mark.parametrize('corner,axis,pocket,stop_index', [
    ('A', 'X', 1, 1), ('A', 'Y', 1, 2), ('B', 'X', 4, 3), ('B', 'Y', 4, 4),
    ('C', 'X', 24, 5), ('C', 'Y', 24, 6), ('D', 'X', 21, 7), ('D', 'Y', 21, 8)])
def test_each_xy_alignment_hold_shows_its_direction_and_requires_new_ok(corner, axis, pocket, stop_index):
    obj = make_gui()
    obj.plate_alignment = True
    obj.step_latest_index, obj.step_total = 1, 2
    record = dict(corner=corner, axis=axis, pocket_cell=pocket, stop_index=stop_index, total=8)
    assert obj.process_step_event('STACK_ALIGNMENT_PAUSED '+json.dumps(record))
    assert obj.alignment_paused
    obj.receive_step_ready(ready(index=2, total=2))
    assert '조정 완료' in obj.approve_button.options['text']
    assert '판을 맞추고 손을 뺀 뒤' in obj.status.get()
    assert axis+'방향' in obj.status.get()
    assert '모서리 홈 '+corner in obj.status.get()
    assert obj.node.clients[-1].calls == []
    obj.approve_next_step()
    assert len(obj.node.clients[-1].calls) == 1
    obj.process_step_event(event('EXECUTING', dict(index=2, total=2, label='나머지 홈 이동')))
    assert not obj.alignment_paused
    assert obj.approve_button.options['text'] == '확인 OK · 표시한 경로 실행'


@pytest.mark.parametrize('state', ['ordinary', 'stopped', 'closing'])
def test_alignment_pause_output_cannot_reenable_unrelated_or_stopped_work(state):
    obj = make_gui()
    obj.plate_alignment = state != 'ordinary'
    obj.user_stopped = state == 'stopped'
    obj.close_requested = state == 'closing'
    obj.process_step_event('STACK_ALIGNMENT_PAUSED '+json.dumps(
        dict(corner='A', axis='X', pocket_cell=1, stop_index=1, total=8)))
    assert not getattr(obj, 'alignment_paused', False)
    assert obj.node.clients == []


@pytest.mark.parametrize('record', [None, {}, {'corner': ''},
    dict(corner='A', axis='Y', pocket_cell=1, stop_index=1, total=8),
    dict(corner='B', axis='X', pocket_cell=4, stop_index=3, total=2),
    dict(corner='A', axis='Z', pocket_cell=1, stop_index=1, total=8)])
def test_invalid_xy_hold_information_cannot_enable_an_approval(record):
    obj = make_gui()
    obj.plate_alignment = True
    obj.process_step_event('STACK_ALIGNMENT_PAUSED '+json.dumps(record))
    assert not getattr(obj, 'alignment_paused', False)
    assert obj.step_request is None
    assert obj.node.clients == []
    obj.approve_next_step()
    assert obj.node.clients == []


def test_ready_displays_current_step_but_never_approves_automatically():
    obj = make_gui()
    assert obj.process_step_event(event('READY', ready()))
    assert obj.step_request['index'] == 1
    assert obj.approve_button.options['state'] == 'normal'
    assert obj.node.clients[0].calls == []
    assert '1/2' in obj.step_status.get()


def test_one_click_dispatches_once_and_locks_double_clicks():
    obj = make_gui()
    obj.receive_step_ready(ready())
    client = obj.step_client
    obj.approve_next_step()
    obj.approve_next_step()
    assert len(client.calls) == 1
    assert obj.approve_button.options['state'] == 'disabled'
    client.future.set_result(SimpleNamespace(success=True, message='approved'))
    obj.refresh_step_approval()
    assert obj.step_request is None
    obj.receive_step_ready(ready())
    assert len(obj.node.clients) == 1
    assert obj.step_request is None


def test_ack_only_unblocks_matching_request():
    obj = make_gui()
    obj.receive_step_ready(ready())
    obj.receive_step_approved(dict(request_id='old_request'))
    assert obj.step_request is not None
    obj.receive_step_approved(dict(request_id='token_1'))
    assert obj.step_request is None
    assert obj.approve_button.options['state'] == 'disabled'


def test_each_new_step_needs_a_new_click_and_stale_ack_is_ignored():
    obj = make_gui()
    obj.receive_step_ready(ready())
    obj.approve_next_step()
    old_client = obj.step_client
    obj.receive_step_ready(ready(index=2))
    new_client = obj.step_client
    obj.receive_step_approved(dict(request_id='token_1'))
    assert obj.step_request['request_id'] == 'token_2'
    assert new_client.calls == []
    assert old_client.future.cancelled()
    obj.approve_next_step()
    assert len(new_client.calls) == 1


@pytest.mark.parametrize('flag', ['stop_pending', 'user_stopped', 'close_requested'])
def test_stop_or_close_blocks_approval_and_new_ready(flag):
    obj = make_gui()
    obj.receive_step_ready(ready())
    client = obj.step_client
    setattr(obj, flag, True)
    obj.approve_next_step()
    assert client.calls == []
    obj.receive_step_ready(ready(index=2))
    assert len(obj.node.clients) == 1
    assert obj.approve_button.options['state'] == 'disabled'


def test_lost_controller_readiness_blocks_click():
    obj = make_gui()
    obj.receive_step_ready(ready())
    obj.execution_ready = lambda: False
    obj.approve_next_step()
    assert obj.step_client.calls == []
    assert obj.approve_button.options['state'] == 'disabled'


@pytest.mark.parametrize('kind', ['send', 'result'])
def test_transport_failure_never_retries_the_same_approval(kind):
    obj = make_gui()
    obj.receive_step_ready(ready())
    client = obj.step_client
    if kind == 'send':
        client.error = RuntimeError('transport uncertain')
    obj.approve_next_step()
    if kind == 'result':
        client.future.set_exception(RuntimeError('result uncertain'))
        obj.refresh_step_approval()
        obj.refresh_step_approval()
    obj.approve_next_step()
    assert len(client.calls) == 1
    assert obj.approve_button.options['state'] == 'disabled'


def test_approval_timeout_keeps_the_same_request_locked(monkeypatch):
    obj = make_gui()
    obj.receive_step_ready(ready())
    obj.approve_next_step()
    client = obj.step_client
    obj.step_approval_sent_at = 0
    monkeypatch.setattr(ui.time, 'monotonic', lambda: 6)
    obj.refresh_step_approval()
    obj.receive_step_ready(ready())
    obj.approve_next_step()
    assert len(client.calls) == 1
    assert not client.future.done()
    assert obj.approve_button.options['state'] == 'disabled'
    assert '중복 승인' in obj.step_status.get()


def test_rejected_approval_is_not_reported_as_running():
    obj = make_gui()
    obj.receive_step_ready(ready())
    client = obj.step_client
    obj.approve_next_step()
    client.future.set_result(SimpleNamespace(success=False, message='old preview'))
    obj.refresh_step_approval()
    assert obj.step_request is None
    assert '거절' in obj.status.get()
    assert obj.approve_button.options['state'] == 'disabled'


@pytest.mark.parametrize('changes', [
    dict(plan_id='other_plan'), dict(index=2), dict(index=True),
    dict(duration=float('nan')), dict(duration=0), dict(duration=True),
    dict(approve_service='/some_other_motion'), dict(preview_sha256='bad'),
    dict(label=''), dict(request_id='../bad'),
])
def test_malformed_or_stale_ready_never_creates_a_client(changes):
    obj = make_gui()
    payload = ready()
    payload.update(changes)
    obj.receive_step_ready(payload)
    assert obj.node.clients == []
    assert obj.step_request is None
    assert obj.approve_button.options['state'] == 'disabled'


def test_real_requires_preview_hash_and_sealed_plan_identity():
    obj = make_gui()
    payload = ready()
    payload.pop('preview_sha256')
    obj.receive_step_ready(payload)
    assert obj.node.clients == []
    obj.job_plan_id = None
    obj.receive_step_ready(ready())
    assert obj.node.clients == []


def test_simulation_can_bind_its_first_run_plan_without_optional_hash():
    obj = make_gui(real=False)
    payload = ready()
    payload.pop('preview_sha256')
    obj.receive_step_ready(payload)
    assert obj.job_plan_id == 'abc123'
    assert obj.step_request is not None
    assert obj.step_client.calls == []


def test_continuous_mode_still_waits_for_one_whole_path_approval():
    obj = make_gui()
    obj.execution_mode.set(obj.EXECUTION_LABELS['continuous'])
    payload = ready(total=1)
    payload['label'] = '전체 경로 · 26개 구간'
    obj.receive_step_ready(payload)
    assert obj.step_client.calls == []
    assert obj.step_request['total'] == 1
    obj.approve_next_step()
    assert len(obj.step_client.calls) == 1


def test_new_executing_segment_cannot_reuse_previous_stop_confirmation():
    obj = make_gui()
    obj.receive_step_ready(ready())
    obj.stop_backend_confirmed = obj.stop_acknowledged = True
    obj.process_step_event(event('EXECUTING', dict(index=1, total=2, label='raise')))
    assert not obj.stop_backend_confirmed
    assert not obj.stop_acknowledged
    assert obj.step_request is None
    assert obj.approve_button.options['state'] == 'disabled'


@pytest.mark.parametrize('stopping', [False, True])
@pytest.mark.parametrize('kind', ['air', 'physical'])
def test_every_new_motion_invalidates_stop_evidence_even_without_matching_ready(stopping, kind):
    obj = make_gui()
    obj.plan_kind = kind
    obj.execution_mode.set(obj.EXECUTION_LABELS['continuous'])
    obj.receive_step_ready(ready(total=1))
    obj.stop_backend_confirmed = obj.stop_acknowledged = True
    obj.user_stopped = obj.stop_pending = stopping
    # A missing/new progress index cannot make the old terminal proof apply to
    # a later goal. It still does not approve or clear the existing request.
    obj.process_step_event(event('EXECUTING', dict(index=2, total=2, label='next goal')))
    assert not obj.stop_backend_confirmed and not obj.stop_acknowledged
    assert obj.step_request['index'] == 1
    assert obj.step_client.calls == []


def test_normal_motion_settled_log_does_not_claim_a_stop_or_change_the_status(monkeypatch):
    obj = make_gui()
    obj.plan_kind = 'physical'
    obj.closing = False
    obj.calibration_window = obj.stop_future = None
    obj.refresh_connection = obj.refresh_step_approval = obj.refresh_stop = Mock()
    obj.log = SimpleNamespace(insert=Mock(), index=lambda *args: '1.0', delete=Mock(), see=Mock())
    obj.root = SimpleNamespace(after=Mock())
    obj.events = queue.Queue()
    obj.events.put(('job', 'STACK_MOTION_SETTLED physical motion terminal and standstill'))
    monkeypatch.setattr(ui.rclpy, 'ok', lambda: False)
    obj.status.set('승인한 경로 실행 중')
    obj.poll()
    assert not obj.stop_backend_confirmed and not obj.stop_acknowledged
    assert obj.status.get() == '승인한 경로 실행 중'


@pytest.mark.parametrize('state', ['planning', 'failed', 'stopped', 'stop_pending', 'other_task'])
def test_physical_validation_progress_updates_only_active_plan_status(monkeypatch, state):
    obj = make_gui()
    obj.closing = False
    obj.calibration_window = obj.stop_future = None
    obj.job_action = 'physical_plan' if state != 'other_task' else 'run'
    obj.job_failed = state == 'failed'
    obj.user_stopped = state == 'stopped'
    obj.stop_pending = state == 'stop_pending'
    obj.refresh_connection = obj.refresh_step_approval = obj.refresh_stop = Mock()
    obj.log = SimpleNamespace(insert=Mock(), index=lambda *args: '1.0', delete=Mock(), see=Mock())
    obj.root = SimpleNamespace(after=Mock())
    obj.events = queue.Queue()
    obj.events.put(('job', '[physical_stack-1] STACK_PHYSICAL_VALIDATING step=10 total=82 samples=1200'))
    monkeypatch.setattr(ui.rclpy, 'ok', lambda: False)
    obj.status.set('이전 상태 또는 오류 원인')
    obj.poll()
    if state == 'planning':
        assert obj.status.get() == '실물 경로 충돌 검사: 10/82단계 · 1200개 상태 검사 · 로봇은 움직이지 않습니다.'
    else:
        assert obj.status.get() == '이전 상태 또는 오류 원인'
    assert obj.node.clients == []
    assert not obj.job_succeeded


@pytest.mark.parametrize('same_poll', [False, True])
@pytest.mark.parametrize('code', ['MONITOR_RPC_TIMEOUT', 'MONITOR_PREWARM_TIMEOUT', 'OTHER_FAILURE'])
def test_failure_cause_survives_stop_proof_process_exit_and_actual_quiet(monkeypatch, same_poll, code):
    obj = make_gui()
    obj.plan_kind = 'physical'
    obj.closing = False
    obj.calibration_window = obj.stop_future = None
    obj.refresh_connection = obj.refresh_step_approval = obj.refresh_stop = Mock()
    obj.busy = Mock()
    obj.job = SimpleNamespace(poll=lambda: 1, returncode=1)
    obj.log = SimpleNamespace(insert=Mock(), index=lambda *args: '1.0', delete=Mock(), see=Mock())
    obj.root = SimpleNamespace(after=Mock())
    obj.events = queue.Queue()
    cause = 'Physical pick/place stopped. Reason: ' + json.dumps(
        dict(code=code, service='/get_planning_scene', elapsed=2.016))
    monkeypatch.setattr(ui.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(ui.time, 'monotonic', lambda: 10.)
    obj.events.put(('job', 'STACK_ERROR ' + cause))
    if not same_poll:
        obj.poll()
        assert cause in obj.status.get()
    obj.events.put(('job', 'STACK_STOPPED_CONFIRMED'))
    if not same_poll:
        obj.poll()
        assert cause in obj.status.get()
        assert obj.stop_backend_confirmed and obj.stop_acknowledged
    obj.events.put(('finished', 'fixture.log'))
    obj.poll()
    assert obj.last_failure_message.endswith(cause)
    if code.startswith('MONITOR_'):
        assert '응답' in obj.last_failure_message and '중지했습니다' in obj.last_failure_message
    assert cause in obj.status.get()
    assert obj.job is None and obj.stop_pending
    assert obj.stop_backend_confirmed and obj.stop_acknowledged

    # Remote terminal proof and GUI-measured standstill still release the
    # controls, while the failure remains visible for a new plan.
    monkeypatch.setattr(ui.time, 'monotonic', lambda: 10.5)
    obj.stop_stationary_since = 10.1
    obj.velocity_times = {name: 10.4 for name in obj.ARM_JOINTS}
    obj.joint_velocities = {name: 0. for name in obj.ARM_JOINTS}
    ui.StackGui.refresh_stop(obj)
    assert not obj.stop_pending
    assert cause in obj.status.get() and '정지 확인' in obj.status.get()
    obj.busy.assert_called_with(False)
    assert obj.node.clients == []


def test_unrelated_output_and_wrong_progress_do_not_approve_or_clear_current_step():
    obj = make_gui()
    obj.receive_step_ready(ready())
    assert not obj.process_step_event('ordinary ROS line')
    assert obj.process_step_event(event('COMPLETED', dict(index=2, total=2, label='wrong')))
    assert obj.step_request['index'] == 1
    assert obj.step_client.calls == []


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_embedded_gripper_window_receives_the_same_selected_mode(monkeypatch, mode):
    obj = make_gui()
    obj.execution_mode.set(obj.EXECUTION_LABELS[mode])
    obj.job = obj.calibration_window = obj.stop_future = None
    obj.closing = False
    obj.model_error = ''
    obj.root = object()
    obj.calibration_file, obj.physical_profile = '/tmp/calibration', '/tmp/profile'
    obj.invalidate_plan, obj.busy = Mock(), Mock()
    factory = Mock(return_value=object())
    fake_module = SimpleNamespace(GripperCalibrationWindow=factory)
    monkeypatch.setitem(sys.modules, 'ur5e_block_bringup.gripper_calibration_gui', fake_module)
    obj.open_gripper_calibration()
    assert factory.call_args.kwargs['execution_mode'] == mode
    obj.busy.assert_called_once_with(True)


def prepare_launch(obj, tmp_path):
    obj.job = obj.calibration_window = None
    obj.model_error = ''
    obj.scene_client = SimpleNamespace(service_is_ready=lambda: True)
    obj.scan_mode = obj.real_backend
    obj.target, obj.obstacles = 21, {7}
    obj.root = object()
    obj.plan_valid = True
    obj.plan_matches_selection = lambda: True
    obj.save_selection = Mock()
    obj.invalidate_plan, obj.busy = Mock(), Mock()
    obj.folder = tmp_path
    obj.selection_file = tmp_path / 'layout.yaml'
    obj.selection_file.write_text('/**:\n  ros__parameters:\n    center_xyz: [0.5, 0.1, 1.1]\n')
    obj.trajectory_file = tmp_path / 'path.cdr'
    obj.trajectory_file.write_bytes(b'reviewed path fixture')
    obj.trajectory_file.with_name('path.cdr.meta.json').write_text('{"plan_id":"abc123"}')
    obj.calibration_file, obj.physical_profile = '/tmp/calibration', '/tmp/profile'
    obj.empty_hand_clearance, obj.obstacle_height = Var('50'), Var('150')


@pytest.mark.parametrize('real', [False, True])
@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_run_forwards_selected_mode_but_waits_for_a_later_preview_approval(monkeypatch, tmp_path, real, mode):
    obj = make_gui(real=real)
    prepare_launch(obj, tmp_path)
    obj.execution_mode.set(obj.EXECUTION_LABELS[mode])
    start = Mock(return_value=SimpleNamespace(stdout=[]))
    modal = Mock(return_value=True)
    monkeypatch.setattr(ui.subprocess, 'Popen', start)
    monkeypatch.setattr(ui.threading, 'Thread', Mock())
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('run')
    command = start.call_args.args[0]
    assert f'execution_mode:={mode}' in command
    assert obj.node.clients == []
    assert obj.step_request is None
    if real:
        assert 'workspace_clear:=true' in command
        assert obj.job_plan_id == 'abc123'
        assert '실제 이동은 경로가 표시된 뒤 상단 OK' in modal.call_args.args[1]
    else:
        modal.assert_not_called()


def test_cancelled_real_preparation_never_starts_backend_or_approval(monkeypatch, tmp_path):
    obj = make_gui()
    prepare_launch(obj, tmp_path)
    start = Mock()
    monkeypatch.setattr(ui.subprocess, 'Popen', start)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', Mock(return_value=False))
    obj.launch_task('run')
    start.assert_not_called()
    assert obj.node.clients == []
    assert obj.job is None


@pytest.mark.parametrize('state', ['success', 'stopped', 'closing', 'failed', 'stop_pending'])
def test_finished_gripper_preparation_only_schedules_a_fresh_plan_when_allowed(monkeypatch, tmp_path, state):
    obj = make_gui()
    prepare_launch(obj, tmp_path)
    obj.job = SimpleNamespace(poll=lambda: 0, returncode=0)
    obj.job_action = 'alignment_gripper'
    obj.job_succeeded, obj.job_failed = state != 'failed', state == 'failed'
    obj.pending_alignment_plan = True
    obj.user_stopped = state == 'stopped'
    obj.close_requested = state == 'closing'
    obj.stop_pending = state == 'stop_pending'
    obj.stop_future = None
    obj.closing = False
    obj.events = queue.Queue()
    obj.events.put(('finished', 'fixture.log'))
    obj.root = SimpleNamespace(after=Mock())
    obj.refresh_connection = obj.refresh_step_approval = obj.refresh_stop = Mock()
    obj.finish_close = Mock()
    obj.launch_task = Mock()
    monkeypatch.setattr(ui.rclpy, 'ok', lambda: False)
    obj.poll()
    callbacks = [call.args[1] for call in obj.root.after.call_args_list
                 if getattr(call.args[1], '__name__', '') == '<lambda>']
    assert len(callbacks) == (1 if state == 'success' else 0)
    assert obj.node.clients == []
    obj.launch_task.assert_not_called()
    if callbacks:
        callbacks[0]()
        obj.launch_task.assert_called_once_with('plate_align')  # Plan only; never run.
