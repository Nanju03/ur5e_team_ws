"""Calibration state-machine regressions without constructing any ROS Node."""
import ast
from collections import deque
import math
from pathlib import Path
import secrets
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'ur5e_block_bringup'
tree = ast.parse((SOURCE / 'gripper_calibration.py').read_text())
backend_class = next(node for node in tree.body
                     if isinstance(node, ast.ClassDef) and node.name == 'CalibrationBackend')
arm_tree = ast.parse((SOURCE / 'air_task.py').read_text())
constants = [node for node in arm_tree.body if isinstance(node, ast.Assign)
             and any(isinstance(target, ast.Name) and target.id in ('ARM', 'HAND') for target in node.targets)]
namespace = dict(math=math, secrets=secrets, deque=deque, Float64MultiArray=lambda **kwargs: SimpleNamespace(**kwargs),
                 GripperCommand=SimpleNamespace(Goal=lambda: SimpleNamespace(command=SimpleNamespace())),
                 LABELS={0:'moving', 1:'empty', 2:'caught', 3:'dropped'},
                 rclpy=SimpleNamespace(spin_once=lambda node, timeout_sec: None))
exec(compile(ast.Module(body=constants+[backend_class], type_ignores=[]), '<actual calibration backend>', 'exec'), namespace)
CalibrationBackend = namespace['CalibrationBackend']


class Clock:
    def __init__(self): self.now = 100.
    def monotonic(self): return self.now


class Future:
    def __init__(self, value=None, *, ready=True, error=None):
        self.value, self.ready, self.error = value, ready, error
    def done(self): return self.ready
    def result(self):
        assert self.ready, 'Result was consumed before completion'
        if self.error: raise self.error
        return self.value


class Goal:
    accepted = True
    def __init__(self, attempts=()):
        self.attempts = deque(attempts); self.cancel_calls = self.get_calls = 0
    def cancel_goal_async(self):
        self.cancel_calls += 1
        return Future(SimpleNamespace(goals_canceling=[self]))
    def get_result_async(self):
        self.get_calls += 1
        if not self.attempts: return Future(ready=False)
        value = self.attempts.popleft()
        if isinstance(value, BaseException): raise value
        return value


def terminal(status=5, stalled=False):
    return SimpleNamespace(status=status, result=SimpleNamespace(stalled=stalled, reached_goal=status == 4))


def trial(**changes):
    value = dict(id=1, context='test', force_percent=20, commanded_q=.73, measured_q=.716,
                 verified_success=None, released=False, sensor_valid=True, grip_state=2,
                 dropped=False, stopped=False, arm_moved=False)
    value.update(changes)
    return value


def history(backend, clock, q, *, since=None):
    start = clock.now-.5 if since is None else since
    backend.q_history = deque((start+(clock.now-start)*fraction, q) for fraction in (0., .5, 1.))
    backend.sensor['position'] = q
    backend.sensor_time = clock.now


@pytest.fixture
def backend(monkeypatch):
    clock = Clock(); monkeypatch.setitem(namespace, 'time', clock)
    obj = CalibrationBackend.__new__(CalibrationBackend)
    obj.clock = clock; obj.node = object(); obj.closed = False; obj.mock = True
    obj.execution_mode = 'continuous'
    obj.pending_approval = obj.approved_preview = None
    obj.command_index = obj.operation_steps = 0
    obj.gripper_only = False; obj.arm_detected = False
    obj.standalone_conflict = False; obj.standalone_conflict_reason = ''
    obj.standalone_discovery_started = clock.now-3
    obj.settings = dict(open_q=.5, max_q=.78, step_q=.01, force_percent=20, settle_s=.4, width_mm=25.)
    obj.configuration_values = obj.settings; obj.context = 'test'; obj.model_sha = 'model'
    obj.sensor = dict(position=.5, raw_position=462, grip_state=1, force_percent=20,
                      communication_age=0., sample_sequence=10.)
    obj.sequence = 10; obj.sensor_time = clock.now
    history(obj, clock, .5)
    obj.phase = 'IDLE'; obj.stage = obj.operation = None; obj.busy = False; obj.error = ''
    obj.queue = deque(); obj.send_future = obj.goal = obj.result_future = obj.goal_terminal = None
    obj.target_q = None; obj.goal_sent = obj.last_cancel = -math.inf
    obj.last_goal_sequence = -1; obj.stop_requested = obj.stop_confirmed = False
    obj.stop_proof_time = -math.inf; obj.stop_proof_sequence = -1; obj.stop_hold_sent = False
    obj.ever_commanded = False; obj.last_force = obj.last_log = -math.inf
    obj.stage_started = clock.now; obj.force_sequence = obj.sequence
    obj.trials = []; obj.current_trial = obj.baseline = obj.current_baseline_max = None
    obj.arm_values = {name:0. for name in namespace['ARM']}; obj.arm_reference = dict(obj.arm_values)
    obj.arm_velocities = dict(obj.arm_values); obj.arm_times = {name:clock.now for name in namespace['ARM']}
    obj.arm_quiet_since = clock.now-1
    obj.events = []; obj.force_messages = []; obj.goals_sent = []
    obj._save = lambda event='': obj.events.append(event)
    obj._log = lambda event='': None
    obj._queries = lambda: None
    obj._controller_ok = lambda: True
    obj._arm_stationary = lambda: True
    obj._recommendation = lambda: None
    obj.force_pub = SimpleNamespace(publish=lambda message: obj.force_messages.append(message.data))
    def send_goal(goal):
        obj.goals_sent.append(goal)
        return Future(ready=False)
    obj.action = SimpleNamespace(send_goal_async=send_goal)
    obj.gap = SimpleNamespace(gap_mm=lambda q: 25.)
    obj.csv_path = Path('/not-created/calibration.csv')
    return obj


def prepare_step_preview(b):
    b.execution_mode = 'step'
    b.start('baseline', workspace_clear=True)
    # The force acknowledgement is independent of the position approval gate.
    b.stage = 'NEXT'
    b._normal_tick()
    assert b.stage == 'WAIT_APPROVAL'
    return dict(b.pending_approval)


def test_step_mode_previews_without_sending_until_exact_approval(backend):
    b = backend
    preview = prepare_step_preview(b)
    for _ in range(5):
        b._normal_tick()
    assert b.goals_sent == []
    assert preview['start_q'] == .5 and preview['target_q'] == .5
    assert preview['start_gap_mm'] == 25. and preview['target_gap_mm'] == 25.
    assert preview['preview_kind'] == 'gripper_angle_and_estimated_gap'
    assert preview['index'] == 1 and preview['total'] == len(b.queue)
    with pytest.raises(RuntimeError, match='승인'):
        b.approve_next('old-token')
    assert b.goals_sent == []
    b.approve_next(preview['request_id'])
    assert b.goals_sent == []  # Actual dispatch revalidates in the next poll.
    with pytest.raises(RuntimeError, match='승인'):
        b.approve_next(preview['request_id'])
    b._normal_tick()
    assert len(b.goals_sent) == 1 and b.goals_sent[0].command.position == .5
    assert b.command_index == 1 and b.pending_approval is None


def test_every_next_gripper_target_needs_a_new_token(backend):
    b = backend
    first = prepare_step_preview(b)
    b.approve_next(first['request_id']); b._normal_tick()
    b.send_future.ready = True
    b.send_future.value = Goal([Future(terminal(status=4))])
    b._action_tick()
    assert b.stage == 'SETTLE'
    b.clock.now += .5; b.sequence += 1; history(b,b.clock,.5)
    b._normal_tick()
    second = b.pending_approval
    assert second['index'] == 2 and second['target_q'] == .51
    assert second['request_id'] != first['request_id']
    assert len(b.goals_sent) == 1
    with pytest.raises(RuntimeError):
        b.approve_next(first['request_id'])


@pytest.mark.parametrize('change', ['pose','grip_state','force','stale','controller'])
def test_step_approval_rechecks_pose_sensor_force_and_controller(backend, change):
    b = backend
    preview = prepare_step_preview(b)
    if change == 'pose':
        history(b,b.clock,.505)
    elif change == 'grip_state':
        b.sensor['grip_state'] = 2
    elif change == 'force':
        b.sensor['force_percent'] = 30
    elif change == 'stale':
        b.sensor_time -= 1.
    else:
        b._controller_ok = lambda: False
    with pytest.raises(RuntimeError):
        b.approve_next(preview['request_id'])
    assert b.goals_sent == []


def test_changed_pose_after_approval_blocks_dispatch_and_invalidates_token(backend):
    b = backend
    preview = prepare_step_preview(b)
    b.approve_next(preview['request_id'])
    history(b,b.clock,.505)
    state = b.poll()
    assert b.goals_sent == [] and b.stop_requested
    assert state['pending_approval'] is None and b.approved_preview is None


def test_stop_while_waiting_invalidates_preview_and_never_sends_goal(backend):
    b = backend
    preview = prepare_step_preview(b)
    b.request_stop()
    with pytest.raises(RuntimeError):
        b.approve_next(preview['request_id'])
    b._normal_tick()
    assert b.goals_sent == [] and b.pending_approval is None


def test_position_send_cannot_bypass_step_approval_but_stop_hold_is_preserved(backend):
    b = backend; b.execution_mode = 'step'
    with pytest.raises(RuntimeError, match='OK'):
        b._send(.51)
    assert b.goals_sent == []
    b.stop_requested = True
    b._send(.5,hold=True)
    assert len(b.goals_sent) == 1 and b.stage == 'HOLD_GOAL'


def test_continuous_mode_dispatches_existing_recipe_without_per_step_gate(backend):
    b = backend
    b.start('baseline',workspace_clear=True); b.stage = 'NEXT'; b._normal_tick()
    assert b.execution_mode == 'continuous' and len(b.goals_sent) == 1
    assert b.pending_approval is None


@pytest.mark.parametrize('first_attempt', [RuntimeError('transport unavailable'), None])
def test_missing_result_future_enters_cancel_and_retries_until_terminal(backend, first_attempt):
    b = backend; pending = Future(ready=False)
    goal = Goal([first_attempt, pending]); b.goal = goal; b.busy = b.ever_commanded = True
    b.goal_sent = b.clock.now; b._action_tick()
    assert b.stop_requested and b.goal is goal and not b.stop_confirmed
    b.clock.now += .6; b._action_tick()
    assert goal.cancel_calls == 1 and b.result_future is pending
    pending.ready = True; pending.value = terminal(5)
    b.clock.now += .1; b._action_tick()
    assert b.goal is None and b.goal_terminal.status == 5
    b._stop_tick()
    assert not b.stop_confirmed, 'Terminal action alone is not post-cancel serial standstill'
    b.clock.now += .5; b.sequence += 3; history(b, b.clock, .5)
    b._stop_tick()
    assert b.stop_confirmed and not b.busy


def test_done_future_exception_keeps_goal_and_retries_cancel(backend):
    b = backend; goal = Goal([Future(ready=False)])
    b.goal = goal; b.result_future = Future(error=RuntimeError('lost result'))
    b.busy = b.ever_commanded = True; b.goal_sent = b.clock.now
    b._action_tick()
    assert b.goal is goal and b.result_future is None and b.stop_requested
    b.clock.now += .6; b._action_tick()
    assert goal.cancel_calls == 1 and b.goal is goal


@pytest.mark.parametrize('result', [None]+[terminal(status) for status in (0, 1, 2, 3, 7, 255)])
def test_unknown_terminal_status_never_releases_accepted_goal(backend, result):
    b = backend; goal = Goal(); b.goal = goal; b.result_future = Future(result)
    b.busy = b.ever_commanded = True; b.goal_sent = b.clock.now
    b._action_tick(); b._stop_tick()
    assert b.goal is goal and b.stop_requested and not b.stop_confirmed
    assert b.poll()['can_close'] is False


@pytest.mark.parametrize('acceptance', ['exception', 'none'])
def test_ambiguous_acceptance_preserves_pending_future_and_prevents_close(backend, acceptance):
    b = backend
    pending = Future(error=RuntimeError('lost acceptance')) if acceptance == 'exception' else Future(None)
    b.send_future = pending; b.busy = b.ever_commanded = True; b.goal_sent = b.clock.now
    status = b.poll()
    assert b.send_future is pending and b.stop_requested
    assert not status['can_close'] and not status['stop_confirmed']
    assert b.close() is False


def test_old_quiet_history_and_sequence_cannot_confirm_stop(backend):
    b = backend; b.ever_commanded = True; b.request_stop()
    assert b._quiet()
    b._stop_tick(); assert not b.stop_confirmed
    b.sequence += 1
    b._stop_tick(); assert not b.stop_confirmed, 'New counter without post-stop quiet interval is insufficient'
    b.clock.now += .2; b.sensor_time = b.clock.now; b.q_history.append((b.clock.now, .5))
    b._stop_tick(); assert not b.stop_confirmed
    b.clock.now += .3; b.sensor_time = b.clock.now; b.sequence += 2
    b.q_history.append((b.clock.now, .5)); b._stop_tick()
    assert b.stop_confirmed and b.events.count('STOP_CONFIRMED') == 1


@pytest.mark.parametrize('status', [4, 5, 6])
def test_terminal_result_resets_stop_proof_to_its_own_time(backend, status):
    b = backend; b.goal = Goal(); b.busy = b.ever_commanded = True; b.request_stop()
    b.clock.now += .6; b.sequence += 2; history(b, b.clock, .5)
    b.result_future = Future(terminal(status)); b._action_tick()
    assert b.stop_proof_time == b.clock.now and b.stop_proof_sequence == b.sequence
    b._stop_tick(); assert not b.stop_confirmed


def test_force_write_counts_as_a_command_before_first_position_goal(backend):
    b = backend; b.stage = 'FORCE'; b.busy = True; b.operation = 'baseline'
    b._normal_tick()
    assert b.force_messages == [[0.]] and b.ever_commanded
    b.request_stop(); b._stop_tick()
    assert not b.stop_confirmed and b.poll()['can_close'] is False


def sensor_message(sequence, raw=462, state=1, force=20, age=0.):
    names = ['position', 'raw_position', 'grip_state', 'force_percent', 'communication_age', 'sample_sequence']
    values = [.93*(1000-raw)/1000, raw, state, force, age, sequence]
    return SimpleNamespace(joint_names=[namespace['HAND']],
                           interface_values=[SimpleNamespace(interface_names=names, values=values)])


def test_repeated_sequence_does_not_refresh_serial_freshness(backend):
    b = backend; b.clock.now += .6
    b._sensor(sensor_message(b.sequence))
    assert not b._fresh() and b.sensor_time == 100.
    b._sensor(sensor_message(b.sequence+1))
    assert b._fresh() and b.sensor_time == b.clock.now


def test_stale_serial_sequence_during_finger_motion_requests_stop(backend):
    b = backend; b.busy = b.ever_commanded = True; b.send_future = Future(ready=False)
    b.goal_sent = b.clock.now; b.stage = 'GOAL'; b.clock.now += .6
    status = b.poll()
    assert b.stop_requested and not status['can_close']


def test_arm_movement_during_active_finger_motion_invalidates_trial(backend):
    b = backend; b.current_trial = trial(sensor_valid=False); b.trials = [b.current_trial]
    b.busy = True; b.stage = 'GOAL'; b.send_future = Future(ready=False); b.goal_sent = b.clock.now
    b.arm_values[namespace['ARM'][0]] = .1
    b.poll()
    assert b.stop_requested and b.current_trial['arm_moved'] and b.current_trial['stopped']


def hold(backend):
    backend.current_trial = trial(); backend.trials = [backend.current_trial]
    backend.phase = 'HOLD'; backend.ever_commanded = True
    backend.sensor['grip_state'] = 2
    history(backend, backend.clock, .716)


def test_manual_arm_lift_during_hold_is_allowed_but_record_waits_for_rest(backend):
    b = backend; hold(b); b.arm_values[namespace['ARM'][0]] = .1
    b._arm_stationary = lambda: False
    status = b.poll()
    assert not b.stop_requested and b.current_trial['sensor_valid'] and not b.current_trial['arm_moved']
    assert status['can_record'] is False
    b._arm_stationary = lambda: True
    assert b.poll()['can_record'] is True
    b.record_result(True)
    assert b.current_trial['verified_success'] is True


@pytest.mark.parametrize('fault', ['stale', 'force', 'position'])
def test_hold_sensor_discontinuity_invalidates_trial(backend, fault):
    b = backend; hold(b)
    if fault == 'stale': b.clock.now += .6
    elif fault == 'force': b.sensor['force_percent'] = 40
    else: b.sensor['position'] += .02
    status = b.poll()
    assert b.current_trial['sensor_valid'] is False and b.current_trial['stopped']
    assert status['can_record'] is False
    with pytest.raises(RuntimeError): b.record_result(True)


def test_hold_failure_cannot_close_before_post_stop_proof(backend):
    b = backend; hold(b); b.sensor['force_percent'] = 40
    status = b.poll()
    assert b.stop_requested and not b.stop_confirmed
    assert status['can_close'] is False
    assert b.close() is False


@pytest.mark.parametrize('flag', ['stopped', 'dropped', 'arm_moved'])
def test_invalidated_trial_can_never_be_operator_approved(backend, flag):
    b = backend; hold(b); b.current_trial[flag] = True
    assert b.poll()['can_record'] is False
    with pytest.raises(RuntimeError): b.record_result(True)


@pytest.mark.parametrize('status,stalled,accepted', [(4, False, True), (6, True, True), (6, False, False), (5, True, False)])
def test_contact_needs_sensor_caught_and_valid_action_termination(backend, status, stalled, accepted):
    b = backend; hold(b); b.busy = True; b.stage = 'SETTLE'; b.operation = 'grasp'
    b.stage_started = b.clock.now-.5; b.last_goal_sequence = b.sequence-1; b.target_q = .73
    b.goal_terminal = terminal(status, stalled); b.queue = deque([.74, .75])
    b._normal_tick()
    assert (b.phase == 'HOLD') is accepted
    if accepted:
        assert not b.busy and not b.queue and b.current_trial['sensor_valid']
        assert not b.goals_sent, 'A contact must hold instead of automatically opening'
    else:
        assert b.stop_requested and b.current_trial['stopped']


def test_baseline_contact_aborts_instead_of_claiming_empty_sweep(backend):
    b = backend; b.sensor['grip_state'] = 2; history(b, b.clock, .716)
    b.busy = True; b.stage = 'SETTLE'; b.operation = 'baseline'; b.stage_started = b.clock.now-.5
    b.last_goal_sequence = b.sequence-1; b.target_q = .73; b.goal_terminal = terminal(6, True)
    b._normal_tick()
    assert b.stop_requested and b.baseline is None


@pytest.mark.parametrize('actual_state', [0, 1, 3])
def test_stall_result_without_caught_sensor_does_not_record_contact(backend, actual_state):
    b = backend; b.current_trial = trial(sensor_valid=False, grip_state=None)
    b.busy = True; b.stage = 'SETTLE'; b.operation = 'grasp'; b.stage_started = b.clock.now-.5
    b.last_goal_sequence = b.sequence-1; b.target_q = .73; b.goal_terminal = terminal(6, True)
    b.sensor['grip_state'] = actual_state
    b._normal_tick()
    assert b.current_trial['sensor_valid'] is False and b.phase != 'HOLD'


@pytest.mark.parametrize('q', [.78, .79])
def test_step_close_at_or_beyond_limit_never_opens_instead(backend, q):
    b = backend; history(b, b.clock, q)
    with pytest.raises(RuntimeError): b.start('step_close', workspace_clear=True)
    assert not b.busy and not b.queue and not b.goals_sent


@pytest.mark.parametrize('successful', [True, False])
def test_release_evidence_requires_successful_open_and_empty_sensor(backend, successful):
    b = backend; hold(b); item = b.current_trial; item['verified_success'] = True
    b.busy = True; b.stage = 'SETTLE'; b.operation = 'open'; b.stage_started = b.clock.now-.5
    b.last_goal_sequence = b.sequence-1; b.target_q = .5
    b.goal_terminal = terminal(4 if successful else 6, stalled=not successful)
    b.sensor['grip_state'] = 1; history(b, b.clock, .5)
    b._normal_tick()
    assert item['released'] is successful
    if successful: assert b.current_trial is None
    else: assert b.stop_requested
