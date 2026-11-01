"""Offline cancellation regressions: no ROS node, executor, socket, or device.

The real cleanup method is exercised with accepted-goal/future fakes. A bounded
fake clock makes an unsafe early return fail and prevents a regression from
hanging pytest indefinitely.
"""

from collections import deque
from concurrent.futures import CancelledError
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


# Load this checkout under an isolated package name so an installed overlay
# cannot silently substitute another copy of the cancellation implementation.
_SOURCE = Path(__file__).resolve().parents[1] / 'ur5e_block_bringup'
_PACKAGE_NAME = 'ur5e_air_cancellation_test_package'
_PACKAGE_SPEC = importlib.util.spec_from_file_location(
    _PACKAGE_NAME, _SOURCE / '__init__.py', submodule_search_locations=[str(_SOURCE)])
_PACKAGE = importlib.util.module_from_spec(_PACKAGE_SPEC)
sys.modules[_PACKAGE_NAME] = _PACKAGE
_PACKAGE_SPEC.loader.exec_module(_PACKAGE)
_SPEC = importlib.util.spec_from_file_location(_PACKAGE_NAME + '.air_task', _SOURCE / 'air_task.py')
air = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = air
_SPEC.loader.exec_module(air)


class FakeFuture:
    def __init__(self, value=None, *, done=True, error=None):
        self.value = value
        self.ready = done
        self.error = error
        self.result_reads = 0

    def done(self):
        return self.ready

    def result(self):
        assert self.ready, 'A result was read before its future completed'
        self.result_reads += 1
        if self.error is not None:
            raise self.error
        return self.value


class AcceptedGoal:
    accepted = True

    def __init__(self, result_attempts=(), *, cancel_failures=0):
        self.result_attempts = deque(result_attempts)
        self.cancel_failures = cancel_failures
        self.cancel_calls = 0
        self.get_result_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.cancel_calls <= self.cancel_failures:
            raise RuntimeError('Transient local cancellation transport failure')
        # An accepted cancellation request is deliberately distinct from a
        # terminal result: accepting the request does not prove motion stopped.
        return FakeFuture(SimpleNamespace(return_code=0, goals_canceling=[object()]))

    def get_result_async(self):
        self.get_result_calls += 1
        if not self.result_attempts:
            raise AssertionError('Unexpected extra result request')
        attempt = self.result_attempts.popleft()
        if isinstance(attempt, BaseException):
            raise attempt
        return attempt


class FakeRuntime:
    def __init__(self):
        self.now = 0.0
        self.spins = 0
        self.on_spin = lambda: None
        self.task = None

    def monotonic(self):
        return self.now

    def spin_once(self, task, timeout_sec):
        assert task is self.task
        assert 0 < timeout_sec <= 0.1
        self.spins += 1
        assert self.spins <= 40, 'Cleanup failed to reach the expected terminal/standstill condition'
        self.now += 0.5
        self.on_spin()


@pytest.fixture
def runtime(monkeypatch):
    fake = FakeRuntime()
    monkeypatch.setattr(air, 'time', fake)
    monkeypatch.setattr(air, 'rclpy', SimpleNamespace(spin_once=fake.spin_once))
    return fake


def task_harness(runtime, goal, result_future, measured_stop):
    task = SimpleNamespace(goal=goal, result_future=result_future,
                           stopped=False, measured_stop=measured_stop)
    runtime.task = task
    return task


def terminal_future(status=5, **kwargs):
    return FakeFuture(SimpleNamespace(status=status), **kwargs)


@pytest.mark.parametrize('first_attempt', [RuntimeError('get_result_async failed'), None])
def test_accepted_goal_without_result_future_retries_acquisition(runtime, capsys, first_attempt):
    terminal = terminal_future()
    goal = AcceptedGoal([first_attempt, terminal])
    measurements = []

    def measured_stop(**kwargs):
        assert goal.get_result_calls == 2, 'A failed/absent result future is not terminal evidence'
        measurements.append(kwargs)
        return True

    task = task_harness(runtime, goal, None, measured_stop)
    air.AirTask.cancel_remaining_goal(task)
    assert task.stopped and task.goal is None
    assert goal.get_result_calls == 2 and goal.cancel_calls >= 1
    assert runtime.spins >= 1 and measurements
    assert capsys.readouterr().out.count('STACK_STOPPED_CONFIRMED') == 1


@pytest.mark.parametrize('error', [RuntimeError('result transport failed'), CancelledError()])
def test_done_future_exception_or_local_cancellation_does_not_prove_remote_stop(runtime, capsys, error):
    failed = FakeFuture(error=error)
    recovery = terminal_future(done=False)
    goal = AcceptedGoal([recovery])
    checks = []

    def measured_stop(**_):
        assert recovery.ready, 'Standstill cannot replace a missing remote terminal result'
        checks.append(True)
        return True

    task = task_harness(runtime, goal, failed, measured_stop)

    def advance():
        assert task.goal is goal, 'Cleanup abandoned the accepted goal after a local future error'
        if runtime.spins == 3:
            recovery.ready = True

    runtime.on_spin = advance
    air.AirTask.cancel_remaining_goal(task)
    assert failed.result_reads == 1
    assert goal.get_result_calls == 1 and runtime.spins >= 3
    assert checks == [True] and task.goal is None
    assert capsys.readouterr().out.count('STACK_STOPPED_CONFIRMED') == 1


@pytest.mark.parametrize('status', [0, 1, 2, 3, 7, 255, None])
def test_unknown_or_nonterminal_result_status_is_not_stop_confirmation(runtime, capsys, status):
    unknown = FakeFuture(None if status is None else SimpleNamespace(status=status))
    terminal = terminal_future()
    goal = AcceptedGoal([terminal])

    def measured_stop(**_):
        assert goal.get_result_calls == 1, 'An unknown status must be replaced with a terminal result'
        return True

    task = task_harness(runtime, goal, unknown, measured_stop)
    air.AirTask.cancel_remaining_goal(task)
    assert unknown.result_reads == 1 and terminal.result_reads >= 1
    assert runtime.spins >= 1 and task.goal is None
    assert capsys.readouterr().out.count('STACK_STOPPED_CONFIRMED') == 1


def test_terminal_result_does_not_emit_confirmation_before_measured_standstill(runtime, capsys):
    goal = AcceptedGoal()
    terminal = terminal_future()
    measurements = []
    output_before_standstill = []

    def measured_stop(**_):
        measurements.append(True)
        return len(measurements) >= 3

    task = task_harness(runtime, goal, terminal, measured_stop)

    def still_waiting():
        text = capsys.readouterr().out
        output_before_standstill.append(text)
        assert 'STACK_STOPPED_CONFIRMED' not in text
        assert task.goal is goal

    runtime.on_spin = still_waiting
    air.AirTask.cancel_remaining_goal(task)
    assert len(measurements) == 3 and runtime.spins == 2
    assert task.goal is None
    assert any('STACK_STOP_PENDING' in text for text in output_before_standstill)
    assert capsys.readouterr().out.count('STACK_STOPPED_CONFIRMED') == 1


@pytest.mark.parametrize('status', [4, 5, 6])
def test_accepted_cancel_waits_for_remote_terminal_result_and_cleanup_is_idempotent(runtime, capsys, status):
    result = terminal_future(status, done=False)
    goal = AcceptedGoal()
    measurements = []

    def measured_stop(**_):
        assert result.ready
        measurements.append(True)
        return True

    task = task_harness(runtime, goal, result, measured_stop)

    def remote_progress():
        assert task.goal is goal
        assert not measurements, 'A cancellation acknowledgement is not a terminal result'
        if runtime.spins == 2:
            result.ready = True

    runtime.on_spin = remote_progress
    air.AirTask.cancel_remaining_goal(task)
    assert goal.cancel_calls == 1 and runtime.spins == 2
    assert measurements == [True] and task.goal is None
    assert capsys.readouterr().out.count('STACK_STOPPED_CONFIRMED') == 1
    counts = goal.cancel_calls, goal.get_result_calls, runtime.spins
    air.AirTask.cancel_remaining_goal(task)
    assert (goal.cancel_calls, goal.get_result_calls, runtime.spins) == counts
    assert capsys.readouterr().out == ''


def test_local_cancel_errors_are_retried_without_abandoning_the_goal(runtime, capsys):
    result = terminal_future(done=False)
    goal = AcceptedGoal(cancel_failures=2)
    task = task_harness(runtime, goal, result, lambda **_: True)

    def remote_progress():
        assert task.goal is goal
        if runtime.now >= 6.0:
            assert goal.cancel_calls >= 3, 'Cancellation was not retried after local transport errors'
            result.ready = True

    runtime.on_spin = remote_progress
    air.AirTask.cancel_remaining_goal(task)
    assert goal.cancel_calls >= 3 and runtime.now >= 6.0
    assert task.goal is None
    assert capsys.readouterr().out.count('STACK_STOPPED_CONFIRMED') == 1
