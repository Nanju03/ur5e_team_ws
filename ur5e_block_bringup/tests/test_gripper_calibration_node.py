"""Headless service and shutdown regressions without ROS Nodes or processes."""
import ast
from collections import deque
import fcntl
import __future__
import json
import math
import os
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace

import pytest

import test_gripper_calibration_backend as backend_base


SOURCE = Path(__file__).resolve().parents[1] / 'ur5e_block_bringup'
backend = backend_base.backend


class Clock:
    now = 100.

    def monotonic(self):
        return self.now

    def time(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class FakeNode:
    overrides = {}

    def __init__(self, name, **kwargs):
        self.name = name
        self.parameters = {}
        self.registered_services = {}
        self.messages = []
        self.logs = []
        self.destroyed = False

    def declare_parameter(self, name, value=None, *args, **kwargs):
        self.parameters[name] = self.overrides.get(name, value)
        return SimpleNamespace(value=self.parameters[name])

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_service(self, kind, name, callback, **kwargs):
        service = SimpleNamespace(name=name, callback=callback)
        self.registered_services[name] = service
        return service

    @property
    def services(self):
        return iter(self.registered_services.values())

    def destroy_service(self, service):
        for name, registered in list(self.registered_services.items()):
            if registered is service:
                del self.registered_services[name]
                return True
        return False

    def get_fully_qualified_name(self):
        return '/' + self.name

    def create_publisher(self, *args, **kwargs):
        return SimpleNamespace(publish=lambda value: self.messages.append(value.data))

    def get_logger(self):
        return SimpleNamespace(**{
            name: lambda message, *args, **kwargs: self.logs.append(str(message))
            for name in ('info', 'warning', 'warn', 'error', 'debug')})

    def destroy_node(self):
        self.destroyed = True


class FakeBackend:
    def __init__(self, events, lock_fd, execution_mode='step'):
        self.events = events
        self.execution_mode = execution_mode
        self.locks = [lock_fd]
        self.close_ready = False
        self.closed = False
        self.poll_calls = self.stop_calls = 0
        self.start_calls, self.record_calls = [], []
        self.apply_calls = 0
        self.approve_calls = []
        self.ever_commanded = False
        self.csv_path = Path('/not-created/calibration.csv')
        self.service_names = []
        self.node = SimpleNamespace(get_service_names_and_types=lambda: self.service_names)
        self.state = dict(ready=True, busy=False, phase='IDLE', can_close=True,
                          can_record=False, feedback_valid=True, age=0., q_rad=.5,
                          raw_position=462, force_percent=20, error='',
                          stop_confirmed=False, recommendation=None,
                          execution_mode=execution_mode, pending_approval=None)

    @property
    def busy(self):
        return self.state['busy']

    def configure(self, **settings):
        self.settings = settings

    def poll(self):
        self.events.append('backend.poll')
        self.poll_calls += 1
        return dict(self.state)

    def start(self, operation, workspace_clear=False):
        if not workspace_clear or not self.state['ready'] or self.state['busy']:
            raise RuntimeError('The physical backend is not ready')
        self.events.append('backend.start.' + operation)
        self.start_calls.append(operation)
        self.state.update(busy=True, ready=False, phase=operation.upper())

    def request_stop(self):
        self.events.append('backend.stop')
        self.stop_calls += 1

    def record_result(self, success):
        if not self.state['can_record']:
            raise RuntimeError('A sensor-verified held trial is required')
        self.record_calls.append(success)

    def apply_recommendation(self):
        if self.state['recommendation'] is None:
            raise RuntimeError('There is no verified recommendation')
        self.apply_calls += 1
        return '/not-created/gripper_calibration.yaml'

    def approve_next(self, token):
        preview = self.state.get('pending_approval')
        if not preview or preview['request_id'] != token:
            raise RuntimeError('The approval token is no longer current')
        self.approve_calls.append(token)
        self.state['pending_approval'] = None

    def close(self):
        self.events.append('backend.close')
        if not self.close_ready:
            return False
        if not self.closed:
            for fd in self.locks:
                os.close(fd)
            self.locks.clear()
            self.closed = True
        return True


class FakeProcess:
    pid = 987654321

    def __init__(self, events):
        self.events = events
        self.returncode = None
        self.signals = []
        self.kill_calls = self.terminate_calls = 0

    def poll(self):
        return self.returncode

    def send_signal(self, value):
        self.events.append('driver.signal.' + str(value))
        self.signals.append(value)

    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired('fake driver', timeout)
        return self.returncode

    def kill(self):
        self.kill_calls += 1
        raise AssertionError('Forced termination must never be used')

    def terminate(self):
        self.terminate_calls += 1
        raise AssertionError('Forced termination must never be used')


@pytest.fixture
def harness(monkeypatch, tmp_path):
    clock, events, descriptors = Clock(), [], []
    process = FakeProcess(events)
    spawned, backends = [], []
    lock_path = tmp_path / 'only-this-test.lock'

    def backend_factory(*args, **kwargs):
        fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        descriptors.append(fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        backend = FakeBackend(events, fd, execution_mode=kwargs.get('execution_mode', 'step'))
        backend.execution_mode = kwargs.get('execution_mode')
        backends.append(backend)
        return backend

    def driver_factory(*args, **kwargs):
        spawned.append((args, kwargs))
        events.append('driver.spawn')
        return process

    def duplicate(fd):
        value = os.dup(fd)
        descriptors.append(value)
        return value

    class SafeOS:
        dup = staticmethod(duplicate)

        def __getattr__(self, name):
            if name == 'killpg':
                return lambda pid, value: process.send_signal(value)
            if name == 'getpgid':
                return lambda pid: pid
            return getattr(os, name)

    FakeNode.overrides = {'start_driver': True, 'workspace_clear': False,
                          'startup_operation': 'monitor', 'use_mock_hardware': False,
                          'allow_mock_hardware': False}
    namespace = dict(
        Node=FakeNode, CalibrationBackend=backend_factory,
        subprocess=SimpleNamespace(Popen=driver_factory, TimeoutExpired=subprocess.TimeoutExpired),
        os=SafeOS(), fcntl=fcntl, signal=signal, time=clock, json=json, math=math, Path=Path,
        deque=deque, String=lambda **kwargs: SimpleNamespace(**kwargs),
        Trigger=SimpleNamespace(Response=reply), SetBool=SimpleNamespace,
        ParameterDescriptor=lambda **kwargs: SimpleNamespace(**kwargs),
        get_package_share_directory=lambda package: str(tmp_path),
        rclpy=SimpleNamespace(spin_once=lambda *args, **kwargs: None))
    data_tree = ast.parse((SOURCE / 'gripper_calibration_data.py').read_text())
    settings = next(node for node in data_tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'validate_settings')
    tree = ast.parse((SOURCE / 'gripper_calibration_node.py').read_text())
    definition_types = (ast.ClassDef, ast.FunctionDef, ast.Assign, ast.AnnAssign)
    definitions = [node for node in tree.body if isinstance(node, definition_types)]
    exec(compile(ast.Module(body=[settings] + definitions, type_ignores=[]),
                 '<actual headless calibration node>', 'exec',
                 flags=__future__.annotations.compiler_flag), namespace)

    def construct(*, backend_factory_override=None, **parameters):
        FakeNode.overrides.update(parameters)
        return namespace['HeadlessCalibration'](
            backend_factory=backend_factory_override or backend_factory,
            driver_factory=driver_factory)

    yield SimpleNamespace(construct=construct, clock=clock, events=events, process=process,
                          spawned=spawned, backends=backends, lock_path=lock_path,
                          namespace=namespace)
    for fd in reversed(descriptors):
        try:
            os.close(fd)
        except OSError:
            pass


def reply():
    return SimpleNamespace(success=None, message='')


def command(node, operation):
    result = reply()
    node.command(operation, result)
    return result


def ready(node, value):
    result = reply()
    node.set_ready(SimpleNamespace(data=value), result)
    return result


def install_approval_probe(node, token):
    backend = node.backend
    backend.approvals = []

    def approve(request_id):
        preview = backend.state.get('pending_approval')
        if not preview or preview['request_id'] != request_id:
            raise RuntimeError('stale approval token')
        backend.approvals.append(request_id)
        backend.state['pending_approval'] = None

    backend.approve_next = approve
    backend.state.update(busy=True, ready=False, phase='WAIT_APPROVAL',
                         pending_approval={'request_id': token, 'target_q': .51,
                                           'target_gap_mm': 48.})


def test_headless_default_step_and_explicit_continuous_reach_backend(harness):
    node = harness.construct(start_driver=False)
    assert node.backend.execution_mode == 'step'
    node.backend.close_ready = True
    node.request_shutdown()
    assert node.close_tick()
    node = harness.construct(start_driver=False, execution_mode='continuous')
    assert node.backend.execution_mode == 'continuous'


def test_headless_approval_service_is_bound_to_exact_current_preview(harness):
    node = harness.construct(start_driver=False, workspace_clear=True)
    token = 'a' * 32
    install_approval_probe(node, token)
    node.tick()
    name = '~/approve_next/r' + token
    old_service = node.registered_services[name]
    assert node.status()['approval_service'] == '/gripper_calibration/approve_next/r' + token
    newer = 'b' * 32
    node.backend.state['pending_approval']['request_id'] = newer
    node.tick()
    assert name not in node.registered_services
    response = old_service.callback(None, reply())
    assert response.success is False and node.backend.approvals == []
    service = node.registered_services['~/approve_next/r' + newer]
    response = service.callback(None, reply())
    assert response.success is True and node.backend.approvals == [newer]
    assert node.status()['approval_service'] is None
    assert service.callback(None, reply()).success is False


def test_headless_readiness_revocation_blocks_pending_approval(harness):
    node = harness.construct(start_driver=False, workspace_clear=True)
    token = 'c' * 32
    install_approval_probe(node, token)
    node.tick()
    service = node.registered_services['~/approve_next/r' + token]
    ready(node, False)
    assert service.callback(None, reply()).success is False
    assert node.backend.approvals == []


def test_headless_shutdown_retires_pending_approval_service(harness):
    node = harness.construct(start_driver=False, workspace_clear=True)
    token = 'd' * 32
    install_approval_probe(node, token)
    node.tick()
    service = node.registered_services['~/approve_next/r' + token]
    node.request_shutdown()
    node.close_tick()
    assert '~/approve_next/r' + token not in node.registered_services
    assert service.callback(None, reply()).success is False


def connect_driver(harness, node):
    harness.clock.now += 2.1
    node.tick()
    assert len(harness.spawned) == 1


def test_monitor_startup_never_sends_an_operation(harness):
    node = harness.construct()
    backend = harness.backends[0]
    for _ in range(4):
        node.tick()
    assert backend.start_calls == [] and backend.record_calls == []
    assert backend.apply_calls == 0
    assert not harness.spawned
    connect_driver(harness, node)
    assert backend.ever_commanded, 'Driver initialization is already a possible physical motion'
    assert harness.spawned[0][1]['start_new_session'] is True


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_selected_execution_mode_is_forwarded_to_the_backend(harness, mode):
    harness.construct(start_driver=False, execution_mode=mode)
    assert harness.backends[0].execution_mode == mode


def test_owned_driver_waits_two_discovery_seconds_before_spawning(harness):
    node = harness.construct()
    harness.clock.now += 1.99
    node.tick()
    assert not harness.spawned
    harness.clock.now += .01
    node.tick()
    assert len(harness.spawned) == 1


def test_existing_gripper_manager_prevents_duplicate_driver_and_latches_the_error(harness):
    node = harness.construct()
    backend = harness.backends[0]
    backend.service_names = [('/gripper/controller_manager/list_hardware_components', [])]
    harness.clock.now += 2.1
    node.tick()
    assert not harness.spawned
    result = reply()
    node.get_status(None, result)
    assert 'start_driver:=false' in json.loads(result.message)['driver_error']
    backend.service_names.clear()
    harness.clock.now += 10.
    node.tick()
    assert not harness.spawned


def test_graph_inspection_error_never_becomes_permission_to_spawn(harness):
    node = harness.construct()
    backend = harness.backends[0]

    def unavailable():
        raise RuntimeError('discovery failed')

    backend.node.get_service_names_and_types = unavailable
    harness.clock.now += 2.1
    node.tick()
    assert not harness.spawned and node.driver_error


def test_backend_lock_conflict_fails_before_driver_creation(harness):
    def locked(*args, **kwargs):
        raise RuntimeError('calibration workspace is already locked')

    with pytest.raises(RuntimeError, match='already locked'):
        harness.construct(backend_factory_override=locked)
    assert not harness.spawned


@pytest.mark.parametrize('operation', ['baseline', 'grasp'])
def test_explicit_startup_waits_for_readiness_then_runs_exactly_once(harness, operation):
    node = harness.construct(start_driver=False, startup_operation=operation, workspace_clear=True)
    backend = harness.backends[0]
    backend.state['ready'] = False
    node.tick()
    assert not backend.start_calls
    backend.state['ready'] = True
    node.tick()
    assert backend.start_calls == [operation]
    backend.state.update(ready=True, busy=False)
    node.tick()
    assert backend.start_calls == [operation]


def test_startup_failure_is_not_retried_when_readiness_returns(harness):
    node = harness.construct(start_driver=False, startup_operation='baseline',
                             workspace_clear=True)
    backend = harness.backends[0]
    attempts = []

    def rejected(*args, **kwargs):
        attempts.append(args)
        raise RuntimeError('physical backend rejected the experiment')

    backend.start = rejected
    node.tick()
    backend.state['ready'] = False
    node.tick()
    backend.state['ready'] = True
    node.tick()
    assert len(attempts) == 1 and not backend.start_calls


def test_startup_timeout_does_not_replay_after_a_late_connection(harness):
    node = harness.construct(start_driver=False, startup_operation='baseline',
                             workspace_clear=True)
    backend = harness.backends[0]
    backend.state['ready'] = False
    harness.clock.now += 31.
    node.tick()
    backend.state['ready'] = True
    node.tick()
    assert not backend.start_calls


@pytest.mark.parametrize('stop_kind', ['stop', 'ready_false', 'shutdown'])
def test_stop_cancels_pending_startup_before_poll_and_does_not_resume_it(harness, stop_kind):
    node = harness.construct(start_driver=False, startup_operation='baseline',
                             workspace_clear=True)
    backend = harness.backends[0]
    harness.events.clear()
    if stop_kind == 'stop':
        assert command(node, 'stop').success
    elif stop_kind == 'ready_false':
        assert ready(node, False).success
    else:
        node.request_shutdown()
    node.tick()
    assert backend.stop_calls >= 1
    assert harness.events.index('backend.stop') < harness.events.index('backend.poll')
    assert not backend.start_calls
    if stop_kind != 'shutdown':
        assert ready(node, True).success
        node.tick()
        assert not backend.start_calls


@pytest.mark.parametrize('stop_kind', ['stop', 'ready_false'])
def test_stop_before_driver_discovery_finishes_cancels_automatic_initialization(
        harness, stop_kind):
    node = harness.construct(workspace_clear=True)
    if stop_kind == 'stop':
        assert command(node, 'stop').success
    else:
        assert ready(node, False).success
    assert node.driver_start_cancelled
    harness.clock.now += 10.
    node.tick()
    assert ready(node, True).success
    node.tick()
    assert not harness.spawned


def test_service_callbacks_do_not_spin_or_poll_the_backend(harness):
    node = harness.construct(start_driver=False)
    backend = harness.backends[0]
    count = backend.poll_calls
    ready(node, True)
    assert command(node, 'baseline').success
    assert backend.poll_calls == count


@pytest.mark.parametrize('operation', ['baseline', 'grasp', 'open', 'step_close'])
def test_motion_service_requires_explicit_workspace_permission(harness, operation):
    node = harness.construct(start_driver=False)
    backend = harness.backends[0]
    assert not command(node, operation).success
    assert not backend.start_calls
    assert ready(node, True).success
    assert command(node, operation).success
    assert backend.start_calls == [operation]


def test_ready_false_cancels_active_motion_and_rejects_new_motion(harness):
    node = harness.construct(start_driver=False, workspace_clear=True)
    backend = harness.backends[0]
    assert command(node, 'baseline').success
    assert ready(node, False).success
    assert backend.stop_calls >= 1
    backend.state.update(ready=True, busy=False)
    assert not command(node, 'grasp').success
    assert backend.start_calls == ['baseline']


def test_driver_exit_requests_stop_before_poll_and_blocks_further_motion(harness):
    node = harness.construct(workspace_clear=True)
    connect_driver(harness, node)
    backend = harness.backends[0]
    assert command(node, 'baseline').success
    harness.process.returncode = 1
    harness.events.clear()
    node.tick()
    assert harness.events.index('backend.stop') < harness.events.index('backend.poll')
    backend.state.update(ready=True, busy=False)
    assert not command(node, 'open').success
    assert backend.start_calls == ['baseline']


@pytest.mark.parametrize('operation', ['baseline', 'grasp', 'open', 'step_close'])
def test_lost_connection_rejects_operations(harness, operation):
    node = harness.construct(start_driver=False, workspace_clear=True)
    backend = harness.backends[0]
    backend.state.update(ready=False, feedback_valid=False, error='lost connection')
    node.tick()
    assert not command(node, operation).success
    assert not backend.start_calls


@pytest.mark.parametrize('operation,result', [('success', True), ('failure', False)])
def test_result_service_preserves_explicit_operator_result(harness, operation, result):
    node = harness.construct(start_driver=False)
    backend = harness.backends[0]
    assert not command(node, operation).success
    backend.state['can_record'] = True
    assert command(node, operation).success
    assert backend.record_calls == [result]


def test_apply_service_delegates_only_to_a_verified_backend_recommendation(harness):
    node = harness.construct(start_driver=False)
    backend = harness.backends[0]
    assert not command(node, 'apply').success and not backend.apply_calls
    backend.state['recommendation'] = {'force_percent': 20, 'commanded_q': .73}
    result = command(node, 'apply')
    assert result.success and backend.apply_calls == 1
    assert '/not-created/gripper_calibration.yaml' in result.message


def test_unknown_command_is_rejected_without_backend_motion(harness):
    node = harness.construct(start_driver=False, workspace_clear=True)
    assert not command(node, 'unrecognized').success
    assert not harness.backends[0].start_calls


def test_approval_service_consumes_exactly_one_displayed_step(harness):
    node = harness.construct(start_driver=False, workspace_clear=True)
    backend = harness.backends[0]
    token = 'a' * 32
    backend.state['pending_approval'] = {'request_id': token, 'q_rad': .51}
    node.tick()
    relative = '~/approve_next/r' + token
    service = node.registered_services[relative]
    assert node.status()['approval_service'] == '/gripper_calibration/approve_next/r' + token
    result = reply()
    service.callback(None, result)
    assert result.success and backend.approve_calls == [token]
    assert node.status()['approval_service'] is None
    result = reply()
    service.callback(None, result)
    assert not result.success and backend.approve_calls == [token]
    node.tick()
    assert relative not in node.registered_services


def test_stale_approval_cannot_authorize_a_later_displayed_step(harness):
    node = harness.construct(start_driver=False, workspace_clear=True)
    backend = harness.backends[0]
    old_token, new_token = 'a' * 32, 'b' * 32
    backend.state['pending_approval'] = {'request_id': old_token}
    node.tick()
    old_service = node.registered_services['~/approve_next/r' + old_token]
    backend.state['pending_approval'] = {'request_id': new_token}
    node.tick()
    result = reply()
    old_service.callback(None, result)
    assert not result.success and not backend.approve_calls
    assert '~/approve_next/r' + new_token in node.registered_services


def test_status_serializes_unavailable_nonfinite_measurements_as_null(harness):
    node = harness.construct(start_driver=False)
    backend = harness.backends[0]
    backend.state.update(ready=False, feedback_valid=False, age=math.inf, q_rad=math.nan)
    node.tick()
    result = reply()
    node.get_status(None, result)
    assert result.success
    parsed = json.loads(result.message)
    assert 'Infinity' not in result.message and 'NaN' not in result.message
    assert parsed['age'] is None and parsed['q_rad'] is None


@pytest.mark.parametrize('mock,allow', [(True, False), (False, True)])
def test_mock_flag_mismatch_fails_before_a_driver_is_spawned(harness, mock, allow):
    with pytest.raises(ValueError):
        harness.construct(use_mock_hardware=mock, allow_mock_hardware=allow)
    assert not harness.spawned


def test_unconfirmed_action_keeps_owned_driver_alive_on_shutdown(harness):
    node = harness.construct()
    connect_driver(harness, node)
    backend = harness.backends[0]
    backend.state.update(busy=True, can_close=False, phase='CANCELLING')
    backend.close_ready = True  # A hypothetical close shortcut cannot bypass can_close.
    node.request_shutdown()
    assert node.close_tick() is False
    assert not backend.closed and not harness.process.signals
    harness.clock.now += 120.
    assert node.close_tick() is False
    assert harness.process.kill_calls == harness.process.terminate_calls == 0


def test_initialization_before_first_sensor_cannot_use_the_never_commanded_close_shortcut(
        harness, backend):
    backend.locks = []
    backend.node = SimpleNamespace(get_service_names_and_types=lambda: [])
    backend.sensor = {}
    backend.sensor_time = -math.inf
    backend.q_history.clear()
    backend.ever_commanded = False
    backend.configure = lambda **settings: None

    def unsafe_close():
        raise AssertionError('Unobserved initialization motion has no physical stop proof')

    backend.close = unsafe_close
    node = harness.construct(backend_factory_override=lambda *args, **kwargs: backend)
    connect_driver(harness, node)
    assert backend.ever_commanded
    node.request_shutdown()
    assert node.close_tick() is False
    assert not backend.stop_confirmed and not harness.process.signals


def test_shutdown_rejects_new_service_commands_and_permission_changes(harness):
    node = harness.construct(start_driver=False, workspace_clear=True)
    node.request_shutdown()
    assert not command(node, 'baseline').success
    assert not command(node, 'success').success
    assert not ready(node, True).success
    assert not harness.backends[0].start_calls


def test_driver_is_interrupted_only_after_backend_stop_proof_then_locks_release(harness):
    node = harness.construct()
    connect_driver(harness, node)
    backend = harness.backends[0]
    node.request_shutdown()
    assert node.close_tick() is False
    assert not harness.process.signals
    backend.close_ready = True
    assert node.close_tick() is False
    assert backend.closed and harness.process.signals == [signal.SIGINT]
    contender = os.open(harness.lock_path, os.O_RDWR)
    try:
        with pytest.raises(BlockingIOError):
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
        harness.process.returncode = 0
        assert node.close_tick() is True
        fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
    finally:
        os.close(contender)
    assert harness.events.index('backend.close') < next(
        i for i, event in enumerate(harness.events) if event.startswith('driver.signal.'))
    assert harness.process.kill_calls == harness.process.terminate_calls == 0


def test_slow_driver_shutdown_never_escalates_past_one_interrupt(harness):
    node = harness.construct()
    connect_driver(harness, node)
    harness.backends[0].close_ready = True
    node.request_shutdown()
    assert node.close_tick() is False
    harness.clock.now += 120.
    assert node.close_tick() is False
    assert harness.process.signals == [signal.SIGINT]
    assert harness.process.kill_calls == harness.process.terminate_calls == 0
    assert node.driver_leases
    assert not command(node, 'baseline').success


def test_main_preserves_ros_context_through_secondary_shutdown_errors(harness):
    events = []
    close_results = iter([RuntimeError('temporary close failure'), False, True])

    class MainNode:
        def tick(self):
            raise RuntimeError('original polling failure')

        def request_shutdown(self):
            events.append('request_stop')

        def close_tick(self):
            result = next(close_results)
            if isinstance(result, Exception):
                events.append('close.error')
                raise result
            events.append('close.' + str(result))
            return result

        def get_logger(self):
            return SimpleNamespace(error=lambda message: events.append('log.error'))

        def destroy_node(self):
            events.append('node.destroy')

    init_calls, spin_calls = [], []

    def spin_once(*args, **kwargs):
        spin_calls.append(True)
        if len(spin_calls) == 2:
            raise RuntimeError('temporary spin failure')

    no_signal_handlers = object()
    harness.namespace.update(
        HeadlessCalibration=MainNode,
        SignalHandlerOptions=SimpleNamespace(NO=no_signal_handlers),
        rclpy=SimpleNamespace(init=lambda **kwargs: init_calls.append(kwargs),
                              spin_once=spin_once, ok=lambda: True,
                              shutdown=lambda: events.append('context.shutdown')),
        signal=SimpleNamespace(SIGINT=signal.SIGINT, SIGTERM=signal.SIGTERM,
                               signal=lambda *args: None))
    with pytest.raises(RuntimeError, match='original polling failure'):
        harness.namespace['main']()
    assert init_calls[0]['signal_handler_options'] is no_signal_handlers
    assert events[:3] == ['request_stop', 'close.error', 'log.error']
    assert events.index('close.True') < events.index('node.destroy')
    assert events.index('node.destroy') < events.index('context.shutdown')
    assert not harness.spawned


def test_backend_node_ignores_launch_global_name_remaps():
    tree = ast.parse((SOURCE / 'gripper_calibration.py').read_text())
    calls = [node for node in ast.walk(tree)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
             and node.func.attr == 'create_node']
    assert len(calls) == 1
    assert any(keyword.arg == 'use_global_arguments'
               and isinstance(keyword.value, ast.Constant) and keyword.value.value is False
               for keyword in calls[0].keywords)
