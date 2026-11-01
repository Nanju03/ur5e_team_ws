"""Standalone AG95 regressions using the real AST backend and no ROS Node."""
import ast
import inspect
from types import SimpleNamespace

import pytest

import test_gripper_calibration_backend as base


backend = base.backend
CalibrationBackend = base.CalibrationBackend
PLUGIN = 'dh_gripper_driver/DHGripperHardwareInterface'
UR_PLUGIN = 'ur_robot_driver/URPositionHardwareInterface'
HARDWARE_SERVICE = '/gripper/controller_manager/list_hardware_components'
HARDWARE_TYPE = 'controller_manager_msgs/srv/ListHardwareComponents'


def restore(backend, *names):
    for name in names:
        backend.__dict__.pop(name, None)


class Client:
    def service_is_ready(self):
        return False


@pytest.fixture
def standalone(backend, monkeypatch):
    b = backend
    b.gripper_only = True
    b.arm_detected = False
    b.standalone_conflict = False
    b.standalone_conflict_reason = ''
    b.standalone_discovery_started = b.clock.now - 3.
    b.arm_values, b.arm_velocities, b.arm_times, b.arm_reference = {}, {}, {}, {}
    b.arm_quiet_since = None
    b.services = [(HARDWARE_SERVICE, [HARDWARE_TYPE])]
    b.node = SimpleNamespace(get_service_names_and_types=lambda: b.services)
    b.queries = {'controllers': Client(), 'hardware': Client()}
    b.query_futures = {}
    b.query_checked = {name: b.clock.now for name in b.queries}
    b.hardware_ok = True
    b.model_ok = False  # Standalone has no robot_state_publisher RPC.
    b.controllers = {'dh_ag95_gripper_controller': 'active',
                     'force_forward_controller': 'active'}
    b.action.server_is_ready = lambda: True
    monkeypatch.setitem(base.namespace, 'PLUGIN', PLUGIN)
    monkeypatch.setitem(base.namespace, 'MOCK', 'mock_components/GenericSystem')
    restore(b, '_arm_stationary', '_controller_ok')
    return b


def component(kind=PLUGIN, *, state=3, command_names=(), state_names=()):
    return SimpleNamespace(
        class_type=kind, state=SimpleNamespace(id=state),
        command_interfaces=[SimpleNamespace(name=name, is_claimed=True)
                            for name in command_names],
        state_interfaces=[SimpleNamespace(name=name, is_available=True)
                          for name in state_names])


def hardware_reply(backend, *components):
    backend.query_futures['hardware'] = (
        base.Future(SimpleNamespace(component=list(components))), backend.clock.now)
    restore(backend, '_queries')
    backend._queries()


def test_no_arm_allows_explicit_standalone_without_remote_model(standalone):
    b = standalone
    assert not b.arm_values and not b.model_ok
    assert b._arm_stationary() and b._controller_ok()
    b.start('baseline', workspace_clear=True)
    assert b.busy and b.stage == 'FORCE'
    assert not b.goals_sent and not b.force_messages


def test_default_attached_mode_stays_blocked_without_all_six_arm_states(standalone):
    b = standalone
    b.gripper_only = False
    assert not b._arm_stationary()
    with pytest.raises(RuntimeError):
        b.start('baseline', workspace_clear=True)
    assert not b.busy and not b.goals_sent and not b.force_messages


def test_discovery_window_prevents_immediate_standalone_commands(standalone):
    b = standalone
    b.standalone_discovery_started = b.clock.now - 1.99
    assert not b._arm_stationary()
    with pytest.raises(RuntimeError):
        b.start('baseline', workspace_clear=True)


def test_two_complete_discovery_seconds_allow_an_otherwise_ready_standalone(standalone):
    b = standalone
    b.standalone_discovery_started = b.clock.now - 2.
    assert b._mode_guard_ok()


@pytest.mark.parametrize('positions,velocities', [([], []), ([0.], []), ([0.], [0.])])
def test_partial_arm_name_latches_even_without_position_or_velocity(
        standalone, positions, velocities):
    b = standalone
    b._arm(SimpleNamespace(name=[base.namespace['ARM'][0]],
                           position=positions, velocity=velocities))
    assert b.arm_detected
    b._arm(SimpleNamespace(name=[base.namespace['HAND']], position=[.5], velocity=[]))
    b.clock.now += 3.
    assert b.arm_detected and not b._arm_stationary()
    with pytest.raises(RuntimeError):
        b.start('baseline', workspace_clear=True)


def test_prefixed_partial_arm_feedback_also_latches(standalone):
    b = standalone
    b._arm(SimpleNamespace(name=['ur5e_' + base.namespace['ARM'][0]],
                           position=[], velocity=[]))
    assert b.arm_detected and not b._mode_guard_ok()


def test_hand_only_joint_state_does_not_claim_an_arm_exists(standalone):
    b = standalone
    b._arm(SimpleNamespace(name=[base.namespace['HAND']], position=[.5], velocity=[]))
    assert not b.arm_detected and b._arm_stationary()


@pytest.mark.parametrize('manager', ['/controller_manager', '/other/arm/controller_manager'])
def test_another_hardware_manager_is_latched_even_after_its_graph_disappears(
        standalone, manager):
    b = standalone
    b.services.append((manager + '/list_hardware_components', [HARDWARE_TYPE]))
    restore(b, '_queries')
    b._queries()
    assert b.standalone_conflict and not b._arm_stationary()
    b.services = [(HARDWARE_SERVICE, [HARDWARE_TYPE])]
    b._queries()
    assert b.standalone_conflict and not b._arm_stationary()


@pytest.mark.parametrize('state', [1, 2, 3])
def test_mixed_gripper_manager_rejects_ur_hardware_in_any_lifecycle_state(standalone, state):
    b = standalone
    b.mock = False
    hardware_reply(b, component(command_names=[base.namespace['HAND'] + '/position']),
                   component(UR_PLUGIN, state=state))
    assert b.arm_detected and not b._arm_stationary()


@pytest.mark.parametrize('interface_kind', ['command', 'state'])
def test_mixed_gripper_manager_rejects_arm_interfaces_even_for_an_unrecognized_plugin(
        standalone, interface_kind):
    b = standalone
    kwargs = {interface_kind + '_names': [base.namespace['ARM'][0] + '/position']}
    hardware_reply(b, component('other/Hardware', **kwargs))
    assert b.arm_detected and not b._arm_stationary()


def test_mixed_hardware_detection_survives_a_later_gripper_only_reply(standalone):
    b = standalone
    b._inspect_standalone_hardware([component(UR_PLUGIN, state=1)])
    b._inspect_standalone_hardware([
        component(command_names=[base.namespace['HAND'] + '/position'])])
    assert b.arm_detected and not b._mode_guard_ok()


def test_graph_inspection_failure_is_unknown_and_latched_instead_of_absent(standalone):
    b = standalone

    def unavailable_graph():
        raise RuntimeError('discovery unavailable')

    b.node.get_service_names_and_types = unavailable_graph
    b._refresh_standalone_graph()
    assert b.standalone_conflict and b.standalone_conflict_reason
    b.node.get_service_names_and_types = lambda: b.services
    b._refresh_standalone_graph()
    assert not b._mode_guard_ok()


def test_standalone_still_requires_both_controllers_and_fresh_hardware_query(standalone):
    b = standalone
    assert b._controller_ok()
    b.controllers.pop('force_forward_controller')
    assert not b._controller_ok()
    b.controllers['force_forward_controller'] = 'active'
    b.query_checked['hardware'] = b.clock.now - 2.1
    assert not b._controller_ok()


def test_attached_mode_still_requires_matching_remote_model(standalone):
    b = standalone
    b.gripper_only = False
    assert not b._controller_ok()
    b.model_ok = True
    assert b._controller_ok()


@pytest.mark.parametrize('changes', [
    {'age': 1.}, {'raw': 1001}, {'state': 4}, {'force': 19}, {'sequence': 10},
])
def test_invalid_or_replayed_sensor_does_not_bypass_standalone_freshness(standalone, changes):
    b = standalone
    b.clock.now += .6
    values = {'sequence': b.sequence + 1, **changes}
    b._sensor(base.sensor_message(**values))
    assert not b._fresh()
    with pytest.raises(RuntimeError):
        b.start('baseline', workspace_clear=True)
    assert not b.goals_sent and not b.force_messages


def test_active_standalone_trial_is_cancelled_when_an_arm_appears(standalone):
    b = standalone
    b.busy = b.ever_commanded = True
    b.stage = 'GOAL'
    b.send_future = base.Future(ready=False)
    b.goal_sent = b.clock.now
    b.current_trial = base.trial(sensor_valid=False)
    b.trials = [b.current_trial]
    b._arm(SimpleNamespace(name=[base.namespace['ARM'][0]], position=[], velocity=[]))
    status = b.poll()
    assert b.stop_requested and b.current_trial['stopped']
    assert not status['ready'] and not status['can_record']


def test_arm_discovery_during_force_stage_prevents_another_force_command(standalone):
    b = standalone
    b.busy = b.ever_commanded = True
    b.stage, b.operation = 'FORCE', 'baseline'
    b.standalone_conflict = True
    b.poll()
    assert b.stop_requested and not b.force_messages and not b.goals_sent


@pytest.mark.parametrize('operation', ['send', 'record', 'apply'])
def test_latched_conflict_blocks_new_goal_and_trial_or_profile_approval(standalone, operation):
    b = standalone
    b.standalone_conflict = True
    b.mock = False
    base.hold(b)
    with pytest.raises(RuntimeError):
        if operation == 'send':
            b._send(.51)
        elif operation == 'record':
            b.record_result(True)
        else:
            b.apply_recommendation()
    assert not b.goals_sent and b.current_trial['verified_success'] is None


def test_emergency_hold_may_still_be_sent_after_an_arm_conflict(standalone):
    b = standalone
    b.standalone_conflict = True
    b._send(b.sensor['position'], hold=True)
    assert len(b.goals_sent) == 1 and b.stage == 'HOLD_GOAL'


def test_emergency_hold_still_rejects_stale_gripper_feedback(standalone):
    b = standalone
    b.standalone_conflict = True
    b.clock.now += .6
    with pytest.raises(RuntimeError):
        b._send(.5, hold=True)
    assert not b.goals_sent


def test_conflict_does_not_bypass_terminal_then_fresh_quiet_stop_proof(standalone):
    b = standalone
    b.standalone_conflict = True
    b.busy = b.ever_commanded = True
    b.goal = base.Goal()
    b.result_future = base.Future(base.terminal(5))
    b.request_stop()
    b._action_tick()
    assert b.goal is None and b.goal_terminal.status == 5
    b._stop_tick()
    assert not b.stop_confirmed
    b.clock.now += .6
    b.sequence += 3
    base.history(b, b.clock, .5)
    b._stop_tick()
    assert b.stop_confirmed and not b.busy


def test_standalone_status_does_not_report_an_observed_stationary_arm(standalone):
    status = standalone.poll()
    assert status['arm_required'] is False
    assert status['arm_stationary'] is None


def test_attached_constructor_remains_default_and_keeps_remote_model_query():
    parameters = inspect.signature(CalibrationBackend.__init__).parameters
    assert parameters['gripper_only'].default is False
    constructor = next(node for node in base.backend_class.body
                       if isinstance(node, ast.FunctionDef) and node.name == '__init__')
    model_guards = []
    for conditional in ast.walk(constructor):
        if not isinstance(conditional, ast.If):
            continue
        for branch, true_branch in ((conditional.body, True), (conditional.orelse, False)):
            values = [n.value for statement in branch for n in ast.walk(statement)
                      if isinstance(n, ast.Constant) and isinstance(n.value, str)]
            if '/robot_state_publisher/get_parameters' in values:
                model_guards.append((ast.unparse(conditional.test), true_branch))
    assert any('gripper_only' in condition and (('not ' in condition) == true_branch)
               for condition, true_branch in model_guards), model_guards
    assert any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
               and node.func.id == 'load_moveit_settings' for node in ast.walk(constructor))


def test_discovery_window_starts_after_ros_node_creation_not_local_model_loading():
    constructor = next(node for node in base.backend_class.body
                       if isinstance(node, ast.FunctionDef) and node.name == '__init__')
    node_lines = [node.lineno for node in ast.walk(constructor)
                  if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                  and node.func.attr == 'create_node']
    discovery_lines = [node.lineno for node in ast.walk(constructor)
                       if isinstance(node, ast.Assign)
                       and any(isinstance(target, ast.Attribute)
                               and target.attr == 'standalone_discovery_started'
                               for target in node.targets)]
    assert len(node_lines) == 1 and discovery_lines
    assert max(discovery_lines) > node_lines[0]
