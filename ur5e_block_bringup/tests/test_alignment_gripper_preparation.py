"""Reviewed gripper-only alignment preparation; no ROS node or hardware."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'ur5e_block_bringup'
PACKAGE = 'ur5e_alignment_gripper_test_package'
SPEC = importlib.util.spec_from_file_location(PACKAGE, SOURCE/'__init__.py', submodule_search_locations=[str(SOURCE)])
package = importlib.util.module_from_spec(SPEC)
sys.modules[PACKAGE] = package
SPEC.loader.exec_module(package)
SPEC = importlib.util.spec_from_file_location(PACKAGE+'.air_task', SOURCE/'air_task.py')
air = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = air
SPEC.loader.exec_module(air)


def test_preview_holds_arm_and_shows_only_the_reviewed_finger_change():
    arm = [0., -1.5, 1.5, -1.5, -1.5, 0.]
    display, hold = air.AirTask.alignment_prepare_preview(arm, .4, .65)
    assert list(display.trajectory_start.joint_state.position) == arm+[.4]
    assert len(display.trajectory) == 1
    trajectory = display.trajectory[0].joint_trajectory
    assert trajectory.joint_names == air.ARM+[air.HAND]
    assert list(trajectory.points[0].positions) == arm+[.4]
    assert list(trajectory.points[-1].positions) == arm+[.65]
    assert all(list(point.positions) == arm for point in hold.joint_trajectory.points)


def test_low_alignment_holds_the_gripper_shape_more_tightly_than_high_air_paths():
    assert air.frozen_hand_tolerance({'path_kind':'plate_alignment'}) == .003
    assert air.frozen_hand_tolerance({'path_kind':'physical_stack'}) == .003
    assert air.frozen_hand_tolerance({'path_kind':'obstacle_xy_scan'}) == .015
    assert air.frozen_hand_tolerance({}) == .015


def task_harness(monkeypatch):
    task = SimpleNamespace()
    task.stopped = False
    task.values = {name: 0. for name in air.ARM}
    task.values[air.HAND] = .4
    task.parameters = dict(workspace_clear=True, preparation_id='prep_unique')
    task.param = task.parameters.__getitem__
    task.fresh = lambda: True
    task.spin_until = lambda predicate: predicate() or pytest.fail('not ready')
    task.measured_stop = lambda: True
    task.hardware = lambda **kw: 'mock_components/GenericSystem'
    task.input_hashes = lambda: {'layout': 'same'}
    task.scene = lambda: object()
    task.alignment_prepare_preview = air.AirTask.alignment_prepare_preview
    task.alignment_prepare_context = Mock()
    task.validate_gripper_sweep = Mock()
    task.await_approval = Mock()
    task.gripper_runtime = lambda: '/dh_ag95_gripper_controller/gripper_cmd'
    task.phase = Mock()
    task.check_stop = lambda: air.AirTask.check_stop(task)
    task.cancel_remaining_goal = Mock()
    task.goal = task.goal_pending = task.result_future = None
    monkeypatch.setattr(air, 'scene_fingerprint', lambda scene: 'fixed_scene')
    clients = []

    class Client:
        def __init__(self, node, action, endpoint):
            assert node is task and action is air.GripperCommand
            assert endpoint == '/dh_ag95_gripper_controller/gripper_cmd'
            self.goals = []
            clients.append(self)
        def server_is_ready(self):
            return True
        def send_goal_async(self, goal, *, goal_uuid):
            assert len(goal_uuid.uuid) == 16
            self.goals.append(goal)
            task.values[air.HAND] = goal.command.position
            result = SimpleNamespace(status=4, result=SimpleNamespace(reached_goal=True, stalled=False))
            handle = SimpleNamespace(accepted=True, get_result_async=lambda: SimpleNamespace(done=lambda: True, result=lambda: result))
            return SimpleNamespace(done=lambda: True, result=lambda: handle)
        def destroy(self):
            self.destroyed = True
    monkeypatch.setattr(air, 'ActionClient', Client)
    return task, clients


def test_one_approval_sends_one_gripper_goal_and_never_an_arm_goal(monkeypatch, capsys):
    task, clients = task_harness(monkeypatch)
    events = []
    task.await_approval.side_effect = lambda *a, **kw: events.append('approved')
    air.AirTask.prepare_alignment_gripper(task)
    assert events == ['approved'] and len(clients) == 1
    goal = clients[0].goals[0]
    assert goal.command.position == .65 and goal.command.max_effort == 0.
    assert task.validate_gripper_sweep.call_args.args[1:3] == (.4, .65)
    assert task.alignment_prepare_context.call_count == 3
    metadata = task.await_approval.call_args.args[1]
    assert metadata['plan_id'] == 'prep_unique'
    assert task.goal is None and task.goal_pending is None
    assert all(task.values[name] == 0. for name in air.ARM)
    assert 'STACK_STOPPED_CONFIRMED' in capsys.readouterr().out
    assert clients[0].destroyed


def test_stop_while_waiting_for_ok_never_creates_action_client(monkeypatch):
    task, clients = task_harness(monkeypatch)
    task.await_approval.side_effect = RuntimeError('stopped before approval')
    with pytest.raises(RuntimeError, match='stopped'):
        air.AirTask.prepare_alignment_gripper(task)
    assert not clients


def test_preparation_requires_workspace_confirmation_before_preview_or_goal(monkeypatch):
    task, clients = task_harness(monkeypatch)
    task.parameters['workspace_clear'] = False
    with pytest.raises(RuntimeError, match='작업 공간'):
        air.AirTask.prepare_alignment_gripper(task)
    task.await_approval.assert_not_called()
    assert not clients


@pytest.mark.parametrize('preparation_id', ['', '../bad', 'a'*129, None])
def test_bad_preparation_id_cannot_get_an_approval_or_goal(monkeypatch, preparation_id):
    task, clients = task_harness(monkeypatch)
    task.parameters['preparation_id'] = preparation_id
    with pytest.raises(RuntimeError, match='preparation_id'):
        air.AirTask.prepare_alignment_gripper(task)
    task.await_approval.assert_not_called()
    assert not clients


def test_changed_context_after_ok_prevents_a_goal(monkeypatch):
    task, clients = task_harness(monkeypatch)
    task.alignment_prepare_context.side_effect = [None, RuntimeError('arm moved during approval')]
    with pytest.raises(RuntimeError, match='arm moved'):
        air.AirTask.prepare_alignment_gripper(task)
    assert not clients


def test_gripper_sweep_checks_all_angles_and_preserves_the_arm(monkeypatch):
    requests = []
    task = SimpleNamespace(check_stop=lambda: None, bounds={name: (-6., 6.) for name in air.ARM+[air.HAND]}, scene=lambda: None)
    task.call = lambda kind, service, request: requests.append(request) or SimpleNamespace(valid=True)
    monkeypatch.setattr(air, 'scene_fingerprint', lambda _: 'same')
    arm = [0., -1.5, 1.5, -1.5, -1.5, 0.]
    air.AirTask.validate_gripper_sweep(task, arm, .4, .65, 'same')
    values = [request.robot_state.joint_state.position for request in requests]
    assert all(list(q[:6]) == arm for q in values)
    assert values[0][-1] == .4 and values[-1][-1] == .65
    assert all(0 < b[-1]-a[-1] <= .01000001 for a, b in zip(values, values[1:]))


def test_a_collision_prevents_gripper_preparation(monkeypatch):
    task = SimpleNamespace(check_stop=lambda: None, bounds={name: (-6., 6.) for name in air.ARM+[air.HAND]}, scene=lambda: None)
    task.call = lambda *a: SimpleNamespace(valid=False)
    with pytest.raises(RuntimeError, match='충돌'):
        air.AirTask.validate_gripper_sweep(task, [0.]*6, .4, .65, 'same')


@pytest.mark.parametrize('mock,plugin,manager,endpoint', [
    (False, 'dh_gripper_driver/DHGripperHardwareInterface', '/gripper/controller_manager', '/gripper/dh_ag95_gripper_controller/gripper_cmd'),
    (True, 'mock_components/GenericSystem', '/controller_manager', '/dh_ag95_gripper_controller/gripper_cmd'),
])
def test_gripper_runtime_checks_the_exact_plugin_and_claimed_joint(mock, plugin, manager, endpoint):
    services = []
    component = SimpleNamespace(class_type=plugin, state=SimpleNamespace(id=3),
                                command_interfaces=[SimpleNamespace(name=air.HAND+'/position', is_claimed=True)])
    def call(kind, service, request):
        services.append(service)
        return SimpleNamespace(component=[component], controller=[SimpleNamespace(name=air.GRIPPER_CONTROLLER, state='active')])
    task = SimpleNamespace(mock=mock, call=call)
    assert air.AirTask.gripper_runtime(task) == endpoint
    assert services == [manager+'/list_hardware_components', manager+'/list_controllers']
    component.class_type = 'unexpected/hardware'
    with pytest.raises(RuntimeError, match='하드웨어'):
        air.AirTask.gripper_runtime(task)


@pytest.mark.parametrize('hand', [.4, .7])
def test_alignment_cannot_plan_with_a_different_frozen_hand(hand):
    task = SimpleNamespace(spin_until=lambda predicate: None, fresh=lambda: True, plate_alignment=True,
                           values={air.HAND: hand})
    with pytest.raises(RuntimeError, match='0.65rad'):
        air.AirTask.plan(task)


def test_old_point_seven_alignment_plan_is_rejected_even_with_valid_fingerprints(tmp_path, monkeypatch):
    path = tmp_path/'reviewed.cdr'
    meta_path = tmp_path/'reviewed.cdr.meta.json'
    display = air.DisplayTrajectory()
    display.trajectory_start.joint_state.name = air.ARM+[air.HAND]
    display.trajectory_start.joint_state.position = [0.]*6+[.7]
    data = air.serialize_message(display)
    path.write_bytes(data)
    inputs = {'selection': 'same', 'model': 'same'}
    metadata = dict(schema=2, path_kind='plate_alignment', gripper_position=.7,
                    trajectory_sha256=air.digest(data), input_sha256=inputs,
                    model_sha256=air.digest(b'model'), semantic_sha256=air.digest(b'semantic'),
                    created_at=air.time.time())
    metadata['plan_id'] = air.digest(air.json.dumps(metadata, sort_keys=True).encode())[:16]
    meta_path.write_text(air.json.dumps(metadata))
    task = SimpleNamespace(path=path, meta_path=meta_path, param=lambda _: True,
                           path_kind='plate_alignment', plate_alignment=True, xy_scan=False,
                           bounds={air.HAND:(-.01,.94)}, input_hashes=lambda: inputs,
                           model='model', semantic='semantic')
    decoded = Mock(side_effect=AssertionError('Old alignment must be rejected before decoding or execution'))
    monkeypatch.setattr(air, 'deserialize_message', decoded)
    with pytest.raises(RuntimeError, match='0.65rad'):
        air.AirTask.load_reviewed_path(task)
    decoded.assert_not_called()


def test_stop_after_gripper_acceptance_cancels_that_exact_goal(monkeypatch, capsys):
    task, _ = task_harness(monkeypatch)
    pending = {'done': False, 'cancel_calls': 0}
    reply = SimpleNamespace(status=5, result=SimpleNamespace(reached_goal=False, stalled=False))
    result = SimpleNamespace(done=lambda: pending['done'], result=lambda: reply)
    def cancel():
        pending['cancel_calls'] += 1
        pending['done'] = True
    goal_handle = SimpleNamespace(accepted=True, get_result_async=lambda: result, cancel_goal_async=cancel)
    class Client:
        def __init__(self, node, action, endpoint):
            assert action is air.GripperCommand
        server_is_ready = lambda self: True
        def send_goal_async(self, goal, *, goal_uuid):
            return SimpleNamespace(done=lambda: True, result=lambda: goal_handle)
        destroy = lambda self: None
    monkeypatch.setattr(air, 'ActionClient', Client)
    monkeypatch.setattr(air, 'rclpy', SimpleNamespace(spin_once=lambda *args, **kw: setattr(task, 'stopped', True)))
    with pytest.raises(RuntimeError, match='정지'):
        air.AirTask.prepare_alignment_gripper(task)
    assert pending['cancel_calls'] == 1
    assert task.goal is None and task.goal_pending is None
    assert 'STACK_STOPPED_CONFIRMED' in capsys.readouterr().out


def test_preparation_stop_proof_also_requires_the_hand_to_be_quiet(monkeypatch):
    clock = {'now': 0.}
    task = SimpleNamespace(mode='alignment_prepare', velocities={})
    def spin(*args, **kwargs):
        clock['now'] += .1
        task.velocities = {name: (0., clock['now']) for name in air.ARM}
        task.velocities[air.HAND] = (.03, clock['now'])
    monkeypatch.setattr(air, 'time', SimpleNamespace(monotonic=lambda: clock['now']))
    monkeypatch.setattr(air, 'rclpy', SimpleNamespace(spin_once=spin))
    assert air.AirTask.measured_stop(task, seconds=.8) is False
    task.mode = 'dry_run'
    assert air.AirTask.measured_stop(task, seconds=.8) is True


def test_acceptance_transport_error_recovers_only_its_known_uuid_until_terminal_and_quiet(monkeypatch, capsys):
    task, _ = task_harness(monkeypatch)
    clock = {'now': 0., 'spins': 0}
    sent_ids, cancel_ids, result_ids, clients = [], [], [], []
    def failed_result():
        raise RuntimeError('acceptance transport lost')
    class Action:
        def __init__(self, node, kind, endpoint):
            assert kind is air.GripperCommand
        server_is_ready = lambda self: True
        def send_goal_async(self, goal, *, goal_uuid):
            sent_ids.append(list(goal_uuid.uuid))
            return SimpleNamespace(done=lambda: True, result=failed_result)
        destroy = lambda self: None
    class Service:
        def __init__(self, kind, endpoint):
            self.kind, self.endpoint = kind, endpoint
            self.result_calls = 0
            clients.append(self)
        service_is_ready = lambda self: True
        def call_async(self, request):
            if self.kind is air.CancelGoal:
                cancel_ids.append(list(request.goal_info.goal_id.uuid))
                return SimpleNamespace(done=lambda: True)
            result_ids.append(list(request.goal_id.uuid))
            self.result_calls += 1
            # A local result error and an unknown remote status do not prove
            # terminal. The third response finally identifies this goal as canceled.
            if self.result_calls == 1:
                return SimpleNamespace(done=lambda: True, result=failed_result)
            status = 0 if self.result_calls == 2 else 5
            return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(status=status))
        remove_pending_request = lambda self, future: None
    def spin(*args, **kwargs):
        clock['now'] += .5
        clock['spins'] += 1
        assert clock['spins'] < 20
    quiet_checks = []
    def quiet(**kwargs):
        quiet_checks.append(True)
        # The first terminal result still finds a moving gripper.
        return len(quiet_checks) >= 2
    task.measured_stop = lambda **kw: True if not kw else quiet(**kw)
    task.create_client = lambda kind, endpoint: Service(kind, endpoint)
    task.destroy_client = Mock()
    task.recover_gripper_goal = lambda endpoint, goal_id: air.AirTask.recover_gripper_goal(task, endpoint, goal_id)
    monkeypatch.setattr(air, 'ActionClient', Action)
    monkeypatch.setattr(air, 'time', SimpleNamespace(monotonic=lambda: clock['now'], time=lambda: clock['now']))
    monkeypatch.setattr(air, 'rclpy', SimpleNamespace(spin_once=spin))
    with pytest.raises(RuntimeError, match='수락 통신 오류'):
        air.AirTask.prepare_alignment_gripper(task)
    assert sent_ids and cancel_ids and len(result_ids) == 3
    assert all(value == sent_ids[0] for value in cancel_ids+result_ids)
    assert len(quiet_checks) == 2
    assert task.stopped and task.goal_pending is None and task.goal is None
    assert task.destroy_client.call_count == 2
    assert capsys.readouterr().out.count('STACK_STOPPED_CONFIRMED') == 1
