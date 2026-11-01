"""Exercise the real approval gate with scripted spins and no ROS node/socket."""
import copy
import json
import math
import numpy as np
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


SOURCE = Path(__file__).resolve().parents[1] / 'ur5e_block_bringup'
PACKAGE = 'ur5e_air_approval_test_package'
package_spec = importlib.util.spec_from_file_location(
    PACKAGE, SOURCE / '__init__.py', submodule_search_locations=[str(SOURCE)])
package = importlib.util.module_from_spec(package_spec)
sys.modules[PACKAGE] = package
package_spec.loader.exec_module(package)
spec = importlib.util.spec_from_file_location(PACKAGE + '.air_task', SOURCE / 'air_task.py')
air = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = air
spec.loader.exec_module(air)


class Runtime:
    def __init__(self):
        self.now, self.spins = 0., 0
        self.on_spin = lambda task: None

    def monotonic(self):
        return self.now

    def spin_once(self, task, timeout_sec):
        assert 0 < timeout_sec <= .1
        self.spins += 1
        assert self.spins <= 30, 'Approval gate did not stop/accept as scripted'
        self.now += .55
        self.on_spin(task)


class Publisher:
    def __init__(self):
        self.messages = []
        self.error = None

    def publish(self, message):
        if self.error:
            raise self.error
        self.messages.append(copy.deepcopy(message))


class GateHarness:
    preview_digest = staticmethod(air.AirTask.preview_digest)

    def __init__(self):
        self.stopped = False
        self.values = {name: 0. for name in air.ARM}
        self.values[air.HAND] = .5
        self.feedback_fresh = True
        self.display = Publisher()
        self.services, self.destroyed, self.phases, self.context_calls = [], [], [], []
        self.context_error = None

    def create_service(self, kind, name, callback):
        service = SimpleNamespace(name=name, callback=callback)
        self.services.append(service)
        return service

    def destroy_service(self, service):
        self.destroyed.append(service)

    def phase(self, name, **payload):
        self.phases.append((name, payload))

    def fresh(self):
        return self.feedback_fresh

    def check_stop(self):
        air.AirTask.check_stop(self)

    def execution_context(self, metadata, trajectory):
        self.context_calls.append((metadata, trajectory))
        if self.context_error:
            raise self.context_error

    def approve(self, service=None):
        service = self.services[-1] if service is None else service
        return service.callback(None, SimpleNamespace(success=None, message=''))


@pytest.fixture
def runtime(monkeypatch):
    result = Runtime()
    monkeypatch.setattr(air, 'time', result)
    monkeypatch.setattr(air, 'rclpy', SimpleNamespace(spin_once=result.spin_once))
    return result


@pytest.fixture
def path():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = air.ARM
    first, last = JointTrajectoryPoint(), JointTrajectoryPoint()
    for point in (first, last):
        point.positions, point.velocities, point.accelerations = [0.] * 6, [0.] * 6, [0.] * 6
    last.positions = [.01] * 6
    last.time_from_start.sec = 2
    trajectory.joint_trajectory.points = [first, last]
    display = DisplayTrajectory()
    display.trajectory = [trajectory]
    display.trajectory_start.joint_state.name = air.ARM + [air.HAND]
    display.trajectory_start.joint_state.position = [0.] * 6 + [.5]
    return display, dict(plan_id='abc123', gripper_position=.5), trajectory


def run_gate(task, path, index=1, total=2):
    return air.AirTask.await_approval(task, *path, index, total, '현재 경로')


def test_no_approval_keeps_waiting_and_stop_destroys_service(runtime, path):
    task = GateHarness()
    def script(current):
        assert [name for name, _ in current.phases] == ['STACK_STEP_READY']
        if runtime.spins == 5:
            current.stopped = True
    runtime.on_spin = script
    with pytest.raises(RuntimeError, match='stopped'):
        run_gate(task, path)
    assert runtime.spins == 5
    assert len(task.display.messages) >= 2
    assert task.destroyed == task.services
    assert not task.approve().success


def test_only_explicit_single_use_approval_releases_gate(runtime, path):
    task = GateHarness()
    responses = []
    def script(current):
        if runtime.spins == 4:
            responses.append(current.approve())
            responses.append(current.approve())
    runtime.on_spin = script
    run_gate(task, path)
    assert runtime.spins == 4
    assert [response.success for response in responses] == [True, False]
    assert [name for name, _ in task.phases] == ['STACK_STEP_READY', 'STACK_STEP_APPROVED']
    ready, approved = task.phases[0][1], task.phases[1][1]
    assert ready['request_id'] == approved['request_id']
    assert ready['approve_service'] == '/ur5e_a3_stack/approve_next_' + ready['request_id']
    assert ready['preview_sha256'] == air.AirTask.preview_digest(path[0])
    assert task.destroyed == task.services
    assert not task.approve().success


def test_stop_before_callback_rejects_approval(runtime, path):
    task = GateHarness()
    responses = []
    def script(current):
        current.stopped = True
        responses.append(current.approve())
    runtime.on_spin = script
    with pytest.raises(RuntimeError, match='stopped'):
        run_gate(task, path)
    assert not responses[0].success
    assert not any(name == 'STACK_STEP_APPROVED' for name, _ in task.phases)
    assert task.destroyed == task.services


@pytest.mark.parametrize('change', ['feedback', 'hand', 'arm'])
def test_changed_robot_does_not_turn_a_click_into_approved_execution(runtime, path, change):
    task = GateHarness()
    def script(current):
        # Even a callback that accepts the token must not release a changed pose.
        assert current.approve().success
        if change == 'feedback':
            current.feedback_fresh = False
        elif change == 'hand':
            current.values[air.HAND] += .02
        else:
            current.values[air.ARM[0]] += .01
    runtime.on_spin = script
    with pytest.raises(RuntimeError, match='changed while waiting'):
        run_gate(task, path)
    assert not any(name == 'STACK_STEP_APPROVED' for name, _ in task.phases)
    assert task.destroyed == task.services


def test_periodic_context_validation_can_reject_changed_scene_before_approval(runtime, path):
    task = GateHarness()
    task.context_error = RuntimeError('scene changed')
    with pytest.raises(RuntimeError, match='scene changed'):
        run_gate(task, path)
    assert runtime.spins == 4
    assert len(task.context_calls) == 1
    assert task.destroyed == task.services


def test_publish_failure_always_closes_the_approval_service(runtime, path):
    task = GateHarness()
    task.display.error = RuntimeError('display unavailable')
    with pytest.raises(RuntimeError, match='display unavailable'):
        run_gate(task, path)
    assert task.phases == []
    assert task.destroyed == task.services
    assert not task.approve().success


def test_previous_closed_callback_cannot_release_the_next_gate(runtime, path):
    task = GateHarness()
    runtime.on_spin = lambda current: current.approve()
    run_gate(task, path)
    old_service = task.services[0]
    second_start = runtime.spins
    stale_results = []
    def script(current):
        stale_results.append(current.approve(old_service).success)
        if runtime.spins == second_start + 3:
            assert current.approve().success
    runtime.on_spin = script
    run_gate(task, path, index=2)
    assert stale_results == [False, False, False]
    assert len({service.name for service in task.services}) == 2
    assert task.destroyed == task.services
    assert [payload['index'] for name, payload in task.phases if name == 'STACK_STEP_APPROVED'] == [1, 2]


def test_preview_fingerprint_survives_transport_but_detects_changed_motion(path):
    display = path[0]
    transported = air.deserialize_message(air.serialize_message(display), DisplayTrajectory)
    assert air.AirTask.preview_digest(display) == air.AirTask.preview_digest(transported)
    transported.trajectory[0].joint_trajectory.points[-1].positions[0] += .001
    assert air.AirTask.preview_digest(display) != air.AirTask.preview_digest(transported)


class ExecutionHarness:
    segment_display = staticmethod(air.AirTask.segment_display)

    def __init__(self, path, mode):
        self.display, self.metadata, self.trajectory = path
        self.metadata.update(metrics={'segment_end_indices': [1, 2]},
                             segment_labels=['수직 상승', 'XY 이동'], scene_sha256='scene')
        self.execution_mode, self.calls = mode, []
        self.fail_approval = self.fail_after_approval = False
        self.approved = False

    def load_reviewed_path(self):
        return self.metadata, self.display, self.trajectory

    def fresh(self):
        return True

    def spin_until(self, predicate):
        assert predicate()

    def execution_context(self, metadata, trajectory):
        self.calls.append(('context', trajectory))
        if self.approved and self.fail_after_approval:
            raise RuntimeError('robot changed after OK')

    def validate_path(self, *args, **kwargs):
        self.calls.append(('validate', args[0]))

    def await_approval(self, preview, metadata, trajectory, index, total, label):
        self.calls.append(('approval', trajectory, index, total, label, preview))
        if self.fail_approval:
            raise RuntimeError('operator stopped while reviewing')
        assert preview.trajectory[0] == trajectory
        self.approved = True

    def execute_arm(self, trajectory, metadata, index, total, label):
        self.calls.append(('arm', trajectory, index, total, label))
        self.approved = False


@pytest.mark.parametrize('mode,count', [('step', 2), ('continuous', 1)])
def test_common_execution_gate_previews_and_approves_each_actual_dispatched_path(monkeypatch, path, mode, count, capsys):
    task = ExecutionHarness(path, mode)
    segments = [copy.deepcopy(path[2]), copy.deepcopy(path[2])]
    monkeypatch.setattr(air, 'split_trajectory', lambda trajectory, ends: segments)
    air.AirTask.execute(task)
    approvals = [call for call in task.calls if call[0] == 'approval']
    commands = [call for call in task.calls if call[0] == 'arm']
    assert len(approvals) == len(commands) == count
    assert [(call[2], call[3]) for call in approvals] == [(index, count) for index in range(1, count + 1)]
    for approved, command in zip(approvals, commands):
        assert approved[1] is command[1]
        assert approved[5].trajectory[0] == command[1]
        position = task.calls.index(command)
        assert task.calls[position - 1] == ('context', command[1])
    if mode == 'continuous':
        assert commands[0][1] is path[2]
    else:
        assert [command[1] for command in commands] == segments
    assert 'STACK_STATUS phase=DONE' in capsys.readouterr().out


@pytest.mark.parametrize('failure', ['approval', 'post_approval_change'])
def test_common_execution_never_dispatches_before_approval_or_after_changed_state(monkeypatch, path, failure):
    task = ExecutionHarness(path, 'step')
    task.fail_approval = failure == 'approval'
    task.fail_after_approval = failure == 'post_approval_change'
    monkeypatch.setattr(air, 'split_trajectory', lambda trajectory, ends: [trajectory, trajectory])
    with pytest.raises(RuntimeError):
        air.AirTask.execute(task)
    assert not any(call[0] == 'arm' for call in task.calls)


def test_action_server_wait_is_followed_by_another_state_check_before_send(monkeypatch, path):
    calls = []
    client = SimpleNamespace(server_is_ready=lambda: True,
                             send_goal_async=lambda goal: calls.append('send'))
    monkeypatch.setattr(air, 'ActionClient', lambda *args: client)
    def context(*args):
        calls.append('context')
        raise RuntimeError('changed during action-server discovery')
    task = SimpleNamespace(spin_until=lambda predicate: calls.append('server_wait'),
                           execution_context=context)
    with pytest.raises(RuntimeError, match='changed during action-server discovery'):
        air.AirTask.execute_arm(task, path[2], path[1], 1, 1, 'path')
    assert calls == ['server_wait', 'context']


def alignment_execution(mode):
    """Four pocket centres with orthogonal stopped holds and high-only turns."""
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = air.ARM
    labels, points, stops = [], [], []
    hand, floor, hover, high = .65, .820, .8306, 1.10
    initial = [.30, .15, high, 0., 0., 0.]
    def add_point(positions):
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = point.accelerations = [0.]*6
        point.time_from_start.sec = 2*len(trajectory.joint_trajectory.points)
        trajectory.joint_trajectory.points.append(point)
    add_point(initial)
    def move(x, y, z, yaw, label):
        add_point([x, y, z, yaw, 0., 0.])
        labels.append(label)
    move(*initial[:3], 0., '현재 방향을 유지하며 수직 상승')
    move(*initial[:3], 0., '높이를 유지하며 그리퍼 방향 맞추기')
    for index, (corner, cell, x, y) in enumerate((('A', 1, .3109, .15), ('B', 4, .4891, .15),
                                               ('C', 24, .4891, -.15), ('D', 21, .3109, -.15))):
        points.append(dict(cell=index+1, pocket_cell=cell, x=x, y=y, z=hover))
        move(x, y, high, 0., f'{corner} X 위 안전 높이로 접근')
        for axis, yaw in (('X', 0.), ('Y', math.pi/2)):
            stop = dict(corner=corner, axis=axis, pocket_cell=cell, yaw_deg=0 if axis == 'X' else 90,
                        x=x, y=y, z=hover,
                        orientation_xyzw=[0., 0., math.sin(yaw/2), math.cos(yaw/2)])
            move(x, y, hover, yaw, air.alignment_hold_label(stop))
            stop['after_segment'] = len(labels)
            stops.append(stop)
            if corner != 'D' or axis != 'Y':
                move(x, y, high, yaw, air.alignment_retract_label(stop))
                next_yaw = math.pi/2 if axis == 'X' else 0.
                move(x, y, high, next_yaw, air.alignment_turn_label(stop))
    display = DisplayTrajectory()
    display.trajectory = [trajectory]
    display.trajectory_start.joint_state.name = air.ARM+[air.HAND]
    display.trajectory_start.joint_state.position = initial+[hand]
    metadata = dict(plan_id='alignment', gripper_position=hand, path_kind='plate_alignment',
                    alignment_policy=air.ALIGNMENT_POLICY, alignment_stops=stops,
                    alignment_rotation_clearance=.05, scan_min_gripper_z=floor, scan_cells=points)
    task = ExecutionHarness((display, metadata, trajectory), mode)
    metadata.update(metrics=dict(segment_end_indices=list(range(1, len(labels)+1))), segment_labels=labels)
    return task


def test_continuous_alignment_ends_each_exact_group_at_one_of_eight_xy_holds(capsys):
    task = alignment_execution('continuous')
    air.AirTask.execute(task)
    approvals = [call for call in task.calls if call[0] == 'approval']
    commands = [call for call in task.calls if call[0] == 'arm']
    assert len(approvals) == len(commands) == 8
    assert [(call[2], call[3]) for call in approvals] == [(i, 8) for i in range(1, 9)]
    original = task.trajectory.joint_trajectory.points
    previous = 0
    for command, stop in zip(commands, task.metadata['alignment_stops']):
        group = command[1].joint_trajectory.points
        end = stop['after_segment']
        for copied, source in zip(group, original[previous:end+1]):
            assert copied.positions == source.positions
            assert copied.velocities == source.velocities
            assert copied.accelerations == source.accelerations
            assert copied.time_from_start.sec == source.time_from_start.sec-original[previous].time_from_start.sec
            assert copied.time_from_start.nanosec == source.time_from_start.nanosec
        assert len(group) == end-previous+1
        previous = end
    pauses = [json.loads(line.split(' ', 1)[1]) for line in capsys.readouterr().out.splitlines()
              if line.startswith('STACK_ALIGNMENT_PAUSED ')]
    assert [p['stop_index'] for p in pauses] == list(range(1, 9))
    assert [(p['corner'], p['axis']) for p in pauses] == [(c, a) for c in 'ABCD' for a in 'XY']
    assert all(p['total'] == 8 for p in pauses)


@pytest.mark.parametrize('withheld', range(2, 9))
def test_each_hold_prevents_the_next_retract_until_a_fresh_ok(withheld, capsys):
    task = alignment_execution('continuous')
    original_gate = task.await_approval
    def gate(preview, metadata, trajectory, index, total, label):
        if index == withheld:
            assert 'STACK_ALIGNMENT_PAUSED' in capsys.readouterr().out
            raise RuntimeError('operator is still adjusting the plate')
        return original_gate(preview, metadata, trajectory, index, total, label)
    task.await_approval = gate
    with pytest.raises(RuntimeError, match='still adjusting'):
        air.AirTask.execute(task)
    commands = [call for call in task.calls if call[0] == 'arm']
    assert len(commands) == withheld-1
    end = task.metadata['alignment_stops'][withheld-2]['after_segment']
    assert commands[-1][1].joint_trajectory.points[-1].positions == task.trajectory.joint_trajectory.points[end].positions


def test_step_alignment_preserves_all_segments_and_eight_stationary_adjustment_stops(capsys):
    task = alignment_execution('step')
    air.AirTask.execute(task)
    approvals = [call for call in task.calls if call[0] == 'approval']
    assert len(approvals) == len(task.metadata['segment_labels']) == 28
    for stop in task.metadata['alignment_stops'][:-1]:
        assert '판 조정 후 손을 빼고 OK' in approvals[stop['after_segment']][4]
    assert capsys.readouterr().out.count('STACK_ALIGNMENT_PAUSED ') == 8


@pytest.mark.parametrize('change', ['old_policy', 'legacy_single_stop', 'missing_stop', 'duplicate_stop',
                                   'wrong_axis', 'wrong_yaw', 'wrong_pocket', 'wrong_xy', 'wrong_orientation',
                                   'non_unit_quaternion', 'missing_retract', 'missing_turn', 'missing_last_hold',
                                   'unsafe_rotation_clearance', 'bad_scan_points', 'moving_acceleration'])
def test_old_or_changed_xy_stop_metadata_is_rejected_before_any_approval(change):
    task = alignment_execution('continuous')
    m = task.metadata
    if change == 'old_policy': m['alignment_policy'] = 'first_A_only'
    elif change == 'legacy_single_stop':
        m.pop('alignment_stops')
        m['alignment_pause_after_segment'] = 4
    elif change == 'missing_stop': m['alignment_stops'].pop()
    elif change == 'duplicate_stop': m['alignment_stops'][1]['after_segment'] = 4
    elif change == 'wrong_axis': m['alignment_stops'][1]['axis'] = 'X'
    elif change == 'wrong_yaw': m['alignment_stops'][1]['yaw_deg'] = False
    elif change == 'wrong_pocket': m['alignment_stops'][3]['pocket_cell'] = 1
    elif change == 'wrong_xy': m['alignment_stops'][1]['y'] += .001
    elif change == 'wrong_orientation': m['alignment_stops'][1]['orientation_xyzw'] = [0., 0., 0., 1.]
    elif change == 'non_unit_quaternion': m['alignment_stops'][0]['orientation_xyzw'] = [0., 0., 0., 2.]
    elif change == 'missing_retract': m['segment_labels'][4] = 'low turn'
    elif change == 'missing_turn': m['segment_labels'][5] = 'low turn'
    elif change == 'missing_last_hold': m['alignment_stops'][-1]['after_segment'] = 27
    elif change == 'unsafe_rotation_clearance': m['alignment_rotation_clearance'] = .005
    elif change == 'bad_scan_points': m['scan_cells'] = m['scan_cells'][:1]
    elif change == 'moving_acceleration': task.trajectory.joint_trajectory.points[4].accelerations = [.01]*6
    with pytest.raises(RuntimeError, match='X/Y'):
        air.AirTask.execute(task)
    assert not any(call[0] in ('approval', 'arm') for call in task.calls)


def test_alignment_pause_cannot_split_a_moving_boundary():
    task = alignment_execution('continuous')
    task.trajectory.joint_trajectory.points[4].velocities = [.01]*6
    with pytest.raises(ValueError, match='nonzero velocity'):
        air.AirTask.execute(task)
    assert not any(call[0] in ('approval', 'arm') for call in task.calls)


class AlignmentKinematics:
    def tcp(self, joints):
        x, y, z, yaw = [joints[name] for name in air.ARM[:4]]
        result = np.eye(4)
        c, s = math.cos(yaw), math.sin(yaw)
        result[:3, :3] = ((c, -s, 0.), (s, c, 0.), (0., 0., 1.))
        result[:3, 3] = (x, y, z)
        return result

    def gripper_min_z(self, joints, hand):
        assert hand == .65
        return joints[air.ARM[2]]-.0106


def alignment_geometry_task():
    task = alignment_execution('continuous')
    task.kinematics = AlignmentKinematics()
    values = dict(plate_alignment_cell_ids=[1, 4, 24, 21], plate_alignment_clearance=.05,
                  plate_alignment_points_xyz=[v for p in task.metadata['scan_cells'] for v in (p['x'], p['y'], .805)])
    task.param = values.__getitem__
    segments = air.split_trajectory(task.trajectory, task.metadata['metrics']['segment_end_indices'])
    return task, segments


def test_all_eight_fk_holds_and_high_only_quarter_turns_match_the_sealed_geometry():
    task, segments = alignment_geometry_task()
    air.AirTask.validate_alignment_geometry(task, task.metadata, segments)


@pytest.mark.parametrize('change', ['actual_hold_xy', 'actual_hold_yaw', 'actual_tip_height',
                                   'sideways_retract', 'low_rotation', 'rotating_retract', 'changed_pocket'])
def test_altered_real_geometry_cannot_pass_valid_labels_or_reseal_a_low_turn(change):
    task, segments = alignment_geometry_task()
    if change == 'actual_hold_xy': segments[3].joint_trajectory.points[-1].positions[0] += .002
    elif change == 'actual_hold_yaw': segments[6].joint_trajectory.points[-1].positions[3] -= .1
    elif change == 'actual_tip_height':
        task.kinematics.gripper_min_z = lambda *args: .80
    elif change == 'sideways_retract':
        original = segments[4].joint_trajectory.points
        middle = copy.deepcopy(original[0])
        middle.time_from_start.sec = 1
        middle.positions = [(a+b)/2 for a, b in zip(original[0].positions, original[-1].positions)]
        middle.positions[0] += .003
        segments[4].joint_trajectory.points.insert(1, middle)
    elif change == 'low_rotation':
        for point in segments[5].joint_trajectory.points: point.positions[2] = .85
    elif change == 'rotating_retract':
        segments[4].joint_trajectory.points[-1].positions[3] = .1
    elif change == 'changed_pocket':
        original_param = task.param
        task.param = lambda name: [4, 1, 24, 21] if name == 'plate_alignment_cell_ids' else original_param(name)
    with pytest.raises(RuntimeError, match='X/Y'):
        air.AirTask.validate_alignment_geometry(task, task.metadata, segments)
