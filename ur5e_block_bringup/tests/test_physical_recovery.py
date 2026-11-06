"""Recovery contracts without ROS nodes, scene writes or controller goals."""
import copy
from concurrent.futures import Future
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject, CollisionObject,
                             PlanningScene, RobotTrajectory)
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5e_block_bringup import physical_recovery as recovery
from ur5e_block_bringup import physical_stack as physical
from ur5e_block_bringup.physical_stack import PhysicalStackTask, JOINTS, scene_fingerprint


def test_recovery_keeps_strict_native_goal_and_matching_result_diagnostics(monkeypatch, capsys):
    path = arm_path(end=[.1]*6)
    step = dict(index=1, kind='arm', phase='approach', label='recovery home',
                start_joints=[0.]*6+[.4], end_joints=[.1]*6+[.4])
    accepted, finished = Future(), Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: finished, cancel_goal_async=Mock())
    accepted.set_result(handle)
    client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=Mock(return_value=accepted), destroy=Mock())
    monkeypatch.setattr(physical, 'ActionClient', lambda *args: client)
    monkeypatch.setattr(physical, 'file_digest', lambda _: 'recipe')
    task = SimpleNamespace(
        carrying=False, release_stage=0, values=dict(zip(JOINTS, [0.]*6+[.4])),
        spin_until=Mock(), guard=Mock(), phase=Mock(), cancel_remaining_goal=Mock(),
        recover_pending=Mock(), destroy_flight_queries=Mock(), poll_flight_queries=Mock(),
        flight_queries=lambda *_: [], prewarm_flight_queries=Mock(), fresh=lambda: True,
        sensor_fresh=lambda **_: True, sensor_sequence=10, stopped=False, check_stop=Mock(),
        measured_stop=lambda **_: True, holding_ok=lambda: True,
        input_hashes=lambda: {}, path=Path('/not/read'), execution_mode='step')
    task.motion_execution_profile = lambda: recovery.PhysicalRecoveryTask.motion_execution_profile(task)
    def spin(*args, **kwargs):
        task.values.update(dict(zip(recovery.ARM, [.1]*6)))
        if not finished.done():
            finished.set_result(SimpleNamespace(status=4,
                result=SimpleNamespace(error_code=0, error_string='success')))
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    PhysicalStackTask.run_motion(task, step, path,
        dict(input_sha256={}, recipe_sha256='recipe'), 'scene', 1, 1)
    goal = client.send_goal_async.call_args.args[0]
    assert [tol.position for tol in goal.path_tolerance] == [.005]*6
    assert all(tol.position == .001 and tol.velocity == .01 for tol in goal.goal_tolerance)
    assert goal.goal_time_tolerance.sec == 5
    record = next(json.loads(line.split('STACK_ARM_RESULT ', 1)[1])
                  for line in capsys.readouterr().out.splitlines() if 'STACK_ARM_RESULT ' in line)
    assert record['path_position_tolerance'] == dict.fromkeys(recovery.ARM, .005)
    assert record['goal_position_tolerance'] == record['endpoint_position_tolerance'] == .001
    assert record['classification'] == 'SUCCESS'
    assert physical.arm_execution_profile()['phase_path_position']['approach'] == [.1]*6


def bounds():
    return {name: (-6.3, 6.3) for name in recovery.ARM} | {recovery.HAND: (0., .93)}


def semantic(values=None):
    values = [0.]*6 if values is None else values
    return '<robot><group_state group="ur5e_arm" name="home">'+''.join(
        f'<joint name="{name}" value="{q}"/>' for name, q in zip(recovery.ARM, values))+'</group_state></robot>'


def arm_path(start=None, end=None):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(recovery.ARM)
    for index, positions in enumerate((start or [0.]*6, end or [.1]*6)):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities, point.accelerations = [0.]*6, [0.]*6
        point.time_from_start.sec = index
        result.joint_trajectory.points.append(point)
    return result


def cube(name):
    obj = CollisionObject()
    obj.id, obj.header.frame_id = name, 'world'
    shape = SolidPrimitive()
    shape.type, shape.dimensions = SolidPrimitive.BOX, [.025]*3
    pose = Pose()
    pose.orientation.w = 1.
    obj.primitives, obj.primitive_poses = [shape], [pose]
    return obj


def scene_with_owned_and_foreign():
    scene = PlanningScene()
    scene.world.collision_objects = [cube('ur5e_stack/block_6'), cube('foreign_fixture')]
    owned, foreign = AttachedCollisionObject(), AttachedCollisionObject()
    owned.object, owned.link_name = cube('ur5e_stack/held'), 'grasp_link'
    foreign.object, foreign.link_name = cube('foreign_tool'), 'tool0'
    scene.robot_state.attached_collision_objects = [owned, foreign]
    matrix = scene.allowed_collision_matrix
    matrix.entry_names = ['foreign_fixture', 'ur5e_stack/held', 'foreign_tool']
    matrix.entry_values = [AllowedCollisionEntry(enabled=row) for row in
                           ([False, True, False], [True, True, True], [False, True, True])]
    matrix.default_entry_names, matrix.default_entry_values = ['ur5e_stack/held', 'foreign_tool'], [True, False]
    return scene


def empty_task(state=1, fresh=True, sensor_fresh=True, quiet=True, confirmed=True):
    obj = SimpleNamespace(check_stop=Mock(), fresh=Mock(return_value=fresh),
        sensor_fresh=Mock(return_value=sensor_fresh), measured_stop=Mock(return_value=quiet),
        mock=False, sensor={'grip_state': state}, has_parameter=Mock(return_value=True),
        param=lambda name: confirmed)
    return obj


@pytest.mark.parametrize('state', [0, 2, 3])
def test_recovery_never_treats_moving_held_or_drop_as_empty(state):
    task = empty_task(state)
    with pytest.raises(RuntimeError, match='빈 그리퍼 상태'):
        recovery.PhysicalRecoveryTask.require_empty(task)
    task.measured_stop.assert_not_called()
    assert not recovery.PhysicalRecoveryTask.holding_ok(task)


@pytest.mark.parametrize('failure', ['joints_stale', 'sensor_stale', 'not_quiet', 'unconfirmed'])
def test_empty_confirmation_is_additional_to_fresh_sensor_and_standstill(failure):
    task = empty_task(fresh=failure != 'joints_stale', sensor_fresh=failure != 'sensor_stale',
                      quiet=failure != 'not_quiet', confirmed=failure != 'unconfirmed')
    with pytest.raises(RuntimeError):
        recovery.PhysicalRecoveryTask.require_empty(task)


def test_confirmed_fresh_empty_stationary_state_passes_without_actuator_command():
    task = empty_task()
    recovery.PhysicalRecoveryTask.require_empty(task)
    task.measured_stop.assert_called_once_with()


@pytest.mark.parametrize('state',[0,1,2,3])
def test_preserving_home_accepts_any_initial_grip_code_without_empty_declaration(state):
    task=empty_task(state)
    task.preserve_gripper=True
    task.recovery_hand_position=.73
    task.values={recovery.HAND:.73}
    task.param=lambda name:name=='workspace_clear'
    recovery.PhysicalRecoveryTask.require_empty(task)
    assert recovery.PhysicalRecoveryTask.holding_ok(task)
    task.measured_stop.assert_called_once_with()


@pytest.mark.parametrize('fault',['stale','moving','hand_changed','not_confirmed'])
def test_preserving_home_still_requires_feedback_and_an_unchanged_stationary_hand(fault):
    task=empty_task(2,sensor_fresh=fault!='stale',quiet=fault!='moving',confirmed=fault!='not_confirmed')
    task.preserve_gripper=True
    task.recovery_hand_position=.73
    task.values={recovery.HAND:.70 if fault=='hand_changed' else .73}
    with pytest.raises(RuntimeError):
        recovery.PhysicalRecoveryTask.require_empty(task)


@pytest.mark.parametrize('initial,current,stop',[(2,3,True),(3,3,False),(2,2,False),(0,1,False)])
def test_preserving_recovery_accepts_initial_stages_but_detects_a_new_payload_drop(monkeypatch,initial,current,stop):
    task=recovery.PhysicalRecoveryTask.__new__(recovery.PhysicalRecoveryTask)
    task.preserve_gripper=True
    task.mock=False
    task.goal_pending=object()
    task.recovery_initial_grip_state=initial
    task.sensor_fresh=lambda:True
    task.sensor={}
    monkeypatch.setattr(physical.PhysicalStackTask,'sensor_feedback',lambda self,msg:self.sensor.update(grip_state=current))
    stopped=Mock()
    monkeypatch.setattr(recovery,'stop_with_reason',stopped)
    task.sensor_feedback(object())
    assert stopped.called==stop
    if stop:assert stopped.call_args.args[1]=='RECOVERY_PAYLOAD_DROPPED'


class RecoveryFeedbackReady(Exception):
    """Stop the preflight before model/services/planning or any motion."""


def startup_feedback_task():
    task = empty_task()
    task.require_empty = lambda: recovery.PhysicalRecoveryTask.require_empty(task)
    task.check_model = Mock(side_effect=RecoveryFeedbackReady)
    task.sensor_stale_details = Mock(return_value=dict(sample_sequence=-1,last_rejection=None))
    return task


def test_recovery_waits_for_first_sensor_after_joint_feedback_before_empty_check():
    task = startup_feedback_task()
    task.sensor_fresh.return_value = False
    def spin(predicate, seconds):
        assert seconds == 5.
        # This is the startup order that used to fail: all joints arrived but
        # the sensor callback has not yet been dispatched by spin_once.
        assert task.fresh() and not predicate()
        task.measured_stop.assert_not_called()
        task.check_model.assert_not_called()
        task.sensor_fresh.return_value = True
        assert predicate()
    task.spin_until = spin
    with pytest.raises(RecoveryFeedbackReady):
        recovery.PhysicalRecoveryTask.plan_recovery(task)
    task.measured_stop.assert_called_once_with()


@pytest.mark.parametrize('state', [0, 2, 3])
def test_sensor_startup_wait_does_not_treat_moving_held_or_drop_feedback_as_empty(state):
    task = startup_feedback_task()
    task.sensor['grip_state'] = state
    task.spin_until = lambda predicate, seconds: predicate()
    with pytest.raises(RuntimeError, match='빈 그리퍼 상태'):
        recovery.PhysicalRecoveryTask.plan_recovery(task)
    task.measured_stop.assert_not_called()
    task.check_model.assert_not_called()


def test_recovery_missing_first_sensor_times_out_without_planning_or_empty_override(capsys):
    task = startup_feedback_task()
    task.sensor_fresh.return_value = False
    task.require_empty = Mock()
    def spin(predicate, seconds):
        assert not predicate()
        raise TimeoutError('feedback unavailable')
    task.spin_until = spin
    with pytest.raises(RuntimeError, match='5초 동안'):
        recovery.PhysicalRecoveryTask.plan_recovery(task)
    task.require_empty.assert_not_called()
    task.check_model.assert_not_called()
    assert 'STACK_RECOVERY_FEEDBACK_TIMEOUT' in capsys.readouterr().out


def test_operator_stop_during_first_sensor_wait_cannot_start_recovery_planning():
    task = startup_feedback_task()
    task.require_empty = Mock()
    task.spin_until = Mock(side_effect=RuntimeError('operator stopped'))
    with pytest.raises(RuntimeError, match='operator stopped'):
        recovery.PhysicalRecoveryTask.plan_recovery(task)
    task.require_empty.assert_not_called()
    task.check_model.assert_not_called()


def test_home_comes_from_exact_named_six_axis_state():
    assert recovery.home_joints(semantic([.1]*6), bounds()) == [.1]*6


@pytest.mark.parametrize('bad', ['missing', 'duplicate_state', 'duplicate_joint', 'nan', 'outside', 'missing_joint'])
def test_invalid_or_guessed_home_is_rejected(bad):
    text = semantic()
    if bad == 'missing':
        text = '<robot/>'
    elif bad == 'duplicate_state':
        text = text.replace('</robot>', text[len('<robot>'):])
    elif bad == 'duplicate_joint':
        text = text.replace('</group_state>', '<joint name="shoulder_pan_joint" value="0"/></group_state>')
    elif bad == 'nan':
        text = semantic([math.nan]*6)
    elif bad == 'outside':
        text = semantic([20.]*6)
    else:
        text = semantic([0.]*5)
    with pytest.raises(ValueError):
        recovery.home_joints(text, bounds())


def test_cleanup_preserves_foreign_world_attachment_octomap_and_acm_pairs():
    live = scene_with_owned_and_foreign()
    before = copy.deepcopy(live)
    result = recovery.without_owned_scene(live)
    assert [o.id for o in result.world.collision_objects] == ['foreign_fixture']
    assert [o.object.id for o in result.robot_state.attached_collision_objects] == ['foreign_tool']
    assert result.world.collision_objects[0] == before.world.collision_objects[1]
    assert result.robot_state.attached_collision_objects[0] == before.robot_state.attached_collision_objects[1]
    assert result.world.octomap == before.world.octomap
    assert result.allowed_collision_matrix.entry_names == ['foreign_fixture', 'foreign_tool']
    assert [list(r.enabled) for r in result.allowed_collision_matrix.entry_values] == [[False, False], [False, True]]
    assert result.allowed_collision_matrix.default_entry_names == ['foreign_tool']
    assert result.allowed_collision_matrix.default_entry_values == [False]
    assert live == before


@pytest.mark.parametrize('which', ['rows', 'width', 'defaults'])
def test_malformed_collision_matrix_is_never_overwritten(which):
    scene = scene_with_owned_and_foreign()
    matrix = scene.allowed_collision_matrix
    if which == 'rows':
        matrix.entry_values.pop()
    elif which == 'width':
        matrix.entry_values[0].enabled.pop()
    else:
        matrix.default_entry_values.pop()
    with pytest.raises(ValueError, match='Malformed live'):
        recovery.without_owned_scene(scene)


@pytest.mark.parametrize('diagonal', [[1., 1., 1.], [1., -1., -1.], [-1., 1., -1.], [-1., -1., 1.]])
def test_fk_orientation_is_preserved_for_identity_and_half_turns(diagonal):
    transform = np.eye(4)
    transform[:3, :3] = np.diag(diagonal)
    transform[:3, 3] = [.3, -.1, .9]
    pose = recovery.transform_pose(transform)
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                       [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                       [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])
    assert np.allclose(matrix, transform[:3, :3])
    assert [pose.position.x, pose.position.y, pose.position.z] == [.3, -.1, .9]


def test_planner_joint_order_and_boundary_derivatives_are_canonicalized():
    source = arm_path()
    source.joint_trajectory.joint_names = list(reversed(recovery.ARM))
    source.joint_trajectory.points[1].positions = [i*.01 for i in range(6)]
    source.joint_trajectory.points[0].velocities = [.01]*6
    source.joint_trajectory.points[-1].accelerations = [.02]*6
    path = recovery.canonical_recovery_path(source)
    assert list(path.joint_trajectory.joint_names) == recovery.ARM
    assert list(path.joint_trajectory.points[-1].positions) == [.05, .04, .03, .02, .01, .0]
    assert path.joint_trajectory.points[-1].time_from_start.sec >= 1
    assert all(list(p.velocities) == [0.]*6 and list(p.accelerations) == [0.]*6
               for p in path.joint_trajectory.points)


def test_smooth_planner_interior_velocity_is_preserved_and_true_limits_are_rechecked():
    source = arm_path(end=[.2]*6)
    source.joint_trajectory.points[-1].time_from_start.sec = 2
    middle = JointTrajectoryPoint()
    middle.positions, middle.velocities, middle.accelerations = [.1]*6, [.1]*6, [0.]*6
    middle.time_from_start.sec = 1
    source.joint_trajectory.points.insert(1, middle)
    task = SimpleNamespace(recovery_steps=[], bounds=bounds(), validate_samples=Mock(),
        joint_velocity_limits={name: 2.1 for name in recovery.ARM},
        joint_acceleration_limits={name: 1. for name in recovery.ARM}, stretch=PhysicalStackTask.stretch)
    task.normalize_arm = lambda *a: PhysicalStackTask.normalize_arm(task, *a)
    task.motion_samples = lambda *a: PhysicalStackTask.motion_samples(task, *a)
    step, path = recovery.PhysicalRecoveryTask.arm_step(task, source, [0.]*6+[.4], 'home')
    assert all(abs(v) > 0. for v in path.joint_trajectory.points[1].velocities)
    assert step['metrics']['max_velocity'] <= .105001
    assert step['metrics']['max_acceleration'] <= .050001
    assert recovery.trajectory_duration(path) < 6.
    for point in (path.joint_trajectory.points[0], path.joint_trajectory.points[-1]):
        assert list(point.velocities) == list(point.accelerations) == [0.]*6


@pytest.mark.parametrize('invalid', ['no_derivatives', 'nonincreasing_time', 'nan_velocity'])
def test_invalid_planner_timing_is_rejected_instead_of_silently_reused(invalid):
    path = arm_path()
    if invalid == 'no_derivatives':
        path.joint_trajectory.points[0].velocities = []
    elif invalid == 'nonincreasing_time':
        path.joint_trajectory.points[-1].time_from_start.sec = 0
    else:
        # Interior NaNs must not be hidden by the deliberate zero end values.
        middle = copy.deepcopy(path.joint_trajectory.points[-1])
        middle.time_from_start.nanosec = 500_000_000
        middle.time_from_start.sec = 0
        middle.velocities = [math.nan]*6
        path.joint_trajectory.points.insert(1, middle)
    with pytest.raises(ValueError):
        recovery.canonical_recovery_path(path)


@pytest.mark.parametrize('failure', ['jump', 'nan', 'wrong_joints', 'too_short'])
def test_planner_jumps_and_invalid_paths_are_rejected(failure):
    source = arm_path()
    if failure == 'jump':
        source.joint_trajectory.points[-1].positions = [.36]*6
    elif failure == 'nan':
        source.joint_trajectory.points[-1].positions = [math.nan]*6
    elif failure == 'wrong_joints':
        source.joint_trajectory.joint_names[-1] = recovery.HAND
    else:
        source.joint_trajectory.points.pop()
    with pytest.raises(ValueError):
        recovery.canonical_recovery_path(source)


def test_recovery_arm_timing_and_all_frozen_hand_samples_respect_slow_profile():
    task = SimpleNamespace(recovery_steps=[], bounds=bounds(), validate_samples=Mock(),
        joint_velocity_limits={name: 2.1 for name in recovery.ARM},
        joint_acceleration_limits={name: 1. for name in recovery.ARM}, stretch=PhysicalStackTask.stretch)
    task.normalize_arm = lambda *a: PhysicalStackTask.normalize_arm(task, *a)
    task.motion_samples = lambda *a: PhysicalStackTask.motion_samples(task, *a)
    step, path = recovery.PhysicalRecoveryTask.arm_step(task, arm_path(), [0.]*6+[.7161], 'home')
    assert step['phase'] == 'approach'
    assert step['metrics']['max_velocity'] <= .105001
    assert step['metrics']['max_acceleration'] <= .050001
    assert all(q[-1] == .7161 for _, q in task.validate_samples.call_args.args[0])


@pytest.mark.parametrize('failure', ['sideways', 'downward', 'rotation'])
def test_a_collision_free_path_cannot_replace_the_required_vertical_lift(failure):
    start, target = np.eye(4), np.eye(4)
    start[2, 3], target[2, 3] = .83, 1.1
    middle = start.copy()
    middle[2, 3] = .9
    if failure == 'sideways':
        middle[0, 3] = .01
    elif failure == 'rotation':
        middle[:3, :3] = np.diag([-1., -1., 1.])
    else:
        middle[2, 3] = .82
    task = SimpleNamespace(motion_samples=lambda *_: [(0., [0.]*7), (1., [0.]*7)],
        kinematics=SimpleNamespace(tcp=Mock(side_effect=[start, middle])))
    with pytest.raises(RuntimeError, match='수직 이동'):
        recovery.PhysicalRecoveryTask.validate_vertical(task, {}, None, start, target)


def completion_task():
    live = scene_with_owned_and_foreign()
    desired = recovery.without_owned_scene(live)
    task = SimpleNamespace(require_empty=Mock(), check_files=Mock(), recovery_home=[0.]*6,
        values=dict(zip(JOINTS, [0.]*6+[.4])), scene=Mock(side_effect=[live, desired]),
        apply_snapshot=Mock(), phase=Mock(), recovery_finished=False)
    metadata = dict(plan_id='recover_test', scene_sha256=scene_fingerprint(live))
    return task, metadata, live


def test_fault_is_cleared_only_after_home_empty_and_acknowledged_exact_scene_cleanup(monkeypatch):
    task, metadata, live = completion_task()
    order = []
    task.apply_snapshot.side_effect = lambda _: order.append('acknowledged cleanup')
    clear = Mock(side_effect=lambda: order.append('clear fault'))
    monkeypatch.setattr(recovery, 'clear_held_fault', clear)
    recovery.PhysicalRecoveryTask.complete_recovery(task, metadata)
    assert order == ['acknowledged cleanup', 'clear fault']
    assert task.recovery_finished
    task.phase.assert_called_once_with('STACK_RECOVERY_DONE', plan_id='recover_test', home_reached=True,
                                      gripper_empty=True, hand_target=.4, scene_cleared=True)
    assert [o.id for o in task.apply_snapshot.call_args.args[0].world.collision_objects] == ['foreign_fixture']
    assert len(live.robot_state.attached_collision_objects) == 2


@pytest.mark.parametrize('failure', ['wrong_home', 'wrong_hand', 'not_empty', 'lost_ack', 'wrong_scene',
                                    'stop_during_cleanup', 'hand_drift_after_cleanup'])
def test_failed_recovery_never_clears_fault_or_reports_done(monkeypatch, failure):
    task, metadata, live = completion_task()
    if failure == 'wrong_home':
        task.values[recovery.ARM[0]] = .01
    elif failure == 'wrong_hand':
        task.values[recovery.HAND] = .72
    elif failure == 'not_empty':
        task.require_empty.side_effect = RuntimeError('still holding')
    elif failure == 'lost_ack':
        task.apply_snapshot.side_effect = TimeoutError('scene ack missing')
    elif failure == 'wrong_scene':
        task.scene.side_effect = [live, live]
    elif failure == 'stop_during_cleanup':
        task.apply_snapshot.side_effect = RuntimeError('operator Stop')
    else:
        task.apply_snapshot.side_effect = lambda _: task.values.update({recovery.HAND: .42})
    clear = Mock()
    monkeypatch.setattr(recovery, 'clear_held_fault', clear)
    with pytest.raises((RuntimeError, TimeoutError)):
        recovery.PhysicalRecoveryTask.complete_recovery(task, metadata)
    clear.assert_not_called()
    task.phase.assert_not_called()
    assert not task.recovery_finished


def run_task(mode='step', stop_on_approval=False):
    steps = [dict(index=i, kind='arm' if i < 3 else 'hand', phase='approach' if i < 3 else 'open',
                  label=str(i), start_joints=[0.]*6+[.7], end_joints=[0.]*6+[.7]) for i in range(1, 4)]
    metadata = dict(plan_id='recover_test', scene_sha256='unchanged')
    order = []
    task = SimpleNamespace(mode='physical_recover', execution_mode=mode,
        acquire_stack_lock=Mock(), recovery_steps=steps, recovery_metadata=metadata,
        plan_recovery=lambda: [(s, arm_path()) for s in steps], guard=Mock(),
        motion_samples=Mock(return_value=[(0., [0.]*7)]), validate_samples=Mock(),
        recipe_display=Mock(return_value=SimpleNamespace(trajectory=[arm_path()])),
        require_empty=Mock(), complete_recovery=Mock(side_effect=lambda _: order.append('complete')), phase=Mock())
    def approve(*args, **kwargs):
        order.append('approve')
        if stop_on_approval:
            raise RuntimeError('operator Stop')
    task.await_approval = Mock(side_effect=approve)
    task.run_motion = Mock(side_effect=lambda *a: order.append('arm' if a[0]['kind'] == 'arm' else 'hand'))
    return task, order


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_every_recovery_motion_requires_approval_and_hand_opening_follows_home(mode):
    task, order = run_task(mode)
    recovery.PhysicalRecoveryTask.run(task)
    assert order == (['approve', 'arm', 'approve', 'arm', 'approve', 'hand', 'complete'] if mode == 'step'
                     else ['approve', 'arm', 'arm', 'hand', 'complete'])


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_stop_before_recovery_approval_sends_no_goals_or_cleanup(mode):
    task, order = run_task(mode, stop_on_approval=True)
    with pytest.raises(RuntimeError, match='operator Stop'):
        recovery.PhysicalRecoveryTask.run(task)
    task.run_motion.assert_not_called()
    task.complete_recovery.assert_not_called()
    assert order == ['approve']


def planning_task(tmp_path, monkeypatch):
    initial = PlanningScene()
    values = dict(zip(JOINTS, [0.]*6+[.4]))
    task = SimpleNamespace(spin_until=Mock(), fresh=Mock(return_value=True), require_empty=Mock(),
        check_model=Mock(), hardware=Mock(return_value='real'), gripper_runtime=Mock(),
        has_parameter=lambda _: True, param=lambda name: {'preparation_id': 'recover_test',
            'obstacle_surface_z': .805, 'obstacle_size_xyz': [.025, .025, .15]}[name],
        model='<robot/>', semantic=semantic([.2]*6), bounds=bounds(), input_hashes=Mock(return_value={}),
        scene=Mock(return_value=initial), values=values, recovery_steps=[], path=tmp_path/'recovery.json',
        validate_samples=Mock(), motion_samples=Mock(return_value=[(0., [0.]*7)]), phase=Mock())
    current = np.eye(4)
    current[2, 3] = 1.1
    task.kinematics = SimpleNamespace(tcp=Mock(return_value=current))
    task.state_at = lambda *args: recovery.PhysicalRecoveryTask.state_at(task, *args)
    task.arm_step = lambda source, start, label: (dict(index=1, kind='arm', phase='approach', label=label,
        start_joints=start, end_joints=[.2]*6+[start[-1]], velocity_scaling=.05,
        acceleration_scaling=.05), recovery.canonical_recovery_path(source))
    reply = SimpleNamespace(motion_plan_response=SimpleNamespace(error_code=SimpleNamespace(val=1),
                                                                 trajectory=arm_path(end=[.2]*6)))
    task.call = Mock(return_value=reply)
    monkeypatch.setattr(recovery, 'read_held_fault', Mock(return_value=None))
    return task


def test_preserving_home_plans_only_arm_commands_and_keeps_the_measured_hand(tmp_path,monkeypatch):
    task=planning_task(tmp_path,monkeypatch)
    task.preserve_gripper=True
    task.values[recovery.HAND]=.73
    task.sensor={'grip_state':2}
    planned=recovery.PhysicalRecoveryTask.plan_recovery(task)
    assert planned and all(s['kind']=='arm' for s,_ in planned)
    assert all(s['start_joints'][-1]==s['end_joints'][-1]==.73 for s,_ in planned)
    assert task.recovery_metadata['policy']==recovery.HOME_PRESERVE_POLICY
    assert task.recovery_metadata['gripper_position']==.73


def test_preserving_home_already_at_home_needs_no_controller_goal(tmp_path,monkeypatch):
    task=planning_task(tmp_path,monkeypatch)
    task.preserve_gripper=True
    task.sensor={'grip_state':3}
    task.semantic=semantic([0.]*6)
    planned=recovery.PhysicalRecoveryTask.plan_recovery(task)
    assert planned==[]
    task.call.assert_not_called()
    task.mode='physical_recover'
    task.acquire_stack_lock=Mock()
    task.plan_recovery=lambda:planned
    task.complete_recovery=Mock()
    task.run_motion=Mock(side_effect=AssertionError('No goal for an already reached home'))
    recovery.PhysicalRecoveryTask.run(task)
    task.complete_recovery.assert_called_once_with(task.recovery_metadata)
    task.run_motion.assert_not_called()


@pytest.mark.parametrize('state,attached,clear',[(1,False,True),(1,True,False),(0,False,False),(2,True,False),(3,False,False)])
def test_preserving_home_never_clears_the_scene_or_a_possibly_held_block_fault(monkeypatch,state,attached,clear):
    scene=scene_with_owned_and_foreign()
    if not attached:scene.robot_state.attached_collision_objects=[]
    metadata=dict(plan_id='recover_current',gripper_position=.73,scene_sha256=scene_fingerprint(scene))
    task=SimpleNamespace(preserve_gripper=True,mock=False,sensor={'grip_state':state},
        values=dict(zip(JOINTS,[0.]*6+[.73])),recovery_home=[0.]*6,require_empty=Mock(),check_files=Mock(),
        scene=lambda:scene,apply_snapshot=Mock(side_effect=AssertionError('Do not clear the scene')),
        phase=Mock())
    reset=Mock()
    monkeypatch.setattr(recovery,'clear_held_fault',reset)
    monkeypatch.setattr(recovery,'read_held_fault',lambda:None if reset.called else {'cause':'stopped'})
    recovery.PhysicalRecoveryTask.complete_recovery(task,metadata)
    assert reset.called==clear
    task.apply_snapshot.assert_not_called()
    event=task.phase.call_args
    assert event.args==('STACK_RECOVERY_DONE',)
    assert event.kwargs['gripper_preserved'] is True
    assert event.kwargs['scene_preserved'] is True
    assert event.kwargs['held_fault_retained'] is not clear
    assert task.recovery_finished


@pytest.mark.parametrize('change', ['arm', 'hand', 'scene', 'fault'])
def test_planning_context_drift_prevents_sealing_or_any_scene_write(tmp_path, monkeypatch, change):
    task = planning_task(tmp_path, monkeypatch)
    old = task.call.side_effect
    reply = task.call.return_value
    def respond(*args, **kwargs):
        if change in ('arm', 'hand'):
            task.values[recovery.ARM[0] if change == 'arm' else recovery.HAND] += .02
        elif change == 'scene':
            task.scene.return_value = scene_with_owned_and_foreign()
        else:
            recovery.read_held_fault.return_value = {'cause': 'changed'}
        return reply
    task.call.side_effect = respond
    with pytest.raises(RuntimeError, match='계획 중 입력'):
        recovery.PhysicalRecoveryTask.plan_recovery(task)
    assert not task.path.exists()


@pytest.mark.parametrize('failure', ['fraction', 'error'])
def test_partial_or_failed_vertical_service_result_never_falls_back_to_joint_plan(tmp_path, monkeypatch, failure):
    task = planning_task(tmp_path, monkeypatch)
    low, high = np.eye(4), np.eye(4)
    low[2, 3], high[2, 3] = .83, 1.1
    task.kinematics.tcp = Mock(side_effect=[low, high])
    task.call.return_value = SimpleNamespace(fraction=.5 if failure == 'fraction' else 1.,
        error_code=SimpleNamespace(val=-1 if failure == 'error' else 1), solution=arm_path())
    with pytest.raises(RuntimeError, match='수직 복구 경로'):
        recovery.PhysicalRecoveryTask.plan_recovery(task)
    task.call.assert_called_once()
    assert task.call.call_args.args[1] == '/compute_cartesian_path'
    request = task.call.call_args.args[2]
    assert request.avoid_collisions and request.max_velocity_scaling_factor == .05
    assert request.start_state.joint_state.position[-1] == .4
    assert not task.path.exists()


@pytest.mark.parametrize('fault', [dict(cause='SCENE_RPC_PENDING: missing', model_sha256='wrong'),
    dict(cause='Unreadable held-block fault record'), dict(cause='held block', model_sha256='wrong')])
def test_uncertain_or_foreign_model_fault_is_not_bypassed(tmp_path, monkeypatch, fault):
    task = planning_task(tmp_path, monkeypatch)
    monkeypatch.setattr(recovery, 'read_held_fault', Mock(return_value=fault))
    clear = Mock()
    monkeypatch.setattr(recovery, 'clear_held_fault', clear)
    with pytest.raises(RuntimeError, match='잠금을 지우지'):
        recovery.PhysicalRecoveryTask.plan_recovery(task)
    task.call.assert_not_called()
    clear.assert_not_called()


def test_recovery_seal_rejects_changed_fault_even_with_same_files(tmp_path, monkeypatch):
    task = SimpleNamespace(path=tmp_path/'recovery.json', model='<robot/>', semantic=semantic(),
                           input_hashes=lambda: {}, recovery_fault={'cause': 'old'})
    task.path.write_text('{}')
    metadata = dict(policy=recovery.RECOVERY_POLICY, input_sha256={},
        model_sha256=recovery.digest(task.model.encode()), semantic_sha256=recovery.digest(task.semantic.encode()),
        recipe_sha256=recovery.file_digest(task.path), created_at=recovery.time.time(), artifact_sha256={})
    task.recovery_metadata = metadata
    monkeypatch.setattr(recovery, 'read_held_fault', lambda: {'cause': 'new'})
    with pytest.raises(RuntimeError, match='중단 기록이 바뀌'):
        recovery.PhysicalRecoveryTask.check_files(task, metadata)


def test_successful_planning_keeps_scene_and_fault_until_reviewed_motion(tmp_path, monkeypatch):
    task = planning_task(tmp_path, monkeypatch)
    fault = dict(cause='held block', model_sha256=recovery.digest(task.model.encode()), recorded_at=1.)
    read = Mock(return_value=fault)
    clear = Mock()
    monkeypatch.setattr(recovery, 'read_held_fault', read)
    monkeypatch.setattr(recovery, 'clear_held_fault', clear)
    planned = recovery.PhysicalRecoveryTask.plan_recovery(task)
    assert len(planned) == 2
    assert [step['kind'] for step, _ in planned] == ['arm', 'hand']
    assert planned[0][0]['start_joints'][-1] == planned[0][0]['end_joints'][-1] == .4
    assert planned[-1][0]['hand_target'] == .4
    assert task.recovery_metadata['plan_id'] == 'recover_test'
    assert task.recovery_fault == fault
    clear.assert_not_called()
    assert json.loads(task.path.read_text())['plan_id'] == 'recover_test'
    request = task.call.call_args.args[2].motion_plan_request
    assert request.start_state.attached_collision_objects == task.scene.return_value.robot_state.attached_collision_objects
    assert request.max_velocity_scaling_factor == request.max_acceleration_scaling_factor == .05


@pytest.mark.parametrize('failure', ['empty_sensor', 'arm_start', 'hand_start', 'scene', 'execution_disabled'])
def test_guard_rechecks_physical_context_after_operator_approval(failure, monkeypatch):
    metadata = dict(hardware_class='real')
    step = dict(start_joints=[0.]*6+[.4])
    task = SimpleNamespace(check_stop=Mock(), check_files=Mock(), check_model=Mock(),
        hardware=Mock(return_value='real'), gripper_runtime=Mock(),
        call=Mock(return_value=SimpleNamespace(values=[SimpleNamespace(bool_value=True)])),
        scene=Mock(return_value=PlanningScene()), require_empty=Mock(),
        values=dict(zip(JOINTS, step['start_joints'])))
    fingerprint = scene_fingerprint(task.scene.return_value)
    if failure == 'empty_sensor':
        task.require_empty.side_effect = RuntimeError('grip_state3 is not empty')
    elif failure == 'arm_start':
        task.values[recovery.ARM[0]] = .01
    elif failure == 'hand_start':
        task.values[recovery.HAND] = .41
    elif failure == 'scene':
        task.scene.return_value = scene_with_owned_and_foreign()
    else:
        task.call.return_value.values[0].bool_value = False
    with pytest.raises(RuntimeError):
        recovery.PhysicalRecoveryTask.guard(task, metadata, step, fingerprint)
