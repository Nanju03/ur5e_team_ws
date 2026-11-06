"""Physical recipe/contact/Stop regressions with no node, driver or real goal."""
from concurrent.futures import Future
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace, MethodType
from unittest.mock import Mock

import pytest
from control_msgs.action import GripperCommand
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, RobotTrajectory
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5e_block_bringup import physical_stack as physical
from ur5e_block_bringup import physical_held
from ur5e_block_bringup.air_task import AirTask


def recipe():
    result = dict(schema=1, joint_names=physical.JOINTS, initial_joints=[0]*6+[.4],
                  initial_scene_file='initial.cdr', initial_state_file='state.cdr',
                  approach_acceleration_scaling=.10, transfer_acceleration_scaling=1.,
                  physical_contact_clearance=.02,
                  physical_approach_clearance=.02, side_entry_clearance=.010,
                  descent_velocity_scaling=.50, descent_acceleration_scaling=.50,
                  placement_policy='raised_tower_hover_setdown_v2', placement_mode='side',
                  vertical_placement_layers=19, placement_approach_clearance=.12,
                  target_cell=11, foundation_layers=1, moves=1, layers=2, steps=[],
                  routing_policy='open_gripper_buffer_v1', source_cells=[10], transport_moves=1, buffer_moves=0)
    result['buffer_selection_policy'] = physical.BUFFER_SELECTION_POLICY
    previous = list(result['initial_joints'])
    def add(kind, phase, target=None):
        nonlocal previous
        end = list(previous)
        if kind == 'arm':
            end[0] += .01
        if kind == 'hand':
            end[-1] = target
        step = dict(index=len(result['steps'])+1, kind=kind, phase=phase, label=phase,
                    file=f'{len(result["steps"])}.cdr', scene_before_file='before.cdr', scene_after_file='after.cdr',
                    joint_names=physical.JOINTS, start_joints=list(previous), end_joints=end,
                    velocity_scaling=physical.ARM_SPEEDS.get(phase, .4),
                    acceleration_scaling=physical.ARM_SPEEDS.get(phase, .4),
                    completed=1 if phase == 'place' else 0)
        if kind == 'hand':
            step['hand_target'] = target
        if phase == 'place':
            step.update(block_id=10, destination_cell=11, temporary=False)
        result['steps'].append(step)
        previous = end
    add('hand', 'open', .4)
    add('arm', 'approach')
    add('hand', 'grasp', .73)
    add('scene', 'attach')
    add('arm', 'lift')
    add('arm', 'approach')
    add('hand', 'release_first', .70)
    add('scene', 'place')
    add('hand', 'release_clearance', .60)
    add('arm', 'retreat')
    return result


def test_complete_recipe_requires_grasp_and_two_stationary_releases_before_retreat():
    value = recipe()
    assert physical.validate_recipe(value) == value['steps']
    assert value['steps'][4]['velocity_scaling'] == 1.
    assert value['steps'][-1]['velocity_scaling'] == 1.
    assert value['steps'][1]['velocity_scaling'] == .10
    assert value['steps'][1]['acceleration_scaling'] == .10
    assert value['steps'][4]['acceleration_scaling'] == 1.
    assert value['steps'][-1]['acceleration_scaling'] == 1.


@pytest.mark.parametrize('change', ['old_policy', 'missing_clearance', 'low_clearance', 'nan_clearance',
                                   'boolean_clearance', 'invalid_mode', 'invalid_layer'])
def test_legacy_or_invalid_placement_policy_is_rejected_before_ros(change):
    value = recipe()
    if change == 'old_policy':
        value.pop('placement_policy')
    elif change == 'missing_clearance':
        value.pop('placement_approach_clearance')
    elif change == 'low_clearance':
        value['placement_approach_clearance'] = .04
    elif change == 'nan_clearance':
        value['placement_approach_clearance'] = math.nan
    elif change == 'boolean_clearance':
        value['placement_approach_clearance'] = True
    elif change == 'invalid_mode':
        value['placement_mode'] = 'unknown'
    else:
        value['vertical_placement_layers'] = 19.5
    with pytest.raises(ValueError, match='raised tower placement policy'):
        physical.validate_recipe(value)


def execution_validation_fixture(monkeypatch, mode):
    """Run the production recipe loop with no ROS node or actuator client."""
    value = recipe()
    for step in value['steps']:
        step.update(scene_before_sha256='same', scene_after_sha256='same')
    events = []
    task = SimpleNamespace(execution_mode=mode, values={physical.HAND: .4}, carrying=False,
                           release_stage=0, scene_request_pending=False, invalid=False,
                           verified_holding_context=None)
    task.acquire_stack_lock = Mock()
    task.load_recipe = lambda: (dict(scene_sha256='same', plan_id='saved'), value)
    task.fresh = lambda: True
    task.spin_until = lambda predicate: predicate()
    task.guard = lambda metadata, step, scene: events.append(('guard', step['index']))
    task.replay_checkpoint = task.guard
    task.prepare_replay_monitors = Mock()
    task.read_message = lambda *args: RobotTrajectory()
    task.recipe_display = lambda *args: SimpleNamespace(trajectory=[RobotTrajectory()])
    task.apply_snapshot = Mock()
    task.scene = lambda: 'same'
    task.holding_ok = lambda: True
    task.holding_context_verified = lambda *args: True
    task.phase = lambda *args, **kwargs: None
    task.await_approval = lambda *args, **kwargs: events.append(('approval',))

    def validate(step, trajectory, scene):
        events.append(('certificate', step['index']))
        if task.invalid:
            raise RuntimeError('Actual pose changed before dispatch')
        if task.carrying and not task.release_stage:
            task.verified_holding_context = dict(key='reviewed')

    task.bind_certified_motion = validate
    task.live_arm_validation = task.live_hand_validation = Mock(side_effect=AssertionError('Runtime collision check'))
    def motion(step, *args):
        events.append(('motion', step['index']))
        if step['kind'] == 'hand':
            task.values[physical.HAND] = step['hand_target']
    task.run_motion = motion
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda _: 'same')
    monkeypatch.setattr(physical, 'read_held_fault', lambda: None)
    return task, value, events


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_recipe_loop_binds_saved_certificates_without_runtime_collision_checks(monkeypatch, mode):
    task, value, events = execution_validation_fixture(monkeypatch, mode)
    physical.PhysicalStackTask.execute(task)
    motions = [step for step in value['steps'] if step['kind'] != 'scene']
    assert [e[1] for e in events if e[0] == 'motion'] == [step['index'] for step in motions]
    assert sum(e[0] == 'approval' for e in events) == (len(motions) if mode == 'step' else 1)
    for step in motions:
        assert events.count(('certificate', step['index'])) == (2 if mode == 'step' else 1)
        dispatched = events.index(('motion', step['index']))
        assert ('certificate', step['index']) in events[:dispatched]
        assert events[dispatched-1] == ('guard', step['index'])
    assert task.apply_snapshot.call_count == 2
    task.live_arm_validation.assert_not_called()
    task.live_hand_validation.assert_not_called()


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_actual_change_after_approval_still_prevents_every_motion(monkeypatch, mode):
    task, _, events = execution_validation_fixture(monkeypatch, mode)
    def approve(*args, **kwargs):
        events.append(('approval',))
        task.invalid = True
    task.await_approval = approve
    with pytest.raises(RuntimeError, match='Actual pose changed before dispatch'):
        physical.PhysicalStackTask.execute(task)
    assert ('approval',) in events
    assert not any(e[0] == 'motion' for e in events)


def test_old_narrow_entry_command_is_rejected_before_any_motion():
    value = recipe()
    preparation = copy.deepcopy(value['steps'][0])
    preparation.update(phase='pregrasp', label='Prepare grasp', hand_target=.70)
    preparation['end_joints'][-1] = .70
    value['steps'].insert(1, preparation)
    value['steps'][2]['start_joints'][-1] = .70
    value['steps'][2]['end_joints'][-1] = .70
    value['steps'][3]['start_joints'][-1] = .70
    for index, step in enumerate(value['steps'], 1):
        step['index'] = index
    with pytest.raises(ValueError, match='Unexpected physical gripper target'):
        physical.validate_recipe(value)

    # A preparation command must never replace the controlled release while
    # a block is attached, even though both targets happen to be 0.70rad.
    value = recipe()
    value['steps'][6]['phase'] = 'pregrasp'
    with pytest.raises(ValueError, match='Unexpected physical gripper target'):
        physical.validate_recipe(value)


def test_pregrasp_cannot_prepare_with_an_unreviewed_narrower_gap():
    value = recipe()
    value['steps'][0].update(phase='pregrasp', hand_target=.72)
    with pytest.raises(ValueError, match='Unexpected physical gripper target'):
        physical.validate_recipe(value)


def buffered_recipe():
    value = recipe()
    template = copy.deepcopy(value['steps'])
    value.update(source_cells=[10, 15, 24], moves=3, layers=4, transport_moves=4, buffer_moves=1, steps=[])
    previous = list(value['initial_joints'])
    for block, destination, temporary, completed in [(10, 24, True, 0), (15, 11, False, 1),
                                                      (10, 11, False, 2), (24, 11, False, 3)]:
        for original in template:
            step = copy.deepcopy(original)
            step.update(index=len(value['steps'])+1, start_joints=list(previous), end_joints=list(previous))
            if step['kind'] == 'arm':
                step['end_joints'][0] += .01
            elif step['kind'] == 'hand':
                step['end_joints'][-1] = step['hand_target']
            if step['phase'] == 'place':
                step.update(block_id=block, destination_cell=destination, temporary=temporary, completed=completed)
            value['steps'].append(step)
            previous = step['end_joints']
    return value


def test_buffered_transport_preserves_target_progress_and_retrieves_before_its_support():
    value = buffered_recipe()
    physical.validate_recipe(value)
    places = [step for step in value['steps'] if step['phase'] == 'place']
    assert [step['completed'] for step in places] == [0, 1, 2, 3]


@pytest.mark.parametrize('fault', ['progress', 'covered_support', 'duplicate_final', 'buffer_count', 'transport_count'])
def test_buffer_bookkeeping_errors_are_rejected_before_ros(fault):
    value = buffered_recipe()
    places = [step for step in value['steps'] if step['phase'] == 'place']
    if fault == 'progress':
        places[0]['completed'] = 1
    elif fault == 'covered_support':
        places[1]['block_id'] = 24
    elif fault == 'duplicate_final':
        places[2]['block_id'] = 15
    elif fault == 'buffer_count':
        value['buffer_moves'] = 0
    else:
        value['transport_moves'] = 3
    with pytest.raises(ValueError):
        physical.validate_recipe(value)


def test_unloaded_pickup_requires_the_original_open_shape():
    value = recipe()
    for step in value['steps'][:3]:
        step['start_joints'][-1] = .6
        if step['phase'] != 'grasp':
            step['end_joints'][-1] = .6
    value['initial_joints'][-1] = .6
    value['steps'].pop(0)
    for index, step in enumerate(value['steps'], 1):
        step['index'] = index
    with pytest.raises(ValueError, match='Pickup approach must keep the gripper open'):
        physical.validate_recipe(value)


@pytest.mark.parametrize('change', ['missing_clearance', 'reordered_release', 'moving_fingers', 'wrong_grasp',
                                   'fast_approach', 'slow_retreat', 'wrong_acceleration', 'wrong_count',
                                   'nan', 'noncontinuous', 'unknown_kind'])
def test_unsafe_or_changed_recipes_are_rejected_before_ros(change):
    value = recipe()
    if change == 'missing_clearance':
        value['steps'].pop(-2)
    elif change == 'reordered_release':
        value['steps'][6]['phase'] = 'release_clearance'
    elif change == 'moving_fingers':
        value['steps'][2]['end_joints'][0] += .01
    elif change == 'wrong_grasp':
        value['steps'][2]['hand_target'] = .8
    elif change == 'fast_approach':
        value['steps'][1]['velocity_scaling'] = .2
    elif change == 'slow_retreat':
        value['steps'][-1]['velocity_scaling'] = .05
    elif change == 'wrong_acceleration':
        value['steps'][-1]['acceleration_scaling'] = .05
    elif change == 'wrong_count':
        value['layers'] = 3
    elif change == 'nan':
        value['steps'][0]['start_joints'][0] = math.nan
    elif change == 'noncontinuous':
        value['steps'][1]['start_joints'][0] = .1
    else:
        value['steps'][0]['kind'] = 'program'
    with pytest.raises(ValueError):
        physical.validate_recipe(value)


@pytest.mark.parametrize('name', ['/tmp/elsewhere', '../outside', 'sub/file', '..', ''])
def test_recipe_cannot_read_outside_its_reviewed_directory(tmp_path, name):
    with pytest.raises(ValueError):
        physical.artifact_path(tmp_path, name)


def test_recipe_rejects_symlink_artifacts(tmp_path):
    (tmp_path/'real').write_bytes(b'x')
    (tmp_path/'link').symlink_to(tmp_path/'real')
    assert physical.artifact_path(tmp_path, 'real') == tmp_path/'real'
    with pytest.raises(ValueError):
        physical.artifact_path(tmp_path, 'link')


def arm_path():
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = physical.ARM
    for seconds, position in [(0, 0.), (1, .5)]:
        point = JointTrajectoryPoint()
        point.positions, point.velocities, point.accelerations = [position]*6, [0.]*6, [0.]*6
        point.time_from_start.sec = seconds
        result.joint_trajectory.points.append(point)
    return result


def test_retiming_respects_each_joint_phase_limit_without_the_old_uniform_point_one_cap():
    task = SimpleNamespace(joint_velocity_limits={name: 2.1 for name in physical.ARM},
                           joint_acceleration_limits={name: 1. for name in physical.ARM}, stretch=AirTask.stretch)
    original = arm_path()
    step = dict(velocity_scaling=.2, acceleration_scaling=.05)
    normalized, metrics = physical.PhysicalStackTask.normalize_arm(task, original, step)
    assert metrics['max_velocity'] <= .42001
    assert metrics['max_acceleration'] <= .05001
    assert physical.trajectory_duration(normalized) > physical.trajectory_duration(original)
    assert original.joint_trajectory.points[-1].time_from_start.sec == 1


def test_actual_contact_shape_is_used_for_held_arm_validation():
    task = SimpleNamespace(bounds={name: (-6., 6.) for name in physical.JOINTS})
    path = arm_path()
    step = dict(kind='arm', start_joints=[0.]*6+[.73], end_joints=[.5]*6+[.73])
    samples = physical.PhysicalStackTask.motion_samples(task, step, path, actual_hand=.705)
    assert samples and all(values[-1] == .705 for _, values in samples)


def test_retreat_cannot_start_until_both_release_positions_are_completed():
    value = recipe()
    pending_clearance = value['steps'][8]
    premature = copy.deepcopy(value['steps'][-1])
    premature['start_joints'] = list(pending_clearance['start_joints'])
    premature['end_joints'] = list(premature['start_joints'])
    premature['end_joints'][0] += .01
    value['steps'].insert(8, premature)
    for index, step in enumerate(value['steps'], 1):
        step['index'] = index
    with pytest.raises(ValueError, match='completed 0.70 -> 0.60 release'):
        physical.validate_recipe(value)


def guard_task(hand=.705, reference=.705):
    obj = SimpleNamespace(check_stop=Mock(), check_files=Mock(), check_model=Mock(),
                          hardware=Mock(return_value='real'), gripper_runtime=Mock(),
                          call=Mock(return_value=SimpleNamespace(values=[SimpleNamespace(bool_value=True)])),
                          scene=Mock(return_value='same'), measured_stop=Mock(return_value=True),
                          fresh=Mock(return_value=True), sensor_fresh=Mock(return_value=True),
                          values=dict(zip(physical.ARM, [0.]*6)), carrying=True, release_stage=0,
                          holding_reference=reference, sensor={'grip_state': 2}, mock=False)
    obj.values[physical.HAND] = hand
    obj.holding_interval = lambda: physical.PhysicalStackTask.holding_interval(obj)
    obj.holding_ok = lambda: physical.PhysicalStackTask.holding_ok(obj)
    return obj


def test_grasp_contact_need_not_reach_point_seven_two_before_a_held_arm_step(monkeypatch):
    obj = guard_task()
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda _: 'same')
    physical.PhysicalStackTask.guard(obj, {'hardware_class': 'real'}, {'phase': 'lift', 'start_joints': [0.]*6+[.73]}, 'same')
    obj.values[physical.HAND] -= .006
    with pytest.raises(RuntimeError, match='그리퍼'):
        physical.PhysicalStackTask.guard(obj, {'hardware_class': 'real'}, {'phase': 'lift', 'start_joints': [0.]*6+[.73]}, 'same')


@pytest.mark.parametrize('failure', ['stale', 'opening_drift'])
def test_command_assumed_block_still_needs_fresh_reviewed_finger_geometry(failure):
    obj = guard_task()
    if failure == 'stale':
        obj.sensor_fresh = lambda: False
    else:
        obj.values[physical.HAND] -= .006
    assert not obj.holding_ok()


@pytest.mark.parametrize('grip_state', [0, 1, 2, 3])
def test_command_assumed_holding_ignores_device_object_classification(grip_state):
    obj = guard_task()
    obj.sensor['grip_state'] = grip_state
    assert obj.holding_ok()


@pytest.mark.parametrize('sensor', [0, 2, 3])
def test_planning_cannot_clear_existing_geometry_while_gripper_is_not_empty(monkeypatch, sensor):
    task = SimpleNamespace(spin_until=Mock(), fresh=lambda: True, check_model=Mock(),
                           hardware=Mock(return_value='real'), gripper_runtime=Mock(),
                           measured_stop=lambda: True, sensor_fresh=lambda: True,
                           mock=False, sensor={'grip_state': sensor}, planner=Mock())
    clear = Mock()
    monkeypatch.setattr(physical, 'clear_held_fault', clear)
    with pytest.raises(RuntimeError, match='그리퍼가 비어'):
        physical.PhysicalStackTask.plan(task)
    task.planner.assert_not_called()
    clear.assert_not_called()


@pytest.mark.parametrize('incremental', [False, True])
def test_validation_failure_preserves_live_scene_even_when_stopped(tmp_path, monkeypatch, incremental):
    from test_incremental_stack import one_block
    value = one_block() if incremental else recipe()
    path = tmp_path/'recipe.json'
    path.with_name(path.name+'.raw.json').write_text(json.dumps(value))
    initial, before, after = PlanningScene(), PlanningScene(), PlanningScene()
    trajectory = RobotTrajectory()
    task = SimpleNamespace(spin_until=Mock(), fresh=lambda: True, check_model=Mock(),
                           hardware=lambda **_: 'mock', gripper_runtime=Mock(), measured_stop=lambda: True,
                           sensor_fresh=lambda: True, mock=True, path=path, meta_path=tmp_path/'meta.json',
                           input_hashes=lambda: {}, planner=Mock(), acquire_stack_lock=Mock(), release_stack_lock=Mock(), check_stop=Mock(),
                           apply_snapshot=Mock(), motion_samples=Mock(return_value=[(0., [0.]*7)]),
                           validate_samples=Mock(side_effect=RuntimeError('collision')))
    task.read_message = lambda name, kind: initial if name == 'initial.cdr' else trajectory if kind is RobotTrajectory else before
    monkeypatch.setattr(physical, 'clear_held_fault', Mock())
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda _: 'scene')
    task.scene = lambda: before
    task.model, task.semantic = '<robot/>', '<semantic/>'
    task.validation_worker = MethodType(physical.PhysicalStackTask.validation_worker, task)
    task.release_validation_worker = MethodType(physical.PhysicalStackTask.release_validation_worker, task)
    worker = SimpleNamespace(validate=Mock(side_effect=RuntimeError('collision')), close=Mock())
    task.certify_motion = lambda checker, scene, step, trajectory, holding: checker.validate(scene, [])
    monkeypatch.setattr(physical, 'CollisionWorker', Mock(return_value=worker))
    with pytest.raises(RuntimeError, match='collision'):
        physical.PhysicalStackTask.plan(task)
    worker.close.assert_called_once()
    if incremental:
        task.apply_snapshot.assert_not_called()
        assert physical.CollisionWorker.call_args.args[0]['bounded_surface_contacts'] is True
    else:
        assert task.apply_snapshot.call_args.args[0] is initial
        assert task.apply_snapshot.call_args.kwargs == {'cancel': False}


def test_stop_cannot_overtake_an_inflight_scene_write_with_restoration(monkeypatch):
    pending = Future()
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(return_value=pending),
                             remove_pending_request=Mock())
    order = []
    task = SimpleNamespace(values=dict(zip(physical.JOINTS, [0.]*7)), model='<robot/>',
                           create_client=lambda *args: client, destroy_client=Mock(),
                           spin_until=lambda predicate, *args, **kwargs: None,
                           check_stop=lambda: order.append(('stop_checked', pending.done())), stopped=False)
    task.cached_service_client = lambda *args: physical.PhysicalStackTask.cached_service_client(task, *args)
    def spin(obj, timeout_sec):
        order.append(('awaiting_ack', pending.done()))
        obj.stopped = True
        pending.set_result(SimpleNamespace(success=True))
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    physical.PhysicalStackTask.apply_scene_message(task, PlanningScene())
    assert order == [('awaiting_ack', False), ('stop_checked', True)]
    assert not task.scene_request_pending
    assert client.call_async.call_count == 1


def scene_box(name, x=0.):
    item = CollisionObject()
    item.id, item.header.frame_id = name, 'world'
    shape = SolidPrimitive()
    shape.type, shape.dimensions = SolidPrimitive.BOX, [.025]*3
    pose = Pose()
    pose.orientation.w = 1.
    item.primitives, item.primitive_poses = [shape], [pose]
    item.pose.orientation.w = 1.
    item.pose.position.x = x
    return item


def test_collision_fingerprint_normalizes_eigen_roundtrips_but_rejects_real_motion():
    left = PlanningScene()
    left.world.collision_objects.append(scene_box('ur5e_stack/block_4', .4))
    right = copy.deepcopy(left)
    right.world.collision_objects[0].pose.position.x += 2e-16
    right.world.collision_objects[0].pose.orientation.w = -1.
    assert physical.scene_fingerprint(left) == physical.scene_fingerprint(right)
    right.world.collision_objects[0].pose.position.x += .000001
    assert physical.scene_fingerprint(left) != physical.scene_fingerprint(right)


def test_reconciliation_detaches_then_removes_auto_world_body_then_adds_placed_body():
    live, desired = PlanningScene(), PlanningScene()
    body = AttachedCollisionObject()
    body.link_name = 'grasp_link'
    body.object = scene_box('ur5e_stack/block_4')
    live.robot_state.attached_collision_objects.append(body)
    foreign = scene_box('workpiece_fixture', .8)
    live.world.collision_objects.append(foreign)
    desired.world.collision_objects = [copy.deepcopy(foreign), scene_box('ur5e_stack/block_4', .4)]
    calls = []
    task = SimpleNamespace(scene_request_pending=False,
                           read_scene_for_reconciliation=lambda **_: live,
                           apply_scene_message=lambda message, **_: calls.append(copy.deepcopy(message)),
                           check_stop=lambda: calls.append('checked'))
    physical.PhysicalStackTask.apply_snapshot(task, desired)
    assert len(calls) == 4 and calls[-1] == 'checked'
    detach, cleanup, add = calls[:3]
    assert detach.robot_state.attached_collision_objects[0].object.operation == CollisionObject.REMOVE
    assert not detach.world.collision_objects
    assert cleanup.world.collision_objects[0].id == 'ur5e_stack/block_4'
    assert not cleanup.robot_state.attached_collision_objects
    assert add.world.collision_objects[0].pose.position.x == .4
    assert all(item.id != 'workpiece_fixture' for item in add.world.collision_objects)


def test_new_foreign_collision_geometry_is_never_deleted_by_snapshot_restoration():
    live, desired = PlanningScene(), PlanningScene()
    live.world.collision_objects.append(scene_box('new_external_obstacle'))
    with pytest.raises(RuntimeError, match='External collision geometry changed'):
        physical.reconcile_snapshot(live, desired)


def test_changed_occupancy_geometry_is_never_overwritten_by_snapshot_restoration():
    live, desired = PlanningScene(), PlanningScene()
    live.world.octomap.octomap.id = 'OcTree'
    live.world.octomap.octomap.resolution = .01
    live.world.octomap.octomap.data = [1, 2]
    with pytest.raises(RuntimeError, match='occupancy data'):
        physical.reconcile_snapshot(live, desired)


@pytest.mark.parametrize('grip_state', [0, 1, 2, 3])
def test_stop_after_grasp_preserves_a_conservative_attachment_and_fault(monkeypatch, grip_state):
    snapshot = PlanningScene()
    body = AttachedCollisionObject()
    body.link_name, body.object.id = 'grasp_link', 'ur5e_stack/block_4'
    snapshot.robot_state.attached_collision_objects.append(body)
    task = SimpleNamespace(mock=False, active_recipe={},
                           grasp_attempt=dict(step={'index': 3}, sensor_sequence=10, arm=[0.]*6,
                                              attachment_snapshot=snapshot),
                           sensor_fresh=lambda **_: True, sensor={'grip_state': grip_state}, fresh=lambda: True,
                           values=dict(zip(physical.JOINTS, [0.]*6+[.715])), model='<robot/>', apply_snapshot=Mock())
    fault = Mock()
    monkeypatch.setattr(physical, 'write_held_fault', fault)
    physical.PhysicalStackTask.preserve_contact_after_stop(task)
    update = task.apply_snapshot.call_args.args[0]
    assert update.world.collision_objects[0].id == 'ur5e_stack/block_4'
    assert update.robot_state.attached_collision_objects[0].object.id == 'ur5e_stack/block_4'
    assert task.apply_snapshot.call_args.kwargs == {'cancel': False, 'full': False}
    assert task.carrying and task.holding_reference == .715
    fault.assert_called_once()


def test_stop_while_acceptance_is_pending_cancels_known_uuid_before_ack(monkeypatch):
    pending = Future()
    client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=Mock(return_value=pending), destroy=Mock())
    monkeypatch.setattr(physical, 'ActionClient', lambda *args: client)
    task = SimpleNamespace(gripper_runtime=lambda: '/grip', spin_until=Mock(), guard=Mock(),
                           sensor_sequence=7, values=dict(zip(physical.JOINTS, [0.]*6+[.4])),
                           stopped=False, phase=Mock(), recover_pending=Mock(), cancel_remaining_goal=Mock(),
                           flight_queries=lambda *_: [], prewarm_flight_queries=Mock(),
                           destroy_flight_queries=Mock(), holding_ok=lambda: True,
                           input_hashes=lambda: {}, path=Path('/not/read'), check_stop=Mock(),
                           poll_flight_queries=Mock(), fresh=lambda: True, sensor_fresh=lambda: True,
                           measured_stop=lambda **_: True)
    monkeypatch.setattr(physical, 'file_digest', lambda _: 'recipe')
    def spin(obj, timeout_sec):
        obj.stopped = True
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    with pytest.raises(RuntimeError, match='acceptance failed'):
        physical.PhysicalStackTask.run_motion(task, {'kind': 'hand', 'phase': 'open', 'hand_target': .4, 'label': 'open'},
                                             RobotTrajectory(), {'input_sha256': {}, 'recipe_sha256': 'recipe'}, 'same', 1, 1)
    assert not pending.done()
    assert task.recover_pending.called
    goal_uuid = client.send_goal_async.call_args.kwargs['goal_uuid']
    assert len(goal_uuid.uuid) == 16
    client.destroy.assert_called_once()


def test_nonresponsive_monitor_does_not_block_stop_or_sensor_checks(monkeypatch):
    task = SimpleNamespace(stopped=False)
    pending = Future()
    query = dict(client=Mock(), request=None, period=.5, valid=lambda _: True, future=pending, sent=0.)
    monkeypatch.setattr(physical.time, 'monotonic', lambda: 3.)
    physical.PhysicalStackTask.poll_flight_queries(task, [query])
    assert task.stopped and not pending.done()


def test_dangling_held_fault_symlink_requires_recovery(tmp_path, monkeypatch):
    path = tmp_path/'held.json'
    path.symlink_to(tmp_path/'missing.json')
    monkeypatch.setattr(physical_held, 'held_fault_path', lambda domain=None: path)
    assert physical_held.read_held_fault() is not None


def test_held_fault_is_domain_local_and_explicit_recovery_only(tmp_path, monkeypatch):
    monkeypatch.setattr(physical_held, 'held_fault_path', lambda domain=None: tmp_path/(str(domain or 0)+'.json'))
    assert physical_held.read_held_fault() is None
    physical_held.write_held_fault('held', 'abc')
    assert physical_held.read_held_fault()['cause'] == 'held'
    assert physical_held.read_held_fault(1) is None
    physical_held.clear_held_fault()
    assert physical_held.read_held_fault() is None


def test_cold_three_second_monitor_ack_is_warmed_before_movement_but_moving_timeout_stays_two(monkeypatch):
    clock, pending = [0.], Future()
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(return_value=pending),
                             remove_pending_request=Mock())
    task = SimpleNamespace(stopped=False, fresh=lambda: True, sensor_fresh=lambda: True)
    def check_stop():
        if task.stopped:
            raise RuntimeError('stopped')
    task.check_stop = check_stop
    task.poll_flight_queries = lambda queries, **kwargs: physical.PhysicalStackTask.poll_flight_queries(task, queries, **kwargs)
    task.destroy_flight_queries = lambda queries: physical.PhysicalStackTask.destroy_flight_queries(task, queries)
    query = dict(client=client, service='/get_planning_scene', request=object(), period=.5,
                 valid=lambda reply: reply == 'valid', future=None, sent=-math.inf,
                 verified=False, last_ok=-math.inf)
    monkeypatch.setattr(physical.time, 'monotonic', lambda: clock[0])
    def spin(*args, **kwargs):
        clock[0] += .25
        if clock[0] >= 3. and not pending.done():
            pending.set_result('valid')
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    physical.PhysicalStackTask.prewarm_flight_queries(task, [query])
    assert clock[0] >= 3. and not task.stopped and query['verified']
    assert query['future'] is None
    query['future'], query['sent'] = Future(), clock[0]
    clock[0] += 2.01
    physical.PhysicalStackTask.poll_flight_queries(task, [query])
    assert task.stopped
    assert task.stop_reason['code'] == 'MONITOR_RPC_TIMEOUT'
    assert task.stop_reason['service'] == '/get_planning_scene'
    assert task.stop_reason['moving'] is True


def test_monitor_clients_persist_but_scene_predicates_and_futures_are_fresh(monkeypatch):
    task = SimpleNamespace(mock=False, model='robot', semantic='semantic',
                           create_client=Mock(side_effect=lambda *_: object()))
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    first = physical.PhysicalStackTask.flight_queries(task, {}, 'scene_before_attach')
    second = physical.PhysicalStackTask.flight_queries(task, {}, 'scene_after_attach')
    assert task.create_client.call_count == 7
    assert len(first) == len(second) == 7
    previous = next(q for q in first if q['service'] == '/get_planning_scene')
    current = next(q for q in second if q['service'] == '/get_planning_scene')
    assert previous['client'] is current['client']
    assert current['future'] is None and not current['verified']
    assert previous['valid'](SimpleNamespace(scene='scene_before_attach'))
    assert not current['valid'](SimpleNamespace(scene='scene_before_attach'))
    assert current['valid'](SimpleNamespace(scene='scene_after_attach'))


def test_monitor_reply_failure_keeps_named_first_cause(monkeypatch, capsys):
    pending = Future()
    pending.set_result('changed scene')
    task = SimpleNamespace(stopped=False)
    query = dict(service='/get_planning_scene', future=pending, sent=1.,
                 valid=lambda _: False, period=.5)
    monkeypatch.setattr(physical.time, 'monotonic', lambda: 1.25)
    physical.PhysicalStackTask.poll_flight_queries(task, [query])
    physical.stop_with_reason(task, 'PROCESS_SIGNAL')
    assert task.stop_reason == dict(code='MONITOR_REPLY_INVALID', service='/get_planning_scene',
                                    elapsed=.25, observed=None)
    assert capsys.readouterr().out.count('STACK_STOP_REASON') == 1


@pytest.mark.parametrize('grip_state', [0, 1, 2, 3])
@pytest.mark.parametrize('contact_position,accepted_contact,status,stalled,reached', [
    (.73, True, 4, False, True), (.73261, True, 4, False, True),
    (.73354, False, 4, False, True), (.4, False, 4, False, True),
    (.73, False, 6, False, False), (.68, True, 6, True, False),
    (.73, False, 4, False, False), (.73, False, 5, False, False)])
def test_grasp_uses_fresh_command_completion_without_object_detection(
        monkeypatch, contact_position, accepted_contact, status, stalled, reached, grip_state):
    finished, accepted = Future(), Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: finished)
    accepted.set_result(handle)
    order = []
    def send(*args, **kwargs):
        order.append('send')
        return accepted
    client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send, destroy=Mock())
    monkeypatch.setattr(physical, 'ActionClient', lambda *args: client)
    monkeypatch.setattr(physical, 'file_digest', lambda _: 'recipe')
    task = SimpleNamespace(gripper_runtime=lambda: '/grip', spin_until=Mock(), guard=Mock(),
                           sensor_sequence=7, sensor={'grip_state': grip_state}, values=dict(zip(physical.JOINTS, [0.]*6+[.4])),
                           stopped=False, phase=Mock(), recover_pending=Mock(), cancel_remaining_goal=Mock(),
                           flight_queries=lambda *_: [], destroy_flight_queries=Mock(), holding_ok=lambda: True,
                           input_hashes=lambda: {}, path=Path('/not/read'), check_stop=Mock(),
                           poll_flight_queries=Mock(), fresh=lambda: True, measured_stop=lambda **_: True,
                           mock=False, execution_mode='continuous',
                           active_recipe={'steps': [dict(index=2, phase='attach', scene_after_file='attached.cdr')]},
                           read_message=lambda *_: PlanningScene(), observe_grasp_settle=Mock())
    fresh_checks = []
    task.sensor_fresh = lambda after=None: fresh_checks.append(after) or True
    def warm(*args):
        order.append('warm')
        task.sensor_sequence = 100
    task.prewarm_flight_queries = warm
    def spin(*args, **kwargs):
        task.sensor_sequence = 101
        task.values[physical.HAND] = contact_position
        finished.set_result(SimpleNamespace(status=status, result=SimpleNamespace(
            reached_goal=reached, stalled=stalled, position=contact_position)))
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    run = lambda: physical.PhysicalStackTask.run_motion(task,
        dict(index=1, kind='hand', phase='grasp', hand_target=.73, label='grasp'),
        RobotTrajectory(), dict(input_sha256={}, recipe_sha256='recipe'), 'same', 1, 1)
    if accepted_contact:
        run()
        task.observe_grasp_settle.assert_called_once()
    else:
        with pytest.raises(RuntimeError, match='0.73 잡기'):
            run()
        task.observe_grasp_settle.assert_not_called()
    assert order == ['warm', 'send']
    assert 100 in fresh_checks
    assert task.grasp_attempt['sensor_sequence'] == 100


def cached_rpc_task(client):
    task = SimpleNamespace(create_client=Mock(return_value=client), destroy_client=Mock(),
                           spin_until=lambda predicate, *args, **kwargs: None,
                           check_stop=Mock())
    task.cached_service_client = lambda *args: physical.PhysicalStackTask.cached_service_client(task, *args)
    task.call = lambda *args, **kwargs: physical.PhysicalStackTask.call(task, *args, **kwargs)
    return task


def test_scene_reads_reuse_transport_but_return_new_replies_and_close_only_at_shutdown():
    first, second = PlanningScene(), PlanningScene()
    first.name, second.name = 'first', 'second'
    futures = [Future(), Future()]
    for future, scene in zip(futures, [first, second]):
        future.set_result(SimpleNamespace(scene=scene))
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(side_effect=futures),
                             remove_pending_request=Mock())
    task = cached_rpc_task(client)
    assert physical.PhysicalStackTask.read_scene_for_reconciliation(task) is first
    assert physical.PhysicalStackTask.read_scene_for_reconciliation(task, cancel=False) is second
    task.create_client.assert_called_once_with(GetPlanningScene, '/get_planning_scene')
    assert client.remove_pending_request.call_count == 2
    task.destroy_client.assert_not_called()
    physical.PhysicalStackTask.close_service_clients(task)
    task.destroy_client.assert_called_once_with(client)
    assert task.service_clients == {}


def test_timed_out_read_is_removed_before_next_fresh_request():
    expired, fresh = Future(), Future()
    fresh.set_result(SimpleNamespace(scene='fresh scene'))
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(side_effect=[expired, fresh]),
                             remove_pending_request=Mock())
    task = cached_rpc_task(client)
    def wait(predicate, *args, **kwargs):
        if not predicate():
            raise TimeoutError('read timed out')
    task.spin_until = wait
    with pytest.raises(TimeoutError, match='read timed out'):
        physical.PhysicalStackTask.read_scene_for_reconciliation(task)
    client.remove_pending_request.assert_called_once_with(expired)
    expired.set_result(SimpleNamespace(scene='late stale scene'))
    assert physical.PhysicalStackTask.read_scene_for_reconciliation(task) == 'fresh scene'
    assert task.create_client.call_count == 1
    assert client.remove_pending_request.call_args.args[0] is fresh


def startup_rpc_task(futures):
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(side_effect=futures),
                             remove_pending_request=Mock())
    task = cached_rpc_task(client)
    task.startup_read_stage = 'physical_plan_preflight'
    def wait(predicate, *args, **kwargs):
        if not predicate():
            raise TimeoutError('test response deadline')
    task.spin_until = Mock(side_effect=wait)
    return task, client


@pytest.mark.parametrize('kind', [physical.GetParameters, physical.ListHardwareComponents,
                                  physical.ListControllers, GetPlanningScene])
def test_startup_read_retries_one_lost_response_with_new_future_after_cleanup(kind, capsys):
    expired, fresh = Future(), Future()
    reply = SimpleNamespace(healthy=True)
    fresh.set_result(reply)
    task, client = startup_rpc_task([expired, fresh])
    order = []
    def send(request):
        order.append('send')
        return expired if order.count('send') == 1 else fresh
    def remove(future):
        order.append('remove old' if future is expired else 'remove new')
        if future is expired:
            # A late first reply never supplies the second call's result.
            expired.set_result(SimpleNamespace(healthy=False))
    client.call_async.side_effect, client.remove_pending_request.side_effect = send, remove
    assert task.call(kind, '/test/read', kind.Request()) is reply
    assert order == ['send', 'remove old', 'send', 'remove new']
    task.create_client.assert_called_once_with(kind, '/test/read')
    record = json.loads(capsys.readouterr().out.split('STACK_READ_RPC_TIMEOUT ')[1])
    assert record['endpoint'] == '/test/read'
    assert record['stage'] == 'response' and record['attempt'] == 1 and record['max_attempts'] == 2
    assert record['context'] == 'physical_plan_preflight' and record['elapsed'] >= 0


def test_startup_read_exhausts_two_response_deadlines_and_names_endpoint(capsys):
    futures = [Future(), Future()]
    task, client = startup_rpc_task(futures)
    with pytest.raises(TimeoutError, match='/controller_manager/list_hardware_components response.*2 request'):
        task.call(physical.ListHardwareComponents, '/controller_manager/list_hardware_components',
                  physical.ListHardwareComponents.Request())
    assert client.call_async.call_count == 2
    assert [call.args[0] for call in client.remove_pending_request.call_args_list] == futures
    records = [json.loads(line.split(' ', 1)[1]) for line in capsys.readouterr().out.splitlines()]
    assert [record['attempt'] for record in records] == [1, 2]
    assert all(record['stage'] == 'response' for record in records)
    assert all(call.args[1] == 10. for call in task.spin_until.call_args_list)


def test_startup_discovery_timeout_does_not_send_or_repeat_discovery(capsys):
    task, client = startup_rpc_task([Future()])
    client.service_is_ready = lambda: False
    with pytest.raises(TimeoutError, match='/test/read discovery.*0 request'):
        task.call(GetPlanningScene, '/test/read', GetPlanningScene.Request())
    task.spin_until.assert_called_once()
    client.call_async.assert_not_called()
    client.remove_pending_request.assert_not_called()
    record = json.loads(capsys.readouterr().out.split('STACK_READ_RPC_TIMEOUT ')[1])
    assert record['stage'] == 'discovery' and record['attempt'] == 0


@pytest.mark.parametrize('block', ['outside_preflight', 'goal', 'goal_pending', 'child',
                                  'scene_request_pending', 'stopped', 'uncancellable'])
def test_response_retry_is_excluded_outside_idle_cancellable_startup(block):
    expired = Future()
    task, client = startup_rpc_task([expired])
    if block == 'outside_preflight':
        task.startup_read_stage = None
    elif block != 'uncancellable':
        setattr(task, block, True)
    with pytest.raises(TimeoutError, match='1 request'):
        task.call(GetPlanningScene, '/test/read', GetPlanningScene.Request(), cancel=block != 'uncancellable')
    client.call_async.assert_called_once()
    client.remove_pending_request.assert_called_once_with(expired)


@pytest.mark.parametrize('kind', [physical.ApplyPlanningScene, GetStateValidity])
def test_mutations_and_collision_queries_never_receive_startup_retries(kind):
    expired = Future()
    task, client = startup_rpc_task([expired])
    with pytest.raises(TimeoutError, match='1 request'):
        task.call(kind, '/test/excluded', kind.Request())
    client.call_async.assert_called_once()
    client.remove_pending_request.assert_called_once_with(expired)


@pytest.mark.parametrize('during', ['wait', 'retry_boundary'])
def test_operator_stop_cleans_up_pending_read_without_retry(during):
    expired = Future()
    task, client = startup_rpc_task([expired])
    if during == 'wait':
        def wait(predicate, *args, **kwargs):
            if not predicate():
                raise RuntimeError('operator Stop')
        task.spin_until = Mock(side_effect=wait)
    else:
        task.check_stop.side_effect = RuntimeError('operator Stop')
    with pytest.raises(RuntimeError, match='operator Stop'):
        task.call(GetPlanningScene, '/test/read', GetPlanningScene.Request())
    client.call_async.assert_called_once()
    client.remove_pending_request.assert_called_once_with(expired)


def test_completed_future_exception_is_not_retried_as_a_missing_reply():
    failed = Future()
    failed.set_exception(TimeoutError('server result failure'))
    task, client = startup_rpc_task([failed])
    with pytest.raises(TimeoutError, match='server result failure'):
        task.call(GetPlanningScene, '/test/read', GetPlanningScene.Request())
    client.call_async.assert_called_once()
    client.remove_pending_request.assert_called_once_with(failed)


def test_startup_model_mismatch_remains_an_immediate_failure():
    bad = Future()
    bad.set_result(SimpleNamespace(values=[SimpleNamespace(string_value='different robot')]))
    task, client = startup_rpc_task([bad])
    task.model, task.semantic = '<robot/>', '<semantic/>'
    with pytest.raises(RuntimeError, match='Robot model changed'):
        AirTask.check_model(task)
    client.call_async.assert_called_once()


def test_startup_retry_scope_is_removed_when_preflight_fails():
    task = SimpleNamespace(spin_until=Mock(), fresh=lambda: True, check_model=Mock(),
                           hardware=Mock(side_effect=RuntimeError('invalid hardware')), gripper_runtime=Mock())
    task.check_model.side_effect = lambda: assert_preflight_context(task)
    with pytest.raises(RuntimeError, match='invalid hardware'):
        physical.PhysicalStackTask.plan(task)
    assert task.startup_read_stage is None
    task.gripper_runtime.assert_not_called()


def assert_preflight_context(task):
    assert task.startup_read_stage == 'physical_plan_preflight'


def test_scene_writes_reuse_client_with_separate_acknowledged_requests():
    futures = [Future(), Future()]
    for future in futures:
        future.set_result(SimpleNamespace(success=True))
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(side_effect=futures),
                             remove_pending_request=Mock())
    task = cached_rpc_task(client)
    task.values = dict(zip(physical.JOINTS, [0.]*7))
    task.check_stop = Mock()
    for name in ('first mutation', 'second mutation'):
        scene = PlanningScene()
        scene.name = name
        physical.PhysicalStackTask.apply_scene_message(task, scene)
    assert task.create_client.call_count == 1
    assert [call.args[0].scene.name for call in client.call_async.call_args_list] == ['first mutation', 'second mutation']
    assert client.remove_pending_request.call_count == 2
    assert not task.scene_request_pending
    task.destroy_client.assert_not_called()


def test_lost_mutation_ack_blocks_reuse_of_the_cached_scene_writer(monkeypatch):
    failed = Future()
    failed.set_exception(RuntimeError('lost transport'))
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(return_value=failed),
                             remove_pending_request=Mock())
    task = cached_rpc_task(client)
    task.values, task.model = dict(zip(physical.JOINTS, [0.]*7)), '<robot/>'
    task.check_stop = Mock()
    monkeypatch.setattr(physical, 'write_held_fault', Mock())
    with pytest.raises(RuntimeError, match='acknowledgement was lost'):
        physical.PhysicalStackTask.apply_scene_message(task, PlanningScene())
    assert task.scene_request_pending
    with pytest.raises(RuntimeError, match='no acknowledgement'):
        physical.PhysicalStackTask.apply_scene_message(task, PlanningScene())
    client.call_async.assert_called_once()
    assert task.create_client.call_count == 1


def test_each_collision_sample_remains_checked_with_a_reused_validity_client(monkeypatch):
    def valid_future(request):
        future = Future()
        future.set_result(SimpleNamespace(valid=True))
        return future
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(side_effect=valid_future),
                             remove_pending_request=Mock())
    task = cached_rpc_task(client)
    task.check_stop = Mock()
    monkeypatch.setattr(physical.rclpy, 'spin_once', lambda *args, **kwargs: None)
    samples = [(0., [0.]*7), (.05, [.01]*7), (.1, [.02]*7)]
    assert physical.PhysicalStackTask.validate_samples(task, samples) == 3
    assert physical.PhysicalStackTask.validate_samples(task, samples) == 3
    task.create_client.assert_called_once_with(GetStateValidity, '/check_state_validity')
    assert client.call_async.call_count == 6
    assert [list(call.args[0].robot_state.joint_state.position) for call in client.call_async.call_args_list] == [q for _, q in samples]*2
    task.destroy_client.assert_not_called()


def test_timed_out_collision_read_is_removed_and_next_validation_uses_fresh_reply(monkeypatch):
    clock, expired, fresh = [0.], Future(), Future()
    fresh.set_result(SimpleNamespace(valid=True))
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(side_effect=[expired, fresh]),
                             remove_pending_request=Mock())
    task = cached_rpc_task(client)
    task.check_stop = Mock()
    monkeypatch.setattr(physical.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(physical.rclpy, 'spin_once', lambda *args, **kwargs: clock.__setitem__(0, clock[0]+11.))
    with pytest.raises(TimeoutError, match='collision validation timed out'):
        physical.PhysicalStackTask.validate_samples(task, [(0., [0.]*7)])
    client.remove_pending_request.assert_called_once_with(expired)
    expired.set_result(SimpleNamespace(valid=False))
    assert physical.PhysicalStackTask.validate_samples(task, [(0., [0.]*7)]) == 1
    assert task.create_client.call_count == 1


def held_validation_task():
    task = guard_task(hand=.70959, reference=.70587)
    task.model, task.semantic = '<robot/>', '<semantic/>'
    task.bounds = {name: (-6., 6.) for name in physical.JOINTS}
    task.verified_holding_context = None
    task.validate_samples = Mock(side_effect=lambda samples: len(samples))
    for name in ('holding_context_key', 'holding_context_verified', 'verified_holding_ok',
                 'motion_samples', 'tracking_motion_samples', 'validate_holding_envelope'):
        setattr(task, name, lambda *args, _name=name, **kwargs: getattr(physical.PhysicalStackTask, _name)(task, *args, **kwargs))
    return task


def held_step():
    return dict(index=8, kind='arm', phase='lift', label='lift',
                start_joints=[0.]*6+[.73], end_joints=[.5]*6+[.73])


def test_command_bounded_closure_keeps_original_contact_anchor_but_opening_and_overclosure_fail():
    task = held_validation_task()
    assert task.holding_ok()  # Four real encoder counts of continued closing.
    task.values[physical.HAND] = .719
    assert task.holding_ok()
    assert task.holding_reference == .70587
    assert task.holding_interval() == (.70587-.005, .733)
    task.values[physical.HAND] = .70587-.00501
    assert not task.holding_ok()
    task.values[physical.HAND] = .73301
    assert not task.holding_ok()


def test_five_encoder_counts_of_contact_slack_need_the_expanded_geometry_check(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    task.values[physical.HAND] = .70122  # Five raw counts below .70587: delta .00465rad.
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    assert task.holding_ok()
    assert not physical.PhysicalStackTask.approval_hand_ok(task, {'holding_context_key': 'not checked'})
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    metadata = {'holding_context_key': task.verified_holding_context['key']}
    assert physical.PhysicalStackTask.approval_hand_ok(task, metadata)
    assert min(q[-1] for _, q in task.validate_samples.call_args.args[0]) == pytest.approx(.70087)
    task.values[physical.HAND] = .70029  # Six raw counts exceed the .005rad margin.
    assert not physical.PhysicalStackTask.approval_hand_ok(task, metadata)


@pytest.mark.parametrize('path_kind', ['physical_stack', 'plate_alignment'])
def test_contact_slack_does_not_relax_empty_hand_or_alignment_approval(path_kind):
    task = SimpleNamespace(values={physical.HAND: .704})
    assert not AirTask.approval_hand_ok(task, dict(path_kind=path_kind, gripper_position=.7))


def test_previous_narrower_contact_validation_cannot_authorize_the_new_slack(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    key = task.verified_holding_context['key']
    task.verified_holding_context['interval'] = (.70587-.003, .733)
    assert not task.holding_context_verified(step, trajectory, 'same')
    assert not task.verified_holding_ok(key)
    assert not physical.PhysicalStackTask.approval_hand_ok(task, {'holding_context_key': key})


def test_every_arm_sample_checks_full_contact_interval_including_lower_opening_bound(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    samples = task.validate_samples.call_args.args[0]
    grouped = {}
    for stamp, q in samples:
        grouped.setdefault(stamp, []).append(q[-1])
    assert len(grouped) > 10
    for fingers in grouped.values():
        assert min(fingers) == pytest.approx(.70087)
        assert max(fingers) == pytest.approx(.733)
        assert .70959 in fingers
        assert max(b-a for a, b in zip(fingers, fingers[1:])) <= .005000001
    assert task.holding_context_verified(step, trajectory, 'same')
    # A second pre/post-OK check can reuse this exact verified interval.
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    assert task.validate_samples.call_count == 1
    # Continued closure does not move the anchor or expand its acceptance band.
    task.values[physical.HAND] = .73
    assert task.verified_holding_ok(task.verified_holding_context['key'])
    assert task.holding_reference == .70587


def test_collision_at_permitted_lower_finger_bound_prevents_any_verified_envelope(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    def reject_lower(samples):
        assert any(q[-1] == pytest.approx(.70087) for _, q in samples)
        raise RuntimeError('finger collision at allowed lower bound')
    task.validate_samples = reject_lower
    with pytest.raises(RuntimeError, match='finger collision'):
        physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    assert task.verified_holding_context is None


@pytest.mark.parametrize('change', ['scene', 'model', 'semantic', 'trajectory', 'interval', 'step', 'start_state'])
def test_verified_holding_context_is_not_reused_after_context_changes(monkeypatch, change):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    scene = 'same'
    if change == 'scene':
        scene = 'changed'
    elif change in ('model', 'semantic'):
        setattr(task, change, 'changed model')
    elif change == 'trajectory':
        trajectory.joint_trajectory.points[-1].time_from_start.sec += 1
    elif change == 'interval':
        task.holding_reference += .00093
    elif change == 'start_state':
        step['start_joints'][0] += .001
    else:
        step['index'] += 1
    assert not task.holding_context_verified(step, trajectory, scene)


def test_step_approval_requires_matching_prevalidated_finger_geometry_not_contact(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    assert not physical.PhysicalStackTask.approval_hand_ok(task, {'holding_context_key': 'unvalidated'})
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    metadata = {'holding_context_key': task.verified_holding_context['key']}
    assert physical.PhysicalStackTask.approval_hand_ok(task, metadata)
    task.sensor['grip_state'] = 3
    assert physical.PhysicalStackTask.approval_hand_ok(task, metadata)
    task.sensor['grip_state'] = 2
    task.sensor_fresh = lambda: False
    assert not physical.PhysicalStackTask.approval_hand_ok(task, metadata)


def test_empty_and_alignment_approval_keep_the_original_frozen_hand_limit():
    task = SimpleNamespace(values={physical.HAND: .404})
    for path_kind in ('physical_stack', 'plate_alignment'):
        metadata = dict(path_kind=path_kind, gripper_position=.4)
        assert not AirTask.approval_hand_ok(task, metadata)
        task.values[physical.HAND] = .402
        assert AirTask.approval_hand_ok(task, metadata)
        task.values[physical.HAND] = .404


def test_release_first_checks_the_entire_held_and_opening_sweep(monkeypatch):
    task = held_validation_task()
    step = dict(index=12, kind='hand', phase='release_first', hand_target=.7,
                start_joints=[0.]*6+[.73], end_joints=[0.]*6+[.7])
    trajectory = RobotTrajectory()
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    physical.PhysicalStackTask.live_hand_validation(task, step, trajectory, 'same')
    samples = task.validate_samples.call_args.args[0]
    assert min(q[-1] for _, q in samples) == pytest.approx(
        physical.PhysicalStackTask.first_release_target(task)-.003)
    assert max(q[-1] for _, q in samples) == .733
    assert task.holding_context_verified(step, trajectory, 'same')


@pytest.mark.parametrize('grip_state', [0, 1, 2, 3])
def test_stationary_geometry_observation_ignores_object_state_and_keeps_anchor(monkeypatch, grip_state):
    task = held_validation_task()
    task.carrying, task.sensor_sequence, task.stopped = False, 10, False
    task.sensor['grip_state'] = grip_state
    task.values[physical.HAND] = task.holding_reference
    clock = [0.]
    monkeypatch.setattr(physical.time, 'monotonic', lambda: clock[0])
    def spin(*args, **kwargs):
        clock[0] += .11
        task.sensor_sequence += 1
        task.values[physical.HAND] = .7068
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    physical.PhysicalStackTask.observe_grasp_settle(task, dict(task.values))
    assert task.holding_reference == .70587
    assert task.values[physical.HAND] == .7068
    assert task.sensor_sequence >= 14


def test_old_uniform_acceleration_recipe_is_rejected_before_ros():
    value = recipe()
    for step in value['steps']:
        if step['kind'] == 'arm':
            step['acceleration_scaling'] = .05
    with pytest.raises(ValueError, match='acceleration must match'):
        physical.validate_recipe(value)


def test_wrong_grasp_classification_policy_is_rejected_before_ros():
    value = recipe()
    value['grasp_policy'] = 'sensor_contact'
    with pytest.raises(ValueError, match='grasp command policy'):
        physical.validate_recipe(value)


def test_old_grasp_detection_metadata_is_rejected_before_file_or_motion_checks(tmp_path, monkeypatch):
    path = tmp_path/'recipe.json'
    path.write_text('{}')
    metadata = dict(schema=1, path_kind='physical_stack', scene_hash_policy=physical.SCENE_HASH_POLICY,
                    recipe_policy=physical.RECIPE_POLICY, arm_execution_profile=physical.arm_execution_profile(),
                    grasp_policy='sensor_contact')
    metadata['plan_id'] = physical.digest(json.dumps(metadata, sort_keys=True).encode())[:16]
    meta_path = tmp_path/'meta.json'
    meta_path.write_text(json.dumps(metadata))
    task = SimpleNamespace(param=lambda _: True, path=path, meta_path=meta_path, check_files=Mock())
    monkeypatch.setattr(physical, 'read_held_fault', lambda: None)
    with pytest.raises(RuntimeError, match='명령 완료 기준'):
        physical.PhysicalStackTask.load_recipe(task)
    task.check_files.assert_not_called()


@pytest.mark.parametrize('old_policy', [None, 'physical_phase_acceleration_contact_envelope_v3',
    'physical_phase_acceleration_contact_envelope_v4', 'physical_phase_acceleration_contact_tracking_v5',
    'physical_phase_acceleration_contact_tracking_v6',
    'physical_phase_acceleration_contact_tracking_v7', 'physical_phase_acceleration_contact_tracking_v8',
    'physical_phase_acceleration_contact_tracking_pregrasp_v10',
    'physical_phase_acceleration_contact_tracking_buffer_v11'])
def test_old_review_policy_requires_replanning(tmp_path, monkeypatch, old_policy):
    path = tmp_path/'recipe.json'
    path.write_text('{}')
    metadata = dict(schema=1, path_kind='physical_stack', scene_hash_policy=physical.SCENE_HASH_POLICY)
    if old_policy is not None:
        metadata['recipe_policy'] = old_policy
    metadata['plan_id'] = physical.digest(json.dumps(metadata, sort_keys=True).encode())[:16]
    meta_path = tmp_path/'meta.json'
    meta_path.write_text(json.dumps(metadata))
    task = SimpleNamespace(param=lambda _: True, path=path, meta_path=meta_path, check_files=Mock())
    monkeypatch.setattr(physical, 'read_held_fault', lambda: None)
    with pytest.raises(RuntimeError, match='다시 계획'):
        physical.PhysicalStackTask.load_recipe(task)
    task.check_files.assert_not_called()


@pytest.mark.parametrize('phase', ['approach', 'lift', 'retreat', 'transfer'])
@pytest.mark.parametrize('event', ['normal_closing', 'small_opening', 'opening', 'overclosing', 'drop', 'no_contact',
                                   'uninitialized_contact', 'stale',
                                   'native_path_abort', 'native_goal_abort', 'endpoint_mismatch'])
def test_inflight_assumed_block_uses_geometry_without_object_classification(monkeypatch, event, phase):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    step['phase'] = phase
    task.values[physical.HAND] = .70587
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    finished, accepted = Future(), Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: finished, cancel_goal_async=Mock())
    accepted.set_result(handle)
    client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=Mock(return_value=accepted), destroy=Mock())
    monkeypatch.setattr(physical, 'ActionClient', lambda *args: client)
    monkeypatch.setattr(physical, 'file_digest', lambda _: 'recipe')
    task.spin_until, task.guard, task.phase, task.cancel_remaining_goal = Mock(), Mock(), Mock(), Mock()
    task.recover_pending, task.destroy_flight_queries, task.poll_flight_queries = Mock(), Mock(), Mock()
    task.flight_queries = lambda *_: []
    task.prewarm_flight_queries = lambda *_: task.values.__setitem__(physical.HAND, .70959)
    task.measured_stop = lambda **_: True
    task.input_hashes, task.path = lambda: {}, Path('/not/read')
    task.sensor_sequence, task.execution_mode, task.stopped = 10, 'step', False
    def check_stop():
        if task.stopped:
            raise RuntimeError('stopped before dispatch')
    task.check_stop = check_stop
    def spin(*args, **kwargs):
        for name in physical.ARM:
            task.values[name] = .5
        task.values[physical.HAND] = {'normal_closing': .71052, 'small_opening': .70122,
                                     'opening': .70029, 'overclosing': .734}.get(event, .71052)
        if event == 'drop':
            task.sensor['grip_state'] = 3
        elif event == 'no_contact':
            task.sensor['grip_state'] = 1
        elif event == 'uninitialized_contact':
            task.sensor['grip_state'] = 0
        if event == 'stale':
            task.sensor_fresh = lambda **_: False
        code = {'native_path_abort': -4, 'native_goal_abort': -5}.get(event, 0)
        if event == 'endpoint_mismatch':
            task.values[physical.ARM[-1]] += .00101
        finished.set_result(SimpleNamespace(status=6 if code else 4,
                                             result=SimpleNamespace(error_code=code, error_string='native detail')))
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    run = lambda: physical.PhysicalStackTask.run_motion(task, step, trajectory,
                   dict(input_sha256={}, recipe_sha256='recipe'), 'same', 1, 1)
    if event in ('normal_closing', 'small_opening', 'drop', 'no_contact', 'uninitialized_contact'):
        run()
        assert not task.stopped
        handle.cancel_goal_async.assert_not_called()
    elif event in ('native_path_abort', 'native_goal_abort', 'endpoint_mismatch'):
        classification = {'native_path_abort': 'ARM_PATH_TOLERANCE_VIOLATED',
                          'native_goal_abort': 'ARM_GOAL_TOLERANCE_VIOLATED',
                          'endpoint_mismatch': 'ARM_ENDPOINT_MISMATCH'}[event]
        with pytest.raises(RuntimeError, match=classification):
            run()
        handle.cancel_goal_async.assert_not_called()
        assert 'STACK_STEP_COMPLETED' not in [call.args[0] for call in task.phase.call_args_list]
    else:
        with pytest.raises(RuntimeError, match='motion stopped'):
            run()
        assert task.stopped
        handle.cancel_goal_async.assert_called_once()
    assert task.holding_reference == .70587
    client.send_goal_async.assert_called_once()
    goal = client.send_goal_async.call_args.args[0]
    assert [item.position for item in goal.path_tolerance] == physical.arm_execution_profile()['phase_path_position'][phase]
    assert all(item.position == .001 and item.velocity == .01 for item in goal.goal_tolerance)
    assert goal.goal_time_tolerance.sec == 5


def test_unvalidated_carrying_motion_never_dispatches_a_goal(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=Mock(), destroy=Mock())
    monkeypatch.setattr(physical, 'ActionClient', lambda *args: client)
    task.spin_until, task.guard, task.cancel_remaining_goal = Mock(), Mock(), Mock()
    task.sensor_sequence, task.stopped = 10, False
    with pytest.raises(RuntimeError, match='no matching validated holding envelope'):
        physical.PhysicalStackTask.run_motion(task, step, trajectory, {}, 'same', 1, 1)
    client.send_goal_async.assert_not_called()


def test_all_physical_phases_use_requested_point_one_with_precise_stationary_boundaries():
    profile = physical.arm_execution_profile()
    assert profile['goal_position'] == profile['start_position'] == physical.START_TOLERANCE == .001
    assert profile['goal_velocity'] == .01 and profile['goal_time_seconds'] == 5
    for phase, tolerances in profile['phase_path_position'].items():
        assert tolerances == [.1]*6
    assert set(profile['phase_path_position']) == {'approach', 'contact', 'descent_fast', 'lift', 'retreat', 'transfer'}
    assert profile['transfer_sweep_limit'] == .010
    assert profile['transfer_sweep_step'] == .005
    assert physical.HAND_HOLD_TOLERANCE == .005
    assert physical.HAND_COMMAND_TOLERANCE == .003


def test_transfer_checks_all_five_wrist_offsets_at_every_original_arm_sample():
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    step['phase'] = 'transfer'
    nominal = task.motion_samples(step, trajectory, actual_hand=.4)
    expanded = task.tracking_motion_samples(step, trajectory, actual_hand=.4)
    assert len(expanded) == 5*len(nominal)
    for index, (stamp, q) in enumerate(nominal):
        checks = expanded[index*5:index*5+5]
        assert all(time == stamp and values[:5] == q[:5] and values[6] == q[6] for time, values in checks)
        assert [values[5]-q[5] for _, values in checks] == pytest.approx([-.010, -.005, 0., .005, .010])
    for phase in ('approach', 'lift', 'retreat'):
        step['phase'] = phase
        assert task.tracking_motion_samples(step, trajectory, actual_hand=.4) == nominal


def test_transfer_held_validation_crosses_every_finger_shape_with_every_wrist_shape(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    step['phase'] = 'transfer'
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    samples = task.validate_samples.call_args.args[0]
    first = [(q[5], q[-1]) for stamp, q in samples if stamp == 0.]
    wrists, fingers = {q for q, _ in first}, {q for _, q in first}
    assert sorted(wrists) == pytest.approx([-.010, -.005, 0., .005, .010])
    assert len(first) == len(wrists)*len(fingers)
    assert set(first) == {(wrist, finger) for wrist in wrists for finger in fingers}
    assert min(fingers) == pytest.approx(.70087) and max(fingers) == .733


def test_collision_in_extra_transfer_wrist_clearance_sweep_blocks_verification(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    step['phase'] = 'transfer'
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    def reject_offset(samples):
        assert any(q[5] < -.009 for _, q in samples)  # Nominal path stays >=0.
        raise RuntimeError('collision only inside expanded wrist tolerance')
    task.validate_samples = reject_offset
    with pytest.raises(RuntimeError, match='expanded wrist tolerance'):
        physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    assert task.verified_holding_context is None


def test_tracking_profile_change_invalidates_cached_holding_geometry(monkeypatch):
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    step['phase'] = 'transfer'
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    task.scene = lambda: 'same'
    physical.PhysicalStackTask.live_arm_validation(task, step, trajectory, 'same')
    assert task.holding_context_verified(step, trajectory, 'same')
    monkeypatch.setattr(physical, 'TRANSFER_WRIST_SWEEP_LIMIT', .015)
    assert not task.holding_context_verified(step, trajectory, 'same')


@pytest.mark.parametrize('status,code,error,quiet,classification', [
    (6, -4, .4, True, 'ARM_PATH_TOLERANCE_VIOLATED'),
    (6, -5, .002, True, 'ARM_GOAL_TOLERANCE_VIOLATED'),
    (4, 0, .00101, True, 'ARM_ENDPOINT_MISMATCH'),
    (4, 0, .00099, True, 'SUCCESS'),
    (4, 0, .00099, False, 'ARM_STANDSTILL_UNCONFIRMED'),
    (5, 0, 0., True, 'ARM_ACTION_CANCELED')])
def test_native_arm_results_keep_tracking_goal_and_success_boundary_failures_distinct(status, code, error, quiet, classification):
    step = held_step()
    measured = dict(zip(physical.ARM, step['end_joints'][:6]))
    measured[physical.ARM[-1]] += error
    result = SimpleNamespace(status=status, result=SimpleNamespace(error_code=code, error_string='native wrist detail'))
    record = physical.arm_result_record(step, result, measured, quiet, {'error': {'wrist_3_joint': .005013}})
    assert record['classification'] == classification
    assert record['error_string'] == 'native wrist detail'
    assert record['joints']['wrist_3_joint']['error'] == pytest.approx(error)
    assert record['last_feedback']['error']['wrist_3_joint'] == .005013


def test_tracking_allowance_cannot_silently_cross_a_joint_limit():
    task, step, trajectory = held_validation_task(), held_step(), arm_path()
    step['phase'] = 'transfer'
    task.bounds[physical.ARM[-1]] = (-.005, .505)
    with pytest.raises(RuntimeError, match='clearance sweep exceeds its joint limit'):
        task.tracking_motion_samples(step, trajectory, actual_hand=.4)


def test_sealed_recipe_cannot_change_the_precision_contract():
    value = recipe()
    value['arm_execution_profile'] = physical.arm_execution_profile()
    physical.validate_recipe(value)
    value['arm_execution_profile']['goal_position'] = .005
    with pytest.raises(ValueError, match='tracking/goal/start profile'):
        physical.validate_recipe(value)


def test_previous_point_zero_zero_five_profile_is_rejected_before_ros():
    value = recipe()
    value['arm_execution_profile'] = physical.arm_execution_profile()
    value['arm_execution_profile']['phase_path_position'] = {
        phase: [.005]*5+[.010 if phase == 'transfer' else .005]
        for phase in physical.ARM_SPEEDS}
    with pytest.raises(ValueError, match='tracking/goal/start profile'):
        physical.validate_recipe(value)


def test_previous_point_seven_two_grasp_recipe_is_rejected_before_ros():
    value = recipe()
    for step in value['steps']:
        if step['kind'] == 'hand' and step['phase'] == 'grasp':
            step['hand_target'] = .72
        for field in ('start_joints', 'end_joints'):
            if step[field][-1] == .73:
                step[field][-1] = .72
    with pytest.raises(ValueError, match='Unexpected physical gripper target'):
        physical.validate_recipe(value)
