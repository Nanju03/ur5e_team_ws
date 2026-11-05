"""Saved playback has no collision RPC; context/freshness/Stop still gate goals."""
from concurrent.futures import Future
import copy
import json
import math
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
from moveit_msgs.msg import PlanningScene, RobotTrajectory
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5e_block_bringup import physical_replay as replay
from ur5e_block_bringup import physical_stack as physical
from test_physical_stack import recipe

SCENE = 'a'*64


def bind(task, *names):
    for name in names:
        setattr(task, name, MethodType(getattr(physical.PhysicalStackTask, name), task))
    return task


def trajectory(step):
    result = RobotTrajectory()
    hand = step['kind'] == 'hand'
    result.joint_trajectory.joint_names = [physical.HAND] if hand else physical.ARM
    for stamp, state in ((0, step['start_joints']), (1, step['end_joints'])):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in (state[-1:] if hand else state[:6])]
        point.velocities = [0.]*len(point.positions)
        point.accelerations = [0.]*len(point.positions)
        point.time_from_start.sec = stamp
        result.joint_trajectory.points.append(point)
    return result


@pytest.fixture
def package(tmp_path):
    task = SimpleNamespace(path=tmp_path/'session.json', meta_path=tmp_path/'session.json.meta.json',
                           model='<robot/>', semantic='<semantic/>',
                           bounds={j: (0., .93) if j == physical.HAND else (-6., 6.) for j in physical.JOINTS})
    task.trajectory_fingerprint = physical.PhysicalStackTask.trajectory_fingerprint
    bind(task, 'motion_samples', 'tracking_motion_samples', 'certify_motion',
         'package_context', 'seal_recipe', 'install_session')
    value = recipe()
    value.update(arm_execution_profile=physical.arm_execution_profile(), grasp_policy=physical.GRASP_POLICY)
    payloads = {'initial.cdr': b'initial', 'state.cdr': b'state', 'before.cdr': b'before', 'after.cdr': b'after'}
    worker = SimpleNamespace(validate=lambda scene, samples: len(samples))
    count = 0
    for step, holding in replay.holding_steps(value['steps']):
        step.update(scene_before_sha256=SCENE, scene_after_sha256=SCENE)
        payload = serialize_message(trajectory(step)) if step['kind'] != 'scene' else b'scene state'
        payloads[step['file']] = payload
        (tmp_path/step['file']).write_bytes(payload)
        if step['kind'] != 'scene':
            count += task.certify_motion(worker, PlanningScene(), step, trajectory(step), holding)
    value.update(validated_samples=count, duration=8.)
    context = task.package_context({'selection_file': 'b'*64}, 'hardware')
    metadata, encoded = task.seal_recipe(value, payloads, context, SCENE)
    for name, payload in payloads.items():
        (tmp_path/name).write_bytes(payload)
    task.path.write_bytes(encoded)
    task.meta_path.write_text(json.dumps(metadata))
    return task, value, metadata, payloads


def test_preparation_covers_every_accepted_finger_range_and_wrist_shape(package):
    task, value, metadata, _ = package
    replay.check_certificates(value, metadata)
    arm = value['steps'][4]
    arm['phase'] = 'transfer'
    worker = SimpleNamespace(validate=Mock(side_effect=lambda scene, samples: len(samples)))
    task.certify_motion(worker, PlanningScene(), arm, trajectory(arm), True)
    samples = worker.validate.call_args.args[1]
    assert min(q[-1] for _, q in samples) == .70
    assert max(q[-1] for _, q in samples) == .733
    assert min(q[5] for _, q in samples) == -.01
    assert max(q[5] for _, q in samples) == .01
    fingers = sorted(set(q[-1] for _, q in samples))
    assert max(b-a for a, b in zip(fingers, fingers[1:])) <= .005000001


def test_initial_open_can_cover_zero_to_open_without_an_invalid_negative_angle():
    step = dict(kind='hand', phase='open', start_joints=[0.]*7,
                end_joints=[0.]*6+[.4], hand_target=.4)
    shapes = replay.finger_shapes(step, False, (0., .93))
    assert shapes[0] == 0. and shapes[-1] == .403
    assert max(b-a for a, b in zip(shapes, shapes[1:])) <= .005000001


@pytest.mark.parametrize('change', ['scene', 'state', 'file', 'range', 'count', 'missing', 'bounds'])
def test_changed_or_incomplete_certificate_cannot_be_loaded(package, change):
    _, value, metadata, _ = package
    step = value['steps'][4]
    if change == 'scene':
        step['scene_before_sha256'] = 'c'*64
    elif change == 'state':
        step['end_joints'][0] += .001
    elif change == 'file':
        step['file'] = 'unsealed.cdr'
    elif change == 'range':
        step['collision_certificate']['finger_range'][0] -= .001
    elif change == 'count':
        step['collision_certificate']['checked_samples'] = 0
    elif change == 'missing':
        step.pop('collision_certificate')
    else:
        metadata['finger_joint_bounds'] = [0., math.nan]
    with pytest.raises(ValueError):
        replay.check_certificates(value, metadata)


def test_package_is_atomic_self_contained_and_detects_corruption(package, tmp_path):
    _, value, metadata, payloads = package
    directory = replay.save_package(tmp_path/'library', metadata, value, payloads)
    assert replay.read_package(directory) == (metadata, value, payloads)
    assert not list(directory.parent.glob('.preparing_*'))
    (directory/next(iter(payloads))).write_bytes(b'changed')
    with pytest.raises(ValueError, match='checksum'):
        replay.read_package(directory)
    with pytest.raises(ValueError):
        replay.save_package(tmp_path/'library', metadata, value, payloads)


@pytest.mark.parametrize('change', ['traversal', 'symlink', 'oversize'])
def test_package_paths_and_size_are_bounded(package, tmp_path, monkeypatch, change):
    _, value, metadata, payloads = package
    directory = replay.save_package(tmp_path/'library', metadata, value, payloads)
    name = next(iter(payloads))
    if change == 'symlink':
        (directory/name).unlink()
        (directory/name).symlink_to(tmp_path/name)
    elif change == 'oversize':
        monkeypatch.setattr(replay, 'MAX_PACKAGE_BYTES', 1)
    else:
        metadata['artifact_sha256']['../outside'] = metadata['artifact_sha256'].pop(name)
        metadata.pop('plan_id')
        metadata['plan_id'] = physical.digest(json.dumps(metadata, sort_keys=True).encode())[:16]
        (directory/'recipe.json.meta.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError):
        replay.read_package(directory)


def restore_task(package, tmp_path):
    task, value, metadata, payloads = package
    task.library = tmp_path/'library'
    replay.save_package(task.library, metadata, value, payloads)
    task.path = tmp_path/'restored.json'
    task.meta_path = tmp_path/'restored.json.meta.json'
    task.check_stop = Mock()
    task.scene = lambda: SCENE
    task.fresh = task.sensor_fresh = lambda: True
    task.values = dict(zip(physical.JOINTS, value['initial_joints']))
    task.announce_plan = Mock()
    bind(task, 'restore_saved_plan')
    return task


def test_old_archive_renews_review_only_after_scene_start_context_match(package, tmp_path, monkeypatch):
    task = restore_task(package, tmp_path)
    value, metadata, payloads = package[1:]
    old = metadata['created_at']
    monkeypatch.setattr(physical.time, 'time', lambda: old+7200.)
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    assert task.restore_saved_plan(metadata['input_sha256'], 'hardware')
    renewed, restored = task.announce_plan.call_args.args
    assert renewed['created_at'] == old+7200.
    assert renewed['valid_for_seconds'] == physical.REVIEW_TTL
    assert renewed['source_plan_id'] == metadata['plan_id']
    assert renewed['collision_rechecks'] == 0
    assert restored['steps'][4]['collision_certificate'] == value['steps'][4]['collision_certificate']
    assert set(renewed['artifact_sha256']).isdisjoint(payloads)
    assert all(len(name) < 80 for name in renewed['artifact_sha256'])
    replay.check_certificates(restored, renewed)


@pytest.mark.parametrize('change', ['scene', 'arm_start', 'hand_start', 'model', 'inputs'])
def test_archive_mismatch_never_installs_a_session(package, tmp_path, monkeypatch, change):
    task = restore_task(package, tmp_path)
    metadata = package[2]
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    inputs = metadata['input_sha256']
    if change == 'scene':
        task.scene = lambda: 'd'*64
    elif change == 'arm_start':
        task.values[physical.ARM[0]] += .0011
    elif change == 'hand_start':
        task.values[physical.HAND] = .7
    elif change == 'model':
        task.model += 'changed'
    else:
        inputs = {'selection_file': 'different'}
    assert not task.restore_saved_plan(inputs, 'hardware')
    task.announce_plan.assert_not_called()
    assert not task.path.exists()


def test_runtime_pins_verified_bytes_and_never_reloads_a_changed_trajectory(package, monkeypatch):
    task, value, metadata, _ = package
    task.param = lambda _: True
    task.input_hashes = lambda: metadata['input_sha256']
    bind(task, 'check_files', 'read_message')
    monkeypatch.setattr(physical, 'read_held_fault', lambda: None)
    physical.PhysicalStackTask.load_recipe(task)
    step = value['steps'][4]
    original = task.read_message(step['file'], RobotTrajectory)
    (task.path.parent/step['file']).write_bytes(b'corrupt')
    assert task.read_message(step['file'], RobotTrajectory) == original


def held_task(package):
    task = package[0]
    task.carrying, task.release_stage = True, 0
    task.holding_reference = .70587
    task.values = {physical.HAND: .70959}
    task.sensor_fresh = lambda: True
    task.verified_holding_context = None
    task.validate_samples = Mock(side_effect=AssertionError('Collision RPC during playback'))
    bind(task, 'holding_interval', 'holding_ok', 'holding_context_key', 'holding_context_verified',
         'verified_holding_ok', 'bind_certified_motion')
    return task


def test_saved_holding_certificate_binds_measured_geometry_without_collision_rpc(package):
    task = held_task(package)
    step = package[1]['steps'][4]
    motion = trajectory(step)
    task.bind_certified_motion(step, motion, SCENE)
    assert task.holding_context_verified(step, motion, SCENE)
    task.values[physical.HAND] = .73
    assert task.verified_holding_ok(task.verified_holding_context['key'])
    task.validate_samples.assert_not_called()


@pytest.mark.parametrize('change', ['reference', 'actual', 'trajectory', 'scene'])
def test_uncovered_runtime_geometry_stops_without_falling_back_to_slow_validation(package, change):
    task = held_task(package)
    step, scene = package[1]['steps'][4], SCENE
    motion = trajectory(step)
    if change == 'reference':
        task.holding_reference = .704
    elif change == 'actual':
        task.values[physical.HAND] = .7
    elif change == 'trajectory':
        motion.joint_trajectory.points[-1].positions[0] += .01
    else:
        scene = 'e'*64
    with pytest.raises(RuntimeError):
        task.bind_certified_motion(step, motion, scene)
    task.validate_samples.assert_not_called()


def test_canonical_trajectory_fingerprint_survives_reserialization(package):
    task = package[0]
    motion = trajectory(package[1]['steps'][4])
    assert len({task.trajectory_fingerprint(copy.deepcopy(motion)) for _ in range(100)}) == 1


@pytest.mark.parametrize('fault', ['none', 'arm_start', 'sensor', 'stop', 'selection', 'recipe', 'expired'])
def test_fast_boundary_keeps_start_feedback_stop_and_integrity_checks(package, monkeypatch, fault):
    task = held_task(package)
    step, metadata = package[1]['steps'][4], package[2]
    task.values.update(dict(zip(physical.ARM, step['start_joints'][:6])))
    task.input_hashes = lambda: metadata['input_sha256']
    task.prepare_replay_monitors = Mock()
    task.measured_stop = lambda: True
    task.fresh = lambda: True
    task.stopped, task.goal = False, None
    task.preserve_contact_after_stop = Mock()
    task.call = Mock(side_effect=AssertionError('Repeated preflight RPC'))
    bind(task, 'check_stop')
    monkeypatch.setattr(physical.rclpy, 'spin_once', lambda *args, **kwargs: None)
    if fault == 'arm_start':
        task.values[physical.ARM[0]] += .0011
    elif fault == 'sensor':
        task.sensor_fresh = lambda: False
    elif fault == 'stop':
        task.stopped = True
    elif fault == 'selection':
        task.input_hashes = lambda: {}
    elif fault == 'recipe':
        task.path.write_text('changed')
    elif fault == 'expired':
        metadata['created_at'] -= 7200.
    if fault == 'none':
        physical.PhysicalStackTask.replay_checkpoint(task, metadata, step, SCENE)
    else:
        with pytest.raises(RuntimeError):
            physical.PhysicalStackTask.replay_checkpoint(task, metadata, step, SCENE)
    task.call.assert_not_called()
    task.validate_samples.assert_not_called()


def test_fast_moving_monitor_preserves_two_second_timeout(monkeypatch):
    task = SimpleNamespace(prevalidated_replay=True, stopped=False)
    query = dict(service='controller', future=Future(), sent=0., valid=lambda _: True)
    monkeypatch.setattr(physical.time, 'monotonic', lambda: 2.001)
    physical.PhysicalStackTask.poll_flight_queries(task, [query])
    assert task.stopped and task.stop_reason['code'] == 'MONITOR_RPC_TIMEOUT'
    assert task.stop_reason['moving'] is True


def test_persistent_monitors_reuse_acknowledgements_but_invalidate_changed_scene(monkeypatch):
    removed = Mock()
    queries = [dict(service=service, client=SimpleNamespace(remove_pending_request=removed),
                    future=Future(), verified=True, last_ok=1., sent=1.)
               for service in ('/get_planning_scene', '/controller_manager/list_controllers')]
    task = SimpleNamespace(replay_queries=queries, replay_monitor_scene='old', prewarm_flight_queries=Mock())
    bind(task, 'destroy_flight_queries')
    monkeypatch.setattr(physical, 'scene_fingerprint', lambda scene: scene)
    old_scene, controller = queries[0]['future'], queries[1]['future']
    physical.PhysicalStackTask.prepare_replay_monitors(task, {}, 'old')
    assert queries[0]['future'] is old_scene
    physical.PhysicalStackTask.prepare_replay_monitors(task, {}, 'new')
    removed.assert_called_once_with(old_scene)
    assert queries[0]['future'] is None and not queries[0]['verified']
    old_scene.set_result(SimpleNamespace(scene='old'))
    assert not queries[0]['valid'](old_scene.result())
    assert queries[0]['valid'](SimpleNamespace(scene='new'))
    assert queries[1]['future'] is controller and queries[1]['verified']


def test_warm_monitors_do_not_repeat_discovery_or_drop_inflight_queries(monkeypatch):
    monkeypatch.setattr(physical.time, 'monotonic', lambda: 5.)
    task = SimpleNamespace(prevalidated_replay=True, check_stop=Mock(), fresh=lambda: True,
                           sensor_fresh=lambda: True, poll_flight_queries=Mock(), destroy_flight_queries=Mock())
    future = Future()
    query = dict(verified=True, last_ok=4.9, future=future, sent=4.9)
    physical.PhysicalStackTask.prewarm_flight_queries(task, [query])
    task.destroy_flight_queries.assert_not_called()
    assert query['future'] is future


def test_stationary_history_is_reused_and_motion_or_feedback_gap_resets_it(monkeypatch):
    now = [1.]
    monkeypatch.setattr(physical.time, 'monotonic', lambda: now[0])
    task = physical.PhysicalStackTask.__new__(physical.PhysicalStackTask)
    task.values, task.velocities, task.received = {}, {}, {}
    task.prevalidated_replay = True
    task.sensor_fresh = lambda: True
    message = JointState(name=physical.JOINTS, position=[0.]*7, velocity=[0.]*7)
    task.joints(message)
    now[0] = 1.31
    task.joints(message)
    monkeypatch.setattr(physical.rclpy, 'spin_once', Mock(side_effect=AssertionError('Duplicate 300ms wait')))
    assert task.measured_stop() and task.measured_stop()
    now[0] = 1.32
    message.velocity[0] = .02
    task.joints(message)
    assert task.stationary_since is None
    now[0], message.velocity[0] = 1.9, 0.
    task.joints(message)
    assert task.stationary_since == 1.9
    now[0] = 2.5
    assert not task.quiet_feedback()


def test_action_clients_persist_across_saved_motions_and_close_once(monkeypatch):
    task = SimpleNamespace(prevalidated_replay=True, mock=False, motion_clients={}, monitor_clients={},
        path=Path('/not-opened/recipe.json'),
        replay_queries=[], carrying=False, release_stage=0, stopped=False, sensor_sequence=10,
        values=dict(zip(physical.JOINTS, [.01]+[0.]*5+[.4])), sensor={}, execution_mode='continuous',
        replay_checkpoint=Mock(), fresh=lambda: True, sensor_fresh=lambda **_: True,
        measured_stop=lambda **_: True, check_stop=Mock(), holding_ok=lambda: True,
        prewarm_flight_queries=Mock(), input_hashes=lambda: {}, cancel_remaining_goal=Mock(), phase=Mock())
    task.spin_until = lambda predicate: predicate()
    result = SimpleNamespace(status=4, result=SimpleNamespace(error_code=0, error_string=''))
    result_future = Future(); result_future.set_result(result)
    goal = SimpleNamespace(accepted=True, get_result_async=lambda: result_future)
    accepted = Future(); accepted.set_result(goal)
    client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=Mock(return_value=accepted), destroy=Mock())
    factory = Mock(return_value=client)
    monkeypatch.setattr(physical, 'ActionClient', factory)
    monkeypatch.setattr(physical, 'file_digest', lambda _: 'recipe')
    step = recipe()['steps'][1]
    for _ in range(2):
        physical.PhysicalStackTask.run_motion(task, step, trajectory(step),
            dict(input_sha256={}, recipe_sha256='recipe'), SCENE, 1, 1)
    assert factory.call_count == 1 and client.send_goal_async.call_count == 2
    client.destroy.assert_not_called()
    physical.PhysicalStackTask.close_monitor_clients(task)
    client.destroy.assert_called_once()
