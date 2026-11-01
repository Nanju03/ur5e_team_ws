"""Alignment adjustment time never extends ordinary arm or gripper guards."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5e_block_bringup import air_task as air


def reviewed_fixture(tmp_path, monkeypatch, age=650, kind='plate_alignment'):
    now = 10_000.
    monkeypatch.setattr(air.time, 'time', lambda: now)
    monkeypatch.setattr(air, 'scene_fingerprint', lambda _: 'same_scene')
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = air.ARM
    for index in range(2):
        point = JointTrajectoryPoint()
        point.positions, point.velocities, point.accelerations = [.01*index]*6, [0.]*6, [0.]*6
        point.time_from_start.sec = index*2
        trajectory.joint_trajectory.points.append(point)
    display = DisplayTrajectory()
    display.trajectory_start.joint_state.name = air.ARM+[air.HAND]
    display.trajectory_start.joint_state.position = [0.]*6+[.65]
    display.trajectory = [trajectory]
    data = air.serialize_message(display)
    path = tmp_path/'reviewed.cdr'
    path.write_bytes(data)
    metadata = dict(schema=2, path_kind=kind, created_at=now-age, gripper_position=.65,
                    input_sha256={'layout': 'same'}, model_sha256=air.digest(b'model'),
                    semantic_sha256=air.digest(b'semantic'), trajectory_sha256=air.digest(data),
                    hardware_class='real', scene_sha256='same_scene', scan_min_gripper_z=.82,
                    metrics={'segment_end_indices': [1]})
    if kind == 'plate_alignment':
        metadata.update(alignment_policy=air.ALIGNMENT_POLICY, valid_for_seconds=1800)
    metadata['plan_id'] = air.digest(json.dumps(metadata, sort_keys=True).encode())[:16]
    meta_path = path.with_name(path.name+'.meta.json')
    meta_path.write_text(json.dumps(metadata))
    task = SimpleNamespace(path=path, meta_path=meta_path, path_kind=kind,
                           plate_alignment=kind == 'plate_alignment', xy_scan=False,
                           model='model', semantic='semantic', scan_floor=.82,
                           param=lambda _: True, bounds={air.HAND: (0., .93)},
                           input_hashes=lambda: {'layout': 'same'},
                           validate_alignment_geometry=Mock(), check_stop=Mock(), check_model=Mock(),
                           hardware=Mock(return_value='real'),
                           call=Mock(return_value=SimpleNamespace(values=[SimpleNamespace(bool_value=True)])),
                           scene=lambda: object(), fresh=lambda: True, measured_stop=lambda: True,
                           values={**dict.fromkeys(air.ARM, 0.), air.HAND: .65})
    return task, metadata, trajectory


@pytest.mark.parametrize('age', [650, 1799, 1800])
def test_xy_alignment_load_and_live_context_allow_adjustment_within_sealed_1800_seconds(
        tmp_path, monkeypatch, age):
    task, metadata, trajectory = reviewed_fixture(tmp_path, monkeypatch, age)
    loaded, _, _ = air.AirTask.load_reviewed_path(task)
    assert loaded == metadata
    task.validate_alignment_geometry.assert_called_once()
    air.AirTask.execution_context(task, metadata, trajectory)
    task.check_model.assert_called_once()
    task.hardware.assert_called_once_with(require_controller=True)


@pytest.mark.parametrize('kind,age', [('plate_alignment', 1800.001),
                                     ('empty_hand_stack_pattern', 600.001)])
def test_expired_alignment_or_ordinary_path_is_rejected_by_load_and_live_context(
        tmp_path, monkeypatch, kind, age):
    task, metadata, trajectory = reviewed_fixture(tmp_path, monkeypatch, age, kind)
    with pytest.raises(RuntimeError, match='stale'):
        air.AirTask.load_reviewed_path(task)
    with pytest.raises(RuntimeError, match='age'):
        air.AirTask.execution_context(task, metadata, trajectory)


@pytest.mark.parametrize('change', ['scene', 'arm', 'hand', 'fresh', 'moving', 'model', 'hardware'])
def test_longer_xy_review_window_keeps_every_live_execution_guard(tmp_path, monkeypatch, change):
    task, metadata, trajectory = reviewed_fixture(tmp_path, monkeypatch)
    if change == 'scene':
        monkeypatch.setattr(air, 'scene_fingerprint', lambda _: 'changed')
    elif change == 'arm':
        task.values[air.ARM[0]] += .002
    elif change == 'hand':
        task.values[air.HAND] += .004
    elif change == 'fresh':
        task.fresh = lambda: False
    elif change == 'moving':
        task.measured_stop = lambda: False
    elif change == 'model':
        task.check_model.side_effect = RuntimeError('changed model')
    else:
        task.hardware.return_value = 'different hardware'
    with pytest.raises(RuntimeError):
        air.AirTask.execution_context(task, metadata, trajectory)


def test_gripper_preparation_keeps_600_seconds_even_if_given_alignment_metadata(tmp_path, monkeypatch):
    task, metadata, trajectory = reviewed_fixture(tmp_path, monkeypatch)
    task.mode = 'alignment_prepare'
    task.gripper_runtime = Mock()
    with pytest.raises(RuntimeError, match='주변 장면'):
        air.AirTask.alignment_prepare_context(task, metadata, trajectory)


@pytest.mark.parametrize('period', [None, 600, 3600, True])
def test_xy_alignment_cannot_silently_change_its_sealed_review_period(period):
    with pytest.raises(RuntimeError, match='유효 시간'):
        air.review_ttl(dict(path_kind='plate_alignment', alignment_policy=air.ALIGNMENT_POLICY,
                            valid_for_seconds=period))


def test_ordinary_and_old_alignment_metadata_cannot_claim_the_longer_window():
    assert air.review_ttl(dict(path_kind='empty_hand_stack_pattern', valid_for_seconds=1800)) == 600
    assert air.review_ttl(dict(path_kind='plate_alignment', alignment_policy='first_A_only',
                               valid_for_seconds=1800)) == 600
