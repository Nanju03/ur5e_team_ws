"""Trajectory fixtures run offline with installed ROS message classes."""

import copy
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


_SOURCE = Path(__file__).resolve().parents[1] / 'ur5e_block_bringup' / 'air_trajectory.py'
_SPEC = importlib.util.spec_from_file_location('air_trajectory_under_test', _SOURCE)
air = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(air)


def point(time, position, velocity=0., acceleration=0.):
    result = JointTrajectoryPoint()
    for field, value in [('positions', position), ('velocities', velocity),
                         ('accelerations', acceleration)]:
        values = [float(value)] * 6 if np.isscalar(value) else list(value)
        setattr(result, field, values)
    nanoseconds = round(time * 1e9)
    result.time_from_start.sec, result.time_from_start.nanosec = divmod(nanoseconds, 10**9)
    return result


def trajectory(*points):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(air.ARM_JOINTS)
    result.joint_trajectory.points = list(points)
    return result


def display(*segments):
    result = DisplayTrajectory()
    result.trajectory = list(segments)
    return result


def test_rest_to_rest_quintic_extrema_are_inside_the_interval():
    metrics = air.trajectory_metrics(trajectory(point(0, 0), point(1, 1)))
    assert metrics['max_velocity'] == pytest.approx(1.875)
    assert metrics['max_acceleration'] == pytest.approx(10 * math.sqrt(3) / 3)
    assert metrics['duration'] == 1
    assert metrics['points'] == 2
    assert metrics['joint_names'] == list(air.ARM_JOINTS)


def test_global_scaling_enforces_actual_spline_limits_and_preserves_shape():
    original = display(trajectory(point(0, 0), point(1, 1)))
    before = copy.deepcopy(original)
    prepared, metrics = air.prepare_trajectory(original)
    assert original == before
    assert metrics['max_velocity'] <= .10
    assert metrics['max_acceleration'] <= .10
    assert metrics['duration'] > 18.75
    assert metrics['time_scale'] == pytest.approx(metrics['duration'], abs=1e-9)
    # q(.5) remains .5 after time scaling, with no new or changed joint points.
    samples = air.sample_trajectory(prepared, max_dt=metrics['duration'] / 2,
                                    max_joint_step=10)
    assert min(samples, key=lambda sample: abs(sample[0] - metrics['duration'] / 2))[1][0] == pytest.approx(.5)
    assert list(prepared.joint_trajectory.points[-1].positions) == [1.] * 6


def test_acceleration_limit_can_dominate_the_time_scale():
    prepared, metrics = air.prepare_trajectory(
        display(trajectory(point(0, 0), point(1, .001))),
        max_velocity=.1, max_acceleration=.00001)
    assert metrics['max_acceleration'] <= .00001
    assert metrics['time_scale'] > 20
    assert air.trajectory_metrics(prepared)['max_velocity'] < .1


def test_merges_segments_without_duplicate_boundary_times():
    prepared, metrics = air.prepare_trajectory(display(
        trajectory(point(0, 0), point(1, .01)),
        trajectory(point(0, .01), point(2, .02))))
    assert metrics['time_scale'] == 1
    assert metrics['segments'] == 2
    assert metrics['segment_end_indices'] == [1, 2]
    points = prepared.joint_trajectory.points
    assert len(points) == 3
    assert [air._time_ns(value) for value in points] == [0, 10**9, 3 * 10**9]


def test_step_boundaries_track_original_segments_after_global_retime():
    source = display(
        trajectory(point(0, 0), point(.2, .003, velocity=.02), point(.7, .01)),
        trajectory(point(0, .01), point(.3, .012, velocity=.01), point(.8, .02)),
        trajectory(point(0, .02), point(.1, .023, velocity=.03), point(.6, .03)))
    whole, metrics = air.prepare_trajectory(source, max_velocity=.003, max_acceleration=.002)
    assert metrics['time_scale'] > 1
    assert metrics['segment_end_indices'] == [2, 4, 6]
    before = copy.deepcopy(whole)
    segments = air.split_trajectory(whole, metrics['segment_end_indices'])
    assert whole == before
    assert len(segments) == 3
    starts = [0, 2, 4]
    for segment, start, end in zip(segments, starts, metrics['segment_end_indices']):
        original = whole.joint_trajectory.points[start:end + 1]
        copied = segment.joint_trajectory.points
        assert len(copied) == len(original)
        assert segment.joint_trajectory.joint_names == list(air.ARM_JOINTS)
        assert segment.joint_trajectory.header.stamp.sec == 0
        assert segment.joint_trajectory.header.stamp.nanosec == 0
        assert not segment.multi_dof_joint_trajectory.points
        origin = air._time_ns(original[0])
        for old, new in zip(original, copied):
            assert list(new.positions) == list(old.positions)
            assert list(new.velocities) == list(old.velocities)
            assert list(new.accelerations) == list(old.accelerations)
            assert air._time_ns(new) == air._time_ns(old) - origin
        # Local interval durations and q/v/a are identical, so the controller's
        # interpolation coefficients remain identical, including curved motion
        # with nonzero velocity at interior (non-cut) points.
        all_records = air._validated(whole)[start:end + 1]
        step_records = air._validated(segment)
        for global_pair, local_pair in zip(zip(all_records, all_records[1:]),
                                          zip(step_records, step_records[1:])):
            global_c, global_dt = air._coefficients(*global_pair)
            local_c, local_dt = air._coefficients(*local_pair)
            assert global_dt == local_dt
            np.testing.assert_array_equal(global_c, local_c)


def test_step_slicing_uses_final_point_times_after_caller_tcp_retime():
    whole, metrics = air.prepare_trajectory(display(
        trajectory(point(0, 0), point(.7, .001)),
        trajectory(point(0, .001), point(.9, .002))))
    # Mirrors the caller's later global TCP-speed stretch without recalculating
    # the original segment indices or using floating-point boundary guesses.
    scale = 1.337
    for value in whole.joint_trajectory.points:
        air._set_time(value, round(air._time_ns(value) * scale))
        value.velocities = [v / scale for v in value.velocities]
        value.accelerations = [a / scale**2 for a in value.accelerations]
    segments = air.split_trajectory(whole, metrics['segment_end_indices'])
    durations = [air._time_ns(segment.joint_trajectory.points[-1]) for segment in segments]
    assert sum(durations) == air._time_ns(whole.joint_trajectory.points[-1])
    assert durations[0] == air._time_ns(whole.joint_trajectory.points[1])
    assert durations[1] == (air._time_ns(whole.joint_trajectory.points[2]) -
                            air._time_ns(whole.joint_trajectory.points[1]))


def test_step_slicing_removes_old_header_and_preserves_stop_accelerations():
    whole = trajectory(point(0, 0, acceleration=.001),
                       point(2, .01, acceleration=.002),
                       point(3, .02, acceleration=.003))
    whole.joint_trajectory.header.stamp.sec = 1234
    whole.joint_trajectory.header.frame_id = 'old_schedule_frame'
    before = copy.deepcopy(whole)
    steps = air.split_trajectory(whole, (1, 2))
    assert whole == before
    assert all(step.joint_trajectory.header.stamp.sec == 0 and
               step.joint_trajectory.header.stamp.nanosec == 0 and
               step.joint_trajectory.header.frame_id == '' for step in steps)
    assert list(steps[0].joint_trajectory.points[-1].accelerations) == [.002] * 6
    assert list(steps[1].joint_trajectory.points[0].accelerations) == [.002] * 6
    # Copies are independent of each other and the sealed source.
    steps[0].joint_trajectory.points[-1].positions[0] = .9
    assert steps[1].joint_trajectory.points[0].positions[0] == .01
    assert whole == before


@pytest.mark.parametrize('boundaries', [
    [], None, {}, '1,2', [True, 2], [1.0, 2], ['1', 2], [0, 2],
    [-1, 2], [1, 1, 2], [2, 1], [1, 3], [1],
])
def test_step_slicing_rejects_invalid_or_incomplete_boundary_metadata(boundaries):
    whole = trajectory(point(0, 0), point(1, .01), point(2, .02))
    with pytest.raises(ValueError, match='segment'):
        air.split_trajectory(whole, boundaries)


@pytest.mark.parametrize('moving_index', [0, 1, 2])
def test_step_slicing_never_cuts_or_waits_at_nonzero_velocity(moving_index):
    whole = trajectory(point(0, 0), point(1, .01), point(2, .02))
    whole.joint_trajectory.points[moving_index].velocities[4] = .00001
    with pytest.raises(ValueError, match='nonzero velocity'):
        air.split_trajectory(whole, [1, 2])


def test_continuous_plan_can_preserve_a_moving_original_boundary_but_steps_reject_it():
    whole, metrics = air.prepare_trajectory(display(
        trajectory(point(0, 0), point(1, .01, velocity=.005)),
        trajectory(point(0, .01, velocity=.005), point(1, .02))))
    assert metrics['segment_end_indices'] == [1, 2]
    assert whole.joint_trajectory.points[1].velocities[0] > 0
    with pytest.raises(ValueError, match='nonzero velocity'):
        air.split_trajectory(whole, metrics['segment_end_indices'])


def test_joint_order_is_canonicalized_in_every_segment():
    first = trajectory(point(0, [float(i) for i in range(6)]),
                       point(10, [i + .01 for i in range(6)]))
    second = trajectory(point(0, [i + .01 for i in reversed(range(6))]),
                        point(10, [i + .02 for i in reversed(range(6))]))
    second.joint_trajectory.joint_names.reverse()
    prepared, _ = air.prepare_trajectory(display(first, second))
    assert prepared.joint_trajectory.joint_names == list(air.ARM_JOINTS)
    assert list(prepared.joint_trajectory.points[-1].positions) == [i + .02 for i in range(6)]


@pytest.mark.parametrize('field', ['positions', 'velocities'])
def test_rejects_discontinuity_at_segment_boundary(field):
    left = trajectory(point(0, 0), point(1, .01))
    right = trajectory(point(0, .01), point(1, .02))
    getattr(right.joint_trajectory.points[0], field)[2] += .001
    with pytest.raises(ValueError, match='discontinuous'):
        air.prepare_trajectory(display(left, right))


def test_stationary_acceleration_boundary_is_normalized_and_recorded():
    left = trajectory(point(0, 0), point(1, .01, acceleration=-.2))
    right = trajectory(point(0, .01, acceleration=.3), point(1, .02))
    source = display(left, right)
    before = copy.deepcopy(source)
    prepared, metrics = air.prepare_trajectory(source)
    assert source == before
    assert metrics['normalized_acceleration_boundaries'] == 1
    change = metrics['acceleration_boundary_changes'][0]
    assert change['point_index'] == 1
    assert change['time_before_scaling'] == 1
    assert change['previous_acceleration'] == [-.2] * 6
    assert change['next_acceleration'] == [.3] * 6
    assert change['normalized_acceleration'] == [0.] * 6
    assert list(prepared.joint_trajectory.points[1].accelerations) == [0.] * 6
    # Recompute from the modified controller spline, not the input segments.
    recomputed = air.trajectory_metrics(prepared)
    assert recomputed['max_velocity'] == metrics['max_velocity']
    assert recomputed['max_acceleration'] == metrics['max_acceleration']
    assert recomputed['max_velocity'] <= .1
    assert recomputed['max_acceleration'] <= .1
    samples = air.sample_trajectory(prepared, max_dt=.01)
    assert samples[0][1] == (0.,) * 6
    assert samples[-1][1] == (.02,) * 6


def test_acceleration_boundary_mismatch_at_nonzero_velocity_is_rejected():
    left = trajectory(point(0, 0), point(1, .01, velocity=.01, acceleration=-.2))
    right = trajectory(point(0, .01, velocity=.01, acceleration=.3), point(1, .02))
    with pytest.raises(ValueError, match='nonzero velocity'):
        air.prepare_trajectory(display(left, right))


def test_matching_nonzero_acceleration_boundary_is_preserved():
    left = trajectory(point(0, 0), point(1, .01, acceleration=.02))
    right = trajectory(point(0, .01, acceleration=.02), point(1, .02))
    prepared, metrics = air.prepare_trajectory(display(left, right))
    assert metrics['normalized_acceleration_boundaries'] == 0
    assert metrics['acceleration_boundary_changes'] == []
    assert prepared.joint_trajectory.points[1].accelerations[0] == pytest.approx(
        .02 / metrics['time_scale']**2)


def test_normalized_quintics_are_retimed_after_acceleration_changes():
    # These segments independently describe monotone cubic position curves:
    # q0->q1 with endpoint a=-6, then q1->q2 with start a=+6.
    # Normalizing their shared stop changes their interior extrema, so caps
    # must be based on the merged, modified curve.
    left = trajectory(point(0, 0, acceleration=6), point(1, 1, acceleration=-6))
    right = trajectory(point(0, 1, acceleration=6), point(1, 2, acceleration=-6))
    prepared, metrics = air.prepare_trajectory(display(left, right),
                                              max_velocity=.03, max_acceleration=.02)
    assert metrics['normalized_acceleration_boundaries'] == 1
    assert metrics['time_scale'] > 1
    assert metrics['max_velocity'] <= .03
    assert metrics['max_acceleration'] <= .02
    samples = air.sample_trajectory(prepared, max_dt=.04, max_joint_step=.002)
    assert max(q[0] for _, q in samples) == pytest.approx(2)
    assert all(right[0] - left[0] <= .040000001
               for left, right in zip(samples, samples[1:]))
    assert all(max(abs(a - b) for a, b in zip(left[1], right[1])) <= .002000001
               for left, right in zip(samples, samples[1:]))


@pytest.mark.parametrize('times', [(0, 0), (1, 2), (0, -1)])
def test_rejects_duplicate_decreasing_or_nonzero_start_times(times):
    with pytest.raises(ValueError):
        air.prepare_trajectory(display(trajectory(point(times[0], 0), point(times[1], .01))))


@pytest.mark.parametrize('field', ['positions', 'velocities', 'accelerations'])
@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
def test_rejects_nonfinite_fields(field, value):
    segment = trajectory(point(0, 0), point(1, .01))
    getattr(segment.joint_trajectory.points[1], field)[1] = value
    with pytest.raises(ValueError, match='nonfinite'):
        air.prepare_trajectory(display(segment))


@pytest.mark.parametrize('field', ['positions', 'velocities', 'accelerations'])
def test_requires_six_complete_position_velocity_acceleration_values(field):
    segment = trajectory(point(0, 0), point(1, .01))
    setattr(segment.joint_trajectory.points[0], field, [])
    with pytest.raises(ValueError, match='six'):
        air.prepare_trajectory(display(segment))


@pytest.mark.parametrize('names', [
    list(air.ARM_JOINTS[:5]),
    list(air.ARM_JOINTS[:5]) + ['left_outer_knuckle_joint'],
    list(air.ARM_JOINTS[:5]) + [air.ARM_JOINTS[0]],
    list(air.ARM_JOINTS) + ['left_outer_knuckle_joint'],
])
def test_rejects_missing_extra_duplicate_or_gripper_joints(names):
    segment = trajectory(point(0, 0), point(1, .01))
    segment.joint_trajectory.joint_names = names
    with pytest.raises(ValueError, match='six UR arm joints'):
        air.prepare_trajectory(display(segment))


def test_rejects_multi_dof_malformed_effort_and_empty_paths():
    segment = trajectory(point(0, 0), point(1, .01))
    segment.multi_dof_joint_trajectory.joint_names = ['floating_base']
    with pytest.raises(ValueError, match='Multi-DOF'):
        air.prepare_trajectory(display(segment))
    segment = trajectory(point(0, 0), point(1, .01))
    segment.joint_trajectory.points[0].effort = [1.]
    with pytest.raises(ValueError, match='effort'):
        air.prepare_trajectory(display(segment))
    with pytest.raises(ValueError):
        air.prepare_trajectory(display())
    with pytest.raises(ValueError):
        air.prepare_trajectory(display(trajectory(point(0, 0))))


def test_measured_effort_is_validated_then_removed_from_controller_commands():
    source = trajectory(point(0, 0), point(1, .01))
    source.joint_trajectory.points[0].effort = [1.] * 6
    prepared, metrics = air.prepare_trajectory(display(source))
    assert source.joint_trajectory.points[0].effort
    assert metrics['input_effort_removed']
    assert all(not point.effort for point in prepared.joint_trajectory.points)
    source.joint_trajectory.points[0].effort[0] = float('nan')
    with pytest.raises(ValueError, match='effort'):
        air.prepare_trajectory(display(source))


def test_sampling_captures_overshoot_when_endpoint_positions_are_equal():
    segment = trajectory(point(0, 0, velocity=1), point(1, 0, velocity=-1))
    samples = air.sample_trajectory(segment, max_dt=10, max_joint_step=.01)
    assert max(q[0] for _, q in samples) == pytest.approx(.3125)
    assert any(time == pytest.approx(.5) for time, _ in samples)
    assert all(max(abs(a - b) for a, b in zip(left[1], right[1])) <= .010000001
               for left, right in zip(samples, samples[1:]))


def test_sampling_enforces_dt_step_and_includes_shared_boundary_once():
    prepared, _ = air.prepare_trajectory(display(
        trajectory(point(0, 0), point(1, .01)),
        trajectory(point(0, .01), point(1, .02))))
    samples = air.sample_trajectory(prepared, max_dt=.03, max_joint_step=.0001)
    times = [time for time, _ in samples]
    assert times[0] == 0 and times[-1] == 2
    assert times.count(1.) == 1
    for left, right in zip(samples, samples[1:]):
        assert 0 < right[0] - left[0] <= .030000001
        assert max(abs(a - b) for a, b in zip(left[1], right[1])) <= .000100001


def test_nonzero_boundary_derivatives_scale_with_time():
    source = display(trajectory(point(0, 0, .2, .1), point(1, .2, .2, -.1)))
    prepared, metrics = air.prepare_trajectory(source, max_velocity=.05, max_acceleration=.01)
    scale = metrics['time_scale']
    first = prepared.joint_trajectory.points[0]
    assert first.velocities[0] == pytest.approx(.2 / scale)
    assert first.accelerations[0] == pytest.approx(.1 / scale**2)
    assert metrics['max_velocity'] <= .05
    assert metrics['max_acceleration'] <= .01


def test_decimal_interval_boundaries_are_not_reordered_by_float_addition():
    source = trajectory(point(0, 0), point(.1, .001), point(.3, .002), point(.4, .003))
    samples = air.sample_trajectory(source, max_dt=.05)
    times = [stamp for stamp, _ in samples]
    assert all(left < right for left, right in zip(times, times[1:]))
    assert times.count(.3) == 1
    assert samples[-1][0] == .4
    assert samples[-1][1] == (.003,) * 6


def test_near_endpoint_roots_quantize_to_one_exact_endpoint(monkeypatch):
    # Regression from the real empty-hand CDR: 80.793481916 + .912950542
    # rounds above 81.706432458. A root just below u=1 used to be emitted
    # after the true endpoint, making the following endpoint go backwards.
    source = trajectory(point(0, 0), point(80.793481916, 0),
                        point(81.706432458, .01))
    original_roots = air._unit_roots

    def near_endpoint_roots(coefficients):
        roots = original_roots(coefficients)
        if len(coefficients) == 5 and np.any(coefficients):
            roots += [np.nextafter(1., 0.), 1. - 1e-12, 1.]
        return roots

    monkeypatch.setattr(air, '_unit_roots', near_endpoint_roots)
    samples = air.sample_trajectory(source)
    times_ns = [round(stamp * 1e9) for stamp, _ in samples]
    assert all(left < right for left, right in zip(times_ns, times_ns[1:]))
    assert times_ns[-1] == 81706432458
    assert times_ns.count(81706432458) == 1
    assert samples[-1][1] == (.01,) * 6


def test_sampling_spacing_is_bounded_after_nanosecond_quantization():
    source = trajectory(point(0, 0), point(.100000001, .001))
    samples = air.sample_trajectory(source, max_dt=.0333333334,
                                    max_joint_step=.0001)
    times_ns = [round(stamp * 1e9) for stamp, _ in samples]
    assert all(right - left <= 33333333
               for left, right in zip(times_ns, times_ns[1:]))
    assert all(max(abs(a - b) for a, b in zip(left[1], right[1])) <= .000100001
               for left, right in zip(samples, samples[1:]))


def test_sub_nanosecond_sampling_requests_are_rejected():
    source = trajectory(point(0, 0), point(1, .01))
    with pytest.raises(ValueError, match='nanosecond'):
        air.sample_trajectory(source, max_dt=1e-12)
    with pytest.raises(ValueError, match='nanosecond'):
        air.sample_trajectory(source, max_joint_step=1e-15)


def test_nanosecond_intervals_are_stretched_and_rechecked_after_rounding():
    prepared, metrics = air.prepare_trajectory(display(
        trajectory(point(0, 0), point(1e-9, .001), point(3e-9, .002))))
    assert metrics['max_velocity'] <= .1
    assert metrics['max_acceleration'] <= .1
    assert air._time_ns(prepared.joint_trajectory.points[1]) > 0
    assert air._time_ns(prepared.joint_trajectory.points[2]) > air._time_ns(prepared.joint_trajectory.points[1])


def test_rest_points_and_samples_do_not_need_nonzero_polynomial_roots():
    source = trajectory(point(0, .5), point(.5, .5))
    metrics = air.trajectory_metrics(source)
    assert metrics['max_velocity'] == 0
    assert metrics['max_acceleration'] == 0
    assert all(q == (.5,) * 6 for _, q in air.sample_trajectory(source))


def test_unrepresentable_scaled_duration_fails_before_ros_field_assignment():
    source = display(trajectory(point(0, 0), point(1, 1)))
    with pytest.raises(ValueError, match='Duration range'):
        air.prepare_trajectory(source, max_velocity=1e-100)


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf'), True, None])
def test_invalid_limits_and_sampling_arguments_are_rejected(value):
    segment = trajectory(point(0, 0), point(1, .01))
    with pytest.raises(ValueError):
        air.prepare_trajectory(display(segment), max_velocity=value)
    with pytest.raises(ValueError):
        air.prepare_trajectory(display(segment), max_acceleration=value)
    with pytest.raises(ValueError):
        air.sample_trajectory(segment, max_dt=value)
    with pytest.raises(ValueError):
        air.sample_trajectory(segment, max_joint_step=value)
