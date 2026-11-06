"""Read-only speed conversion and freshness checks; no ROS nodes or hardware."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


_SPEC = importlib.util.spec_from_file_location(
    'speed_monitor_under_test', Path(__file__).resolve().parents[1]
    / 'ur5e_block_bringup' / 'speed_monitor.py')
speed = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(speed)


def ready_speed():
    result = speed.PositionSpeed()
    result.add(10., (0., 0., 0.), 'base', 20.)
    result.add(10.1, (.01, -.02, .03), 'base', 20.1)
    return result


def test_position_speed_recovers_signed_constant_velocity_in_metres_per_second():
    estimate = speed.PositionSpeed()
    expected = (.2, -.4, .1)
    for index in range(21):
        elapsed = index * .02
        estimate.add(100.+elapsed, tuple(x*elapsed for x in expected), 'base', 10.+elapsed)
    assert estimate.velocity(10.4) == pytest.approx(expected)
    # The estimator retains a short recent window instead of the entire motion.
    assert estimate.samples[-1][0]-estimate.samples[0][0] <= .12 + 1e-9


def test_pose_requires_a_nonzero_measurement_window_and_expires_without_feedback():
    estimate = speed.PositionSpeed()
    estimate.add(10., (0., 0., 0.), 'base', 20.)
    estimate.add(10.01, (1., 0., 0.), 'base', 20.01)
    assert estimate.velocity(20.01) is None
    estimate = ready_speed()
    assert estimate.velocity(20.1) == pytest.approx((.1, -.2, .3))
    assert estimate.velocity(20.601) is None


def test_repeated_stamp_does_not_refresh_feedback_or_fabricate_motion():
    estimate = ready_speed()
    estimate.add(10.1, (100., 200., 300.), 'base', 20.59)
    assert estimate.received == 20.1
    assert estimate.velocity(20.2) == pytest.approx((.1, -.2, .3))
    assert estimate.velocity(20.601) is None


@pytest.mark.parametrize('stamp', [1., 11.])
def test_clock_rewind_or_long_sample_gap_resets_differentiation(stamp):
    estimate = ready_speed()
    estimate.add(stamp, (10., 0., 0.), 'base', 20.2)
    assert estimate.velocity(20.2) is None
    estimate.add(stamp+.1, (10.02, 0., 0.), 'base', 20.3)
    assert estimate.velocity(20.3) == pytest.approx((.2, 0., 0.))


def test_coordinate_frame_change_cannot_be_mistaken_for_fast_translation():
    estimate = ready_speed()
    estimate.add(10.2, (500., 200., 100.), 'other_base', 20.2)
    assert estimate.velocity(20.2) is None
    estimate.add(10.3, (500.01, 200., 100.), 'other_base', 20.3)
    assert estimate.velocity(20.3) == pytest.approx((.1, 0., 0.))


@pytest.mark.parametrize('stamp,xyz,frame', [
    (0., (0., 0., 0.), 'base'),
    (math.nan, (0., 0., 0.), 'base'),
    (10.2, (math.inf, 0., 0.), 'base'),
    (10.2, (0., 0., 0.), ''),
])
def test_invalid_pose_sample_does_not_extend_freshness(stamp, xyz, frame):
    estimate = ready_speed()
    estimate.add(stamp, xyz, frame, 20.59)
    assert estimate.received == 20.1
    assert estimate.velocity(20.601) is None


def message(names=None, position=None, velocity=None):
    return SimpleNamespace(
        name=list(speed.ARM if names is None else names),
        position=list(range(6)) if position is None else position,
        velocity=[index/10 for index in range(6)] if velocity is None else velocity)


def test_joint_values_use_names_and_ignore_unrelated_gripper_values():
    names = ['left_outer_knuckle_joint', *reversed(speed.ARM)]
    sample = message(names, [math.nan, 5, 4, 3, 2, 1, 0],
                     [math.nan, .5, .4, .3, .2, .1, 0.])
    q, dq = speed.arm_values(sample)
    assert q == tuple(range(6))
    assert dq == pytest.approx((0., .1, .2, .3, .4, .5))


@pytest.mark.parametrize('sample', [
    message(['left_outer_knuckle_joint'], [.4], [0.]),
    message(speed.ARM[:-1], [0.]*5, [0.]*5),
    message([*speed.ARM, speed.ARM[0]], [0.]*7, [0.]*7),
    message(position=[0.]*5),
    message(velocity=[]),
    message(position=[0., 0., math.nan, 0., 0., 0.]),
    message(velocity=[0., 0., 0., math.inf, 0., 0.]),
])
def test_incomplete_or_nonfinite_arm_feedback_is_rejected(sample):
    assert speed.arm_values(sample) is None


class RotatingPoint:
    """Known geometry: a point 0.2m from a Z-axis revolute joint."""
    def tcp(self, joints):
        angle = joints[speed.ARM[0]]
        result = np.eye(4)
        result[:3, 3] = (.2*math.cos(angle), .2*math.sin(angle), .35)
        return result


def test_model_speed_matches_analytic_tangential_velocity():
    angle, angular_velocity = math.pi/6, 2.
    actual = speed.grasp_velocity(RotatingPoint(), (angle, 0., 0., 0., 0., 0.),
                                  (angular_velocity, 0., 0., 0., 0., 0.))
    expected = (-.2*math.sin(angle)*angular_velocity,
                .2*math.cos(angle)*angular_velocity, 0.)
    assert actual == pytest.approx(expected, rel=1e-7, abs=1e-10)
    assert math.sqrt(sum(x*x for x in actual)) == pytest.approx(.4, rel=1e-7)


def test_zero_joint_velocity_gives_zero_model_speed():
    assert speed.grasp_velocity(RotatingPoint(), (.7, 0., 0., 0., 0., 0.),
                                (0.,)*6) == (0., 0., 0.)
