"""Validate, time-scale, and sample six-axis UR trajectories offline.

The trajectory controller's position/velocity/acceleration interpolation is a
quintic Hermite polynomial on each interval. Extrema and samples below use
that polynomial, rather than linear interpolation between the positions.
No actions, ROS nodes, hardware interfaces, or network connections are used.
"""

import math

import numpy as np
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)
_NANOSECONDS = 1_000_000_000
_CONTINUITY_TOLERANCE = 1e-8
_MAX_SAMPLES = 1_000_000


def _positive(value, name):
    if isinstance(value, bool):
        raise ValueError(f'{name} must be finite and positive')
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must be finite and positive') from exc
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def _time_ns(point):
    seconds = point.time_from_start.sec
    nanoseconds = point.time_from_start.nanosec
    if seconds < 0 or not 0 <= nanoseconds < _NANOSECONDS:
        raise ValueError('Trajectory times must be nonnegative normalized durations')
    return seconds * _NANOSECONDS + nanoseconds


def _set_time(point, nanoseconds):
    seconds, remainder = divmod(nanoseconds, _NANOSECONDS)
    if not 0 <= seconds <= 2**31 - 1:
        raise ValueError('Time-scaled trajectory exceeds the ROS Duration range')
    point.time_from_start.sec = seconds
    point.time_from_start.nanosec = remainder


def _vectors(point, indices):
    if point.effort:
        # MoveIt can copy measured effort from a RobotState into its planned
        # message. Validate that optional sensor data, then omit it from the
        # new position/velocity/acceleration-only controller trajectory.
        if len(point.effort) != 6 or not np.isfinite(point.effort).all():
            raise ValueError('Optional measured effort must contain six finite values')
    result = []
    for field in ('positions', 'velocities', 'accelerations'):
        values = getattr(point, field)
        if len(values) != 6:
            raise ValueError(f'Every point must contain six {field}')
        values = np.asarray([values[index] for index in indices], dtype=float)
        if not np.isfinite(values).all():
            raise ValueError(f'Trajectory {field} contain nonfinite values')
        result.append(values)
    return tuple(result)


def _validated(trajectory):
    """Return integer times and canonical q/v/a arrays without mutating input."""
    multi = trajectory.multi_dof_joint_trajectory
    if multi.joint_names or multi.points:
        raise ValueError('Multi-DOF trajectories are not supported for arm motion')
    joint = trajectory.joint_trajectory
    names = list(joint.joint_names)
    if len(names) != 6 or set(names) != set(ARM_JOINTS):
        raise ValueError('A trajectory must contain exactly the six UR arm joints')
    if len(joint.points) < 2:
        raise ValueError('Each trajectory requires at least two points')
    indices = [names.index(name) for name in ARM_JOINTS]
    result = []
    for point in joint.points:
        stamp = _time_ns(point)
        if result and stamp <= result[-1][0]:
            raise ValueError('Trajectory point times must be strictly increasing')
        result.append((stamp, *_vectors(point, indices)))
    if result[0][0] != 0:
        raise ValueError('Each trajectory must start at time zero')
    return result


def _make_point(stamp, q, v, a):
    point = JointTrajectoryPoint()
    point.positions = [float(value) for value in q]
    point.velocities = [float(value) for value in v]
    point.accelerations = [float(value) for value in a]
    _set_time(point, stamp)
    return point


def _coefficients(left, right):
    """Return q(u) coefficients, u=(t-left.time)/duration, without padding."""
    duration = (right[0] - left[0]) / _NANOSECONDS
    _, q0, v0, a0 = left
    _, q1, v1, a1 = right
    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            c0 = q0
            c1 = v0 * duration
            c2 = a0 * duration**2 / 2
            dq = q1 - c0 - c1 - c2
            dv = v1 * duration - c1 - 2 * c2
            da = a1 * duration**2 - 2 * c2
            coefficients = np.array((
                c0, c1, c2, 10 * dq - 4 * dv + da / 2,
                -15 * dq + 7 * dv - da, 6 * dq - 3 * dv + da / 2,
            ))
    except (FloatingPointError, OverflowError) as exc:
        raise ValueError('Trajectory polynomial is numerically out of range') from exc
    if not np.isfinite(coefficients).all():
        raise ValueError('Trajectory polynomial contains nonfinite coefficients')
    return coefficients, duration


def _differentiate(coefficients, duration):
    with np.errstate(over='raise', invalid='raise', divide='raise'):
        try:
            result = coefficients[1:] * np.arange(
                1, len(coefficients), dtype=float)[:, None] / duration
        except FloatingPointError as exc:
            raise ValueError('Trajectory derivative is numerically out of range') from exc
    return result


def _unit_roots(coefficients):
    """Real roots of an ascending-order polynomial on the closed unit interval."""
    if not np.isfinite(coefficients).all():
        raise ValueError('Polynomial root calculation requires finite coefficients')
    try:
        roots = np.polynomial.polynomial.polyroots(coefficients)
    except (np.linalg.LinAlgError, FloatingPointError, ValueError) as exc:
        raise ValueError('Cannot determine trajectory polynomial extrema') from exc
    result = []
    for root in roots:
        if not np.isfinite(root):
            raise ValueError('Trajectory polynomial produced nonfinite roots')
        if abs(root.imag) <= 1e-8 * max(1.0, abs(root.real)):
            real = float(root.real)
            if -1e-10 <= real <= 1 + 1e-10:
                result.append(min(1.0, max(0.0, real)))
    return result


def _derivative_maxima(coefficients, duration):
    velocity = _differentiate(coefficients, duration)
    acceleration = _differentiate(velocity, duration)
    jerk = _differentiate(acceleration, duration)
    maximum_v = np.zeros(6)
    maximum_a = np.zeros(6)
    for index in range(6):
        velocity_times = [0.0, 1.0] + _unit_roots(acceleration[:, index])
        acceleration_times = [0.0, 1.0] + _unit_roots(jerk[:, index])
        maximum_v[index] = max(abs(float(np.polynomial.polynomial.polyval(
            value, velocity[:, index]))) for value in velocity_times)
        maximum_a[index] = max(abs(float(np.polynomial.polynomial.polyval(
            value, acceleration[:, index]))) for value in acceleration_times)
    if not np.isfinite(maximum_v).all() or not np.isfinite(maximum_a).all():
        raise ValueError('Trajectory derivative extrema are nonfinite')
    return maximum_v, maximum_a


def trajectory_metrics(trajectory):
    """Return exact polynomial velocity/acceleration extrema for each joint.

    Limits are angular units: radians/second and radians/second squared.
    Position bounds, FK, and collision validity require the caller's robot model.
    """
    records = _validated(trajectory)
    maximum_v = np.zeros(6)
    maximum_a = np.zeros(6)
    for left, right in zip(records, records[1:]):
        velocity, acceleration = _derivative_maxima(*_coefficients(left, right))
        maximum_v = np.maximum(maximum_v, velocity)
        maximum_a = np.maximum(maximum_a, acceleration)
    return {
        'joint_names': list(ARM_JOINTS),
        'points': len(records),
        'duration': records[-1][0] / _NANOSECONDS,
        'max_velocity': float(max(maximum_v)),
        'max_acceleration': float(max(maximum_a)),
        'joint_velocity_maxima': [float(value) for value in maximum_v],
        'joint_acceleration_maxima': [float(value) for value in maximum_a],
    }


def prepare_trajectory(display, max_velocity=.10, max_acceleration=.10):
    """Merge arm segments and globally slow their exact quintic motion shape.

    Shared boundary points require matching q/v. At a stationary boundary only,
    mismatching accelerations are normalized to zero on both sides and recorded
    in the metrics. This changes the adjacent quintics; the caller must preview
    and collision-check this returned artifact before sealing it for execution.
    All inputs are left untouched. The returned header has zero stamp so it does
    not inherit an expired execution schedule.
    """
    max_velocity = _positive(max_velocity, 'max_velocity')
    max_acceleration = _positive(max_acceleration, 'max_acceleration')
    if not display.trajectory:
        raise ValueError('DisplayTrajectory contains no trajectories')
    merged = RobotTrajectory()
    merged.joint_trajectory.joint_names = list(ARM_JOINTS)
    offset = 0
    previous = None
    acceleration_changes = []
    segment_end_indices = []
    for trajectory in display.trajectory:
        records = _validated(trajectory)
        last = records[-1]
        if previous is not None:
            for field, old, new in zip(('position', 'velocity'),
                                       previous[1:3], records[0][1:3]):
                if not np.allclose(old, new, rtol=0.0, atol=_CONTINUITY_TOLERANCE):
                    raise ValueError(f'Segment boundary {field} is discontinuous')
            if not np.allclose(previous[3], records[0][3], rtol=0.0,
                               atol=_CONTINUITY_TOLERANCE):
                if (np.max(np.abs(previous[2])) > _CONTINUITY_TOLERANCE or
                        np.max(np.abs(records[0][2])) > _CONTINUITY_TOLERANCE):
                    raise ValueError(
                        'Segment boundary acceleration differs at nonzero velocity')
                acceleration_changes.append({
                    'point_index': len(merged.joint_trajectory.points) - 1,
                    'time_before_scaling': offset / _NANOSECONDS,
                    'previous_acceleration': [float(value) for value in previous[3]],
                    'next_acceleration': [float(value) for value in records[0][3]],
                    'normalized_acceleration': [0.0] * 6,
                })
                # These arrays are private copies from _validated(), never
                # views into source messages. The merged shared endpoint now
                # supplies a=0 to both adjacent controller quintics.
                previous[3][:] = 0.0
                records[0][3][:] = 0.0
                merged.joint_trajectory.points[-1].accelerations = [0.0] * 6
            records = records[1:]
        for stamp, q, v, a in records:
            merged.joint_trajectory.points.append(_make_point(offset + stamp, q, v, a))
        segment_end_indices.append(len(merged.joint_trajectory.points) - 1)
        previous = last
        offset += previous[0]

    original = _validated(merged)
    metrics = trajectory_metrics(merged)
    scale = max(1.0, metrics['max_velocity'] / max_velocity,
                math.sqrt(metrics['max_acceleration'] / max_acceleration))
    if scale > 1.0:
        # Re-evaluate after integer-nanosecond rounding. A global scale keeps
        # q(t/s), v/s, and a/s² together instead of changing individual slopes.
        scale *= 1.000001
        for _ in range(8):
            if not math.isfinite(scale):
                raise ValueError('Required trajectory time scaling is nonfinite')
            points = []
            for stamp, q, v, a in original:
                stretched = stamp * scale
                if (not math.isfinite(stretched) or
                        stretched > (2**31 - 1) * _NANOSECONDS + _NANOSECONDS - 1):
                    raise ValueError('Time-scaled trajectory exceeds the ROS Duration range')
                points.append(_make_point(round(stretched), q, v / scale, a / scale / scale))
            merged.joint_trajectory.points = points
            metrics = trajectory_metrics(merged)
            if (metrics['max_velocity'] <= max_velocity and
                    metrics['max_acceleration'] <= max_acceleration):
                break
            correction = max(metrics['max_velocity'] / max_velocity,
                             math.sqrt(metrics['max_acceleration'] / max_acceleration))
            scale *= max(1.000001, correction * 1.000001)
        else:
            raise ValueError('Could not enforce trajectory limits after duration rounding')
    metrics['time_scale'] = float(scale)
    metrics['velocity_limit'] = max_velocity
    metrics['acceleration_limit'] = max_acceleration
    metrics['segments'] = len(display.trajectory)
    # Point indices survive both the global retime above and the caller's
    # optional TCP-speed retime. They identify original planner segments,
    # rather than inventing cuts inside a moving quintic interval.
    metrics['segment_end_indices'] = segment_end_indices
    for change in acceleration_changes:
        change['time_after_scaling'] = _time_ns(
            merged.joint_trajectory.points[change['point_index']]) / _NANOSECONDS
    metrics['normalized_acceleration_boundaries'] = len(acceleration_changes)
    metrics['acceleration_boundary_changes'] = acceleration_changes
    metrics['input_effort_removed'] = any(
        bool(point.effort) for trajectory in display.trajectory
        for point in trajectory.joint_trajectory.points)
    return merged, metrics


def split_trajectory(trajectory, segment_end_indices):
    """Copy exact reviewed planner segments into separate rest-to-rest goals.

    ``segment_end_indices`` must come from ``prepare_trajectory`` and be sealed
    with the reviewed trajectory. Inclusive shared endpoints belong to both
    neighboring segments. Integer-nanosecond time subtraction preserves each
    controller quintic exactly while making every segment start at local t=0.
    This never retimes, interpolates, or changes endpoint derivatives. A moving
    boundary is rejected because waiting there would alter the reviewed path.
    """
    records = _validated(trajectory)
    if not isinstance(segment_end_indices, (list, tuple)) or not segment_end_indices:
        raise ValueError('Planner segment end indices must be a nonempty list')
    previous = 0
    for index in segment_end_indices:
        if isinstance(index, bool) or not isinstance(index, int):
            raise ValueError('Planner segment end indices must be integers')
        if not previous < index < len(records):
            raise ValueError('Planner segment end indices must increase and stay within the trajectory')
        previous = index
    if segment_end_indices[-1] != len(records) - 1:
        raise ValueError('Planner segments must cover the complete reviewed trajectory')
    boundaries = [0] + list(segment_end_indices)
    for index in boundaries:
        if np.max(np.abs(records[index][2])) > _CONTINUITY_TOLERANCE:
            raise ValueError(f'Planner segment boundary {index} has nonzero velocity')
    result = []
    for start, end in zip(boundaries, boundaries[1:]):
        segment = RobotTrajectory()
        segment.joint_trajectory.joint_names = list(ARM_JOINTS)
        origin = records[start][0]
        segment.joint_trajectory.points = [
            _make_point(stamp - origin, q, v, a)
            for stamp, q, v, a in records[start:end + 1]
        ]
        result.append(segment)
    return result


def sample_trajectory(trajectory, max_dt=.05, max_joint_step=.01):
    """Sample actual quintics with both a time and a per-joint distance bound.

    The global velocity bound on each interval limits motion between samples,
    including a returning/overshooting joint with equal endpoint positions.
    Exact position-extremum times are included as additional samples.
    Returns [(seconds_from_start, (six joint positions))].
    """
    max_dt = _positive(max_dt, 'max_dt')
    max_joint_step = _positive(max_joint_step, 'max_joint_step')
    max_dt_ns = math.floor(max_dt * _NANOSECONDS)
    if max_dt_ns < 1:
        raise ValueError('Sampling time spacing must allow at least one nanosecond')
    records = _validated(trajectory)
    samples = []
    last_ns = None
    for left, right in zip(records, records[1:]):
        coefficients, duration = _coefficients(left, right)
        maximum_v, _ = _derivative_maxima(coefficients, duration)
        velocity_bound = float(max(maximum_v))
        spacing_ns = max_dt_ns
        if velocity_bound:
            distance_spacing = max_joint_step / velocity_bound * _NANOSECONDS
            if math.isfinite(distance_spacing):
                spacing_ns = min(spacing_ns, math.floor(distance_spacing))
        if spacing_ns < 1:
            raise ValueError('Requested joint spacing requires sub-nanosecond samples')
        interval_ns = right[0] - left[0]
        count = max(1, (interval_ns + spacing_ns - 1) // spacing_ns)
        if len(samples) + count + 32 > _MAX_SAMPLES:
            raise ValueError('Trajectory requires more than one million validity samples')
        # Integer time is the canonical sample identity. Evaluating q at that
        # same quantized time prevents nearly repeated roots near u=1 from
        # placing a floating-point sample past the exact interval endpoint.
        times_ns = {
            left[0] + interval_ns * index // count for index in range(count + 1)
        }
        velocity = _differentiate(coefficients, duration)
        for index in range(6):
            for root in _unit_roots(velocity[:, index]):
                offset_ns = min(interval_ns, max(0, round(root * interval_ns)))
                times_ns.add(left[0] + offset_ns)
        for stamp_ns in sorted(times_ns):
            if stamp_ns == left[0]:
                position = left[1]
            elif stamp_ns == right[0]:
                position = right[1]
            else:
                normalized = (stamp_ns - left[0]) / interval_ns
                position = np.polynomial.polynomial.polyval(normalized, coefficients)
            if not np.isfinite(position).all():
                raise ValueError('Trajectory sample contains nonfinite joint positions')
            if last_ns is not None and stamp_ns <= last_ns:
                if stamp_ns == last_ns:
                    if not np.allclose(position, samples[-1][1], rtol=0.0,
                                       atol=_CONTINUITY_TOLERANCE):
                        raise ValueError('Different positions share one sample timestamp')
                    continue
                raise ValueError('Trajectory sampling produced non-increasing times')
            stamp = stamp_ns / _NANOSECONDS
            if samples and stamp <= samples[-1][0]:
                raise ValueError('Sample times exceed floating-point time resolution')
            samples.append((float(stamp), tuple(float(value) for value in position)))
            last_ns = stamp_ns
    return samples
