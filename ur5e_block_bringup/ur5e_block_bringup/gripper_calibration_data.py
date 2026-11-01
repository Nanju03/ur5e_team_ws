"""Offline geometry and evidence rules for AG95 calibration.

The gap is a URDF estimate. Force is a controller setting in percent, never a
measured gripping force. Recommendations contain only commands actually tried.
"""
import math
from statistics import median

import numpy as np

from .air_scene import RobotKinematics, _revolute_motion, _transform_vertices


def validate_settings(open_q=.50, max_q=.78, step_q=.01, force_percent=20,
                      settle_s=.4, width_mm=25):
    values = dict(open_q=open_q, max_q=max_q, step_q=step_q,
                  force_percent=force_percent, settle_s=settle_s, width_mm=width_mm)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in values.values()):
        raise ValueError('All calibration settings must be finite numbers')
    if not 0 <= open_q < max_q <= .85:
        raise ValueError('Require 0 <= open angle < maximum angle <= 0.85 rad')
    if not .001 <= step_q <= .03:
        raise ValueError('The closing step must be 0.001..0.03 rad')
    if not 20 <= force_percent <= 100 or int(force_percent) != force_percent:
        raise ValueError('Force setting must be an integer percentage from 20 to 100')
    if not .3 <= settle_s <= 2 or not 0 < width_mm <= 95:
        raise ValueError('Require settle time 0.3..2 s and block width 0..95 mm')
    return {**{k: float(v) for k, v in values.items()}, 'force_percent': int(force_percent)}


class GripperGapModel:
    def __init__(self, xml):
        self.robot = RobotKinematics(xml)
        self.robot._prepare_gripper_geometry()
        self.pads = ('left_finger_pad', 'right_finger_pad')
        if any(p not in self.robot._gripper_geometry for p in self.pads):
            raise ValueError('AG95 finger-pad collision geometry is required')

    def gap_mm(self, q):
        if isinstance(q, bool) or not math.isfinite(q) or not 0 <= q <= .93:
            raise ValueError('AG95 angle must be 0..0.93 rad')
        r = self.robot
        positions = {'left_outer_knuckle_joint': q}

        def value(name):
            if name not in positions:
                source, multiplier, offset = r._gripper_mimics[name]
                positions[name] = value(source)*multiplier+offset
            return positions[name]

        poses, pending = {'ag95_base_link': np.eye(4)}, ['ag95_base_link']
        while pending:
            parent = pending.pop()
            for child in r._children.get(parent, []):
                name, kind, origin, axis = r._parents[child][1]
                poses[child] = poses[parent] @ origin
                if kind != 'fixed':
                    poses[child] = poses[child] @ _revolute_motion(axis, value(name))
                pending.append(child)
        left, right = [_transform_vertices(poses[p], r._gripper_geometry[p]) for p in self.pads]
        direction = left.mean(axis=0)-right.mean(axis=0)
        direction /= np.linalg.norm(direction)
        return max(0., float((left @ direction).min()-(right @ direction).max())*1000)


def recommend(trials, baseline):
    """Require an empty sweep and three released, operator-verified grasps.

    Reject force groups with a failed/dropped trial or inconsistent contact
    positions. Select the lowest *tested* successful force. No extra squeeze
    margin or untested force is inferred from the data.
    """
    if not baseline or baseline.get('passed') is not True:
        return None
    bounds = [baseline.get('open_q'), baseline.get('max_q')]
    if (any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) for v in bounds)
            or not 0 <= bounds[0] < bounds[1] <= .85):
        return None
    groups = {}
    for t in trials:
        force = t.get('force_percent')
        if (isinstance(force, bool) or not isinstance(force, (int, float)) or not math.isfinite(force)
                or int(force) != force):
            continue
        groups.setdefault(force, []).append(t)
    for force, group in sorted(groups.items()):
        if not 20 <= force <= 100:
            continue
        if any(t.get('verified_success') is False or t.get('dropped') for t in group):
            continue
        good = [t for t in group if t.get('verified_success') is True and t.get('released') is True
                and t.get('sensor_valid') is True and t.get('grip_state') == 2
                and t.get('dropped') is False and t.get('stopped') is False and t.get('arm_moved') is False]
        if len(good) < 3:
            continue
        ids = [t.get('id') for t in good]
        if (any(isinstance(v,bool) or not isinstance(v,int) or v < 1 for v in ids)
                or len(set(ids)) != len(ids)):
            continue
        q = [t.get('measured_q') for t in good]
        commands = [t.get('commanded_q') for t in good]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               or not 0 <= v <= .85 for v in q+commands):
            continue
        command = max(commands)
        if (max(q)-min(q) > .02+1e-12 or bounds[1] < command
                or any(c < measured-.0015 for c,measured in zip(commands,q))):
            continue
        if not bounds[0] < min(q):
            continue
        return dict(commanded_q=command, measured_q=median(q), force_percent=int(force),
                    measured_min=min(q), measured_max=max(q), trials=len(good),
                    trial_ids=[t['id'] for t in good])
    return None
