"""Plan all eight XY holds and exercise approved stops on mock domain 105."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import yaml

from ur5e_block_bringup.air_task import ARM, HAND, AirTask
from ur5e_block_bringup.air_scene import RobotKinematics
from ur5e_block_bringup.air_trajectory import sample_trajectory, split_trajectory
from ur5e_block_bringup.model import load_moveit_settings
from ur5e_block_bringup.plate_geometry import plate_parameters

assert os.environ.get('ROS_DOMAIN_ID') == '105', 'Probe is restricted to mock domain 105'
ROOT = Path('/home/ksn/ur5e_ws/validation')
BASE = ROOT.parent
PATH = ROOT/'xy_alignment_plan.cdr'
PLAN_LOG = ROOT/'xy_alignment_plan.log'
RUN_LOG = ROOT/'xy_alignment_run.log'
RESULT = ROOT/'xy_alignment_runtime.result.json'
FAULT = Path('/tmp/ur5e_physical_held_105.json')
jobs, streams, q, samples, previews = [], [], {}, [], []
node, active = None, None
log = PLAN_LOG


def process_row(pid):
    try:
        item = Path('/proc')/str(pid)
        stat = (item/'stat').read_text().split(') ', 1)[1].split()
        args = (item/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        env = (item/'environ').read_bytes().split(b'\0')
        domain = next((value.split(b'=', 1)[1].decode() for value in env if value.startswith(b'ROS_DOMAIN_ID=')), 'unset')
        return dict(pid=int(pid), ppid=int(stat[1]), start_ticks=int(stat[19]), domain=domain, args=args[:600])
    except (OSError, ValueError, IndexError):
        return None


def driver_rows():
    rows = []
    for item in Path('/proc').iterdir():
        if not item.name.isdecimal():
            continue
        row = process_row(item.name)
        if row and row['domain'] == '105' and row['args']:
            name = Path(row['args'].split()[0]).name
            if name in {'ros2_control_node', 'move_group', 'robot_state_publisher', 'stack_blocks', 'air_task'}:
                rows.append(row)
    return rows


def start(command, target):
    assert os.environ['ROS_DOMAIN_ID'] == '105'
    stream = target.open('w')
    streams.append(stream)
    child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                             start_new_session=True, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
    jobs.append((child, process_row(child.pid)))
    return child


def content():
    return log.read_text() if log.exists() else ''


def records(marker):
    return [json.loads(line.split(marker+' ', 1)[1]) for line in content().splitlines() if marker+' ' in line]


def feedback(message):
    q.update(zip(message.name, message.position))
    if all(name in q for name in ARM+[HAND]):
        samples.append((time.monotonic(), dict(q)))


def until(predicate, seconds=30., allow_exit=False):
    deadline = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(content()[-4500:])
        if active is not None and active.poll() is not None and not allow_exit:
            raise RuntimeError(content()[-5000:])
        rclpy.spin_once(node, timeout_sec=.02)


def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)


def call(kind, endpoint, request, seconds=10.):
    client = node.create_client(kind, endpoint)
    try:
        if not client.wait_for_service(timeout_sec=seconds):
            raise TimeoutError(endpoint+' discovery')
        future = client.call_async(request)
        until(future.done, seconds, allow_exit=True)
        return future.result()
    finally:
        node.destroy_client(client)


def stationary(reference, tolerance=1e-9):
    return all(abs(q[name]-reference[name]) <= tolerance for name in ARM+[HAND])


def quaternion_matrix(raw):
    x, y, z, w = [float(value) for value in raw]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    x, y, z, w = [value/norm for value in (x, y, z, w)]
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def verify_offline(metadata, display, kinematics):
    assert metadata['alignment_policy'] == 'plate_corner_xy_v1'
    assert abs(metadata['gripper_position']-.65) < .003
    segments = split_trajectory(display.trajectory[0], metadata['metrics']['segment_end_indices'])
    stops, holds = metadata['alignment_stops'], []
    assert len(stops) == 8
    for index, stop in enumerate(stops):
        assert stop['corner'] == 'ABCD'[index//2] and stop['axis'] == ('X' if index % 2 == 0 else 'Y')
        endpoint = segments[stop['after_segment']-1].joint_trajectory.points[-1]
        assert max(abs(v) for v in endpoint.velocities+endpoint.accelerations) < 1e-8
        joints = dict(zip(ARM, endpoint.positions))
        tcp = kinematics.tcp(joints)
        assert math.dist(tcp[:3, 3], [stop[axis] for axis in 'xyz']) < .001
        rotation_error = max(abs(v) for v in (tcp[:3, :3]-quaternion_matrix(stop['orientation_xyzw'])).flat)
        assert rotation_error < .002
        minimum = kinematics.gripper_min_z(joints, .65)
        assert minimum >= .820-1e-5
        holds.append(dict(corner=stop['corner'], axis=stop['axis'], pocket_cell=stop['pocket_cell'],
                          tcp=tcp[:3, 3].tolist(), minimum_gripper_z=minimum,
                          orientation=tcp[:3, :3].tolist(), segment=stop['after_segment']))
    rz = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    for a, b in zip(holds[::2], holds[1::2]):
        assert math.dist(a['tcp'], b['tcp']) < .001
        assert max(abs(v) for v in (np.array(b['orientation'])@np.array(a['orientation']).T-rz).flat) < .002
    rotation_segments = []
    for index, label in enumerate(metadata['segment_labels']):
        if 'turn' not in label and '방향 맞추기' not in label and '회전' not in label:
            continue
        dense = sample_trajectory(segments[index], max_dt=.05, max_joint_step=.02)
        minimum = min(kinematics.gripper_min_z(dict(zip(ARM, point)), .65) for _, point in dense)
        assert minimum >= .870-1e-5, (label, minimum)
        rotation_segments.append(dict(index=index+1, label=label, samples=len(dense), minimum_gripper_z=minimum))
    assert rotation_segments
    return holds, rotation_segments


def main():
    global node, active, log
    assert not driver_rows(), 'Domain 105 is already occupied'
    assert not FAULT.exists(), 'Do not overwrite a pre-existing domain-105 fault'
    before_fault = None
    selection = yaml.safe_load((Path.home()/'.ros/ur5e_a3_stack/layout_domain_0_real_cube_plate_v1.yaml').read_text())
    parameters = selection['/**']['ros__parameters']
    parameters.update(plate_alignment_test=True, empty_xy_scan=True, physical_stack_task=False,
                      plate_alignment_gripper_position=.65)
    parameters.update(plate_parameters(parameters))
    selection_path = ROOT/'xy_alignment_selection.yaml'
    selection_path.write_text(yaml.safe_dump(selection))
    start(['ros2', 'launch', str(ROOT/'xy_alignment_mock_system.launch.py')], ROOT/'xy_alignment_fixture.log')
    rclpy.init()
    node = rclpy.create_node('xy_alignment_mock_runtime_probe')
    node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
    node.create_subscription(DisplayTrajectory, '/display_planned_path', previews.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    components = call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                      ListHardwareComponents.Request(), 20.).component
    assert components and all(c.class_type == 'mock_components/GenericSystem' and c.state.id == 3 for c in components)
    until(lambda: all(name in q for name in ARM+[HAND]), 15.)
    until(lambda: any(c.name == 'scaled_joint_trajectory_controller' and c.state == 'active' for c in
        call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller), 20.)
    assert abs(q[HAND]-.65) < .001
    initial = dict(q)
    common = ['ros2', 'launch', 'ur5e_block_bringup', 'block_task.launch.py',
              'selection_file:='+str(selection_path), 'trajectory_file:='+str(PATH),
              'allow_mock_hardware:=true', 'execution_mode:=continuous']
    active = start(common+['stack_mode:=dry_plan'], PLAN_LOG)
    until(lambda: active.poll() is not None, 180., allow_exit=True)
    assert active.returncode == 0, content()[-6000:]
    assert stationary(initial), 'Planning moved the mock robot'
    metadata = json.loads(PATH.with_name(PATH.name+'.meta.json').read_text())
    display = deserialize_message(PATH.read_bytes(), DisplayTrajectory)
    model = load_moveit_settings(str(BASE/'robot_calibration/ur5e_20261031.yaml'), str(BASE/'physical_setup.yaml'))
    kinematics = RobotKinematics(model['robot_description'])
    holds, rotations = verify_offline(metadata, display, kinematics)
    print(json.dumps(dict(stage='plan_verified', stops=8, rotation_segments=len(rotations),
                          duration=metadata['metrics']['duration'])), flush=True)
    log = RUN_LOG
    log.write_text('')
    active = start(common+['stack_mode:=dry_run', 'workspace_clear:=true'], RUN_LOG)
    until(lambda: records('STACK_STEP_READY') and previews, 45.)
    first = records('STACK_STEP_READY')[0]
    assert first['total'] == 8
    spin(.5)
    assert stationary(initial) and not records('STACK_STEP_EXECUTING')
    full = metadata['metrics']['duration'] <= 135.
    run_groups = 8 if full else 2
    approved, measured_holds = [], []
    for index in range(1, run_groups+1):
        until(lambda: any(row['index'] == index for row in records('STACK_STEP_READY')), 40.)
        ready = next(row for row in records('STACK_STEP_READY') if row['index'] == index)
        assert ready['total'] == 8 and ready['plan_id'] == metadata['plan_id']
        assert ready['request_id'] not in approved
        until(lambda: any(AirTask.preview_digest(p) == ready['preview_sha256'] for p in previews), 8.)
        held = dict(q)
        executions = len(records('STACK_STEP_EXECUTING'))
        spin(.5)
        assert stationary(held) and len(records('STACK_STEP_EXECUTING')) == executions
        assert call(Trigger, ready['approve_service'], Trigger.Request()).success
        approved.append(ready['request_id'])
        until(lambda: any(row['index'] == index for row in records('STACK_STEP_COMPLETED')),
              min(180., max(40., ready['duration']+25.)))
        spin(.1)
        actual = kinematics.tcp({name:q[name] for name in ARM})
        stop = metadata['alignment_stops'][index-1]
        assert math.dist(actual[:3, 3], [stop[a] for a in 'xyz']) < .003
        assert abs(q[HAND]-.65) < .003
        measured_holds.append(dict(index=index, corner=stop['corner'], axis=stop['axis'], tcp=actual[:3, 3].tolist()))
        print(json.dumps(dict(stage='hold_completed', index=index, corner=stop['corner'], axis=stop['axis'])), flush=True)
    if full:
        until(lambda: 'STACK_STATUS phase=DONE' in content(), 10.)
        until(lambda: active.poll() is not None, 10., allow_exit=True)
        assert active.returncode == 0 and len(records('STACK_STEP_READY')) == 8
        stopped_at_pair = False
    else:
        until(lambda: any(row['index'] == 3 for row in records('STACK_STEP_READY')), 15.)
        held = dict(q)
        spin(.5)
        assert stationary(held)
        assert call(Trigger, '/ur5e_a3_stack/stop', Trigger.Request()).success
        until(lambda: 'STACK_STOPPED_CONFIRMED' in content(), 15., allow_exit=True)
        until(lambda: active.poll() is not None, 10., allow_exit=True)
        spin(.4)
        assert stationary(held) and len(records('STACK_STEP_EXECUTING')) == 2
        assert 'STACK_STATUS phase=DONE' not in content()
        stopped_at_pair = True
    assert (FAULT.read_bytes() if FAULT.exists() else None) == before_fault
    assert max(abs(value[HAND]-.65) for _, value in samples) < .003
    result = dict(result='passed', domain=105, hardware='mock_components/GenericSystem',
                  actual_domain_touched=False, actual_robot_goals_sent=False,
                  plan_policy=metadata['alignment_policy'], all_eight_stops_verified=holds,
                  rotation_segments_verified_high=rotations, full_plan_duration=metadata['metrics']['duration'],
                  execution_groups=run_groups, measured_holds=measured_holds, approvals=len(approved),
                  no_motion_before_OK=True, mandatory_dwell_before_each_OK_seconds=.5,
                  stopped_after_first_XY_pair=stopped_at_pair, whole_eight_executed=full,
                  gripper_frozen=.65, held_latch_unchanged=True, feedback_samples=len(samples))
    RESULT.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(result='passed', groups=run_groups, report=str(RESULT))), flush=True)


def stop_job(child, birth):
    current = process_row(child.pid)
    if child.poll() is not None or current is None or current['start_ticks'] != birth['start_ticks']:
        return
    assert current['domain'] == '105'
    for sig, seconds in ((signal.SIGINT, 8.), (signal.SIGTERM, 4.), (signal.SIGKILL, 2.)):
        try:
            os.killpg(child.pid, sig)
            child.wait(timeout=seconds)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


if __name__ == '__main__':
    error = None
    try:
        main()
    except BaseException as exc:
        error = exc
        RESULT.write_text(json.dumps(dict(result='failed', error=str(exc), traceback=traceback.format_exc(),
                                         domain=105, actual_domain_touched=False), indent=2)+'\n')
        traceback.print_exc()
    finally:
        owned = driver_rows()
        for child, birth in reversed(jobs):
            stop_job(child, birth)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for stream in streams:
            stream.close()
        remaining = driver_rows()
        report = dict(domain=105, owned_launches=[birth for _, birth in jobs], children_before_stop=owned,
                      all_exited=not remaining, remaining=remaining, actual_domain_touched=False)
        (ROOT/'xy_alignment_runtime_cleanup.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(cleanup_all_exited=not remaining, remaining=remaining)), flush=True)
    if error is not None:
        raise SystemExit(1)
