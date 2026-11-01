#!/usr/bin/env python3
"""Sequential, resumable plan-only checks and saved-path review for an isolated mock scene."""
import argparse
import csv
from datetime import datetime, timezone
import fcntl
import gzip
import hashlib
import html
import io
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from moveit_msgs.msg import DisplayTrajectory, PlanningScene, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

FINISHED = {'passed', 'no_plan_found', 'timeout'}
LABELS = {'passed': '경로 찾음', 'no_plan_found': '경로 미발견', 'timeout': '시간 초과',
          'environment_error': '환경 오류', 'interrupted': '중단', 'pending': '미검사'}
COLORS = {'passed': '#b8e5c4', 'no_plan_found': '#ffe3a0', 'timeout': '#ffc392',
          'environment_error': '#f6b6b6', 'interrupted': '#c6d5eb', 'pending': '#eceff3'}


def boolean(value):
    if value.lower() not in ('true', 'false'):
        raise argparse.ArgumentTypeError('Use true or false')
    return value.lower() == 'true'


def cells(value, size=24):
    if value.strip().lower() == 'all':
        return list(range(1, size+1))
    found = set()
    for item in value.split(','):
        item = item.strip()
        if not item:
            raise ValueError('Empty cell list item')
        if '-' in item:
            bounds = item.split('-')
            if len(bounds) != 2:
                raise ValueError('Invalid range '+item)
            low, high = map(int, bounds)
            if not 1 <= low <= high <= size:
                raise ValueError('Range must be ordered and within 1..'+str(size))
            found.update(range(low, high+1))
        else:
            found.add(int(item))
    if not found or min(found) < 1 or max(found) > size:
        raise ValueError(f'Cell numbers must be in 1..{size}')
    return sorted(found)


def case_key(obstacle, target):
    return f'o{obstacle:03d}_t{target:03d}'


def selected_pairs(obstacles, targets):
    return [(o, t) for o in cells(obstacles) for t in cells(targets) if o != t]


def pending_pairs(pairs, results, retry_failed=False):
    return [(o, t) for o, t in pairs if results.get(case_key(o, t), {}).get('status') not in FINISHED or
            (retry_failed and results[case_key(o, t)]['status'] != 'passed')]


def transient_communication_error(record):
    return record.get('status') == 'environment_error' and record.get('reason', '').startswith((
        'ROS service response timed out:', 'ROS service unavailable',
        'Planning scene request timed out', 'Robot model publisher is unavailable',
        'MoveIt planning scene is unavailable', 'Apply planning scene is unavailable'))


def atomic_text(path, text):
    path = Path(path)
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(text, encoding='utf-8')
    temporary.replace(path)


def read_saved_path(path, model_id, home):
    """Load the backend's complete plan without relying on its DDS lifetime."""
    display = deserialize_message(Path(path).read_bytes(), DisplayTrajectory)
    if display.model_id != model_id or not display.trajectory:
        raise RuntimeError('Saved trajectory has a different model or no trajectory')
    start = dict(zip(display.trajectory_start.joint_state.name,
                     display.trajectory_start.joint_state.position))
    if not home.keys() <= start.keys() or any(not math.isfinite(start[name]) or
            abs(start[name]-value) > .003 for name, value in home.items()):
        raise RuntimeError('Saved trajectory does not start at the expected home pose')
    for part in display.trajectory:
        path = part.joint_trajectory
        if not home.keys() <= set(path.joint_names) or not path.points or any(
                len(point.positions) != len(path.joint_names) for point in path.points):
            raise RuntimeError('Saved trajectory is missing joints or points')
    return display


def save_message(path, message):
    temporary = path.with_name(path.name+'.tmp')
    with gzip.open(temporary, 'wb') as stream:
        stream.write(serialize_message(message))
    temporary.replace(path)


def write_reports(folder, state, pairs):
    results = state['results']
    atomic_text(folder/'state.json', json.dumps(state, ensure_ascii=False, indent=2)+'\n')
    counts = {name: 0 for name in LABELS}
    for o, t in pairs:
        status = results.get(case_key(o, t), {}).get('status', 'pending')
        counts[status if status in counts else 'pending'] += 1
    summary = {'selected_cases': len(pairs), 'all_single_obstacle_cases': 552, 'counts': counts,
               'updated_utc': datetime.now(timezone.utc).isoformat(), 'signature': state['signature'],
               'note': 'No-plan/timeouts do not prove mathematical impossibility. No trajectories are executed.'}
    atomic_text(folder/'summary.json', json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    output = io.StringIO()
    columns = ['obstacle', 'target', 'status', 'attempts', 'elapsed_s', 'last_source', 'last_layer', 'last_stage', 'reason', 'log', 'path']
    writer = csv.DictWriter(output, fieldnames=columns)
    writer.writeheader()
    for o, t in pairs:
        record = results.get(case_key(o, t), {'obstacle': o, 'target': t, 'status': 'pending', 'attempts': 0})
        writer.writerow({key: record.get(key, '') for key in columns})
    atomic_text(folder/'results.csv', output.getvalue())
    selected = set(pairs)
    parts = ['<!doctype html><meta charset="utf-8"><title>UR5e 경로 검사</title>',
             '<style>body{font:14px sans-serif;margin:24px}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:8px;text-align:center}a{color:#182e42}p{max-width:1000px}</style>',
             '<h1>UR5e · 장애물 1개 / 쌓을 칸 경로 검사</h1>',
             '<p>행 = 장애물 칸, 열 = 쌓을 칸. 모든 나머지 칸에 30mm 블록이 있는 23층 작업의 사전 경로 검사입니다. 경로 미발견·시간 초과는 불가능 판정이 아닙니다. 실제 이동 명령은 보내지 않습니다.</p>',
             '<p>'+html.escape(' / '.join(f'{LABELS[k]} {v}' for k, v in counts.items()))+'</p>',
             '<p><a href="results.csv">CSV</a> · <a href="summary.json">요약 JSON</a> · <a href="cases.json">검사 조합</a><br>셀을 누르면 마지막 로그가 열립니다. 성공 경로는 stack_sweep.launch.py review:=장애물,목표 use_rviz:=true 로 다시 볼 수 있습니다.</p>',
             '<table><tr><th>장애물 ↓ / 목표 →</th>'+''.join(f'<th>{t}</th>' for t in range(1, 25))+'</tr>']
    for o in range(1, 25):
        parts.append(f'<tr><th>{o}</th>')
        for t in range(1, 25):
            if o == t:
                parts.append('<td style="background:#39434c;color:white">—</td>')
                continue
            if (o, t) not in selected:
                parts.append('<td>·</td>')
                continue
            record = results.get(case_key(o, t), {})
            status = record.get('status', 'pending')
            label = LABELS.get(status, status)
            title = html.escape(record.get('reason') or record.get('last_stage', ''), quote=True)
            text = html.escape(label)
            if record.get('log'):
                text = '<a href="'+html.escape(record['log'], quote=True)+'">'+text+'</a>'
            parts.append(f'<td style="background:{COLORS.get(status, COLORS["pending"])}" title="{title}">{text}</td>')
        parts.append('</tr>')
    parts.append('</table><p>모델·알고리즘 지문: '+html.escape(state['signature'])+'</p>')
    atomic_text(folder/'report.html', '\n'.join(parts))


class Sweep:
    def __init__(self, args, node):
        self.args, self.node = args, node
        self.folder = Path(args.output_dir).expanduser().resolve()
        self.folder.mkdir(parents=True, exist_ok=True)
        self.lock = (self.folder/'.lock').open('a+')
        if not args.review:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.backend = json.loads(node.get_parameter('backend_json').value)
        self.model_id = ET.fromstring(self.backend['robot_description']).get('name')
        self.home = json.loads(node.get_parameter('home_json').value)
        self.binary = Path(get_package_prefix('ur5e_a3_grid'))/'lib/ur5e_a3_grid/stack_blocks'
        model_bytes = json.dumps({'backend': self.backend, 'home': self.home}, sort_keys=True).encode()
        model_signature = hashlib.sha256(model_bytes).hexdigest()
        signature = hashlib.sha256(model_bytes+self.binary.read_bytes()).hexdigest()
        self.pairs = selected_pairs(args.obstacles, args.targets)
        self.state = {'version': 1, 'signature': signature, 'model_signature': model_signature, 'results': {}}
        state_path = self.folder/'state.json'
        if state_path.exists():
            previous = json.loads(state_path.read_text())
            same = previous.get('signature') == signature
            if args.review and previous.get('model_signature') == model_signature:
                same = True
            if not same:
                raise RuntimeError('Result folder belongs to different model/settings/backend. Use a new output_dir.')
            if not args.resume and not args.review:
                raise RuntimeError('Result folder exists. Use resume:=true or a new output_dir.')
            self.state = previous
            self.state.setdefault('model_signature', model_signature)
        elif args.review:
            raise RuntimeError('No saved sweep in this output_dir')
        self.stopping = False
        self.child = None
        self.feedback = None
        self.feedback_at = 0.
        self.display = None
        self.lines = queue.Queue()
        self.events = {}
        self.get_scene = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.hardware = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.stop_client = node.create_client(Trigger, '/ur5e_a3_stack/stop')
        self.feedback_sub = node.create_subscription(JointState, '/joint_states', self.on_feedback, qos_profile_sensor_data)
        self.display_pub = node.create_publisher(DisplayTrajectory, '/display_planned_path', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path_sub = None
        self.last_preview = None
        self.preview_subscribers = 0
        self.preview_repeat_at = None

    def show_preview(self, message):
        self.last_preview = message
        self.preview_subscribers = self.display_pub.get_subscription_count()
        self.display_pub.publish(message)
        self.preview_repeat_at = time.monotonic()+3.

    def clear_preview(self):
        # Replace an earlier case's animation with a stationary home preview.
        # This is only a display message, never a controller command.
        display = DisplayTrajectory()
        display.trajectory_start.joint_state.name = list(self.home)
        display.trajectory_start.joint_state.position = list(self.home.values())
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(self.home)
        point = JointTrajectoryPoint()
        point.positions = list(self.home.values())
        trajectory.joint_trajectory.points = [point]
        display.trajectory = [trajectory]
        self.show_preview(display)

    def on_feedback(self, message):
        values = dict(zip(message.name, message.position))
        if self.home.keys() <= values.keys() and all(math.isfinite(values[name]) for name in self.home):
            self.feedback, self.feedback_at = values, time.monotonic()

    def pump(self, timeout=.02):
        rclpy.spin_once(self.node, timeout_sec=timeout)
        for _ in range(16):
            rclpy.spin_once(self.node, timeout_sec=0)
        count = self.display_pub.get_subscription_count()
        # The Humble demo RViz requests volatile durability. Send the current
        # saved preview again when a new viewer joins, without resetting an
        # existing viewer's animation on every poll.
        if self.last_preview is not None and count > self.preview_subscribers:
            self.display_pub.publish(self.last_preview)
            self.preview_repeat_at = time.monotonic()+3.
        if self.last_preview is not None and self.preview_repeat_at is not None and time.monotonic() >= self.preview_repeat_at:
            # RViz creates the subscription just before loading its robot model.
            # One delayed repeat covers that startup race, without an endless
            # animation reset for an already connected viewer.
            self.display_pub.publish(self.last_preview)
            self.preview_repeat_at = None
        self.preview_subscribers = count
        while True:
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break
            match = re.search(r'STACK_PLANNING source=(\d+) target=(\d+) layer=(\d+)/(\d+)', line)
            if match:
                self.events.update(last_source=int(match[1]), last_layer=int(match[3]), last_stage=line.strip())
            if 'STACK_EXECUTING ' in line or 'STACK_ROUTE ' in line:
                self.events['last_stage'] = line.strip()
            if 'STACK_ERROR ' in line:
                self.events['reason'] = line.split('STACK_ERROR ', 1)[1].strip()
            if 'STACK_PREFLIGHT_PASSED ' in line:
                self.events['preflight'] = True
            if 'STACK_STATUS phase=PLAN_READY' in line:
                self.events['ready'] = True

    def call(self, client, request, timeout=8.):
        for attempt in range(1, 4):
            if self.stopping:
                raise RuntimeError('Sweep interrupted during ROS service request')
            future = client.call_async(request)
            deadline = time.monotonic()+timeout
            while not future.done() and time.monotonic() < deadline and not self.stopping:
                self.pump()
            if not future.done():
                future.cancel()
                client.remove_pending_request(future)
                if self.stopping:
                    raise RuntimeError('Sweep interrupted during ROS service request')
                if attempt == 3:
                    raise RuntimeError('ROS service response timed out: '+client.srv_name)
                print(f'SWEEP_SERVICE_RETRY service={client.srv_name} attempt={attempt}/3', flush=True)
                until = time.monotonic()+.25
                while time.monotonic() < until and not self.stopping:
                    self.pump()
                continue
            result = future.result()
            if result is None:
                raise RuntimeError('ROS service failed: '+client.srv_name)
            return result

    def wait_environment(self):
        deadline = time.monotonic()+40
        while time.monotonic() < deadline and not self.stopping:
            self.pump(.05)
            if not all(c.service_is_ready() for c in (self.get_scene, self.apply_scene, self.controllers, self.hardware)) or self.feedback is None:
                continue
            states = {item.name: item.state for item in self.call(self.controllers, ListControllers.Request()).controller}
            if all(states.get(name) == 'active' for name in ('ur5e_arm_controller', 'dh_ag95_gripper_controller', 'joint_state_broadcaster')):
                components = self.call(self.hardware, ListHardwareComponents.Request()).component
                claimed = {command.name for component in components
                           if component.class_type == 'mock_components/GenericSystem' and component.state.id == 3
                           for command in component.command_interfaces if command.is_claimed}
                if not all(name+'/position' in claimed for name in self.home):
                    raise RuntimeError('Sweep requires active mock_components/GenericSystem for every joint')
                self.check_stationary()
                return
        raise RuntimeError('Isolated MoveIt/mock environment did not become ready')

    def check_stationary(self):
        if self.feedback is None or time.monotonic()-self.feedback_at > 3:
            raise RuntimeError('Joint feedback disappeared')
        if any(abs(self.feedback[name]-value) > .003 for name, value in self.home.items()):
            raise RuntimeError('Mock start pose changed; aborting sweep before another scene update')

    def stop_child(self):
        child = self.child
        if child is None or child.poll() is not None:
            return
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
            deadline = time.monotonic()+.5
            while child.poll() is None and time.monotonic() < deadline:
                self.pump()
        for sig, delay in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
            if child.poll() is not None:
                break
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic()+delay
            while child.poll() is None and time.monotonic() < deadline:
                self.pump()
        child.wait(timeout=2)

    def attempt(self, obstacle, target, number):
        directory = self.folder/'cases'/case_key(obstacle, target)
        directory.mkdir(parents=True, exist_ok=True)
        stem = f'attempt_{number:02d}'
        parameters = dict(self.backend)
        parameters.update(stack_mode='plan', target_cell=target, obstacle_cells=[obstacle], empty_cells=[])
        raw_path = directory/(stem+'.path.cdr')
        raw_path.unlink(missing_ok=True)
        raw_path.with_name(raw_path.name+'.tmp').unlink(missing_ok=True)
        parameters['display_trajectory_file'] = str(raw_path)
        parameter_file = directory/(stem+'.yaml')
        atomic_text(parameter_file, yaml.safe_dump({'/**': {'ros__parameters': {k: v for k, v in parameters.items() if v != []}}}))
        self.events, self.display = {}, None
        self.clear_preview()
        topic = f'/ur5e_stack_sweep/{case_key(obstacle, target)}_{number}/display'
        def capture(message):
            self.display = message
            self.show_preview(message)
        self.path_sub = self.node.create_subscription(DisplayTrajectory, topic, capture, QoSProfile(depth=1))
        command = [str(self.binary), '--ros-args', '--params-file', str(parameter_file),
                   '-r', '__node:=ur5e_stack_sweep_case', '-r', '/display_planned_path:='+topic]
        log_path = directory/(stem+'.log')
        record = {'obstacle': obstacle, 'target': target, 'attempt': number, 'status': 'environment_error',
                  'log': str(log_path.relative_to(self.folder)), 'input': str(parameter_file.relative_to(self.folder)),
                  'path': '', 'scene': '', 'reason': '', 'started_utc': datetime.now(timezone.utc).isoformat()}
        started = time.monotonic()
        next_heartbeat = started+10.
        timed_out = False
        try:
            self.child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                          text=True, start_new_session=True, env={**os.environ, 'RCUTILS_COLORIZED_OUTPUT': '0'})
            output_stream = self.child.stdout
            def read():
                with output_stream, log_path.open('w') as stream:
                    for line in output_stream:
                        stream.write(line)
                        stream.flush()
                        self.lines.put(line)
            reader = threading.Thread(target=read, daemon=True)
            reader.start()
            while self.child.poll() is None and not self.stopping:
                self.pump()
                self.check_stationary()
                if time.monotonic() >= next_heartbeat:
                    stage = self.events.get('last_stage', 'Waiting for planner')
                    if ']: ' in stage:
                        stage = stage.split(']: ', 1)[1]
                    print(f'SWEEP_WORKING obstacle={obstacle} target={target} '
                          f'seconds={time.monotonic()-started:.0f} '
                          f'source={self.events.get("last_source", "?")} '
                          f'layer={self.events.get("last_layer", "?")}/23 stage={stage}', flush=True)
                    next_heartbeat = time.monotonic()+10.
                if time.monotonic()-started > self.args.case_timeout:
                    timed_out = True
                    break
            if self.stopping or timed_out:
                self.stop_child()
            reader.join(timeout=3)
            self.pump()
            if self.stopping:
                record.update(status='interrupted', reason='Interrupted; resume will repeat this case')
            elif timed_out:
                record.update(status='timeout', reason=f'No completed plan within {self.args.case_timeout:g} seconds')
            elif self.child.returncode == 0 and self.events.get('preflight') and self.events.get('ready') and not self.events.get('reason'):
                # The child saves before PLAN_READY. Its short-lived publisher
                # may disappear before the large preview reaches DDS readers.
                # Use that exact file as the result; DDS is only for the viewer.
                received = self.display is not None
                self.display = read_saved_path(raw_path, self.model_id, self.home)
                self.show_preview(self.display)
                points = sum(len(part.joint_trajectory.points) for part in self.display.trajectory)
                print(f'SWEEP_PATH source=file points={points} topic_received={received}', flush=True)
                path = directory/(stem+'.path.cdr.gz')
                save_message(path, self.display)
                request = GetPlanningScene.Request()
                request.components.components = 1023
                scene = self.call(self.get_scene, request).scene
                scene_path = directory/(stem+'.scene.cdr.gz')
                save_message(scene_path, scene)
                record.update(status='passed', path=str(path.relative_to(self.folder)), scene=str(scene_path.relative_to(self.folder)),
                              path_source='file', preview_received=received)
                raw_path.unlink()
            else:
                reason = self.events.get('reason', f'Backend exited {self.child.returncode} without a complete plan')
                environment_terms = ('No joint feedback', 'models differ', 'unavailable', 'timed out', 'Another stacking task',
                                     'Cannot save planned trajectory', 'Robot model publisher returned', 'ROS service',
                                     'MoveIt rejected a scene update')
                kind = 'environment_error' if not self.events.get('reason') or any(term in reason for term in environment_terms) else 'no_plan_found'
                record.update(status=kind, reason=reason)
        except Exception as error:
            record.update(status='interrupted' if self.stopping else 'environment_error', reason=str(error))
        finally:
            self.stop_child()
            self.child = None
            if self.path_sub is not None:
                self.node.destroy_subscription(self.path_sub)
                self.path_sub = None
            record['elapsed_s'] = round(time.monotonic()-started, 3)
            for key in ('last_source', 'last_layer', 'last_stage'):
                record[key] = self.events.get(key, '')
        return record

    def review(self):
        pair = tuple(map(int, self.args.review.split(',')))
        if len(pair) != 2 or pair[0] == pair[1] or not all(1 <= number <= 24 for number in pair):
            raise ValueError('review must be obstacle,target, e.g. 10,11')
        record = self.state['results'].get(case_key(*pair), {})
        if record.get('status') != 'passed':
            raise RuntimeError('No passing saved robot path for this pair')
        with gzip.open(self.folder/record['scene'], 'rb') as stream:
            scene = deserialize_message(stream.read(), PlanningScene)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        if not self.call(self.apply_scene, request).success:
            raise RuntimeError('Saved starting scene was rejected')
        with gzip.open(self.folder/record['path'], 'rb') as stream:
            display = deserialize_message(stream.read(), DisplayTrajectory)
        self.show_preview(display)
        print(f'SWEEP_REVIEW obstacle={pair[0]} target={pair[1]} · robot path preview; no trajectory execution. Ctrl+C to close.', flush=True)
        while not self.stopping:
            self.pump(.1)

    def run(self):
        if self.args.review:
            self.wait_environment()
            self.review()
            return 0
        atomic_text(self.folder/'cases.json', json.dumps([{'obstacle': o, 'target': t} for o, t in self.pairs], indent=2)+'\n')
        write_reports(self.folder, self.state, self.pairs)
        queue_pairs = pending_pairs(self.pairs, self.state['results'], self.args.retry_failed)
        if self.args.max_cases:
            queue_pairs = queue_pairs[:self.args.max_cases]
        print(f'SWEEP_SELECTED total={len(self.pairs)} queued={len(queue_pairs)} output={self.folder}', flush=True)
        if self.args.dry_run:
            print('SWEEP_DRY_RUN · no MoveIt fixture, scene changes or motion commands', flush=True)
            return 0
        self.wait_environment()
        started = time.monotonic()
        for index, (obstacle, target) in enumerate(queue_pairs, 1):
            if self.stopping:
                break
            self.check_stationary()
            key = case_key(obstacle, target)
            previous = self.state['results'].get(key, {})
            attempts = list(previous.get('history', []))
            planning_attempts, communication_retries = 0, 0
            while planning_attempts < self.args.attempts:
                if self.stopping:
                    break
                number = len(attempts)+1
                print(f'SWEEP_CASE {index}/{len(queue_pairs)} obstacle={obstacle} target={target} attempt={number}', flush=True)
                record = self.attempt(obstacle, target, number)
                attempts.append(record)
                current = {**record, 'attempts': len(attempts), 'history': attempts,
                           'elapsed_s': round(sum(item['elapsed_s'] for item in attempts), 3)}
                self.state['results'][key] = current
                write_reports(self.folder, self.state, self.pairs)
                print(f'SWEEP_RESULT obstacle={obstacle} target={target} status={record["status"]} seconds={record["elapsed_s"]} reason={record["reason"]}', flush=True)
                if record['status'] == 'environment_error':
                    if transient_communication_error(record) and communication_retries < self.args.communication_retries:
                        communication_retries += 1
                        print(f'SWEEP_RETRY obstacle={obstacle} target={target} '
                              f'communication_retry={communication_retries}/{self.args.communication_retries}', flush=True)
                        until = time.monotonic()+1.
                        while time.monotonic() < until and not self.stopping:
                            self.pump()
                        if self.stopping:
                            break
                        self.wait_environment()
                        continue
                    raise RuntimeError('Environment error; sweep stopped to avoid invalid remaining results')
                planning_attempts += 1
                if record['status'] in ('passed', 'interrupted'):
                    break
            remaining = len(queue_pairs)-index
            elapsed = time.monotonic()-started
            print(f'SWEEP_PROGRESS done={index} remaining={remaining} eta_minutes={elapsed/index*remaining/60:.1f} report={self.folder / "report.html"}', flush=True)
            until = time.monotonic()+self.args.pause_seconds
            while not self.stopping and time.monotonic() < until:
                self.pump(.1)
        print(f'SWEEP_FINISHED stopped={self.stopping} report={self.folder / "report.html"}', flush=True)
        while self.args.keep_alive and not self.stopping:
            self.pump(.1)
        return 130 if self.stopping else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='/workspaces/cobot_ws/log/stack_sweep')
    parser.add_argument('--obstacles', default='all')
    parser.add_argument('--targets', default='all')
    parser.add_argument('--max-cases', type=int, default=0)
    parser.add_argument('--attempts', type=int, default=1)
    parser.add_argument('--communication-retries', type=int, default=2)
    parser.add_argument('--case-timeout', type=float, default=180)
    parser.add_argument('--pause-seconds', type=float, default=0)
    parser.add_argument('--review', default='')
    for name, default in (('resume', True), ('retry-failed', False), ('keep-alive', False), ('dry-run', False)):
        parser.add_argument('--'+name, type=boolean, default=default)
    args, ros_args = parser.parse_known_args()
    if (args.max_cases < 0 or not 1 <= args.attempts <= 10 or not 0 <= args.communication_retries <= 5 or not math.isfinite(args.case_timeout) or
        not math.isfinite(args.pause_seconds) or args.case_timeout <= 0 or args.pause_seconds < 0):
        parser.error('Invalid limit, attempts, timeout or pause')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ur5e_stack_sweep', automatically_declare_parameters_from_overrides=True)
    sweep = None
    def stop(*_):
        if sweep is not None:
            sweep.stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    code = 2
    try:
        sweep = Sweep(args, node)
        code = sweep.run()
    except Exception as error:
        print('SWEEP_ERROR '+str(error), file=sys.stderr, flush=True)
    finally:
        if sweep is not None:
            sweep.stop_child()
            sweep.lock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
