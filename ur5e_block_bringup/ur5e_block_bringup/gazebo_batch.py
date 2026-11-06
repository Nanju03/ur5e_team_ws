"""Resumable adjacent-diagonal contact trials in isolated localhost Gazebo sessions.

Each attempt owns a fresh simulator and the existing continuous stacking launch.
There is no alternate executor, collision exemption, or physical-driver startup.
"""
import argparse
import csv
from datetime import datetime
import fcntl
import hashlib
import html
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
import zipfile
from zoneinfo import ZoneInfo

import yaml

from .path_archive import checked_artifact, seal_successful_archive, sha256
from .plate_geometry import plate_parameters
from .gazebo_session import GazeboSession, MAX_CASE_JOBS, session_from_environment

FAILED = {'failed', 'timeout', 'environment_error', 'archive_error'}
FINISHED = FAILED | {'succeeded'}
LABELS = {'pending': '미시험', 'running': '진행 중', 'succeeded': '성공·경로 저장',
          'failed': '적재/계획 실패', 'timeout': '시간 초과', 'environment_error': '환경 오류',
          'archive_error': '저장 오류', 'interrupted': '사용자/프로세스 중단'}


def now():
    return datetime.now(ZoneInfo('Asia/Seoul')).isoformat(timespec='seconds')


def adjacent_diagonals(columns=4, rows=6):
    """Ordered obstacle/target pairs, with no row wrapping or symmetry merging."""
    pairs = []
    for obstacle in range(1, columns*rows+1):
        row, col = divmod(obstacle-1, columns)
        for dr in (-1, 1):
            for dc in (-1, 1):
                if 0 <= row+dr < rows and 0 <= col+dc < columns:
                    pairs.append((obstacle, (row+dr)*columns+col+dc+1))
    pairs.sort()
    # Exercise the reported working layout first, then its reversed roles.
    first = [p for p in ((11, 14), (14, 11)) if p in pairs]
    return first+[p for p in pairs if p not in first]


def key(pair):
    return f'obstacle_{pair[0]}_target_{pair[1]}'


def available_cpus():
    try:
        return len(os.sched_getaffinity(0))
    except AttributeError:
        return os.cpu_count() or 1


def parallel_world(source, workers):
    """Change scheduling only: retain the 1ms step, solver and contact model."""
    tree = ET.fromstring(source)
    physics = tree.find('world/physics')
    if physics is None or physics.get('type') != 'ode':
        raise ValueError('병렬 시험에는 기존 ODE world가 필요합니다.')
    solver = physics.find('ode/solver')
    if solver is None:
        raise ValueError('ODE solver 설정이 없습니다.')
    # ODE's zero disables its island thread pool. A one-worker pool still
    # queues every island, so the one-CPU setting should use the serial path.
    for name, value in (('island_threads', str(workers if workers > 1 else 0)), ('thread_position_correction', 'true' if workers > 1 else 'false')):
        element = solver.find(name)
        if element is None:
            element = ET.SubElement(solver, name)
        element.text = value
    # Keep the startup time cap: an unthrottled empty world can advance 120
    # simulation seconds while the robot's meshes/plugins are still loading,
    # causing gazebo_ros_factory to report a spawn timeout.
    return ET.tostring(tree, encoding='unicode')+'\n'


def atomic_text(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+'.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def write_json(path, value):
    atomic_text(path, json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def pending_pairs(state, retry_failed=False):
    records = state['results']
    return [tuple(p) for p in state['cases'] if
            (records.get(key(p), {}).get('status') in FAILED if retry_failed else
             records.get(key(p), {}).get('status') not in FINISHED)]


def write_reports(folder, state):
    """Every checkpoint is enough to resume; pending is never called a failure."""
    folder = Path(folder)
    state['updated_at'] = now()
    write_json(folder/'state.json', state)
    records = [{'obstacle': o, 'target': t, **state['results'].get(key((o, t)), {})}
               for o, t in state['cases']]
    for row in records:
        row.setdefault('status', 'pending')
    failures = [row for row in records if row['status'] in FAILED]
    counts = {status: sum(r['status'] == status for r in records) for status in LABELS}
    write_json(folder/'summary.json', dict(total=len(records), counts=counts,
               status=state.get('status'), message=state.get('message', ''), updated_at=state['updated_at']))
    write_json(folder/'failed_cases.json', dict(count=len(failures), cases=failures))
    columns = ['obstacle', 'target', 'worker_id', 'ros_domain', 'status', 'stage', 'attempts', 'moved', 'last_source', 'last_layer', 'elapsed_s',
               'reason', 'log', 'archive_directory', 'successful_path']
    for name, rows in (('results.csv', records), ('failed_cases.csv', failures)):
        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
        atomic_text(folder/name, output.getvalue())
    text = [f'# 대각선 Gazebo 시험 — 실패 {len(failures)}건', '',
            f"갱신: {state['updated_at']} · 전체 {len(records)}건 · 성공 {counts['succeeded']}건 · "
            f"미시험 {counts['pending']}건 · 중단 {counts['interrupted']}건", '',
            '현재 시도에서 실패한 배치만 표시합니다. 미시험·중단은 실패 판정에 넣지 않습니다.', '',
            '| 장애물 | 목표 | 구분 | 단계 | 마지막 적재 | 원인 / 로그 |',
            '| --- | --- | --- | --- | --- | --- |']
    for row in failures:
        reason = str(row.get('reason', '')).replace('|', '\\|').replace('\n', ' ')
        log = row.get('log', '')
        text.append(f"| {row['obstacle']} | {row['target']} | {LABELS[row['status']]} | "
                    f"{row.get('stage', '')} | {row.get('moved', 0)}/22 | {reason} [로그]({log}) |")
    if not failures:
        text += ['', '현재 기록된 실패가 없습니다. 아직 끝나지 않은 경우 전체 성공을 뜻하지 않습니다.']
    atomic_text(folder/'failed_cases.md', '\n'.join(text)+'\n')
    esc = lambda value: html.escape(str(value), quote=True)
    body = ['<!doctype html><meta charset="utf-8"><meta http-equiv="refresh" content="15">',
            '<title>UR5e 대각선 자동 시험</title><style>body{font:15px sans-serif;margin:24px}'
            'table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:7px}'
            '.succeeded{background:#cff0d7}.failed,.timeout,.archive_error,.environment_error{background:#ffd8d8}'
            '.running{background:#fff0b5}td:last-child{max-width:650px;overflow-wrap:anywhere}</style>',
            f'<h1>Gazebo · 이웃 대각선 {len(records)}가지</h1>',
            '<p>'+esc(state.get('message', ''))+'</p>',
            '<p>'+esc(' / '.join(f'{LABELS[s]} {n}' for s, n in counts.items()))+'</p>',
            '<p>15초마다 갱신 · <a href="failed_cases.md">실패 목록</a> · '
            '<a href="failed_cases.csv">실패 CSV</a> · <a href="results.csv">전체 CSV</a></p>',
            '<table><tr><th>장애물</th><th>목표</th><th>결과</th><th>단계 / 적재</th><th>기록</th></tr>']
    for row in records:
        links = []
        for field, label in (('log', '로그'), ('successful_path', '성공 경로 ZIP')):
            if row.get(field):
                target = os.path.relpath(row[field], folder) if Path(row[field]).is_absolute() else row[field]
                links.append(f'<a href="{esc(target)}">{label}</a>')
        worker = f"작업자 {row['worker_id']} · ROS {row.get('ros_domain')} · " if row.get('worker_id') else ''
        body.append(f'<tr class="{esc(row["status"])}"><td>{row["obstacle"]}</td><td>{row["target"]}</td>'
                    f'<td>{esc(LABELS[row["status"]])}</td><td>{esc(worker)}{esc(row.get("stage", ""))} · {row.get("moved", 0)}/22</td>'
                    f'<td>{" · ".join(links)} {esc(row.get("reason", ""))}</td></tr>')
    atomic_text(folder/'report.html', '\n'.join(body+['</table>']))


class Campaign:
    def __init__(self, folder, signature, metadata, cases=None):
        cases = adjacent_diagonals() if cases is None else cases
        self.folder = Path(folder).expanduser().resolve()
        self.folder.mkdir(parents=True, exist_ok=True)
        self.lock = (self.folder/'.lock').open('a+')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            path = self.folder/'state.json'
            if path.exists():
                self.state = read_json(path)
                if self.state.get('signature') != signature or self.state.get('cases') != [list(p) for p in cases]:
                    raise ValueError('모델·설정·코드가 다른 시험 폴더입니다. --output-dir로 새 폴더를 지정하세요.')
                for row in self.state['results'].values():
                    if row['status'] == 'running':
                        row.update(status='interrupted', reason='이전 시험 프로세스가 완료 기록 없이 종료됨')
                        row.setdefault('history', []).append({k: v for k, v in row.items() if k != 'history'})
                    if row['status'] == 'succeeded':
                        bundle = Path(row['successful_path'])
                        if not bundle.is_file() or sha256(bundle) != row['zip_sha256']:
                            raise ValueError('보관한 성공 ZIP이 없거나 변경되었습니다: '+str(bundle))
            else:
                self.state = dict(schema=1, signature=signature, metadata=metadata,
                                  created_at=now(), cases=[list(p) for p in cases], results={})
        except BaseException:
            self.lock.close()
            raise

    def checkpoint(self):
        write_reports(self.folder, self.state)

    def close(self):
        self.lock.close()


class Interrupted(Exception):
    pass


class TrialError(Exception):
    def __init__(self, reason, status='failed'):
        super().__init__(reason)
        self.status = status


class ManagedProcess:
    """Capture full output, signal only this launch's process group, reap on exit."""
    def __init__(self, command, logfile, env):
        self.logfile, self.lines = Path(logfile), queue.Queue()
        self.read_error = None
        self.output = self.logfile.open('w', encoding='utf-8')
        try:
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding='utf-8', errors='replace', start_new_session=True, env=env)
        except BaseException:
            self.output.close()
            raise
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                self.output.write(line)
                self.output.flush()
                self.lines.put(line)
        except Exception as error:
            self.read_error = error
        finally:
            self.process.stdout.close()
            self.output.close()

    def poll(self):
        return self.process.poll()

    def drain(self):
        if self.read_error is not None:
            raise TrialError('전체 로그 저장 실패: '+str(self.read_error), 'archive_error')
        result = []
        while True:
            try:
                result.append(self.lines.get_nowait())
            except queue.Empty:
                return result

    def finished(self):
        if self.poll() is None:
            return False
        self.reader.join(timeout=.02)
        return not self.reader.is_alive()

    def stop(self):
        # ROS launch relays SIGINT to its children. The group fallback also
        # covers descendants left behind when the launcher itself has exited.
        for sig, seconds in ((signal.SIGINT, 10.), (signal.SIGTERM, 3.), (signal.SIGKILL, 2.)):
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                self.process.poll()
                try:
                    os.killpg(self.process.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(.05)
            else:
                continue
            break
        self.process.wait(timeout=2.)
        self.reader.join(timeout=2.)
        if self.reader.is_alive():
            raise RuntimeError('시험 출력 프로세스가 종료되지 않았습니다: '+str(self.logfile))


def validate_hardware(urdf, components, controllers):
    root = ET.fromstring(urdf)
    hardware = root.findall('ros2_control')
    if not hardware or any(h.findtext('hardware/plugin') != 'gazebo_ros2_control/GazeboSystem' for h in hardware):
        raise TrialError('Gazebo 전용 로봇 모델이 아닙니다.', 'environment_error')
    names = {h.get('name') for h in hardware}
    covered = set()
    for component in components:
        imported = not component.class_type and component.name in names
        if (component.class_type != 'gazebo_ros2_control/GazeboSystem' and not imported):
            raise TrialError('Gazebo 외 하드웨어가 발견되었습니다.', 'environment_error')
        if component.state.id == 3:
            covered.update(i.name for i in component.command_interfaces if i.is_available and i.is_claimed)
    for joint in hardware[0].findall('joint'):
        if joint.find('command_interface') is None:
            continue
        interface = '/effort' if joint.get('name') == 'left_outer_knuckle_joint' else '/position'
        if joint.get('name')+interface not in covered:
            raise TrialError('Gazebo 관절 제어 인터페이스가 준비되지 않았습니다.', 'environment_error')
    states = {c.name: c.state for c in controllers}
    if any(states.get(n) != 'active' for n in ('joint_state_broadcaster', 'ur5e_arm_controller', 'dh_ag95_gripper_controller')):
        raise TrialError('Gazebo 팔·그리퍼 제어기가 active가 아닙니다.', 'environment_error')


def initial_feedback_problem(home, feedback, stamp):
    for name, expected in home.items():
        if name not in feedback or stamp-feedback[name][1] > 3. or not math.isfinite(feedback[name][0]):
            return '관절 피드백 없음/지연: '+name
        actual = feedback[name][0]
        if name == 'left_outer_knuckle_joint':
            # The effort-controlled unloaded fingers can drift before their
            # first GripperCommand. The backend plans from this measured value
            # and opens at staging; waiting for an uncommanded 0.4 is a deadlock.
            if not 0. <= actual <= .93:
                return '그리퍼 측정값 범위 초과'
        # Controllers capture their initial hold command after gravity has
        # already loaded the spawned arm. Observed startup sag is 0.0255rad;
        # the planner uses measured joints, not this nominal-home gate. This
        # does not change the executor's 0.005rad trajectory settling limit.
        elif abs(actual-expected) > .05:
            return f'팔 초기 자세 정착 대기: {name} 오차 {actual-expected:.6f}rad'
    return None


class RosMonitor:
    def __init__(self, urdf, home, cancelled, session=None):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from rosgraph_msgs.msg import Clock
        from gazebo_msgs.msg import ModelStates
        from gazebo_msgs.srv import GetEntityState
        from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
        from rcl_interfaces.srv import GetParameters
        from moveit_msgs.srv import GetPlanningScene
        from std_srvs.srv import Trigger
        self.rclpy, self.Trigger = rclpy, Trigger
        self.session = session or GazeboSession()
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=self.session.domain, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(f'ur5e_gazebo_diagonal_batch_{self.session.domain}', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.urdf, self.home, self.cancelled = urdf, home, cancelled
        self.feedback, self.bodies = {}, {}
        self.clock, self.clock_at, self.bodies_at = 0., 0., 0.
        self.readiness_problem = ''
        self.node.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.node.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.node.create_subscription(ModelStates, '/gazebo/model_states', self.on_bodies, qos_profile_sensor_data)
        self.parameters = self.node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        self.hardware = self.node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.controllers = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.entity = self.node.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.scene = self.node.create_client(GetPlanningScene, '/get_planning_scene')
        self.stop_client = self.node.create_client(Trigger, '/ur5e_a3_stack/stop')

    def on_joints(self, message):
        if len(message.name) == len(message.position):
            stamp = time.monotonic()
            self.feedback.update({n: (q, stamp) for n, q in zip(message.name, message.position) if math.isfinite(q)})

    def on_clock(self, message):
        value = message.clock.sec+message.clock.nanosec*1e-9
        if value != self.clock:
            self.clock, self.clock_at = value, time.monotonic()

    def on_bodies(self, message):
        self.bodies = {n: (p.position.x, p.position.y, p.position.z) for n, p in zip(message.name, message.pose)}
        self.bodies_at = time.monotonic()

    def pump(self, seconds=.05):
        self.executor.spin_once(timeout_sec=seconds)
        for _ in range(12):
            self.executor.spin_once(timeout_sec=0.)

    def call(self, client, request, timeout=6.):
        if self.cancelled():
            raise Interrupted()
        deadline = time.monotonic()+timeout
        while not client.service_is_ready():
            if self.cancelled():
                raise Interrupted()
            if time.monotonic() > deadline:
                raise TrialError('ROS 서비스 없음: '+client.srv_name, 'environment_error')
            self.pump()
        future = client.call_async(request)
        while not future.done():
            if self.cancelled():
                future.cancel()
                raise Interrupted()
            if time.monotonic() > deadline:
                future.cancel()
                raise TrialError('ROS 서비스 응답 시간 초과: '+client.srv_name, 'environment_error')
            self.pump()
        if future.result() is None:
            raise TrialError('ROS 서비스 응답 오류: '+client.srv_name, 'environment_error')
        return future.result()

    def wait_idle(self, seconds=30.):
        """Never attach to or stop an existing user-owned simulator."""
        deadline = time.monotonic()+seconds
        clear_since = None
        while time.monotonic() < deadline:
            if self.cancelled():
                raise Interrupted()
            self.pump()
            others = set(self.node.get_node_names())-{self.node.get_name()}
            with socket.socket() as probe:
                busy = probe.connect_ex(('127.0.0.1', self.session.port)) == 0
            if not others and not busy:
                if clear_since is None:
                    clear_since = time.monotonic()
                if time.monotonic()-clear_since >= 2.5:
                    self.feedback.clear()
                    self.bodies.clear()
                    self.clock = self.clock_at = self.bodies_at = 0.
                    return
            else:
                clear_since = None
        raise TrialError(f'ROS {self.session.domain} / Gazebo {self.session.port}가 사용 중입니다. 기존 Gazebo 시작 터미널을 Ctrl+C로 종료하세요. '
                         +', '.join(sorted(others)), 'environment_error')

    def check_model(self):
        response = self.call(self.parameters, self.parameters.srv_type.Request(names=['robot_description']))
        if len(response.values) != 1 or response.values[0].type != 4 or response.values[0].string_value != self.urdf:
            raise TrialError('실행 중인 로봇과 시험 모델이 다릅니다.', 'environment_error')
        hardware = self.call(self.hardware, self.hardware.srv_type.Request()).component
        controllers = self.call(self.controllers, self.controllers.srv_type.Request()).controller
        validate_hardware(self.urdf, hardware, controllers)

    def ready(self):
        if not all(c.service_is_ready() for c in (self.parameters, self.hardware, self.controllers, self.entity, self.scene)):
            self.readiness_problem = 'Gazebo·MoveIt 서비스 준비 대기'
            return False
        if time.monotonic()-self.clock_at > 3.:
            self.readiness_problem = 'Gazebo 시계 진행 대기'
            return False
        self.readiness_problem = initial_feedback_problem(self.home, self.feedback, time.monotonic()) or ''
        if self.readiness_problem:
            return False
        self.check_model()
        response = self.call(self.entity, self.entity.srv_type.Request(name='ur5e_stack_robot', reference_frame='world'))
        return response.success

    def approve(self, service):
        self.check_model()
        if self.cancelled():
            raise Interrupted()
        client = self.node.create_client(self.Trigger, service)
        try:
            response = self.call(client, self.Trigger.Request())
            if not response.success:
                raise TrialError('자동 승인 거절: '+response.message, 'environment_error')
        finally:
            self.node.destroy_client(client)

    def request_stop(self):
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(self.Trigger.Request())
            self.pump(.1)

    def verify_tower(self, expected, keep_alive, logfile):
        """Observe all 23 free bodies twice, separated by two simulation seconds."""
        samples, start, deadline = [], self.clock, time.monotonic()+90.
        while True:
            if self.cancelled():
                raise Interrupted()
            if not keep_alive():
                raise TrialError('최종 탑 관측 중 Gazebo가 종료됨', 'environment_error')
            self.pump()
            if time.monotonic() > deadline:
                raise TrialError('최종 탑 안정 확인 시간 초과', 'timeout')
            if time.monotonic()-self.bodies_at > 3.:
                continue
            if not samples or self.clock-start >= 2.:
                errors = {name: math.dist(self.bodies.get(name, (math.inf,)*3), position)
                          for name, position in expected.items()}
                invalid = {name: value for name, value in errors.items() if not math.isfinite(value) or value > .010}
                if invalid:
                    raise TrialError('최종 23층 위치 오차 10mm 초과/블록 누락: '+str(invalid))
                samples.append(dict(sim_time=self.clock, positions={n: self.bodies[n] for n in expected}, errors_m=errors))
                if len(samples) == 1:
                    start = self.clock
                if len(samples) == 2:
                    write_json(logfile, dict(status='passed', tolerance_m=.010, simulation_separation_s=self.clock-start, samples=samples))
                    return

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown(context=self.context)


class RunEvents:
    def __init__(self):
        self.archive, self.plan_id, self.ready, self.approved = None, None, None, False
        self.preflight, self.done, self.success, self.scene = False, False, False, False
        self.moved, self.reason = 0, ''
        self.bound = False
        self.last_source = self.last_layer = None

    def feed(self, line):
        planning = re.search(r'STACK_PLANNING source=(\d+) target=(\d+) layer=(\d+)/(\d+)', line)
        if planning:
            self.last_source, self.last_layer = int(planning[1]), int(planning[3])
        if 'STACK_ERROR ' in line:
            self.reason = line.split('STACK_ERROR ', 1)[1].strip()
        if 'STACK_EXECUTION_ARCHIVE ' in line:
            data = json.loads(line.split('STACK_EXECUTION_ARCHIVE ', 1)[1])
            if self.archive is not None:
                raise TrialError('한 실행에서 기록 폴더가 중복됨', 'environment_error')
            self.archive, self.plan_id = Path(data['directory']).resolve(), data['plan_id']
        if 'STACK_PREFLIGHT_PASSED ' in line:
            self.preflight = bool(re.search(r'\bmoves=22 layers=23\b', line))
        if 'STACK_STEP_READY ' in line:
            data = json.loads(line.split('STACK_STEP_READY ', 1)[1])
            if self.ready is not None and self.ready != data:
                raise TrialError('연속 실행 외 추가 승인 요청이 발생함', 'environment_error')
            self.ready = data
        match = re.search(r'STACK_STATUS phase=(\w+) moved=(\d+) total=(\d+) target=(\d+) layers=(\d+)', line)
        if match:
            self.scene |= match[1] == 'SCENE'
            self.moved = int(match[2])
            self.done |= match[1] == 'DONE' and int(match[2]) == 22 and int(match[3]) == 22 and int(match[5]) == 23
        if 'STACK_SUCCESS ' in line:
            self.success = line.split('STACK_SUCCESS ', 1)[1].strip()

    def approval_service(self, pair, archive_root, expected_model_hash, expected_domain=None):
        data = self.ready
        if (not self.preflight or self.archive is None or self.reason or not isinstance(data, dict)
                or data.get('plan_id') != self.plan_id or data.get('index') != 1 or data.get('total') != 1
                or data.get('request_id') != self.plan_id+'_1'
                or data.get('approve_service') != '/ur5e_a3_stack/approve_next_'+self.plan_id+'_1'
                or not math.isfinite(data.get('duration', math.nan)) or data['duration'] <= 0):
            raise TrialError('현재 전체 Gazebo 경로에 결합되지 않은 승인 요청', 'environment_error')
        parent = Path(archive_root).resolve()/key(pair)
        if self.archive.parent != parent or self.archive.is_symlink():
            raise TrialError('예상 배치 밖의 경로 기록 폴더', 'environment_error')
        context = read_json(self.archive/'context.json')
        result = read_json(self.archive/'execution_result.json')
        recipe = read_json(self.archive/'plan.recipe.json')
        settings = yaml.safe_load((self.archive/'selection.yaml').read_text())['/**']['ros__parameters']
        if (context.get('backend') != 'gazebo' or context.get('obstacle_cells') != [pair[0]]
                or (expected_domain is not None and context.get('ros_domain') != str(expected_domain))
                or context.get('target_cell') != pair[1] or context.get('model_sha256') != expected_model_hash
                or result.get('status') not in ('planned', 'running') or result.get('plan_id') != self.plan_id
                or result.get('backend') != 'gazebo' or result.get('target_cell') != pair[1]
                or result.get('completed_moves') != 0 or result.get('completed_commands') != 0
                or recipe.get('layers') != 23 or recipe.get('moves') != 22 or recipe.get('target_cell') != pair[1]
                or settings.get('execution_mode') != 'continuous' or settings.get('stack_mode') != 'run'
                or settings.get('empty_cells') != [] or settings.get('simulation_backend') != 'gazebo'):
            raise TrialError('계획·배치·기록 정보가 현재 23층 시험과 다릅니다.', 'environment_error')
        self.bound = True
        return data['approve_service']


def final_tower_positions(archive, pair, settings):
    recipe = read_json(checked_artifact(Path(archive), 'plan.recipe.json'))
    data = {**settings, 'target_cell': pair[1]}
    data.update(plate_parameters(data))
    x, y = data['stack_target_xy']
    height = data['block_size_xyz'][2]
    result = {f'ur5e_stack_block_{pair[1]}': (x, y, data['stack_surface_z']+height/2)}
    placements = [s for s in recipe['steps'] if s.get('phase') == 'place' and not s.get('temporary')]
    sources = set(range(1, 25))-{pair[0], pair[1]}
    if (len(placements) != 22 or {s['block_id'] for s in placements} != sources
            or [s['completed'] for s in placements] != list(range(1, 23))
            or any(s['destination_cell'] != pair[1] for s in placements)):
        raise TrialError('최종 적재 22개 순서/배치가 기록과 다릅니다.', 'archive_error')
    for step in placements:
        result[f'ur5e_stack_block_{step["block_id"]}'] = (x, y, data['stack_surface_z']+height*(step['completed']+.5))
    return result


def load_configuration(workspace, height):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    share = Path(get_package_share_directory('ur5e_a3_grid'))
    source = share/'launch/stack_model.py'
    spec = importlib.util.spec_from_file_location('batch_stack_model', source)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    model = helper.make_model(height, simulation_backend='gazebo').to_dict()
    settings = {}
    for name in ('a3_grid.yaml', 'stack_blocks.yaml'):
        settings.update(yaml.safe_load((share/'config'/name).read_text())['/**']['ros__parameters'])
    settings.update(plate_parameters(settings))
    settings.update(yaml.safe_load((share/'config/gazebo_physics.yaml').read_text())['task'])
    if (settings['columns'], settings['rows']) != (4, 6):
        raise ValueError('이 시험은 4열×6행 플레이트 전용입니다.')
    urdf, srdf = model['robot_description'], model['robot_description_semantic']
    semantic = ET.fromstring(srdf)
    home = {j.get('name'): float(j.get('value')) for j in semantic.findall("group_state[@name='home'][@group='ur5e_arm']/joint")}
    if len(home) != 6:
        raise ValueError('팔 초기 자세 6개 관절이 없습니다.')
    home['left_outer_knuckle_joint'] = .4
    prefix = Path(get_package_prefix('ur5e_a3_grid'))
    files = [prefix/'lib/ur5e_a3_grid/stack_blocks', prefix/'lib/libur5e_a3_mimic.so', prefix/'lib/libur5e_parallel_ompl.so',
             prefix/'lib/ur5e_a3_grid/gazebo_layout', Path(__file__), Path(__file__).with_name('path_archive.py'),
             Path(__file__).with_name('plate_geometry.py'), Path(__file__).with_name('gazebo_session.py'),
             Path(__file__).with_name('gazebo_parallel.py'), workspace/'scripts/gazebo_env.bash']
    files += [p for sub in ('config', 'launch', 'worlds') for p in sorted((share/sub).glob('*')) if p.is_file()]
    metadata = dict(backend='gazebo', ros_domain=44, master_uri='http://127.0.0.1:11355',
                    scope='adjacent_diagonal_directed_60_v1', riser_height=height,
                    model_sha256=hashlib.sha256(urdf.encode()).hexdigest(),
                    semantic_sha256=hashlib.sha256(srdf.encode()).hexdigest(),
                    files_sha256={str(p): sha256(p) for p in files})
    signature = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode()).hexdigest()
    return settings, urdf, srdf, home, signature, metadata


class Runner:
    def __init__(self, args, campaign, settings, monitor, process_factory=ManagedProcess):
        self.args, self.campaign, self.settings, self.monitor = args, campaign, settings, monitor
        self.process_factory = process_factory
        self.stopping, self.server, self.task = False, None, None
        self.env = {**os.environ, 'PYTHONUNBUFFERED': '1', 'RCUTILS_COLORIZED_OUTPUT': '0'}

    def check(self):
        if self.stopping or (getattr(self, 'external_cancelled', lambda: False)()):
            self.stopping = True
            raise Interrupted()
        if self.server is not None and self.server.poll() is not None:
            raise TrialError('Gazebo/MoveIt 실행 프로세스가 종료됨. simulator.log를 확인하세요.', 'environment_error')
        if self.server is not None:
            self.server.drain()  # Full simulator output is already on disk.

    def update(self, row, **values):
        row.update(values)
        self.campaign.state['message'] = f"장애물 {row['obstacle']} / 목표 {row['target']} · {row['stage']} · {row.get('moved', 0)}/22개"
        self.campaign.checkpoint()
        if 'stage' in values:
            print(f"BATCH_STAGE 장애물={row['obstacle']} 목표={row['target']} {row['stage']}", flush=True)

    def command(self, mode, selection):
        return ['ros2', 'launch', 'ur5e_a3_grid', 'stack_blocks.launch.py', f'stack_mode:={mode}',
                f'selection_file:={selection}', 'simulation_backend:=gazebo', 'execution_mode:=continuous',
                f'riser_height:={self.args.riser_height}', f'path_archive_root:={self.args.archive_root}']

    def wait_task(self, row, events, pair, execution=False):
        stage = 'planning' if execution else 'scene'
        self.update(row, stage=stage, log=str(self.task.logfile.relative_to(self.campaign.folder)))
        deadline = time.monotonic()+(self.args.planning_timeout if execution else self.args.scene_timeout)
        next_heartbeat = time.monotonic()+60.
        while True:
            self.monitor.pump()
            self.check()
            for line in self.task.drain():
                events.feed(line)
                if events.archive is not None:
                    row['archive_directory'] = str(events.archive)
                row.update(last_source=events.last_source, last_layer=events.last_layer)
                if events.moved != row.get('moved', 0):
                    print(f"BATCH_PROGRESS 장애물={pair[0]} 목표={pair[1]} 적재={events.moved}/22", flush=True)
                    self.update(row, moved=events.moved)
            if events.reason:
                raise TrialError(events.reason)
            if execution and events.ready is not None and not events.approved:
                service = events.approval_service(pair, self.args.archive_root,
                    self.campaign.state['metadata']['model_sha256'], expected_domain=self.env.get('ROS_DOMAIN_ID'))
                write_json(events.archive/'batch_verification.json', dict(status='pending',
                    obstacle_cells=[pair[0]], target_cell=pair[1], campaign_signature=self.campaign.state['signature']))
                if getattr(self.args, 'world_file', None):
                    shutil.copy2(self.args.world_file, events.archive/'parallel.world')
                    write_json(events.archive/'parallelism.json', dict(
                        planner_workers=self.args.workers, physics_threads=self.args.physics_threads,
                        worker_id=getattr(self.args, 'worker_id', None),
                        cpu_affinity=getattr(self.args, 'cpu_affinity', None),
                        cpu_pinning=getattr(self.args, 'pin_cpus', False),
                        ros_domain=self.env.get('ROS_DOMAIN_ID'), master_uri=self.env.get('GAZEBO_MASTER_URI'),
                        world_sha256=sha256(self.args.world_file)))
                self.check()
                self.monitor.approve(service)
                events.approved = True
                stage = 'executing'
                self.update(row, stage=stage, plan_id=events.plan_id)
                deadline = time.monotonic()+self.args.execution_timeout
                print(f'BATCH_APPROVED 장애물={pair[0]} 목표={pair[1]} 전체 연속 실행', flush=True)
            if self.task.finished():
                # A last line can arrive between drain() and the reader closing.
                for line in self.task.drain():
                    events.feed(line)
                if events.reason or self.task.poll() != 0:
                    raise TrialError(events.reason or f'작업 프로세스 종료 코드 {self.task.poll()}')
                if execution:
                    expected = f'moved=22 layers=23 target={pair[1]}'
                    if not events.approved or not events.done or events.success != expected:
                        raise TrialError('전체 23층 완료/자동 승인 기록이 없습니다.')
                elif not events.scene:
                    raise TrialError('Gazebo 배치/계획 장면 준비 확인이 없습니다.', 'environment_error')
                return
            if time.monotonic() > deadline:
                raise TrialError(f'{stage} 실제 시간 제한 초과', 'timeout')
            if time.monotonic() >= next_heartbeat:
                self.update(row)
                print(f"BATCH_WAIT 장애물={pair[0]} 목표={pair[1]} {stage} 적재={events.moved}/22 "
                      f"계획층={events.last_layer}", flush=True)
                next_heartbeat = time.monotonic()+60.

    def attempt(self, pair):
        previous = self.campaign.state['results'].get(key(pair), {})
        number = previous.get('attempts', 0)+1
        directory = self.campaign.folder/'cases'/key(pair)/f'attempt_{number:03d}'
        while directory.exists():
            number += 1
            directory = directory.with_name(f'attempt_{number:03d}')
        directory.mkdir(parents=True, exist_ok=False)
        row = dict(obstacle=pair[0], target=pair[1], status='running', stage='starting', moved=0,
                   attempts=number, started_at=now(), history=previous.get('history', []),
                   log=str((directory/'simulator.log').relative_to(self.campaign.folder)))
        if hasattr(self.args, 'worker_id'):
            row.update(worker_id=self.args.worker_id, ros_domain=int(self.env['ROS_DOMAIN_ID']),
                       master_uri=self.env['GAZEBO_MASTER_URI'], cpu_affinity=self.args.cpu_affinity)
        self.campaign.state['results'][key(pair)] = row
        started = time.monotonic()
        self.update(row)
        print(f'BATCH_START 장애물={pair[0]} 목표={pair[1]} 시도={number}', flush=True)
        events = RunEvents()
        startup_gate = getattr(self, 'startup_gate', None)
        startup_acquired = False
        try:
            self.check()
            self.monitor.wait_idle()
            if startup_gate is not None:
                self.update(row, stage='waiting_startup')
                while not startup_gate.acquire(timeout=.1):
                    self.check()
                startup_acquired = True
                self.check()
                self.update(row, stage='starting')
            server_command = ['ros2', 'launch', 'ur5e_a3_grid', 'stack_gazebo.launch.py',
                'use_gui:=false', f'gui:={str(self.args.show_gazebo).lower()}',
                f'use_rviz:={str(self.args.show_rviz).lower()}', f'riser_height:={self.args.riser_height}']
            if getattr(self.args, 'world_file', None):
                server_command.append(f'world_file:={self.args.world_file}')
            self.server = self.process_factory(server_command,
                directory/'simulator.log', self.env)
            deadline = time.monotonic()+self.args.startup_timeout
            next_startup_report = time.monotonic()+15.
            while not self.monitor.ready():
                self.monitor.pump()
                self.check()
                if time.monotonic() >= next_startup_report:
                    problem = getattr(self.monitor, 'readiness_problem', '')
                    self.update(row, startup_problem=problem)
                    print(f'BATCH_STARTUP_WAIT 장애물={pair[0]} 목표={pair[1]} {problem}', flush=True)
                    next_startup_report = time.monotonic()+15.
                if time.monotonic() > deadline:
                    raise TrialError('Gazebo 초기 자세·제어기 준비 시간 초과: '+
                                     getattr(self.monitor, 'readiness_problem', ''), 'environment_error')
            selection = directory/'selection.yaml'
            settings = {**self.settings, 'target_cell': pair[1], 'obstacle_cells': [pair[0]], 'empty_cells': [],
                        'geometry_profile': 'cube_plate_v1', 'empty_hand_test': False,
                        'empty_xy_scan': False, 'plate_alignment_test': False}
            atomic_text(selection, yaml.safe_dump({'/**': {'ros__parameters': settings}}))
            self.task = self.process_factory(self.command('scene', selection), directory/'scene.log', self.env)
            self.wait_task(row, RunEvents(), pair)
            self.task.stop()
            if startup_acquired:
                startup_gate.release()
                startup_acquired = False
            self.task = self.process_factory(self.command('run', selection), directory/'run.log', self.env)
            self.wait_task(row, events, pair, execution=True)
            self.update(row, stage='verifying_tower', archive_directory=str(events.archive))
            expected = final_tower_positions(events.archive, pair, settings)
            self.monitor.verify_tower(expected, lambda: self.server.poll() is None, directory/'tower_verification.json')
            # Add final whole-tower evidence before the archive's manifest is sealed.
            shutil.copy2(directory/'tower_verification.json', events.archive/'tower_verification.json')
            write_json(events.archive/'batch_verification.json', dict(status='passed', obstacle_cells=[pair[0]],
                target_cell=pair[1], campaign_signature=self.campaign.state['signature'], layers=23))
            self.update(row, stage='saving')
            try:
                bundle = seal_successful_archive(events.archive, directory/'run.log')
                with zipfile.ZipFile(bundle) as zipped:
                    if zipped.testzip() is not None:
                        raise ValueError('ZIP 무결성 검사 실패')
                row.update(successful_path=str(bundle), zip_sha256=sha256(bundle))
            except (OSError, ValueError, KeyError, zipfile.BadZipFile) as error:
                raise TrialError(str(error), 'archive_error') from error
            row.update(status='succeeded', stage='completed', moved=22, reason='')
        except Interrupted:
            row.update(status='interrupted', reason='사용자가 중단했거나 종료 신호를 받음')
        except TrialError as error:
            row.update(status=error.status, reason=str(error))
        except (OSError, ValueError, KeyError, TypeError) as error:
            row.update(status='environment_error', reason=f'{type(error).__name__}: {error}')
        finally:
            if startup_acquired:
                startup_gate.release()
            cleanup_errors = []
            if row['status'] != 'succeeded' and self.task is not None:
                try:
                    self.monitor.request_stop()
                except Exception as error:
                    cleanup_errors.append(str(error))
            for child in (self.task, self.server):
                if child is not None:
                    try:
                        child.stop()
                    except Exception as error:
                        cleanup_errors.append(str(error))
            self.task = self.server = None
            row.update(ended_at=now(), elapsed_s=round(time.monotonic()-started, 2))
            if cleanup_errors:
                row['cleanup_errors'] = cleanup_errors
                self.stopping = True  # Never start another case with uncertain ownership.
            if events.bound and row['status'] not in ('succeeded', 'archive_error'):
                write_json(events.archive/'batch_verification.json', dict(status=row['status'],
                    obstacle_cells=[pair[0]], target_cell=pair[1], reason=row.get('reason', '')))
            if not (self.campaign.folder/row['log']).is_file():
                row['log'] = str((directory/'result.json').relative_to(self.campaign.folder))
            snapshot = {k: v for k, v in row.items() if k != 'history'}
            write_json(directory/'result.json', snapshot)
            row['history'].append(snapshot)
            self.update(row)
        print(f"BATCH_RESULT 장애물={pair[0]} 목표={pair[1]} {LABELS[row['status']]} {row.get('reason', '')}", flush=True)
        return row

    def run(self):
        pairs = pending_pairs(self.campaign.state, self.args.retry_failed)
        if self.args.max_cases:
            pairs = pairs[:self.args.max_cases]
        self.campaign.state.update(status='running', message=f"이번 실행 {len(pairs)}건 · 전체 {len(self.campaign.state['cases'])}건")
        self.campaign.checkpoint()
        environments = 0
        for pair in pairs:
            if self.stopping:
                break
            if shutil.disk_usage(self.campaign.folder).free < self.args.min_free_gb*1024**3:
                self.campaign.state['message'] = '저장 공간 부족으로 중지했습니다. 미시험 배치는 그대로 남깁니다.'
                self.stopping = True
                break
            row = self.attempt(pair)
            environments = environments+1 if row['status'] == 'environment_error' else 0
            if environments >= 3:
                self.campaign.state['message'] = '환경 오류 3건 연속으로 자동 시험을 중지했습니다. 로그를 확인하세요.'
                self.stopping = True
        remaining = pending_pairs(self.campaign.state)
        self.campaign.state['status'] = 'interrupted' if self.stopping else 'partial' if remaining else 'completed'
        if not self.stopping:
            self.campaign.state['message'] = '선택한 시험 완료. 실패 목록을 확인하세요.'
        elif not ('부족' in self.campaign.state.get('message', '') or '3건 연속' in self.campaign.state.get('message', '')):
            self.campaign.state['message'] = '자동 시험이 중단되었습니다. 같은 명령으로 미완료 배치를 이어서 시험할 수 있습니다.'
        self.campaign.checkpoint()
        failures = [(p, self.campaign.state['results'].get(key(p), {})) for p in self.campaign.state['cases']]
        failures = [(p, row) for p, row in failures if row.get('status') in FAILED]
        print(f"BATCH_FINISHED 상태={self.campaign.state['status']} 실패={len(failures)} 미완료={len(remaining)}", flush=True)
        for pair, row in failures:
            print(f"  장애물 {pair[0]} / 목표 {pair[1]}: {LABELS[row['status']]} · {row.get('reason', '')}", flush=True)
        print('실패 목록: '+str(self.campaign.folder/'failed_cases.md'), flush=True)
        print('전체 결과: '+str(self.campaign.folder/'report.html'), flush=True)
        return 130 if self.stopping else 1 if failures else 0


def parse_args(argv=None):
    workspace = Path(os.environ.get('UR5E_WS', Path.home()/'ur5e_team_ws')).resolve()
    parser = argparse.ArgumentParser(description='Gazebo 이웃 대각선 60가지 자동 시험·성공 궤적 저장·실패 목록. 기존 Gazebo는 먼저 종료하세요.')
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument('--jobs', type=int, default=None, help='케이스별 병렬 작업자 수 1..8 (권장 4). 각 작업자가 독립 Gazebo 사용')
    parser.add_argument('--pin-cpus', action='store_true', help='병렬 작업자 CPU 고정 (기본은 운영체제 자동 배분)')
    parser.add_argument('--obstacle', type=int, help='단일 시험의 장애물 칸 (--target과 함께 지정)')
    parser.add_argument('--target', type=int, help='단일 시험의 쌓을 칸 (--obstacle과 함께 지정)')
    parser.add_argument('--workers', type=int, default=0, help='OMPL 병렬 후보 수. 0=자동(CPU 수-2), 최대 64')
    parser.add_argument('--physics-threads', type=int, default=1, help='ODE 스레드 수. 기본 1=직렬, 0=workers와 동일')
    parser.add_argument('--archive-root', type=Path, default=workspace/'saved_paths/gazebo')
    parser.add_argument('--retry-failed', action='store_true', help='기존 실패한 배치만 재시험 (성공·미시험은 제외)')
    parser.add_argument('--max-cases', type=int, default=0, help='이번에 실행할 최대 배치 수. 0=전체')
    parser.add_argument('--show-gazebo', action='store_true', help='물리 시험 중 Gazebo 화면 표시')
    parser.add_argument('--show-rviz', action='store_true', help='RViz도 표시')
    parser.add_argument('--riser-height', type=float, default=.016)
    parser.add_argument('--startup-timeout', type=float, default=180.)
    parser.add_argument('--scene-timeout', type=float, default=180.)
    parser.add_argument('--planning-timeout', type=float, default=900., help='계획 단계 실제 시간 제한(초)')
    parser.add_argument('--execution-timeout', type=float, default=7200., help='승인 후 실행 실제 시간 제한(초), 기본 2시간/배치')
    parser.add_argument('--min-free-gb', type=float, default=2.)
    parser.add_argument('--list', action='store_true', help='60가지 목록만 출력. 파일/ROS 노드/시뮬레이터를 만들지 않음')
    args = parser.parse_args(argv)
    args.case_parallel = args.jobs is not None
    args.jobs = 1 if args.jobs is None else args.jobs
    if not 1 <= args.jobs <= MAX_CASE_JOBS:
        parser.error('jobs는 1..8 범위여야 합니다.')
    if (args.obstacle is None) != (args.target is None):
        parser.error('--obstacle과 --target을 함께 지정하세요.')
    if args.obstacle is not None and (args.obstacle, args.target) not in adjacent_diagonals():
        parser.error('이웃 대각선 배치만 지정할 수 있습니다 (예: --obstacle 1 --target 6).')
    if not 0 <= args.workers <= 64 or not 0 <= args.physics_threads <= 64:
        parser.error('workers와 physics-threads는 0..64 범위여야 합니다.')
    args.workers = args.workers or (min(2, max(1, (available_cpus()-2)//args.jobs))
                                    if args.case_parallel else min(64, max(1, available_cpus()-2)))
    args.physics_threads = args.physics_threads or args.workers
    args.cases = [(args.obstacle, args.target)] if args.obstacle is not None else adjacent_diagonals()
    if args.output_dir is None:
        name = f'gazebo_{args.obstacle}_{args.target}_parallel' if args.obstacle is not None else 'gazebo_diagonal_tests'
        if args.case_parallel:
            name = f'gazebo_{args.obstacle}_{args.target}_case_parallel' if args.obstacle is not None else 'gazebo_diagonal_case_parallel'
        args.output_dir = workspace/'log'/name
    if (args.max_cases < 0 or not math.isfinite(args.riser_height) or not 0 <= args.riser_height <= .3
            or any(not math.isfinite(v) or v <= 0 for v in (args.startup_timeout, args.scene_timeout,
                args.planning_timeout, args.execution_timeout, args.min_free_gb))):
        parser.error('시간·공간은 유한한 양수, max-cases는 0 이상, riser-height는 0..0.3이어야 합니다.')
    args.workspace = workspace
    args.output_dir = args.output_dir.expanduser().resolve()
    args.archive_root = args.archive_root.expanduser().resolve()
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.list:
        for o, t in args.cases:
            print(f'장애물 {o:2d} / 목표 {t:2d}')
        print(f'총 {len(args.cases)}가지 (장애물·목표 역할을 바꾸면 별도 시험)')
        return 0
    try:
        session = session_from_environment()
    except ValueError:
        print('BATCH_ERROR test_block_diagonals.sh로 localhost Gazebo 44 환경에서 실행하세요.', file=sys.stderr)
        return 2
    if args.case_parallel:
        if session.slot is not None:
            print('BATCH_ERROR 병렬 조정기는 기본 Gazebo 실행 환경에서 시작하세요.', file=sys.stderr)
            return 2
        from .gazebo_parallel import main as parallel_main
        return parallel_main(args)
    campaign = monitor = None
    domain_lock = None
    old_signals = {}
    try:
        # A second output folder must not bypass the shared domain lock.
        fd = os.open(f'/tmp/ur5e_gazebo_diagonal_batch_{session.domain}.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        domain_lock = os.fdopen(fd, 'a+')
        try:
            fcntl.flock(domain_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError(f'다른 대각선 자동 시험이 ROS {session.domain}를 사용 중입니다.') from None
        settings, urdf, srdf, home, signature, metadata = load_configuration(args.workspace, args.riser_height)
        from ament_index_python.packages import get_package_share_directory
        source = (Path(get_package_share_directory('ur5e_a3_grid'))/'worlds/stack.world').read_text()
        world = parallel_world(source, args.physics_threads)
        metadata.update(cases=[list(pair) for pair in args.cases], planner_workers=args.workers,
                        physics_threads=args.physics_threads, world_sha256=hashlib.sha256(world.encode()).hexdigest())
        signature = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode()).hexdigest()
        campaign = Campaign(args.output_dir, signature, metadata, cases=args.cases)
        args.world_file = campaign.folder/'parallel.world'
        atomic_text(args.world_file, world)
        settings['planner_workers'] = args.workers
        print(f'BATCH_CPU available={available_cpus()} planner_workers={args.workers} physics_threads={args.physics_threads} cases={len(args.cases)}', flush=True)
        if args.retry_failed and not any(r['status'] in FAILED for r in campaign.state['results'].values()):
            print('기록된 실패 배치가 없습니다.', flush=True)
            return 0
        runner = Runner(args, campaign, settings, None)
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signals[sig] = signal.signal(sig, lambda *_: setattr(runner, 'stopping', True))
        monitor = RosMonitor(urdf, home, lambda: runner.stopping, session=session)
        runner.monitor = monitor
        monitor.wait_idle(seconds=5.)  # Fail before recording any case if a user's session exists.
        atomic_text(campaign.folder/'robot.urdf', urdf)
        atomic_text(campaign.folder/'robot.srdf', srdf)
        atomic_text(campaign.folder/'base_parameters.yaml', yaml.safe_dump(settings))
        return runner.run()
    except Interrupted:
        if campaign is not None:
            campaign.state.update(status='interrupted', message='사용자가 자동 시험을 중단했습니다.')
            campaign.checkpoint()
        return 130
    except (OSError, ValueError, TrialError) as error:
        print('BATCH_ERROR '+str(error), file=sys.stderr, flush=True)
        return 2
    finally:
        if monitor is not None:
            monitor.close()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
        if campaign is not None:
            campaign.close()
        if domain_lock is not None:
            domain_lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
