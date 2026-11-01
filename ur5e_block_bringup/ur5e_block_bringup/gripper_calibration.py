"""Nonblocking, gripper-only calibration backend for the Tk client.

There is no arm action client. Only validated, fresh AG95 telemetry can advance
the experiment. Closing this tool never releases a held block automatically.
"""
from collections import deque
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import GripperCommand
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml

from .air_task import ARM, HAND
from .model import load_moveit_settings
from .gripper_calibration_data import GripperGapModel, recommend, validate_settings

PLUGIN = 'dh_gripper_driver/DHGripperHardwareInterface'
MOCK = 'mock_components/GenericSystem'
LABELS = {0: '이동 중', 1: '목표 도달 / 물체 없음', 2: '물체 잡힘', 3: '물체 놓침'}
PROFILE = Path.home()/'ur5e_ws/gripper_calibration.yaml'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def atomic_json(path, value):
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    tmp.replace(path)


def load_calibration_profile(path=PROFILE, *, model_sha256=None, block_width_mm=25):
    """Read only a real-hardware profile backed by the recorded trial evidence."""
    path = Path(path).expanduser()
    if not path.exists():
        return None
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict) or value.get('schema') != 1 or value.get('hardware_class') != PLUGIN:
        raise ValueError('Invalid real AG95 calibration profile')
    if model_sha256 is not None and value.get('model_sha256') != model_sha256:
        raise ValueError('Gripper calibration belongs to another model or physical setup; recalibrate')
    if value.get('block_width_mm') != block_width_mm:
        raise ValueError('Gripper calibration block width differs from the selected block')
    settings = validate_settings(**value['settings'])
    context = sha(json.dumps({k:v for k,v in settings.items() if k != 'force_percent'},sort_keys=True).encode())
    baseline, trials = value.get('baseline'), value.get('trials')
    if (settings['width_mm'] != value['block_width_mm'] or not isinstance(baseline,dict)
            or not isinstance(trials,list) or baseline.get('context') != context
            or baseline.get('model_sha256') != value.get('model_sha256')
            or baseline.get('open_q') != settings['open_q'] or baseline.get('max_q') != settings['max_q']
            or any(not isinstance(t,dict) or t.get('context') != context for t in trials)):
        raise ValueError('Gripper profile mixes different settings, model or trial contexts')
    candidate = recommend(trials, baseline)
    if (candidate is None or candidate != value.get('recommendation')
            or value.get('gripper_grasp_position') != candidate['commanded_q']
            or value.get('gripper_open_position') != settings['open_q']
            or value.get('force_percent') != candidate['force_percent']):
        raise ValueError('Gripper profile does not match its verified calibration evidence')
    return value


class CalibrationBackend:
    def __init__(self, calibration_file, physical_profile, output_dir=None, allow_mock=False,
                 gripper_only=False, execution_mode='step'):
        self.node = None
        self.locks = []
        self.closed = False
        self.mock = bool(allow_mock)
        if execution_mode not in ('step', 'continuous'):
            raise ValueError('execution_mode must be step or continuous')
        self.execution_mode = execution_mode
        self.pending_approval = self.approved_preview = None
        self.command_index = self.operation_steps = 0
        if not isinstance(gripper_only, bool):
            raise ValueError('gripper_only must be a boolean')
        self.gripper_only = gripper_only
        self.arm_detected = False
        self.standalone_conflict = False
        self.standalone_conflict_reason = ''
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        if not domain.isdigit():
            raise ValueError('ROS_DOMAIN_ID must be numeric')
        try:
            for prefix in ('ur5e_air_task', 'ur5e_stack'):
                fd = os.open(f'/tmp/{prefix}_{domain}.lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
                self.locks.append(fd)
                fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
            self.model = load_moveit_settings(calibration_file, physical_profile)['robot_description']
            self.model_sha = sha(self.model.encode())
            self.gap = GripperGapModel(self.model)
            self.node = rclpy.create_node('ur5e_gripper_calibration',use_global_arguments=False)
            self.standalone_discovery_started = time.monotonic()
            self.action = ActionClient(self.node, GripperCommand,
                                       '/gripper/dh_ag95_gripper_controller/gripper_cmd')
            self.force_pub = self.node.create_publisher(Float64MultiArray,
                                       '/gripper/force_forward_controller/commands', 1)
            self.subs = [self.node.create_subscription(DynamicJointState,
                         '/gripper/dynamic_joint_states', self._sensor, qos_profile_sensor_data),
                         self.node.create_subscription(JointState, '/joint_states',
                         self._arm, qos_profile_sensor_data)]
            self.queries = {
                'controllers': self.node.create_client(ListControllers, '/gripper/controller_manager/list_controllers'),
                'hardware': self.node.create_client(ListHardwareComponents, '/gripper/controller_manager/list_hardware_components')}
            if not self.gripper_only:
                self.queries['model'] = self.node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
            self.query_futures = {}
            self.query_checked = {}
            self.controllers, self.hardware_ok, self.model_ok = {}, False, False
            self.settings = validate_settings()
            self.context = ''
            self.sensor = {}
            self.sequence = -1
            self.sensor_time = -math.inf
            self.q_history = deque()
            self.arm_values, self.arm_velocities, self.arm_times = {}, {}, {}
            self.arm_quiet_since = None
            self.phase, self.error, self.busy = 'IDLE', '', False
            self.operation, self.stage = None, None
            self.queue = deque()
            self.send_future = self.goal = self.result_future = None
            self.goal_terminal = None
            self.target_q = None
            self.goal_sent = self.last_cancel = -math.inf
            self.stop_requested = self.stop_confirmed = False
            self.stop_proof_time, self.stop_proof_sequence = -math.inf, -1
            self.ever_commanded = False
            self.stop_hold_sent = False
            self.last_force = self.last_log = -math.inf
            self.stage_started = time.monotonic()
            self.last_goal_sequence = -1
            self.trials, self.baseline, self.current_trial = [], None, None
            self.current_baseline_max = None
            self.arm_reference = {}
            self.configuration_values = dict(self.settings)
            directory = Path(output_dir or Path.home()/'.ros/ur5e_a3_stack/gripper_calibration').expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            name = time.strftime('%Y%m%d_%H%M%S')+f'_domain{domain}_{os.getpid()}'
            self.csv_path, self.json_path = directory/(name+'.csv'), directory/(name+'.json')
            self.csv = self.csv_path.open('w', newline='')
            self.writer = csv.DictWriter(self.csv, fieldnames=[
                'unix_time','phase','trial_id','commanded_q_rad','measured_q_rad','model_gap_mm',
                'raw_position','grip_state','force_setting_percent','communication_age_s','sample_sequence',
                'arm_feedback_required','arm_stationary','action_status','event'])
            self.writer.writeheader()
            self.configure(**self.settings)
        except Exception:
            if self.node is not None:
                self.node.destroy_node()
            for fd in self.locks:
                os.close(fd)
            raise

    def _sensor(self, message):
        if len(message.joint_names) != len(message.interface_values) or HAND not in message.joint_names:
            return
        record = message.interface_values[message.joint_names.index(HAND)]
        if len(record.interface_names) != len(record.values) or len(set(record.interface_names)) != len(record.interface_names):
            return
        values = dict(zip(record.interface_names, record.values))
        required = ('position','raw_position','grip_state','force_percent','communication_age','sample_sequence')
        if any(k not in values or not math.isfinite(values[k]) for k in required):
            return
        q, raw, state, force, age, seq = [values[k] for k in required]
        if (not 0 <= q <= .93 or not 0 <= raw <= 1000 or raw != int(raw)
                or state not in LABELS or not 20 <= force <= 100 or not 0 <= age <= .5
                or seq != int(seq) or seq < 1 or abs(q-.93*(1000-raw)/1000) > .0015):
            return
        if seq <= self.sequence:
            return  # A repeated broadcaster timestamp is not a new serial sample.
        now = time.monotonic()
        self.sequence, self.sensor_time, self.sensor = int(seq), now-age, values
        self.q_history.append((now-age, q))
        while self.q_history and self.q_history[0][0] < now-3:
            self.q_history.popleft()
        if self.current_trial and not self.current_trial['released'] and state == 3:
            self.current_trial.update(dropped=True, verified_success=False)
            self._save('DROPPED')
            self._fail('그리퍼 센서에서 물체 놓침이 감지되었습니다.')

    def _arm(self, message):
        # Partial and velocity-less feedback still proves an arm source exists.
        if any(any(name.endswith(joint) for joint in ARM) for name in message.name):
            self.arm_detected = True
        if len(message.name) != len(message.position) or len(message.velocity) != len(message.name):
            return
        now = time.monotonic()
        for name, q, velocity in zip(message.name, message.position, message.velocity):
            if name in ARM and math.isfinite(q) and math.isfinite(velocity):
                self.arm_values[name], self.arm_velocities[name], self.arm_times[name] = q, velocity, now

    def _fresh(self):
        return bool(self.sensor) and time.monotonic()-self.sensor_time <= .5

    def _quiet(self, since=-math.inf):
        if not self._fresh() or self.sensor['grip_state'] == 0:
            return False
        now = time.monotonic()
        history = [(t,q) for t,q in self.q_history if t >= since and now-t <= self.settings['settle_s']+.15]
        return (len(history) >= 3 and history[-1][0]-history[0][0] >= self.settings['settle_s']
                and max(q for _,q in history)-min(q for _,q in history) <= .0015)

    def _arm_stationary(self):
        if self.gripper_only:
            return self._mode_guard_ok()
        now = time.monotonic()
        quiet = all(n in self.arm_times and now-self.arm_times[n] <= .5
                    and abs(self.arm_velocities[n]) <= .01 for n in ARM)
        self.arm_quiet_since = (now if self.arm_quiet_since is None else self.arm_quiet_since) if quiet else None
        return self.arm_quiet_since is not None and now-self.arm_quiet_since >= .5

    def _mode_guard_ok(self):
        return (not self.gripper_only or not self.arm_detected and not self.standalone_conflict
                and time.monotonic()-self.standalone_discovery_started >= 2.)

    def _refresh_standalone_graph(self):
        if not self.gripper_only:
            return
        try:
            other = [name for name, _ in self.node.get_service_names_and_types()
                     if name.endswith('/list_hardware_components')
                     and name != '/gripper/controller_manager/list_hardware_components']
            if other:
                self.standalone_conflict = True
                self.standalone_conflict_reason = '다른 로봇 제어기가 검색되었습니다: '+', '.join(sorted(other))
        except Exception:
            self.standalone_conflict = True
            self.standalone_conflict_reason = '다른 로봇 제어기의 연결 여부를 확인하지 못했습니다.'

    def _inspect_standalone_hardware(self, components):
        if not self.gripper_only:
            return
        for component in components:
            interfaces = list(component.command_interfaces)+list(component.state_interfaces)
            if ('URPositionHardwareInterface' in component.class_type or
                    any(any(i.name.split('/')[0].endswith(joint) for joint in ARM) for i in interfaces)):
                self.arm_detected = True

    def _queries(self):
        self._refresh_standalone_graph()
        now = time.monotonic()
        for name, client in self.queries.items():
            pending = self.query_futures.get(name)
            if pending:
                future, sent = pending
                if future.done():
                    try:
                        reply = future.result()
                        if name == 'controllers':
                            self.controllers = {c.name:c.state for c in reply.controller}
                        elif name == 'hardware':
                            self._inspect_standalone_hardware(reply.component)
                            wanted = MOCK if self.mock else PLUGIN
                            self.hardware_ok = any(c.class_type == wanted and c.state.id == 3
                                and any(i.name == HAND+'/position' and i.is_claimed for i in c.command_interfaces)
                                for c in reply.component)
                        else:
                            self.model_ok = bool(reply.values and reply.values[0].string_value == self.model)
                        self.query_checked[name] = now
                    except Exception:
                        self.query_checked[name] = -math.inf
                    del self.query_futures[name]
                elif now-sent > 2:
                    client.remove_pending_request(future)
                    del self.query_futures[name]
                    self.query_checked[name] = -math.inf
            elif client.service_is_ready() and now-self.query_checked.get(name, -math.inf) > .5:
                request = (ListControllers.Request() if name == 'controllers' else
                           ListHardwareComponents.Request() if name == 'hardware' else GetParameters.Request())
                if name == 'model':
                    request.names = ['robot_description']
                self.query_futures[name] = (client.call_async(request), now)

    def _controller_ok(self):
        now = time.monotonic()
        return (all(now-self.query_checked.get(n,-math.inf) < 2 for n in self.queries)
                and self.hardware_ok and (self.gripper_only or self.model_ok)
                and self.controllers.get('dh_ag95_gripper_controller') == 'active'
                and self.controllers.get('force_forward_controller') == 'active'
                and self.action.server_is_ready())

    def configure(self, **values):
        if self.busy:
            raise RuntimeError('진행 중인 시험을 먼저 정지하세요.')
        new = validate_settings(**values)
        context = sha(json.dumps({k:v for k,v in new.items() if k != 'force_percent'},sort_keys=True).encode())
        if context != self.context:
            self.context, self.baseline = context, None
        self.settings = self.configuration_values = new
        self._save('CONFIGURE')

    def _recommendation(self):
        return recommend([t for t in self.trials if t['context'] == self.context], self.baseline)

    def _save(self, event=''):
        if not hasattr(self, 'json_path'):
            return
        atomic_json(self.json_path, dict(schema=1, model_sha256=self.model_sha,
                    gripper_only=self.gripper_only,
                    execution_mode=self.execution_mode, pending_approval=self.pending_approval,
                    hardware_class=MOCK if self.mock else PLUGIN, settings=self.settings,
                    baseline=self.baseline, trials=self.trials, recommendation=self._recommendation(),
                    csv_path=str(self.csv_path), event=event))
        self._log(event)

    def _log(self, event=''):
        if not hasattr(self, 'writer'):
            return
        s = self.sensor
        q = s.get('position')
        self.writer.writerow(dict(unix_time=time.time(),phase=self.phase,
            trial_id=self.current_trial['id'] if self.current_trial else '',
            commanded_q_rad=self.target_q if self.target_q is not None else '', measured_q_rad=q,
            model_gap_mm=self.gap.gap_mm(q) if q is not None else '',
            raw_position=s.get('raw_position'),grip_state=s.get('grip_state'),
            force_setting_percent=s.get('force_percent'),communication_age_s=time.monotonic()-self.sensor_time,
            sample_sequence=s.get('sample_sequence'),arm_feedback_required=not self.gripper_only,
            arm_stationary=self._arm_stationary() if not self.gripper_only else '',
            action_status=self.goal_terminal.status if self.goal_terminal else '',event=event))
        self.csv.flush()

    def start(self, mode, workspace_clear=False):
        if self.busy or self.send_future is not None or self.goal is not None:
            raise RuntimeError('다른 그리퍼 명령이 진행 중입니다.')
        if workspace_clear is not True:
            raise RuntimeError('손이 없는 작업 공간과 블록 받침을 확인하세요.')
        if mode not in ('baseline','grasp','open','step_close'):
            raise ValueError('Unknown calibration operation')
        self._refresh_standalone_graph()
        if not self._mode_guard_ok():
            raise RuntimeError(self.standalone_conflict_reason or
                               '팔 피드백이 검색되었거나 단독 모드 연결 확인 중입니다. 팔 연결 시 기본 보정 모드를 사용하세요.')
        if not self._fresh() or not self._quiet() or not self._arm_stationary() or not self._controller_ok():
            raise RuntimeError('실제 센서·힘 제어기·팔 정지 상태를 먼저 확인하세요.')
        q, state = self.sensor['position'], self.sensor['grip_state']
        if mode == 'baseline' and state != 1:
            raise RuntimeError('빈 상태 기준 측정은 물체가 없는 상태에서 시작하세요.')
        if mode == 'grasp' and (state != 1 or abs(q-self.settings['open_q']) > .003):
            raise RuntimeError('블록을 받침에 놓고 먼저 설정된 열림 각도로 여세요.')
        if mode == 'step_close' and state == 2:
            raise RuntimeError('이미 잡힘 상태입니다. 더 닫지 말고 성공 여부를 확인하세요.')
        if mode == 'step_close' and q >= self.settings['max_q']-1e-6:
            raise RuntimeError('설정한 닫힘 상한에 도달했습니다.')
        if mode == 'open' and q < self.settings['open_q']-.003:
            raise RuntimeError('현재 더 넓게 열려 있습니다. 열림 기준각을 현재 각도 이하로 설정하세요.')
        self.operation, self.phase, self.stage = mode, mode.upper(), 'FORCE'
        self.busy, self.error = True, ''
        self.stop_requested = self.stop_confirmed = self.stop_hold_sent = False
        self.arm_reference = dict(self.arm_values)
        self.stage_started = time.monotonic()
        self.last_force, self.force_sequence = -math.inf, self.sequence
        self.queue.clear()
        self.pending_approval = self.approved_preview = None
        self.command_index = 0
        self.current_baseline_max = None
        self.target_q = None
        if mode == 'baseline':
            self.baseline = None
            self.queue.append(self.settings['open_q'])
            self.queue.extend(self._steps(self.settings['open_q']))
            self.queue.append(self.settings['open_q'])
        elif mode == 'grasp':
            self.current_trial = dict(id=len(self.trials)+1,context=self.context,
                force_percent=self.settings['force_percent'],commanded_q=None,measured_q=None,
                verified_success=None,released=False,grip_state=None,sensor_valid=False,
                dropped=False,stopped=False,arm_moved=False)
            self.trials.append(self.current_trial)
            self.queue.extend(self._steps(q))
        elif mode == 'step_close':
            self.queue.append(min(self.settings['max_q'],q+self.settings['step_q']))
        else:
            if self.current_trial and self.current_trial['verified_success'] is None:
                self.current_trial['verified_success'] = False
            self.queue.append(self.settings['open_q'])
        self.operation_steps = len(self.queue)
        self._save('START_'+mode)

    def _preview_next(self):
        q = float(self.queue[0])
        self.pending_approval = dict(
            request_id=secrets.token_hex(16), operation=self.operation,
            index=self.command_index+1, total=self.operation_steps,
            start_q=float(self.sensor['position']), target_q=q,
            start_gap_mm=float(self.gap.gap_mm(self.sensor['position'])),
            target_gap_mm=float(self.gap.gap_mm(q)),
            force_percent=self.settings['force_percent'], context=self.context,
            grip_state=int(self.sensor['grip_state']),
            preview_kind='gripper_angle_and_estimated_gap')
        self.stage, self.phase = 'WAIT_APPROVAL', 'WAIT_APPROVAL'
        self._save('PREVIEW_NEXT')

    def _check_preview(self, preview):
        if (self.stop_requested or not self.busy or not self.queue
                or self.send_future is not None or self.goal is not None):
            raise RuntimeError('현재 승인할 그리퍼 단계가 없습니다.')
        if (not self._mode_guard_ok() or not self._fresh() or not self._quiet()
                or not self._arm_stationary() or not self._controller_ok()):
            raise RuntimeError('각도·간격 미리보기 후 연결 또는 정지 조건이 바뀌었습니다.')
        if (preview['context'] != self.context or preview['operation'] != self.operation
                or preview['target_q'] != self.queue[0]
                or abs(preview['start_q']-self.sensor['position']) > .0015
                or preview['grip_state'] != int(self.sensor['grip_state'])
                or abs(preview['force_percent']-self.sensor['force_percent']) > .5):
            raise RuntimeError('그리퍼 설정이나 현재 자세가 미리보기와 달라졌습니다. 정지 후 다시 시작하세요.')

    def approve_next(self, request_id):
        """Approve exactly the displayed target; old or duplicate tokens fail."""
        preview = self.pending_approval
        if (self.execution_mode != 'step' or self.stage != 'WAIT_APPROVAL'
                or preview is None or not isinstance(request_id, str)
                or request_id != preview['request_id']):
            raise RuntimeError('이전 단계의 승인 또는 유효하지 않은 승인 요청입니다.')
        self._check_preview(preview)
        self.approved_preview, self.pending_approval = preview, None
        self.stage, self.phase = 'NEXT', self.operation.upper()
        self._save('APPROVED_NEXT')

    def _steps(self, q):
        values = []
        while q < self.settings['max_q']-1e-8:
            q = min(self.settings['max_q'], q+self.settings['step_q'])
            values.append(q)
        return values

    def _send(self, q, hold=False):
        if not hold and self.stop_requested:
            raise RuntimeError('정지 요청 후 새 그리퍼 이동 명령을 보낼 수 없습니다.')
        if not hold and not self._mode_guard_ok():
            raise RuntimeError('그리퍼 단독 모드에서 팔 또는 다른 로봇 제어기가 검색되었습니다.')
        if not self._fresh() or not self._controller_ok():
            raise RuntimeError('새 센서 데이터와 활성 그리퍼 제어기가 필요합니다.')
        if not hold and self.execution_mode == 'step':
            preview = self.approved_preview
            if preview is None or preview['target_q'] != q:
                raise RuntimeError('각도·간격 미리보기를 확인하고 해당 단계의 OK를 누르세요.')
            self._check_preview(preview)
            self.approved_preview = None  # One approval authorizes one send only.
        goal = GripperCommand.Goal()
        goal.command.position = float(q)
        # PositionGripperActionController does not command actual force through
        # max_effort. The separately acknowledged force controller does that.
        goal.command.max_effort = 0.
        self.send_future = self.action.send_goal_async(goal)
        self.goal = self.result_future = self.goal_terminal = None
        self.target_q, self.goal_sent, self.last_goal_sequence = q, time.monotonic(), self.sequence
        self.ever_commanded = True
        if not hold:
            self.command_index += 1
        self.stage = 'HOLD_GOAL' if hold else 'GOAL'
        self._log('SEND_HOLD' if hold else 'SEND')

    def _action_tick(self):
        now = time.monotonic()
        if self.send_future is not None:
            if not self.send_future.done():
                if now-self.goal_sent > 5:
                    self._fail('목표 수락 응답 대기 중입니다. 수락 여부를 확인한 뒤 정지합니다.')
                return
            try:
                goal = self.send_future.result()
            except Exception:
                # An ambiguous send failure is not proof that no remote goal
                # exists. Preserve the pending state for operator inspection.
                self._fail('그리퍼 목표 수락 상태가 불명확합니다. 창을 유지하고 연결을 확인하세요.')
                return
            if goal is None:
                self._fail('그리퍼 목표 수락 상태가 불명확합니다. 창을 유지하고 연결을 확인하세요.')
                return
            self.send_future = None
            if not goal.accepted:
                self._fail('그리퍼 제어기가 명령을 거절했습니다.')
                self.stage = 'REJECTED'
                return
            self.goal = goal
        if self.goal is not None:
            if self.stop_requested and now-self.last_cancel > .5:
                try:
                    self.goal.cancel_goal_async()
                    self.last_cancel = now
                except Exception:
                    pass
            if self.result_future is None:
                try:
                    self.result_future = self.goal.get_result_async()
                    if self.result_future is None:
                        self._fail('그리퍼 종료 응답을 받지 못해 취소하고 다시 확인합니다.')
                        return
                except Exception:
                    self._fail('그리퍼 종료 응답 오류로 취소하고 다시 확인합니다.')
                    return
            if self.result_future.done():
                try:
                    result = self.result_future.result()
                except Exception:
                    self.result_future = None
                    self._fail('종료 결과를 다시 확인하고 있습니다.')
                    return
                if result is not None and result.status in (4,5,6):
                    self.goal_terminal = result
                    self.goal = self.result_future = None
                    self.stage = 'SETTLE'
                    self.stage_started = now
                    if self.stop_requested:
                        self.stop_proof_time, self.stop_proof_sequence = now, self.sequence
                else:
                    self.result_future = None
                    self._fail('그리퍼의 종료 상태가 확인되지 않았습니다.')
            elif now-self.goal_sent > 5:
                self._fail('그리퍼 동작 시간이 초과되어 취소하고 있습니다.')

    def _fail(self, message):
        self.error = message
        if not self.stop_requested:
            self.stop_proof_time, self.stop_proof_sequence = time.monotonic(), self.sequence
        self.stop_requested = True
        self.busy = True
        self.phase = 'CANCELLING'
        self.queue.clear()
        self.pending_approval = self.approved_preview = None
        if self.current_trial and not self.current_trial['released']:
            self.current_trial['stopped'] = True

    def request_stop(self):
        if self.closed:
            return
        if not self.stop_requested:
            self.stop_proof_time, self.stop_proof_sequence = time.monotonic(), self.sequence
        self.stop_requested, self.stop_confirmed = True, False
        self.busy, self.phase = True, 'CANCELLING'
        self.queue.clear()
        self.pending_approval = self.approved_preview = None
        if self.current_trial and not self.current_trial['released']:
            self.current_trial['stopped'] = True

    def _stop_tick(self):
        if self.send_future is not None or self.goal is not None:
            return
        if not self.ever_commanded:
            self.busy, self.stop_confirmed, self.phase = False, True, 'IDLE'
        elif self.sequence > self.stop_proof_sequence and self._quiet(since=self.stop_proof_time):
            self.busy, self.stop_confirmed = False, True
            self.phase = 'ERROR' if self.error else ('HOLD' if self.sensor['grip_state'] == 2 else 'IDLE')
        elif self._fresh() and self._controller_ok() and self.sensor['grip_state'] == 0 and not self.stop_hold_sent:
            self.stop_hold_sent = True
            self._send(self.sensor['position'],hold=True)
        if self.stop_confirmed:
            self.stop_requested = False
            self._save('STOP_CONFIRMED')

    def _normal_tick(self):
        if self.stop_requested:
            return
        now = time.monotonic()
        if self.stage == 'FORCE':
            if now-self.last_force > .2:
                self.force_pub.publish(Float64MultiArray(data=[(self.settings['force_percent']-20)/80.]))
                self.ever_commanded = True
                self.last_force = now
            if self.sequence > self.force_sequence and abs(self.sensor['force_percent']-self.settings['force_percent']) <= .5:
                self.stage = 'NEXT'
            elif now-self.stage_started > 3:
                self._fail('그리퍼에서 설정한 힘 비율을 확인하지 못했습니다.')
            return
        if self.send_future is not None or self.goal is not None:
            return
        if self.stage == 'SETTLE':
            if not self._quiet(since=self.stage_started) or self.sequence <= self.last_goal_sequence:
                if now-self.stage_started > 3:
                    self._fail('새 센서 데이터로 그리퍼 정지를 확인하지 못했습니다.')
                return
            state = int(self.sensor['grip_state'])
            result = self.goal_terminal
            caught = state == 2
            if self.operation == 'baseline' and caught:
                self._fail('빈 상태 기준 측정 중 접촉이 감지되었습니다. 블록과 간섭을 확인하세요.')
                return
            if caught and self.operation in ('grasp','step_close'):
                if result.status not in (4,6) or (result.status == 6 and not result.result.stalled):
                    self._fail('잡힘 센서는 있지만 제어기 동작이 정상 종료되지 않았습니다.')
                    return
                if self.current_trial and self.operation == 'grasp':
                    self.current_trial.update(commanded_q=self.target_q,
                        measured_q=self.sensor['position'],grip_state=2,sensor_valid=True)
                self.queue.clear()
                self.busy, self.phase = False, 'HOLD'
                self._save('CONTACT_RECORDED')
                return
            if result.status != 4 or state != 1 or abs(self.sensor['position']-self.target_q) > .003:
                self._fail('목표 각도 도달 또는 물체 없는 상태를 확인하지 못했습니다.')
                return
            if self.operation == 'baseline':
                self.current_baseline_max = max(self.current_baseline_max or 0,self.sensor['position'])
            self.stage = 'NEXT'
        if self.stage == 'NEXT':
            if self.queue:
                if self.execution_mode == 'step' and self.approved_preview is None:
                    self._preview_next()
                else:
                    # Leave the reviewed target in the queue until _send has
                    # revalidated its pose/settings and consumed its approval.
                    self._send(self.queue[0])
                    self.queue.popleft()
            else:
                self.busy = False
                self.phase = 'IDLE'
                if self.operation == 'baseline':
                    self.baseline = dict(passed=True,max_q=self.settings['max_q'],
                        measured_max_q=self.current_baseline_max,
                        open_q=self.settings['open_q'],force_percent=self.settings['force_percent'],
                        model_sha256=self.model_sha,context=self.context)
                elif self.operation == 'open' and self.current_trial:
                    self.current_trial['released'] = True
                    self.current_trial = None
                elif self.operation == 'grasp':
                    self.current_trial.update(verified_success=False,grip_state=1,sensor_valid=True)
                    self.error = '닫힘 상한까지 물체 잡힘이 감지되지 않았습니다. 정렬을 확인하세요.'
                self._save('COMPLETE_'+self.operation)

    def record_result(self, success):
        if not isinstance(success, bool):
            raise ValueError('A boolean operator result is required')
        self._refresh_standalone_graph()
        if not self._mode_guard_ok():
            raise RuntimeError('단독 모드의 연결 조건을 확인한 뒤 기록하세요.')
        if (self.busy or self.current_trial is None or self.current_trial['verified_success'] is not None
                or self.current_trial['grip_state'] != 2 or not self._quiet() or not self._arm_stationary()
                or self.current_trial['sensor_valid'] is not True or self.current_trial['stopped']
                or self.current_trial['dropped'] or self.current_trial['arm_moved']
                or self.sensor['grip_state'] != 2
                or abs(self.sensor['force_percent']-self.current_trial['force_percent']) > .5
                or abs(self.sensor['position']-self.current_trial['measured_q']) > .01):
            raise RuntimeError('센서 잡힘과 정지 상태가 확인된 회차에서 기록하세요.')
        self.current_trial['verified_success'] = success
        self._save('OPERATOR_SUCCESS' if success else 'OPERATOR_FAILURE')

    def apply_recommendation(self):
        if self.mock:
            raise RuntimeError('가상 시험 기록은 실물 파지 보정값으로 적용할 수 없습니다.')
        self._refresh_standalone_graph()
        if not self._mode_guard_ok():
            raise RuntimeError('단독 모드의 연결 조건이 바뀌어 추천값을 적용할 수 없습니다.')
        candidate = self._recommendation()
        if self.busy or candidate is None or not self._quiet() or self.sensor['grip_state'] != 1:
            raise RuntimeError('빈 상태 기준과 성공·놓기 3회 이상을 확인한 뒤 적용하세요.')
        trials = [t for t in self.trials if t['context'] == self.context]
        profile = dict(schema=1,hardware_class=PLUGIN,model_sha256=self.model_sha,
            gripper_only=self.gripper_only,
            block_width_mm=self.settings['width_mm'],settings=self.settings,baseline=self.baseline,
            trials=trials,recommendation=candidate,gripper_open_position=self.settings['open_q'],
            gripper_grasp_position=candidate['commanded_q'],force_percent=candidate['force_percent'],
            session_json=str(self.json_path),created_at=time.time())
        PROFILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = PROFILE.with_suffix('.yaml.tmp')
        tmp.write_text(yaml.safe_dump(profile,allow_unicode=True,sort_keys=False))
        load_calibration_profile(tmp,model_sha256=self.model_sha,block_width_mm=self.settings['width_mm'])
        tmp.replace(PROFILE)
        self._save('PROFILE_APPLIED')
        return str(PROFILE)

    def poll(self):
        if self.closed:
            return {'can_close':True,'busy':False,'phase':'CLOSED'}
        for _ in range(30):
            rclpy.spin_once(self.node,timeout_sec=0)
        self._queries()
        stationary = self._arm_stationary()
        # The operator may manually lift the arm while the fingers HOLD to
        # check the grasp. Sensor continuity and the tested force must remain
        # valid; arm motion is forbidden while finger commands are active.
        trial = self.current_trial
        if trial and not trial['released'] and trial['sensor_valid'] and not self.stop_requested:
            if (not self._fresh() or abs(self.sensor['force_percent']-trial['force_percent']) > .5
                    or self.sensor['grip_state'] == 3
                    or self.phase == 'HOLD' and abs(self.sensor['position']-trial['measured_q']) > .01):
                trial['sensor_valid'] = False
                self._fail('파지 유지 중 센서 연속성·힘 설정·각도가 바뀌어 해당 회차를 무효화합니다.')
        if self.busy and not self.stop_requested:
            moved = not self.gripper_only and any(n not in self.arm_values or abs(self.arm_values[n]-self.arm_reference.get(n,math.inf)) > .01 for n in ARM)
            if not self._fresh() or not self._controller_ok() or not stationary or moved:
                if self.current_trial and moved:
                    self.current_trial['arm_moved'] = True
                self._fail('센서·제어기 연결 또는 팔 정지 조건이 바뀌어 그리퍼를 정지합니다.')
            elif self.stage != 'FORCE' and abs(self.sensor['force_percent']-self.settings['force_percent']) > .5:
                self._fail('시험 도중 힘 설정이 바뀌어 그리퍼를 정지합니다.')
        self._action_tick()
        if self.stop_requested:
            self._stop_tick()
        elif self.busy:
            try:
                self._normal_tick()
            except Exception as error:
                self._fail(str(error))
        now = time.monotonic()
        if now-self.last_log > .05:
            self._log()
            self.last_log = now
        s = self.sensor
        fresh, quiet = self._fresh(), self._quiet()
        idle = not self.busy and not self.stop_requested and self.send_future is None and self.goal is None
        mode_error = ''
        if self.gripper_only:
            if self.arm_detected:
                mode_error = '팔 피드백이 감지되었습니다. 팔·가상 팔 실행을 종료한 뒤 단독 보정 창을 다시 여세요.'
            elif self.standalone_conflict:
                mode_error = self.standalone_conflict_reason+' · 해당 실행을 종료한 뒤 단독 보정 창을 다시 여세요.'
            elif now-self.standalone_discovery_started < 2.:
                mode_error = '그리퍼 단독 연결을 확인하고 있습니다. 잠시 기다려 주세요.'
        can_record = bool(idle and self.current_trial and self.current_trial['verified_success'] is None
                          and self.current_trial['grip_state'] == 2 and s.get('grip_state') == 2 and quiet and stationary
                          and self.current_trial['sensor_valid'] is True and not self.current_trial['stopped']
                          and not self.current_trial['dropped'] and not self.current_trial['arm_moved']
                          and abs(s['force_percent']-self.current_trial['force_percent']) <= .5
                          and abs(s['position']-self.current_trial['measured_q']) <= .01)
        return dict(phase=self.phase,busy=self.busy,ready=idle and fresh and quiet and stationary and self._controller_ok(),
            execution_mode=self.execution_mode,
            pending_approval=dict(self.pending_approval) if self.pending_approval else None,
            q_rad=s.get('position'),raw_position=s.get('raw_position'),
            model_gap_mm=self.gap.gap_mm(s['position']) if s else None,
            grip_state=s.get('grip_state'),grip_label=LABELS.get(s.get('grip_state'),'센서 없음'),
            force_percent=s.get('force_percent'),feedback_valid=fresh,age=now-self.sensor_time,
            gripper_only=self.gripper_only,arm_required=not self.gripper_only,
            arm_feedback_present=self.arm_detected,standalone_conflict=self.standalone_conflict,
            arm_stationary=stationary if not self.gripper_only else None,controller_active=self._controller_ok(),
            force_supported=self._controller_ok(),csv_path=str(self.csv_path),
            recommendation=self._recommendation(),stop_confirmed=self.stop_confirmed,
            can_close=idle and (not self.ever_commanded or fresh and quiet),can_record=can_record,
            configuration_values=dict(self.settings),baseline_passed=bool(self.baseline),error=self.error or mode_error)

    def close(self):
        if self.closed:
            return True
        status = self.poll()
        if not status['can_close']:
            if not self.stop_requested:
                self.request_stop()
            return False
        self._save('CLOSED')
        self.csv.close()
        self.action.destroy()
        self.node.destroy_node()
        for fd in self.locks:
            os.close(fd)
        self.locks.clear()
        self.closed = True
        return True
