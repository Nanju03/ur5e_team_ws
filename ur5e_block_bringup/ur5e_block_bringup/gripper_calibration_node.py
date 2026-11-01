"""Launchable terminal interface and ordered driver lifecycle for calibration.

No Tk, arm driver, MoveIt server or interactive stdin is used. ROS services
control experiments; responses acknowledge requests and ~/state reports results.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

from .gripper_calibration import CalibrationBackend
from .gripper_calibration_data import validate_settings

OPERATIONS = ('baseline', 'grasp', 'open', 'step_close')
SETTING_NAMES = ('open_q','max_q','step_q','force_percent','settle_s','width_mm')


def clean_json(value):
    if isinstance(value,float) and not math.isfinite(value):
        return None
    if isinstance(value,dict):
        return {k:clean_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):
        return [clean_json(v) for v in value]
    return value


def driver_command(com_port, force_percent, use_mock_hardware):
    if not isinstance(com_port,str) or not com_port or '\x00' in com_port:
        raise ValueError('com_port must name a serial device')
    return ['ros2','launch','dh_gripper_driver','dh_ag95_control.launch.py',
            'com_port:='+com_port, 'force_percent:='+str(force_percent),
            'use_mock_hardware:='+str(use_mock_hardware).lower(),
            'controller_mode:=action','gui:=false',
            'launch_robot_state_publisher:=false','joint_states_topic:=/joint_states']


class HeadlessCalibration(Node):
    def __init__(self, backend_factory=CalibrationBackend, driver_factory=subprocess.Popen):
        super().__init__('gripper_calibration')
        self.backend = None
        self.driver_process = None
        self.driver_factory = driver_factory
        self.driver_leases = []
        self.driver_attempted = False
        self.driver_start_cancelled = False
        self.driver_error = ''
        self.driver_stop_sent = False
        self.last_driver_warning = -math.inf
        self.driver_check_started = time.monotonic()
        self.shutdown_requested = self.shutdown_started = False
        self.backend_closed = False
        self.latest = {}
        self.approval_service = None
        self.approval_request_id = None
        self.approval_service_name = None
        self.operation_id = 0
        self.last_operation = ''
        self.last_phase = None
        self.last_status_time = self.last_log = -math.inf
        self.last_error = ''
        try:
            defaults = dict(calibration_file=str(Path.home()/'ur5e_ws/robot_calibration/ur5e_20261031.yaml'),
                physical_profile=str(Path.home()/'ur5e_ws/physical_setup.yaml'),
                gripper_only=True,allow_mock_hardware=False,start_driver=True,use_mock_hardware=False,
                com_port='/dev/robot/dh_ag95_gripper',startup_operation='monitor',workspace_clear=False,
                execution_mode='step',
                **validate_settings())
            for name,value in defaults.items():
                # Runtime permission changes use the explicit ~/ready service.
                self.declare_parameter(name,value,ParameterDescriptor(read_only=name in ('workspace_clear','execution_mode')))
            get = lambda name:self.get_parameter(name).value
            self.start_driver = get('start_driver')
            self.workspace_clear = get('workspace_clear')
            if get('execution_mode') not in ('step', 'continuous'):
                raise ValueError('execution_mode must be step or continuous')
            if any(not isinstance(get(n),bool) for n in
                   ('gripper_only','allow_mock_hardware','start_driver','use_mock_hardware','workspace_clear')):
                raise ValueError('Launch mode and permission values must be boolean')
            if self.start_driver and get('allow_mock_hardware') != get('use_mock_hardware'):
                raise ValueError('Mock backend and owned driver modes must match')
            self.startup_pending = get('startup_operation')
            if self.startup_pending not in ('monitor',)+OPERATIONS:
                raise ValueError('Unknown startup_operation')
            if self.startup_pending != 'monitor' and not self.workspace_clear:
                raise ValueError('Automatic startup motion requires workspace_clear:=true')
            self.startup_pending = None if self.startup_pending == 'monitor' else self.startup_pending
            self.startup_deadline = time.monotonic()+30.
            settings = self.settings()
            self.driver_args = driver_command(get('com_port'),settings['force_percent'],get('use_mock_hardware'))
            self.backend = backend_factory(get('calibration_file'),get('physical_profile'),
                allow_mock=get('allow_mock_hardware'),gripper_only=get('gripper_only'),
                execution_mode=get('execution_mode'))
            self.driver_check_started = time.monotonic()
            self.backend.configure(**settings)
            if self.start_driver:
                # Duplicated descriptors retain the exact same flock lease
                # after backend.close(), until the owned driver also exits.
                self.driver_leases = [os.dup(fd) for fd in self.backend.locks]
            self.state_pub = self.create_publisher(String,'~/state',10)
            self.calibration_services = [self.create_service(Trigger,'~/'+operation,
                lambda request,reply,operation=operation:self.command(operation,reply))
                for operation in OPERATIONS+('success','failure','stop','apply')]
            self.calibration_services += [self.create_service(Trigger,'~/status',self.get_status),
                              self.create_service(SetBool,'~/ready',self.set_ready)]
            self.get_logger().info('터미널 보정 시작 · 기본 연결 대기. /gripper_calibration/status로 상태 확인')
            self.get_logger().info('준비 확인 후 /gripper_calibration/ready {data: true}, baseline → grasp → success/failure → open')
            self.get_logger().info('CSV: '+str(self.backend.csv_path))
        except Exception:
            if self.backend is not None:
                self.backend.close()  # Constructor/configure have sent no commands.
            self._release_driver_leases()
            self.destroy_node()
            raise

    def settings(self):
        return validate_settings(**{n:self.get_parameter(n).value for n in SETTING_NAMES})

    def request_shutdown(self):
        if not self.shutdown_requested and self.backend is not None and not self.backend_closed:
            # request_stop only latches Python state; it performs no ROS calls.
            # This also interrupts an already-running backend poll on SIGINT.
            self.backend.request_stop()
            self.shutdown_started = True
        self.shutdown_requested = True
        self.startup_pending = None

    def _driver_available(self):
        return not self.start_driver or (self.driver_process is not None and self.driver_process.poll() is None)

    def _driver_tick(self):
        if not self.start_driver or self.driver_attempted or self.driver_start_cancelled or self.shutdown_requested:
            return
        if time.monotonic()-self.driver_check_started < 2.:
            return
        self.driver_attempted = True
        try:
            names = {name for name,_ in self.backend.node.get_service_names_and_types()}
            if '/gripper/controller_manager/list_hardware_components' in names:
                raise RuntimeError('이미 그리퍼 드라이버가 실행 중입니다. start_driver:=false로 다시 실행하세요.')
            # A separate session isolates the driver from the launch parent's
            # Ctrl+C. It remains alive until the backend proves physical stop.
            self.driver_process = self.driver_factory(self.driver_args,start_new_session=True)
            # Lifecycle activation itself may move the fingers. Even before
            # the first calibration goal, closing must prove a fresh stop.
            self.backend.ever_commanded = True
            self.get_logger().info('AG95 드라이버 연결 중 · 초기화로 손가락이 움직일 수 있습니다.')
        except Exception as error:
            self.driver_error = str(error)
            self.startup_pending = None
            self.get_logger().error(self.driver_error)

    def command(self, operation, reply):
        try:
            if self.shutdown_requested:
                raise RuntimeError('종료 중에는 새 명령을 받을 수 없습니다.')
            if operation == 'stop':
                self.startup_pending = None
                self._cancel_pending_driver_start()
                self.backend.request_stop()
            elif operation in OPERATIONS:
                if not self.workspace_clear:
                    raise RuntimeError('고정·받침·손 제외·전원 차단 준비를 확인한 뒤 /ready를 true로 설정하세요.')
                if not self._driver_available() or self.driver_error:
                    raise RuntimeError(self.driver_error or '그리퍼 드라이버 연결 확인 중입니다.')
                if self.backend.busy:
                    raise RuntimeError('현재 시험이 끝나거나 정지가 확인된 뒤 명령을 보내세요.')
                self.backend.configure(**self.settings())
                self.backend.start(operation,workspace_clear=True)
                self.startup_pending = None
            elif operation in ('success','failure'):
                self.backend.record_result(operation == 'success')
            elif operation == 'apply':
                path = self.backend.apply_recommendation()
                reply.success,reply.message = True,'보정 프로파일 저장 완료: '+str(path)
                return reply
            else:
                raise ValueError('Unknown calibration command')
            self.operation_id += 1
            self.last_operation = operation
            reply.success = True
            reply.message = f'접수 id={self.operation_id} operation={operation}; 완료 여부는 /status 또는 /state에서 확인하세요.'
        except Exception as error:
            reply.success,reply.message = False,str(error)
        return reply

    def set_ready(self, request, reply):
        if self.shutdown_requested:
            reply.success,reply.message = False,'종료 중입니다.'
            return reply
        self.workspace_clear = bool(request.data)
        if not self.workspace_clear:
            self.startup_pending = None
            self._cancel_pending_driver_start()
            self.backend.request_stop()
        reply.success = True
        reply.message = ('작업 공간 준비 확인을 받았습니다. 물리 명령은 별도 서비스로 시작하세요.' if self.workspace_clear else
                         '허용을 철회하고 현재 시험의 정지를 요청했습니다.')
        return reply

    def _cancel_pending_driver_start(self):
        if self.start_driver and not self.driver_attempted and self.driver_process is None:
            self.driver_start_cancelled = True
            self.driver_error = '드라이버 시작 전에 정지 요청을 받아 자동 연결을 취소했습니다. 다시 launch하세요.'

    def status(self):
        return clean_json({**self.latest,'workspace_clear':self.workspace_clear,
            'approval_service':self.approval_service_name,
            'operation_id':self.operation_id,'last_operation':self.last_operation,
            'startup_pending':self.startup_pending,'closing':self.shutdown_requested,
            'driver_owned':self.start_driver,
            'driver_running':self._driver_available() if self.start_driver else bool(self.latest.get('controller_active')),
            'driver_error':self.driver_error})

    def get_status(self, _, reply):
        reply.success = True
        reply.message = json.dumps(self.status(),ensure_ascii=False,allow_nan=False)
        return reply

    def _retire_approval_service(self):
        if self.approval_service is not None:
            self.destroy_service(self.approval_service)
        self.approval_service = self.approval_request_id = self.approval_service_name = None

    def _sync_approval_service(self):
        preview = self.latest.get('pending_approval')
        token = preview.get('request_id') if isinstance(preview, dict) else None
        if token == self.approval_request_id:
            return
        self._retire_approval_service()
        if token is not None and not self.shutdown_requested:
            if not isinstance(token, str) or len(token) != 32 or any(c not in '0123456789abcdef' for c in token):
                raise ValueError('Invalid gripper approval token')
            relative = '~/approve_next/r'+token
            self.approval_service = self.create_service(Trigger, relative,
                lambda request, reply, token=token: self.approve(token, reply))
            self.approval_request_id = token
            self.approval_service_name = self.get_fully_qualified_name()+'/approve_next/r'+token
            self.get_logger().info('그리퍼 각도/추정 간격 확인 후 승인: '+self.approval_service_name)

    def approve(self, token, reply):
        try:
            if self.shutdown_requested or not self.workspace_clear or not self._driver_available() or self.driver_error:
                raise RuntimeError('현재 그리퍼 단계를 승인할 실행 준비가 되지 않았습니다.')
            self.backend.approve_next(token)
            # Retire in the next tick, after rclpy has sent this response.
            # Destroying the service inside its own callback can destroy the
            # handle that the executor still needs to acknowledge this OK.
            self.approval_service_name = None
            reply.success, reply.message = True, '표시한 그리퍼 단계 1개를 승인했습니다. 새 센서 검사 후 실행합니다.'
        except Exception as error:
            reply.success, reply.message = False, str(error)
        return reply

    def tick(self):
        if self.shutdown_requested:
            return self.close_tick()
        self._driver_tick()
        if self.driver_process is not None and self.driver_process.poll() is not None and not self.driver_error:
            self.driver_error = '그리퍼 드라이버가 종료되었습니다. 연결과 센서 정지 상태를 확인하세요.'
            self.startup_pending = None
            self.backend.request_stop()
        # Service stop/ready=False runs before this poll in the main loop.
        # There is no delayed queue whose commands could replay after a stop.
        self.latest = self.backend.poll()
        self._sync_approval_service()
        if self.start_driver and not self._driver_available():
            self.latest['ready'] = False
        if self.startup_pending:
            if time.monotonic() > self.startup_deadline:
                self.get_logger().error('시작 동작의 준비 대기 시간이 초과됐습니다. 자동 재시도하지 않습니다.')
                self.startup_pending = None
            elif self.latest.get('ready'):
                operation,self.startup_pending = self.startup_pending,None
                response = self.command(operation,Trigger.Response())
                self.get_logger().info(response.message)
        self.publish_status()
        return False

    def publish_status(self):
        now = time.monotonic()
        if now-self.last_status_time >= .1:
            self.state_pub.publish(String(data=json.dumps(self.status(),ensure_ascii=False,allow_nan=False)))
            self.last_status_time = now
        phase,error = self.latest.get('phase'),self.latest.get('error')
        if phase != self.last_phase or error != self.last_error:
            self.get_logger().info(f'phase={phase} busy={self.latest.get("busy")} '+str(error or ''))
            self.last_phase,self.last_error = phase,error
        if now-self.last_log >= 1.:
            self.get_logger().info('q={q_rad} raw={raw_position} grip={grip_label} force_setting={force_percent}% ready={ready}'.format(
                **{k:self.latest.get(k) for k in ('q_rad','raw_position','grip_label','force_percent','ready')}))
            self.last_log = now

    def _release_driver_leases(self):
        for fd in self.driver_leases:
            os.close(fd)
        self.driver_leases.clear()

    def close_tick(self):
        self._retire_approval_service()
        if not self.shutdown_started:
            self.shutdown_started = True
            self.startup_pending = None
            self.backend.request_stop()
        if not self.backend_closed:
            self.latest = self.backend.poll()
            self.publish_status()
            if not self.latest.get('can_close') or self.backend.close() is not True:
                return False
            self.backend_closed = True
        # Leave the owned driver alive while cancellation/fresh-stop proof is
        # pending. Only the verified-safe backend.close() reaches this point.
        if self.driver_process is not None:
            if self.driver_process.poll() is None:
                if not self.driver_stop_sent:
                    try:
                        os.killpg(self.driver_process.pid,signal.SIGINT)
                    except ProcessLookupError:
                        pass  # It exited between poll() and the signal.
                    self.driver_stop_sent = True
                    self.get_logger().info('그리퍼 정지 확인 완료 · 소유한 드라이버 종료 중')
                elif time.monotonic()-self.last_driver_warning > 10.:
                    self.last_driver_warning = time.monotonic()
                    self.get_logger().warning('그리퍼 드라이버의 정상 종료를 기다립니다.')
                return False
        self._release_driver_leases()
        return True


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous = {}
    try:
        node = HeadlessCalibration()
        for signum in (signal.SIGINT,signal.SIGTERM):
            previous[signum] = signal.signal(signum,lambda *_:node.request_shutdown())
        done = False
        while not done:
            rclpy.spin_once(node,timeout_sec=.02)
            done = node.tick()
    except Exception:
        if node is not None:
            node.request_shutdown()
            while True:
                try:
                    if node.close_tick():
                        break
                except Exception as error:
                    node.get_logger().error('정지 확인을 계속 기다립니다: '+str(error))
                try:
                    rclpy.spin_once(node,timeout_sec=.02)
                except Exception:
                    time.sleep(.05)
        raise
    finally:
        for signum,handler in previous.items():
            signal.signal(signum,handler)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
