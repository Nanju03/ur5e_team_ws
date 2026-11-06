"""Plan, certify and execute one block at a time, preserving the committed scene.

Continuous mode requires explicit consent to the repeating session on its first
block. Every subsequent block still passes the existing geometric certificate,
hardware/model/scene/start checks and live Stop/watchdog gates. There is no
competition timer or automatic time-budget cutoff.
"""
import copy
import json
import os
import time
import uuid

import rclpy
from rclpy.serialization import serialize_message
from moveit_msgs.msg import PlanningScene

from .air_task import digest, atomic_write, ARM, START_TOLERANCE
from .physical_stack import PhysicalStackTask, HAND, JOINTS, scene_fingerprint, validate_recipe
from .physical_motion import incremental_motion_profile, SURFACE_CONTACT_POLICY
from .incremental_state import (POLICY, IncrementalFinished, SessionAuthorization,
                                recipe_layout, validate_state)
from .incremental_prefetch import PrefetchJob
from . import physical_replay as replay


class IncrementalStackTask(PhysicalStackTask):
    def __init__(self):
        super().__init__()
        if not self.has_parameter('incremental_stack') or self.param('incremental_stack') is not True:
            raise ValueError('The incremental executor requires incremental_stack:=true')
        self.expected_incremental_state = None
        self.incremental_authorization = None
        self.prefetch_job = None
        self.prefetch_enabled = (self.param('incremental_prefetch')
            if self.has_parameter('incremental_prefetch') else True)
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        self.incremental_journal = self.path.parent/f'incremental_session_{domain}_{uuid.uuid4().hex}.jsonl'

    def recipe_policy(self):
        return POLICY

    def package_context(self, inputs, hardware):
        return dict(super().package_context(inputs, hardware),
                    incremental_motion_profile=incremental_motion_profile(),
                    surface_contact_policy=SURFACE_CONTACT_POLICY)

    def validation_worker(self, bounded_contacts):
        if not bounded_contacts:
            raise RuntimeError('한 블록 연속 경로는 접촉 깊이 검사가 필요합니다.')
        worker = getattr(self, '_validation_worker', None)
        if worker is None:
            self._validation_worker = super().validation_worker(True)
        return self._validation_worker

    def release_validation_worker(self, worker):
        # Reuse the local model across blocks; it has no robot endpoints.
        pass

    def close_monitor_clients(self):
        try:
            self.cancel_prefetch()
            super().close_monitor_clients()
        finally:
            worker = getattr(self, '_validation_worker', None)
            if worker is not None:
                worker.close()
                self._validation_worker = None

    def planner(self, output):
        super().planner(output)
        result = json.loads(output.read_text())
        if result.get('incremental_status') in ('no_remaining_blocks', 'no_feasible_direct_pick'):
            raise IncrementalFinished(result['incremental_status'], result['state'])
        recipe_layout(result)
        if (self.expected_incremental_state is not None
                and result['incremental_state_before'] != self.expected_incremental_state):
            raise RuntimeError('현재 장면의 탑·남은 블록이 직전 실행 결과와 다릅니다. 자동 반복을 중지합니다.')

    def plan(self):
        # Only the foreground accesses ROS clients/session artifacts. The
        # speculative worker owns a separate directory and a private scene.
        if self.replay_queries is not None:
            self.destroy_flight_queries(self.replay_queries)
            self.replay_queries = None
        self.prevalidated_replay = False
        self.execution_artifacts = None
        if not self.consume_prefetch():
            super().plan()
        recipe = json.loads(self.path.read_text())
        recipe_layout(recipe)
        if (self.expected_incremental_state is not None
                and recipe['incremental_state_before'] != self.expected_incremental_state):
            raise RuntimeError('저장된 한 블록 경로의 시작 상태가 직전 적층 결과와 다릅니다.')
        print('STACK_INCREMENTAL_PLAN '+json.dumps(dict(
            plan_id=json.loads(self.meta_path.read_text())['plan_id'],
            source=recipe['source_cells'][0], layer=recipe['layers'],
            remaining=len(recipe['incremental_state_after']['remaining_sources']),
            duration=recipe['duration'], timer_enabled=False)), flush=True)

    def cancel_prefetch(self):
        job = getattr(self, 'prefetch_job', None)
        self.prefetch_job = None
        if job is not None:
            job.close()

    def on_recipe_execution_ready(self, metadata, recipe):
        # Step mode intentionally remains sequential. Start only after the
        # first continuous-session approval and its current-state checkpoint.
        if (not getattr(self, 'prefetch_enabled', False) or self.execution_mode != 'continuous'
                or not recipe['incremental_state_after']['remaining_sources']):
            return
        self.cancel_prefetch()
        directory = self.path.parent/'incremental_prefetch'/uuid.uuid4().hex
        inputs = directory.parent/(directory.name+'_inputs')
        inputs.mkdir(parents=True, exist_ok=False)
        final = recipe['steps'][-1]
        scene = self.read_message(final['scene_after_file'], PlanningScene)
        if scene_fingerprint(scene) != final['scene_after_sha256']:
            raise RuntimeError('다음 블록 예측 장면이 현재 경로의 검증된 완료 장면과 다릅니다.')
        state = copy.deepcopy(scene.robot_state)
        state.is_diff = False
        state.joint_state.name = list(JOINTS)
        state.joint_state.position = [float(v) for v in final['end_joints']]
        state.joint_state.velocity = [0.]*len(JOINTS)
        state.joint_state.effort = []
        scene_file, state_file = inputs/'scene.cdr', inputs/'state.cdr'
        atomic_write(scene_file, serialize_message(scene))
        atomic_write(state_file, serialize_message(state))
        parameters = {name: parameter.value for name, parameter
                      in self.get_parameters_by_prefix('').items()}
        # A bounded low-priority worker leaves CPU time for real feedback.
        parameters['planner_workers'] = 2
        request = dict(planner_parameters=parameters,
            context=self.authorization_context(metadata), bounds=self.bounds,
            joint_velocity_limits=self.joint_velocity_limits,
            joint_acceleration_limits=self.joint_acceleration_limits,
            expected_state=copy.deepcopy(recipe['incremental_state_after']),
            expected_scene_sha256=final['scene_after_sha256'],
            scene_file=str(scene_file), state_file=str(state_file))
        try:
            self.prefetch_job = PrefetchJob(directory, request)
        except OSError as error:
            print('STACK_PREFETCH_FALLBACK '+json.dumps(dict(reason=str(error))), flush=True)
            return
        print('STACK_PREFETCH_STARTED '+json.dumps(dict(layer=recipe['layers']+1,
            planner_workers=2, directory=str(directory), robot_commands=0)), flush=True)

    def consume_prefetch(self):
        job = getattr(self, 'prefetch_job', None)
        if job is None:
            return False
        began = time.monotonic()
        try:
            while job.running():
                self.check_stop()
                if time.monotonic()-job.started > 600.:
                    print('STACK_PREFETCH_FALLBACK '+json.dumps(dict(
                        reason='offline preparation exceeded 600 seconds', directory=str(job.directory))), flush=True)
                    return False
                rclpy.spin_once(self, timeout_sec=.02)
            self.check_stop()
            try:
                result = job.result()
            except (RuntimeError, ValueError, OSError) as error:
                print('STACK_PREFETCH_FALLBACK '+json.dumps(dict(reason=str(error),
                    directory=str(job.directory))), flush=True)
                return False
            if self.carrying or self.release_stage:
                raise RuntimeError('블록 놓기 완료를 확인하기 전에는 다음 경로를 사용할 수 없습니다.')
            if job.request['expected_state'] != self.expected_incremental_state:
                raise RuntimeError('다음 경로의 예측 상태가 실제 완료한 적층 상태와 다릅니다.')
            self.spin_until(self.fresh)
            self.check_model()
            hardware = self.hardware(require_controller=True)
            self.gripper_runtime()
            context = self.package_context(self.input_hashes(), hardware)
            if context != job.request['context']:
                raise RuntimeError('다음 경로 계산 중 배치·모델·실행 설정이 바뀌었습니다.')
            if (job.request['joint_velocity_limits'] != self.joint_velocity_limits
                    or job.request['joint_acceleration_limits'] != self.joint_acceleration_limits
                    or {k: list(v) for k, v in job.request['bounds'].items()}
                        != {k: list(v) for k, v in self.bounds.items()}):
                raise RuntimeError('다음 경로 계산 중 관절 속도·가속도·위치 한계가 바뀌었습니다.')
            live_hash = scene_fingerprint(self.scene())
            if live_hash != job.request['expected_scene_sha256']:
                raise RuntimeError('실제 완료 장면이 다음 경로를 계산한 예측 장면과 다릅니다.')
            if not self.measured_stop() or not self.sensor_fresh():
                raise RuntimeError('다음 경로 시작 전 팔·그리퍼 정지와 최신 피드백을 확인하지 못했습니다.')
            if result['status'] == 'finished':
                if (result.get('state') != self.expected_incremental_state
                        or result.get('reason') not in ('no_remaining_blocks', 'no_feasible_direct_pick')):
                    raise RuntimeError('다음 경로의 종료 상태가 실제 완료한 적층 상태와 다릅니다.')
                raise IncrementalFinished(result['reason'], result['state'])
            metadata, recipe, payloads = replay.read_package(job.directory)
            validate_recipe(recipe)
            recipe_layout(recipe)
            replay.check_certificates(recipe, metadata)
            if (any(metadata.get(key) != value for key, value in context.items())
                    or metadata['scene_sha256'] != live_hash
                    or recipe['incremental_state_before'] != self.expected_incremental_state):
                raise RuntimeError('사전 계산 경로의 설정·장면·블록 순서가 현재 작업과 다릅니다.')
            if (any(abs(self.values[name]-q) > START_TOLERANCE
                    for name, q in zip(ARM, recipe['initial_joints']))
                    or not replay.initial_hand_matches(recipe, self.values[HAND], self.bounds[HAND])):
                # A completed action is still required to arrive at its exact
                # reviewed boundary; never shift or stretch a saved joint path.
                raise RuntimeError('팔·그리퍼가 사전 계산 경로의 시작 자세와 다릅니다.')
            self.acquire_stack_lock()
            destination = replay.save_package(self.library, metadata, recipe, payloads)
            installed, recipe = self.install_session(recipe, payloads, context, live_hash,
                source_plan_id=metadata['plan_id'], source_package=str(destination), collision_rechecks=0)
            self.announce_plan(installed, recipe)
            print('STACK_PREFETCH_HIT '+json.dumps(dict(layer=recipe['layers'],
                wait_seconds=round(time.monotonic()-began, 3),
                planning_timing=recipe.get('planning_timing'),
                validated_samples=recipe['validated_samples'], collision_rechecks=0)), flush=True)
            return True
        finally:
            self.cancel_prefetch()

    def authorization_context(self, metadata):
        return self.package_context(metadata['input_sha256'], metadata['hardware_class'])

    def approve_continuous_recipe(self, metadata, recipe, steps, expected_scene):
        recipe_layout(recipe)
        context = self.authorization_context(metadata)
        if self.incremental_authorization is None:
            approval_metadata = dict(metadata, gripper_position=self.values[HAND])
            self.await_approval(lambda: self.recipe_display(recipe, steps), approval_metadata, None, 1, 1,
                '한 블록씩 자동 적층 시작 · 이후 같은 배치에서 계획·검사 후 반복 실행 · 시간 제한 없음',
                context=lambda *_: self.replay_checkpoint(metadata, steps[0], expected_scene))
            self.incremental_authorization = SessionAuthorization(context, recipe['incremental_state_before'])
        self.incremental_authorization.check(context, recipe)
        self.replay_checkpoint(metadata, steps[0], expected_scene)

    def execute(self):
        try:
            self.execute_blocks()
        finally:
            # No speculative result can survive a Stop, aborted arm action,
            # uncertain placement, or a completed incremental session.
            self.cancel_prefetch()

    def execute_blocks(self):
        first = True
        while True:
            self.check_stop()
            if not first:
                print('STACK_INCREMENTAL_PLANNING_NEXT', flush=True)
                self.plan()
            # The base executor enforces all certified step/scene/feedback
            # checks. Our approval hook only scopes autonomous repetition.
            super().execute()
            recipe = self.active_recipe
            recipe_layout(recipe)
            if self.carrying or self.release_stage:
                raise RuntimeError('한 블록 실행이 빈 그리퍼 상태로 완료되지 않았습니다.')
            if self.incremental_authorization is not None:
                self.incremental_authorization.advance(recipe)
            self.expected_incremental_state = copy.deepcopy(recipe['incremental_state_after'])
            record = dict(time=time.time(), policy=POLICY, target_cell=recipe['target_cell'],
                source=recipe['source_cells'][0], layers=recipe['layers'],
                state=self.expected_incremental_state, model_sha256=digest(self.model.encode()),
                timer_enabled=False)
            with self.incremental_journal.open('a') as stream:
                stream.write(json.dumps(record, sort_keys=True)+'\n')
                stream.flush()
            print('STACK_INCREMENTAL_PROGRESS '+json.dumps(dict(
                layers=recipe['layers'], completed=len(self.expected_incremental_state['target_blocks'])-1,
                remaining=len(self.expected_incremental_state['remaining_sources']),
                source=recipe['source_cells'][0], journal=str(self.incremental_journal))), flush=True)
            if not self.expected_incremental_state['remaining_sources']:
                self.finish_incremental('no_remaining_blocks', self.expected_incremental_state)
                return
            first = False

    @staticmethod
    def finish_incremental(reason, state):
        state = validate_state(state)
        print('STACK_INCREMENTAL_FINISHED '+json.dumps(dict(reason=reason,
            layers=len(state['target_blocks']), remaining=len(state['remaining_sources']),
            timer_enabled=False)), flush=True)

    def run(self):
        try:
            super().run()
        except IncrementalFinished as done:
            if self.expected_incremental_state is not None and done.state != self.expected_incremental_state:
                raise RuntimeError('다음 경로의 종료 상태가 직전 실행 결과와 다릅니다.')
            self.finish_incremental(done.reason, done.state)


def main():
    from .physical_stack import main as run_physical
    run_physical(task_class=IncrementalStackTask)
