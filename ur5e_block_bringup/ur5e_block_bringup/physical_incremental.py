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

from .air_task import digest
from .physical_stack import PhysicalStackTask, HAND
from .physical_motion import incremental_motion_profile, SURFACE_CONTACT_POLICY
from .incremental_state import (POLICY, IncrementalFinished, SessionAuthorization,
                                recipe_layout, validate_state)


class IncrementalStackTask(PhysicalStackTask):
    def __init__(self):
        super().__init__()
        if not self.has_parameter('incremental_stack') or self.param('incremental_stack') is not True:
            raise ValueError('The incremental executor requires incremental_stack:=true')
        self.expected_incremental_state = None
        self.incremental_authorization = None
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
        # Planning is sequential with motion. Retire read-only monitoring
        # queries before temporary validation snapshots, retaining clients.
        if self.replay_queries is not None:
            self.destroy_flight_queries(self.replay_queries)
            self.replay_queries = None
        self.prevalidated_replay = False
        self.execution_artifacts = None
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
