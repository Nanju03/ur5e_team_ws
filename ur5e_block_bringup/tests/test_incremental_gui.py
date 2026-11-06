"""New plan IDs and partial completion in the GUI; no node or robot process."""
import json
from unittest.mock import Mock

import pytest

from ur5e_block_bringup.incremental_state import POLICY
from test_stack_gui_execution_modes import make_gui, ready, Var


def gui(tmp_path):
    obj = make_gui()
    obj.plan_kind = 'physical'
    obj.incremental_mode = Var(True)
    obj.job_incremental_mode = True
    obj.incremental_finished_info = None
    obj.target = 11
    obj.trajectory_file = tmp_path/'next.recipe.json'
    obj.trajectory_file.with_name(obj.trajectory_file.name+'.meta.json').write_text(json.dumps(dict(
        plan_id='block2', recipe_policy=POLICY, path_kind='physical_stack')))
    return obj


def plan_line(plan='block2'):
    return 'STACK_INCREMENTAL_PLAN '+json.dumps(dict(source=12, layer=3, remaining=20, plan_id=plan))


def test_new_block_resets_step_numbers_only_for_its_saved_metadata(tmp_path):
    obj = gui(tmp_path)
    obj.receive_step_ready(ready(index=1, total=8))
    assert obj.step_latest_index == 1
    assert obj.process_incremental_event(plan_line())
    assert obj.step_latest_index == 0 and obj.step_total is None
    assert obj.job_plan_id == 'block2'
    payload = ready(index=1, total=12)
    payload.update(plan_id='block2', request_id='block2_token', approve_service='/ur5e_a3_stack/approve_next_block2_token')
    obj.receive_step_ready(payload)
    assert obj.step_request['plan_id'] == 'block2'
    assert obj.step_request['total'] == 12
    assert all(not client.calls for client in obj.node.clients)


def test_continuous_next_block_shows_motion_progress_without_a_new_approval(tmp_path):
    obj = gui(tmp_path)
    obj.job_execution_mode = 'continuous'
    obj.process_incremental_event(plan_line())
    obj.process_step_event('STACK_STEP_EXECUTING '+json.dumps(dict(index=1,total=1,label='next block')))
    assert '실행 중' in obj.status.get()
    assert obj.step_request is None
    assert obj.node.clients == []


@pytest.mark.parametrize('case', ['metadata_mismatch', 'stopped', 'unrelated', 'closing', 'failed'])
def test_new_plan_output_cannot_enable_unrelated_or_stopped_approvals(tmp_path, case):
    obj = gui(tmp_path)
    obj.user_stopped = case == 'stopped'
    obj.close_requested = case == 'closing'
    obj.job_incremental_mode = case != 'unrelated'
    obj.job_failed = case == 'failed'
    obj.process_incremental_event(plan_line('unknown' if case == 'metadata_mismatch' else 'block2'))
    assert obj.job_plan_id == 'abc123'
    assert obj.step_request is None
    assert obj.node.clients == []


def test_cpp_diagnostic_cannot_claim_session_completion(tmp_path):
    obj = gui(tmp_path)
    obj.process_incremental_event('STACK_INCREMENTAL_FINISHED reason=no_feasible_direct_pick')
    assert not obj.job_succeeded
    assert obj.incremental_finished_info is None


@pytest.mark.parametrize('reason,remaining', [('no_remaining_blocks', 0), ('no_feasible_direct_pick', 9)])
def test_no_feasible_sources_are_reported_separately_from_all_blocks_complete(tmp_path, reason, remaining):
    obj = gui(tmp_path)
    obj.process_incremental_event('STACK_INCREMENTAL_FINISHED '+json.dumps(
        dict(reason=reason, remaining=remaining, layers=14)))
    assert obj.job_succeeded
    assert obj.incremental_finished_info['remaining'] == remaining
    assert ('적층 완료' in obj.status.get()) == (remaining == 0)
    assert ('직접 집기 경로' in obj.status.get()) == (remaining > 0)


def test_switching_modes_invalidates_preview_and_keeps_whole_stack_file_separate(tmp_path):
    obj = gui(tmp_path)
    obj.full_physical_trajectory_file = tmp_path/'full.recipe.json'
    obj.incremental_trajectory_file = tmp_path/'next.recipe.json'
    obj.invalidate_plan = Mock()
    obj.incremental_mode.set(False)
    obj.incremental_changed()
    obj.invalidate_plan.assert_called_once()
    assert obj.trajectory_file == obj.full_physical_trajectory_file
    obj.incremental_mode.set(True)
    obj.incremental_changed()
    assert obj.trajectory_file == obj.incremental_trajectory_file
