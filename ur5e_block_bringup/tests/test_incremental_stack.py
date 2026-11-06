"""Incremental state, execution and consent contracts; no ROS node or goal."""
import copy
import json
from types import MethodType
from unittest.mock import Mock

import pytest

from ur5e_block_bringup import incremental_state as state
from ur5e_block_bringup import physical_incremental as incremental
from ur5e_block_bringup import physical_stack as physical
from ur5e_block_bringup.physical_motion import incremental_motion_profile, incremental_arm_scaling, SURFACE_CONTACT_POLICY, FAST_LAYERS, LOW_TOWER_SPEED, HIGH_TOWER_SPEED
from test_physical_stack import recipe as full_recipe, execution_validation_fixture


def layout():
    return dict(schema=1, target_cell=11, target_blocks=[11], remaining_sources=[10, 12],
                obstacle_cells=[1], empty_cells=[n for n in range(1, 25) if n not in (1, 10, 11, 12)])


def one_block(before=None, source=10):
    before = before or layout()
    result = full_recipe()
    result.update(routing_policy=state.ROUTING_POLICY, source_cells=[source],
                  target_cell=before['target_cell'], foundation_layers=len(before['target_blocks']),
                  layers=len(before['target_blocks'])+1, post_place_retreat_clearance=.05,
                  incremental_pickup_approach_clearance=.08,
                  incremental_transfer_clearance=.05, incremental_pickup_policy=state.PICKUP_POLICY, incremental_route_policy=state.PICKUP_ROUTE_POLICY,
                  incremental_motion_profile=incremental_motion_profile(), surface_contact_policy=SURFACE_CONTACT_POLICY,
                  incremental_state_before=before, incremental_state_after=state.after_placement(before, source))
    roles = iter(['pickup','carry','place','retreat'])
    for step in result['steps']:
        if step['kind'] == 'arm':
            role = next(roles)
            velocity, acceleration = incremental_arm_scaling(role, result['layers'])
            step.update(motion_role=role, surface_contact_policy=SURFACE_CONTACT_POLICY,
                        phase={'pickup':'transfer','carry':'transfer','place':'approach','retreat':'retreat'}[role],
                        velocity_scaling=velocity, acceleration_scaling=acceleration)
    previous=result['initial_joints'][:]
    for index,step in enumerate(result['steps'],1):
        step.update(index=index,file=f'step{index}.cdr',start_joints=previous[:])
        end=previous[:]
        if step['kind']=='arm':end[0]+=.01
        elif step['kind']=='hand':end[-1]=step['hand_target']
        step['end_joints']=end;previous=end
        if step['phase'] == 'place':
            step.update(block_id=source, destination_cell=before['target_cell'])
    return result


def test_one_block_recipe_retains_every_unpicked_block_and_the_complete_tower():
    first = one_block()
    towers, after = state.recipe_layout(first)
    assert towers == {11: [11], 10: [10], 12: [12]}
    assert after['target_blocks'] == [11, 10]
    assert physical.validate_recipe(first) == first['steps']
    second = one_block(after, 12)
    assert state.recipe_layout(second)[0] == {11: [11, 10], 12: [12]}
    assert physical.validate_recipe(second) == second['steps']
    assert second['incremental_state_after']['remaining_sources'] == []


@pytest.mark.parametrize('layer,expected', [(2,LOW_TOWER_SPEED), (FAST_LAYERS,LOW_TOWER_SPEED), (FAST_LAYERS+1,HIGH_TOWER_SPEED), (FAST_LAYERS+2,HIGH_TOWER_SPEED)])
def test_layer_profile_slows_loaded_transport_and_keeps_empty_pickup_fast(layer, expected):
    before = dict(schema=1,target_cell=11,target_blocks=[11]+list(range(30,layer+28)),
                  remaining_sources=[10],obstacle_cells=[1],empty_cells=[])
    item = one_block(before)
    physical.validate_recipe(item)
    motions = {s['motion_role']:s for s in item['steps'] if s['kind']=='arm'}
    for role in ('carry','retreat'):
        assert tuple(motions[role][k] for k in ('velocity_scaling','acceleration_scaling')) == expected
    assert motions['pickup']['velocity_scaling'] == motions['pickup']['acceleration_scaling'] == 1.
    assert motions['place']['velocity_scaling'] == motions['place']['acceleration_scaling'] == .10


@pytest.mark.parametrize('field,value', [('velocity_scaling',1.),('acceleration_scaling',1.),
                                        ('surface_contact_policy','unbounded'),('motion_role','pickup')])
def test_high_tower_recipe_cannot_reuse_fast_or_unbounded_carry(field,value):
    before = dict(schema=1,target_cell=11,target_blocks=[11]+list(range(30,30+FAST_LAYERS-1)),
                  remaining_sources=[10],obstacle_cells=[1],empty_cells=[])
    item = one_block(before)
    next(s for s in item['steps'] if s.get('motion_role')=='carry')[field] = value
    with pytest.raises(ValueError):
        physical.validate_recipe(item)


def test_motion_role_and_surface_policy_are_bound_to_collision_certificate():
    from ur5e_block_bringup.physical_replay import motion_context
    step = next(s for s in one_block()['steps'] if s['kind']=='arm')
    baseline = motion_context(step)
    for key in ('motion_role','surface_contact_policy'):
        modified = dict(step, **{key:'modified'})
        assert motion_context(modified) != baseline


@pytest.mark.parametrize('change', [
    {'schema': True}, {'target_cell': True}, {'target_blocks': [10, 11]},
    {'remaining_sources': [10, 10]}, {'remaining_sources': [10, 11]},
    {'remaining_sources': [10, False]}, {'empty_cells': [1]}, {'obstacle_cells': [101]},
])
def test_invalid_or_overlapping_roles_cannot_enter_a_session(change):
    current = layout()
    current.update(change)
    with pytest.raises(ValueError):
        state.validate_state(current)


@pytest.mark.parametrize('key,value', [
    ('moves', 2), ('transport_moves', 2), ('buffer_moves', 1), ('source_cells', [10, 12]),
    ('layers', 23), ('foundation_layers', 0), ('target_cell', 6),
    ('post_place_retreat_clearance', .12), ('incremental_pickup_approach_clearance', .16),
    ('incremental_transfer_clearance', .12), ('incremental_pickup_policy', 'split_pickup'),
    ('incremental_route_policy', 'wide_side_staging_first'),
])
def test_whole_stack_buffer_or_changed_clearance_is_not_an_incremental_recipe(key, value):
    recipe = one_block()
    recipe[key] = value
    with pytest.raises(ValueError):
        physical.validate_recipe(recipe)


@pytest.mark.parametrize('change', ['split_descent','split_lift','side_staging','missing_lift'])
def test_double_pickup_stops_and_carrying_detours_are_rejected_before_execution(change):
    recipe=one_block()
    steps=recipe['steps']
    grasp=next(i for i,s in enumerate(steps) if s['phase']=='grasp')
    lift=next(i for i,s in enumerate(steps) if s.get('motion_role')=='carry')
    if change=='missing_lift':
        del steps[lift]
    else:
        index=grasp if change=='split_descent' else lift+1
        duplicate=copy.deepcopy(steps[index-1])
        duplicate['kind']='arm'
        duplicate['phase']={'split_descent':'approach','split_lift':'contact','side_staging':'transfer'}[change]
        steps.insert(index,duplicate)
    with pytest.raises(ValueError,match='one continuous pickup'):
        state.recipe_layout(recipe)


def test_after_state_cannot_silently_lose_unpicked_blocks_or_move_an_obstacle():
    for key, value in [('remaining_sources', []), ('obstacle_cells', []), ('target_blocks', [11, 12])]:
        recipe = one_block()
        recipe['incremental_state_after'][key] = value
        with pytest.raises(ValueError):
            state.recipe_layout(recipe)


@pytest.mark.parametrize('changed', ['inputs', 'model', 'controller', 'profile', 'target', 'tower'])
def test_session_consent_does_not_authorize_a_changed_configuration_or_scene(changed):
    context = dict(inputs={'selection': 'a'}, model='b', controller='c', profile='d')
    recipe = one_block()
    consent = state.SessionAuthorization(context, layout())
    if changed in ('target', 'tower'):
        before = layout()
        if changed == 'target':
            before['target_cell'], before['target_blocks'] = 6, [6]
            before['empty_cells'].remove(6)
            before['empty_cells'].append(11)
        else:
            before = state.after_placement(before, 10)
        recipe = one_block(before, 12)
    else:
        context[changed] = {'selection': 'changed'} if changed == 'inputs' else 'changed'
    with pytest.raises(ValueError, match='configuration or committed scene'):
        consent.check(context, recipe)


def test_only_successful_sequential_commit_advances_consent():
    context = {'model': 'same'}
    first = one_block()
    consent = state.SessionAuthorization(context, layout())
    consent.check(context, first)
    assert consent.expected == layout()
    consent.advance(first)
    with pytest.raises(ValueError, match='skipped or repeated'):
        consent.advance(first)
    consent.check(context, one_block(consent.expected, 12))
    assert first['incremental_state_before'] == layout()


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_real_recipe_executor_preserves_motion_gates_in_both_modes(monkeypatch, mode):
    mock_task, _, events = execution_validation_fixture(monkeypatch, mode)
    task = mock_task
    task.approve_continuous_recipe = MethodType(incremental.IncrementalStackTask.approve_continuous_recipe, task)
    value = one_block()
    for step in value['steps']:
        step.update(scene_before_sha256='same', scene_after_sha256='same')
    task.load_recipe = lambda: (dict(scene_sha256='same', plan_id='block', input_sha256={}, hardware_class='mock'), value)
    task.incremental_authorization = None
    task.authorization_context = lambda metadata: {'model': 'unchanged'}
    physical.PhysicalStackTask.execute(task)
    count = sum(s['kind'] != 'scene' for s in value['steps'])
    assert sum(e[0] == 'approval' for e in events) == (count if mode == 'step' else 1)
    assert sum(e[0] == 'motion' for e in events) == count
    task.live_arm_validation.assert_not_called()
    assert (task.incremental_authorization is not None) == (mode == 'continuous')


def loop_task(tmp_path):
    task = incremental.IncrementalStackTask.__new__(incremental.IncrementalStackTask)
    task.check_stop = Mock()
    task.carrying, task.release_stage = False, 0
    task.incremental_authorization = None
    task.expected_incremental_state = None
    task.incremental_journal = tmp_path/'journal.jsonl'
    task.model = '<plain_robot/>'
    return task


def test_each_placement_finishes_before_next_plan_and_preserves_committed_layout(monkeypatch, tmp_path, capsys):
    task = loop_task(tmp_path)
    first = one_block()
    second = one_block(first['incremental_state_after'], 12)
    events = []
    task.active_recipe = first
    task.incremental_authorization = state.SessionAuthorization({}, layout())
    def plan():
        assert task.expected_incremental_state == first['incremental_state_after']
        events.append('plan12')
        task.active_recipe = second
    task.plan = plan
    monkeypatch.setattr(physical.PhysicalStackTask, 'execute',
                        lambda self: events.append('execute'+str(self.active_recipe['source_cells'][0])))
    task.execute()
    assert events == ['execute10', 'plan12', 'execute12']
    assert task.check_stop.call_count == 2
    records = [json.loads(line) for line in task.incremental_journal.read_text().splitlines()]
    assert [r['source'] for r in records] == [10, 12]
    assert task.incremental_authorization.expected['target_blocks'] == [11, 10, 12]
    assert all(r['timer_enabled'] is False for r in records)
    assert '"reason": "no_remaining_blocks"' in capsys.readouterr().out


@pytest.mark.parametrize('failure', ['execute', 'held', 'release', 'stop'])
def test_failure_or_stop_never_starts_the_next_plan_or_commits_progress(monkeypatch, tmp_path, failure):
    task = loop_task(tmp_path)
    task.active_recipe = one_block()
    task.plan = Mock(side_effect=AssertionError('Next plan must not run'))
    task.incremental_authorization = state.SessionAuthorization({}, layout())
    def execute(self):
        if failure == 'execute':
            raise RuntimeError('failed action')
        task.carrying = failure == 'held'
        task.release_stage = 1 if failure == 'release' else 0
    monkeypatch.setattr(physical.PhysicalStackTask, 'execute', execute)
    if failure == 'stop':
        task.check_stop.side_effect = RuntimeError('stop requested')
    with pytest.raises(RuntimeError):
        task.execute()
    task.plan.assert_not_called()
    assert task.expected_incremental_state is None
    assert task.incremental_authorization.expected == layout()
    assert not task.incremental_journal.exists()


def test_auto_repeat_approves_once_and_rechecks_context_before_each_new_block():
    task = incremental.IncrementalStackTask.__new__(incremental.IncrementalStackTask)
    task.incremental_authorization = None
    task.values = {physical.HAND: .4}
    task.authorization_context = lambda metadata: metadata['context']
    task.await_approval = Mock()
    task.replay_checkpoint = Mock()
    first = one_block()
    metadata = {'context': {'model': 'same'}}
    task.approve_continuous_recipe(metadata, first, first['steps'], 'scene')
    assert task.await_approval.call_count == 1
    assert '시간 제한 없음' in task.await_approval.call_args.args[5]
    task.incremental_authorization.advance(first)
    second = one_block(first['incremental_state_after'], 12)
    task.approve_continuous_recipe(metadata, second, second['steps'], 'nextscene')
    assert task.await_approval.call_count == 1
    task.replay_checkpoint.assert_called_with(metadata, second['steps'][0], 'nextscene')
    with pytest.raises(ValueError):
        task.approve_continuous_recipe({'context': {'model': 'changed'}}, second, second['steps'], 'nextscene')


def test_next_plan_retires_monitor_queries_without_destroying_action_clients(monkeypatch, tmp_path):
    task = loop_task(tmp_path)
    task.path = tmp_path/'next.recipe.json'
    task.meta_path = tmp_path/'next.meta.json'
    task.replay_queries = {'old': object()}
    task.destroy_flight_queries = Mock()
    task.close_monitor_clients = Mock(side_effect=AssertionError('Retain action clients'))
    recipe = one_block()
    def plan(self):
        assert self.replay_queries is None and not self.prevalidated_replay
        self.path.write_text(json.dumps(recipe | {'duration': 12.}))
        self.meta_path.write_text(json.dumps({'plan_id': 'new'}))
    monkeypatch.setattr(physical.PhysicalStackTask, 'plan', plan)
    task.plan()
    task.destroy_flight_queries.assert_called_once()
    task.close_monitor_clients.assert_not_called()


def test_no_feasible_path_reports_remaining_instead_of_false_complete(monkeypatch, tmp_path, capsys):
    task = loop_task(tmp_path)
    def run(self):
        raise state.IncrementalFinished('no_feasible_direct_pick', layout())
    monkeypatch.setattr(physical.PhysicalStackTask, 'run', run)
    task.run()
    data = json.loads(capsys.readouterr().out.split('STACK_INCREMENTAL_FINISHED ')[1])
    assert data == dict(reason='no_feasible_direct_pick', layers=1, remaining=2, timer_enabled=False)
