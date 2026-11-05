"""Current hover/descent policy, phase timing and legacy-path exclusion."""
import json
from types import SimpleNamespace

import pytest

from ur5e_block_bringup import physical_stack as physical
from test_physical_stack import recipe, arm_path


@pytest.mark.parametrize('field,value', [
    ('placement_policy', 'raised_tower_clearance_v1'),
    ('side_entry_clearance', None), ('side_entry_clearance', 0.),
    ('side_entry_clearance', .005), ('side_entry_clearance', .02),
    ('side_entry_clearance', float('nan')), ('side_entry_clearance', True),
    ('physical_approach_clearance', None), ('physical_approach_clearance', .02),
    ('physical_approach_clearance', float('inf')),
    ('descent_velocity_scaling', .05), ('descent_acceleration_scaling', 1.),
])
def test_old_or_changed_hover_descent_contract_is_rejected_before_ros(field, value):
    item = recipe()
    item[field] = value
    with pytest.raises(ValueError):
        physical.validate_recipe(item)


@pytest.mark.parametrize('scaling', [.05, 1.])
def test_fast_descent_cannot_silently_use_contact_or_transfer_timing(scaling):
    item = recipe()
    step = item['steps'][1]
    step.update(phase='descent_fast', velocity_scaling=scaling, acceleration_scaling=scaling)
    with pytest.raises(ValueError, match='phase'):
        physical.validate_recipe(item)


def test_fast_descent_is_a_distinct_bounded_phase_and_keeps_controller_precision():
    item = recipe()
    item['steps'][1].update(phase='descent_fast', velocity_scaling=.20, acceleration_scaling=.20)
    physical.validate_recipe(item)
    profile = physical.arm_execution_profile()
    assert physical.ARM_SPEEDS['descent_fast'] == .20
    assert physical.ARM_SPEEDS['approach'] == physical.ARM_SPEEDS['contact'] == .05
    assert profile['phase_path_position']['descent_fast'] == [.1]*6
    assert profile['goal_position'] == profile['start_position'] == .001
    assert profile['goal_velocity'] == .01


def test_exact_quintic_is_retimed_to_twenty_percent_velocity_and_acceleration():
    task = SimpleNamespace(joint_velocity_limits={name: 1. for name in physical.ARM},
                           joint_acceleration_limits={name: 1. for name in physical.ARM},
                           stretch=physical.AirTask.stretch)
    path, metrics = physical.PhysicalStackTask.normalize_arm(
        task, arm_path(), dict(velocity_scaling=.20, acceleration_scaling=.20))
    assert max(metrics['joint_velocity_maxima']) <= .200001
    assert max(metrics['joint_acceleration_maxima']) <= .200001
    for point in (path.joint_trajectory.points[0], path.joint_trajectory.points[-1]):
        assert max(abs(v) for v in list(point.velocities)+list(point.accelerations)) == 0.


@pytest.mark.parametrize('preparing', [False, True])
@pytest.mark.parametrize('policy', ['physical_fast_transfer_local_validation_v13', 'physical_prevalidated_replay_v14',
                                   'physical_hover_setdown_fast_descent_v15'])
def test_old_stored_path_cannot_bypass_the_new_placement_policy(tmp_path, monkeypatch, preparing, policy):
    path = tmp_path/'recipe.json'
    path.write_text('{}')
    metadata = dict(schema=1, path_kind='physical_stack', scene_hash_policy=physical.SCENE_HASH_POLICY,
                    recipe_policy=policy)
    metadata['plan_id'] = physical.digest(json.dumps(metadata, sort_keys=True).encode())[:16]
    meta_path = tmp_path/'meta.json'
    meta_path.write_text(json.dumps(metadata))
    task = SimpleNamespace(path=path, meta_path=meta_path, param=lambda _: True)
    monkeypatch.setattr(physical, 'read_held_fault', lambda: None)
    with pytest.raises(RuntimeError, match='다시 계획'):
        physical.PhysicalStackTask.load_recipe(task, preparing=preparing)


@pytest.mark.parametrize('policy', [None, 'farthest_clearance', 'unknown'])
def test_recipe_must_record_the_near_target_buffer_selection_policy(policy):
    item = recipe()
    if policy is None:
        item.pop('buffer_selection_policy')
    else:
        item['buffer_selection_policy'] = policy
    with pytest.raises(ValueError, match='nearest-target'):
        physical.validate_recipe(item)
