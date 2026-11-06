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
    ('physical_approach_clearance', None), ('physical_approach_clearance', .04),
    ('physical_approach_clearance', float('inf')),
    ('physical_contact_clearance', .04), ('physical_contact_clearance', .01),
    ('descent_velocity_scaling', .20), ('descent_acceleration_scaling', .20),
    ('descent_velocity_scaling', .05), ('descent_acceleration_scaling', 1.),
    ('approach_acceleration_scaling', .05),
])
def test_old_or_changed_hover_descent_contract_is_rejected_before_ros(field, value):
    item = recipe()
    item[field] = value
    with pytest.raises(ValueError):
        physical.validate_recipe(item)


@pytest.mark.parametrize('scaling', [.05, .10, .20, 1.])
def test_fast_descent_cannot_silently_use_contact_or_transfer_timing(scaling):
    item = recipe()
    step = item['steps'][1]
    step.update(phase='descent_fast', velocity_scaling=scaling, acceleration_scaling=scaling)
    with pytest.raises(ValueError, match='phase'):
        physical.validate_recipe(item)


def test_fast_descent_is_a_distinct_bounded_phase_and_keeps_controller_precision():
    item = recipe()
    item['steps'][1].update(phase='descent_fast', velocity_scaling=.50, acceleration_scaling=.50)
    physical.validate_recipe(item)
    profile = physical.arm_execution_profile()
    assert physical.ARM_SPEEDS['descent_fast'] == .50
    assert physical.ARM_SPEEDS['approach'] == physical.ARM_SPEEDS['contact'] == .10
    assert profile['phase_path_position']['descent_fast'] == [.1]*6
    assert profile['goal_position'] == profile['start_position'] == .001
    assert profile['goal_velocity'] == .01


@pytest.mark.parametrize('scaling', [.10, .50, 1.])
def test_exact_quintic_respects_the_higher_phase_velocity_and_acceleration_limits(scaling):
    task = SimpleNamespace(joint_velocity_limits={name: 1. for name in physical.ARM},
                           joint_acceleration_limits={name: 1. for name in physical.ARM},
                           stretch=physical.AirTask.stretch)
    path, metrics = physical.PhysicalStackTask.normalize_arm(
        task, arm_path(), dict(velocity_scaling=scaling, acceleration_scaling=scaling))
    assert max(metrics['joint_velocity_maxima']) <= scaling+1e-6
    assert max(metrics['joint_acceleration_maxima']) <= scaling+1e-6
    for point in (path.joint_trajectory.points[0], path.joint_trajectory.points[-1]):
        assert max(abs(v) for v in list(point.velocities)+list(point.accelerations)) == 0.


@pytest.mark.parametrize('old,new', [(.05, .10), (.20, .50)])
def test_higher_profile_shortens_the_same_motion_within_its_scaled_joint_limits(old, new):
    task = SimpleNamespace(joint_velocity_limits={name: 1. for name in physical.ARM},
                           joint_acceleration_limits={name: 1. for name in physical.ARM},
                           stretch=physical.AirTask.stretch)
    durations = []
    for scaling in (old, new):
        path, metrics = physical.PhysicalStackTask.normalize_arm(
            task, arm_path(), dict(velocity_scaling=scaling, acceleration_scaling=scaling))
        assert max(metrics['joint_velocity_maxima']) <= scaling+1e-6
        assert max(metrics['joint_acceleration_maxima']) <= scaling+1e-6
        durations.append(physical.trajectory_duration(path))
    assert durations[1] < durations[0]


@pytest.mark.parametrize('field', ['velocity_scaling', 'acceleration_scaling'])
@pytest.mark.parametrize('scaling', [0., -.1, 1.01, float('nan'), float('inf'), True])
def test_higher_phase_policy_still_rejects_invalid_scaling_before_ros(field, scaling):
    item = recipe()
    item['steps'][1][field] = scaling
    with pytest.raises(ValueError, match='phase'):
        physical.validate_recipe(item)


@pytest.mark.parametrize('preparing', [False, True])
@pytest.mark.parametrize('policy', ['physical_fast_transfer_local_validation_v13', 'physical_prevalidated_replay_v14',
                                   'physical_hover_setdown_fast_descent_v15',
                                   'physical_nearest_target_buffer_v16', 'physical_nearest_target_buffer_v17',
                                   'physical_incremental_retreat50_v17'])
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
