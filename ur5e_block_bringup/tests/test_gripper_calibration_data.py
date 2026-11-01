"""Real AG95 evidence and native geometry checks without ROS nodes or devices."""
import ast
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from urllib.parse import urlsplit

import pytest
import yaml


SOURCE = Path(__file__).resolve().parents[1] / 'ur5e_block_bringup'
REPOSITORY = SOURCE.parents[1]
PACKAGE_NAME = 'ur5e_gripper_calibration_data_test_package'
package_spec = importlib.util.spec_from_file_location(
    PACKAGE_NAME, SOURCE / '__init__.py', submodule_search_locations=[str(SOURCE)])
package = importlib.util.module_from_spec(package_spec)
sys.modules[PACKAGE_NAME] = package
package_spec.loader.exec_module(package)
data_spec = importlib.util.spec_from_file_location(
    PACKAGE_NAME + '.gripper_calibration_data', SOURCE / 'gripper_calibration_data.py')
data = importlib.util.module_from_spec(data_spec)
sys.modules[data_spec.name] = data
data_spec.loader.exec_module(data)

# Exercise the checkout's exact loader function without importing its ROS
# backend module. In particular, collection cannot create a Node or executor.
backend_tree = ast.parse((SOURCE / 'gripper_calibration.py').read_text())
loader_function = next(node for node in backend_tree.body
                       if isinstance(node, ast.FunctionDef) and node.name == 'load_calibration_profile')
PLUGIN = 'dh_gripper_driver/DHGripperHardwareInterface'
namespace = dict(Path=Path, yaml=yaml, json=json, math=math, PLUGIN=PLUGIN,
                 PROFILE=Path('/unused/gripper_calibration.yaml'),
                 validate_settings=data.validate_settings, recommend=data.recommend,
                 sha=lambda value: hashlib.sha256(value).hexdigest())
exec(compile(ast.Module(body=[loader_function], type_ignores=[]), '<actual profile loader>', 'exec'), namespace)
load_profile = namespace['load_calibration_profile']


def baseline(**changes):
    value = dict(passed=True, open_q=.50, max_q=.78, measured_max_q=.779,
                 force_percent=20, context='test-context', model_sha256='a'*64)
    value.update(changes)
    return value


def trials(force=20, start_id=1, **changes):
    result = []
    for offset, measured in enumerate((.715, .716, .717)):
        item = dict(id=start_id+offset, context='test-context', force_percent=force,
                    commanded_q=(.72 if offset == 0 else .73), measured_q=measured,
                    verified_success=True, released=True, sensor_valid=True, grip_state=2,
                    dropped=False, stopped=False, arm_moved=False)
        item.update(changes)
        result.append(item)
    return result


def test_defaults_and_boundary_settings():
    settings = data.validate_settings()
    assert settings == dict(open_q=.5, max_q=.78, step_q=.01, force_percent=20,
                            settle_s=.4, width_mm=25.)
    assert type(settings['force_percent']) is int
    assert data.validate_settings(open_q=0, max_q=.85, step_q=.001,
                                  force_percent=100, settle_s=2, width_mm=95)['max_q'] == .85


@pytest.mark.parametrize('field', ['open_q', 'max_q', 'step_q', 'force_percent', 'settle_s', 'width_mm'])
@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf, True, None, '0.5'])
def test_settings_reject_nonfinite_boolean_and_nonnumeric(field, invalid):
    with pytest.raises(ValueError):
        data.validate_settings(**{field: invalid})


@pytest.mark.parametrize('change', [
    dict(open_q=-.001), dict(open_q=.78), dict(max_q=.851), dict(max_q=0),
    dict(step_q=.0009), dict(step_q=.0301), dict(force_percent=19),
    dict(force_percent=101), dict(force_percent=20.5), dict(settle_s=.299),
    dict(settle_s=2.001), dict(width_mm=0), dict(width_mm=95.001),
])
def test_settings_reject_out_of_range_values(change):
    with pytest.raises(ValueError):
        data.validate_settings(**change)


def test_recommendation_contains_only_tested_values_without_extra_margin():
    evidence = trials()
    answer = data.recommend(evidence, baseline())
    assert answer['commanded_q'] == max(item['commanded_q'] for item in evidence)
    assert answer['force_percent'] == 20
    assert answer['measured_q'] == .716
    assert answer['measured_min'] == .715 and answer['measured_max'] == .717
    assert answer['trial_ids'] == [1, 2, 3] and answer['trials'] == 3


@pytest.mark.parametrize('empty_baseline', [None, {}, {'passed': False}, {'passed': 1}])
def test_empty_space_baseline_is_required(empty_baseline):
    assert data.recommend(trials(), empty_baseline) is None


@pytest.mark.parametrize('field', ['open_q', 'max_q'])
@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf, True, None, '0.7'])
def test_baseline_rejects_invalid_bounds(field, invalid):
    assert data.recommend(trials(), baseline(**{field: invalid})) is None


@pytest.mark.parametrize('change', [dict(open_q=-.1), dict(open_q=.78), dict(max_q=.86), dict(max_q=.49)])
def test_baseline_requires_ordered_physical_bounds(change):
    assert data.recommend(trials(), baseline(**change)) is None


def test_three_distinct_successful_trials_are_required():
    evidence = trials()
    assert data.recommend(evidence[:2], baseline()) is None
    assert data.recommend([evidence[0]]*3, baseline()) is None


@pytest.mark.parametrize('invalid_id', [0, -1, True, 1.5, None, '1'])
def test_trial_ids_are_positive_unique_integers(invalid_id):
    evidence = trials(); evidence[0]['id'] = invalid_id
    assert data.recommend(evidence, baseline()) is None


@pytest.mark.parametrize('field,invalid', [
    ('verified_success', False), ('verified_success', None), ('released', False),
    ('sensor_valid', False), ('grip_state', 0), ('grip_state', 1), ('grip_state', 3),
    ('dropped', True), ('stopped', True), ('arm_moved', True),
])
def test_each_trial_requires_verified_released_sensor_backed_grasp(field, invalid):
    evidence = trials(); evidence[0][field] = invalid
    assert data.recommend(evidence, baseline()) is None


@pytest.mark.parametrize('field', ['dropped', 'stopped', 'arm_moved'])
def test_absent_no_drop_stop_or_arm_motion_evidence_is_not_success(field):
    evidence = trials(); del evidence[0][field]
    assert data.recommend(evidence, baseline()) is None


def test_lowest_successful_tested_force_is_selected_independent_of_order():
    evidence = trials(60, 1)+trials(40, 4)+trials(20, 7)
    assert data.recommend(evidence, baseline())['force_percent'] == 20


@pytest.mark.parametrize('failure', [dict(verified_success=False), dict(dropped=True)])
def test_failed_force_group_is_excluded_even_with_three_other_successes(failure):
    evidence = trials(20, 1)+trials(40, 4)
    failed = trials(20, 7)[0]; failed.update(failure); evidence.append(failed)
    answer = data.recommend(evidence, baseline())
    assert answer['force_percent'] == 40 and answer['trial_ids'] == [4, 5, 6]


@pytest.mark.parametrize('invalid', [19, 101, 20.5, math.nan, math.inf, True, None, '20'])
def test_invalid_trial_force_never_creates_an_untested_recommendation(invalid):
    assert data.recommend(trials(invalid), baseline()) is None


@pytest.mark.parametrize('field', ['measured_q', 'commanded_q'])
@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf, True, None, '0.7', -.01, .86])
def test_invalid_trial_angle_excludes_group(field, invalid):
    evidence = trials(); evidence[0][field] = invalid
    assert data.recommend(evidence, baseline()) is None


def test_contact_spread_accepts_exact_boundary_and_rejects_larger_spread():
    evidence = trials()
    for item, value in zip(evidence, [.74, .75, .76]):
        item.update(measured_q=value, commanded_q=.77)
    assert data.recommend(evidence, baseline()) is not None
    evidence[-1]['measured_q'] = .7601
    assert data.recommend(evidence, baseline()) is None


def test_contact_must_be_inside_tested_empty_sweep_and_after_open_angle():
    assert data.recommend(trials(), baseline(max_q=.72)) is None
    assert data.recommend(trials(), baseline(open_q=.715)) is None


def test_command_cannot_be_materially_more_open_than_measured_contact():
    evidence = trials(); evidence[0]['commanded_q'] = .70
    assert data.recommend(evidence, baseline()) is None


@pytest.fixture
def valid_profile():
    settings = data.validate_settings()
    context = hashlib.sha256(json.dumps(
        {key:value for key,value in settings.items() if key != 'force_percent'}, sort_keys=True).encode()).hexdigest()
    sweep = baseline(context=context)
    evidence = trials(context=context)
    recommendation = data.recommend(evidence, sweep)
    return dict(schema=1, hardware_class=PLUGIN, model_sha256='a'*64, block_width_mm=25.,
                settings=settings, baseline=sweep, trials=evidence, recommendation=recommendation,
                gripper_open_position=settings['open_q'],
                gripper_grasp_position=recommendation['commanded_q'],
                force_percent=recommendation['force_percent'])


def write_profile(tmp_path, profile):
    path = tmp_path / 'gripper.yaml'
    path.write_text(yaml.safe_dump(profile))
    return path


def test_profile_is_optional_but_valid_real_evidence_loads(tmp_path, valid_profile):
    assert load_profile(tmp_path/'missing.yaml') is None
    path = write_profile(tmp_path, valid_profile)
    assert load_profile(path, model_sha256='a'*64, block_width_mm=25) == valid_profile


@pytest.mark.parametrize('field,value', [
    ('hardware_class', 'mock_components/GenericSystem'), ('hardware_class', 'unknown'),
    ('schema', 2), ('model_sha256', 'b'*64), ('block_width_mm', 30),
    ('gripper_grasp_position', .74), ('gripper_open_position', .49), ('force_percent', 40),
])
def test_profile_rejects_mock_wrong_metadata_or_q_tampering(tmp_path, valid_profile, field, value):
    valid_profile[field] = value
    with pytest.raises(ValueError):
        load_profile(write_profile(tmp_path, valid_profile), model_sha256='a'*64)


@pytest.mark.parametrize('field,value', [
    ('commanded_q', .74), ('measured_q', .70), ('trials', 4), ('force_percent', 30),
])
def test_profile_recomputes_recommendation_from_evidence(tmp_path, valid_profile, field, value):
    valid_profile['recommendation'][field] = value
    with pytest.raises(ValueError):
        load_profile(write_profile(tmp_path, valid_profile), model_sha256='a'*64)


@pytest.mark.parametrize('mutation', [
    'fractional_trial_force', 'nan_baseline_max', 'duplicate_trial', 'wrong_trial_context',
    'wrong_baseline_context', 'wrong_baseline_model', 'wrong_settings_width',
    'wrong_baseline_open', 'wrong_baseline_max', 'wrong_settings_context',
])
def test_profile_evidence_cannot_be_relabelled_or_mixed(tmp_path, valid_profile, mutation):
    if mutation == 'fractional_trial_force':
        for item in valid_profile['trials']: item['force_percent'] = 20.5
    elif mutation == 'nan_baseline_max': valid_profile['baseline']['max_q'] = math.nan
    elif mutation == 'duplicate_trial': valid_profile['trials'] = [copy.deepcopy(valid_profile['trials'][0])]*3
    elif mutation == 'wrong_trial_context': valid_profile['trials'][0]['context'] = 'other-context'
    elif mutation == 'wrong_baseline_context': valid_profile['baseline']['context'] = 'other-context'
    elif mutation == 'wrong_baseline_model': valid_profile['baseline']['model_sha256'] = 'b'*64
    elif mutation == 'wrong_settings_width': valid_profile['settings']['width_mm'] = 30
    elif mutation == 'wrong_baseline_open': valid_profile['baseline']['open_q'] = .49
    elif mutation == 'wrong_baseline_max': valid_profile['baseline']['max_q'] = .79
    elif mutation == 'wrong_settings_context': valid_profile['settings']['step_q'] = .02
    with pytest.raises(ValueError):
        load_profile(write_profile(tmp_path, valid_profile), model_sha256='a'*64)


@pytest.fixture(scope='module')
def native_gap_model():
    # Resolve the project's actual native URDF package URIs to its real source
    # STL files. Geometry, scales and mimic chains are kept exactly as supplied;
    # only ament discovery is avoided, so no ROS environment is required.
    scene = sys.modules[PACKAGE_NAME+'.air_scene']
    resolver = scene._mesh_path
    description = REPOSITORY/'dh_ag95_gripper_ros2/dh_ag95_description'
    def source_mesh(uri):
        parsed = urlsplit(uri)
        assert parsed.scheme == 'package' and parsed.netloc == 'dh_ag95_description'
        return resolver(str(description / parsed.path.lstrip('/')))
    with pytest.MonkeyPatch.context() as monkeypatch:
        monkeypatch.setattr(scene, '_mesh_path', source_mesh)
        xml = (REPOSITORY/'ur5e_block_description/urdf/ur5e_on_box.urdf').read_text()
        yield data.GripperGapModel(xml)


@pytest.mark.parametrize('q,expected_mm', [(.5, 48.538), (.67, 30.255), (.718, 25.0)])
def test_native_ag95_gap_matches_25mm_geometry(native_gap_model, q, expected_mm):
    assert native_gap_model.gap_mm(q) == pytest.approx(expected_mm, abs=.03)


def test_native_gap_decreases_as_joint_closes(native_gap_model):
    gaps = [native_gap_model.gap_mm(q) for q in [.5, .6, .67, .718, .78, .85]]
    assert all(left > right for left,right in zip(gaps, gaps[1:]))


@pytest.mark.parametrize('invalid', [True, math.nan, math.inf, -math.inf, -.001, .931])
def test_gap_rejects_nonfinite_bool_and_out_of_range(native_gap_model, invalid):
    with pytest.raises(ValueError):
        native_gap_model.gap_mm(invalid)
