"""Launch parameter preparation only: no launch service, node or device."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('plate_block_launch', ROOT/'ur5e_block_bringup/launch/block_task.launch.py')
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


@pytest.fixture
def prepare(monkeypatch, tmp_path):
    monkeypatch.setattr(module, 'LaunchConfiguration', lambda name: SimpleNamespace(perform=lambda context: context[name]))
    monkeypatch.setattr(module, 'Node', lambda **values: values)
    monkeypatch.setattr(module, 'load_moveit_settings', lambda *args: {'robot_description': '<robot />'})
    monkeypatch.setattr(module, 'load_calibration_profile', lambda **kwargs: None)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(ROOT/name))
    monkeypatch.setattr(module, 'physical_held_path', lambda: tmp_path/'held.json')

    def run(mode='dry_plan', preparation_id='', recovery_empty_confirmed='false', **params):
        selected = dict(geometry_profile='cube_plate_v1', plate_enabled=True,
                        plate_center_xy=[.4, 0.], plate_yaw_deg=-90.,
                        empty_hand_clearance=.05, velocity_scaling=.05)
        selected.update(params)
        path = tmp_path/'selection.yaml'
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': selected}}))
        context = dict(calibration_file='calibration.yaml', physical_profile='physical.yaml',
                       selection_file=str(path), stack_mode=mode, trajectory_file=str(tmp_path/'plan.cdr'),
                       workspace_clear='false', allow_mock_hardware='false', execution_mode='step',
                       preparation_id=preparation_id, recovery_empty_confirmed=recovery_empty_confirmed)
        context['recipe_manifest_file'] = ''
        run.last_node = module.start(context)[0]
        return run.last_node['parameters'][0]
    return run


def test_air_pattern_raises_logical_targets_but_never_the_plate_or_obstacle(prepare):
    p = prepare()
    assert p['table_surface_z'] == pytest.approx(.800)
    assert p['plate_bottom_z'] == pytest.approx(.800)
    assert p['plate_origin_xyz'][2] == pytest.approx(.800)
    assert p['source_surface_z'] == pytest.approx(.855)
    assert p['stack_surface_z'] == pytest.approx(.855)
    assert p['obstacle_surface_z'] == pytest.approx(.805)
    assert p['independent_stack_target'] is False and p['target_has_foundation'] is True


def test_alignment_uses_plate_top_clearance_and_physical_supports(prepare):
    p = prepare(plate_alignment_test=True)
    assert p['plate_alignment_test'] and p['empty_xy_scan']
    assert p['scan_min_gripper_z'] == pytest.approx(.820)
    assert p['source_surface_z'] == pytest.approx(.805)
    assert p['stack_surface_z'] == pytest.approx(.805)
    assert p['workspace_clear'] is False


def test_obstacle_scan_keeps_the_obstacle_anchored_to_its_pocket_floor(prepare):
    p = prepare(empty_xy_scan=True, obstacle_cells=[7])
    assert not p['plate_alignment_test']
    assert p['scan_min_gripper_z'] == pytest.approx(1.005)
    assert p['obstacle_surface_z'] == pytest.approx(.805)


def test_scene_display_cannot_reuse_a_motion_mode_flag(prepare):
    p = prepare('scene', empty_xy_scan=True, plate_alignment_test=True)
    assert not p['empty_xy_scan'] and not p['plate_alignment_test']


def test_old_uniform_grid_selection_requires_explicit_migration(prepare):
    with pytest.raises(ValueError, match='새 GUI'):
        prepare(geometry_profile='old_uniform_grid')


def test_saved_cell_array_is_recomputed_from_the_mesh_pose(prepare):
    p = prepare(cell_centers_xy=[99.] * 48, plate_yaw_deg=0.)
    assert (p['columns'], p['rows']) == (6, 4)
    assert p['cell_centers_xy'][:2] == pytest.approx([.25, .0891])


def test_legacy_mode_remains_available_without_a_plate(prepare):
    p = prepare(plate_enabled=False, geometry_profile='old_uniform_grid')
    assert not p['plate_enabled']
    assert p['table_surface_z'] == pytest.approx(.850)
    assert p['obstacle_surface_z'] == pytest.approx(.800)


def test_alignment_cannot_run_without_the_measured_plate(prepare):
    with pytest.raises(ValueError, match='requires'):
        prepare(plate_enabled=False, plate_alignment_test=True)


def test_alignment_preparation_is_separate_from_arm_replay_and_has_its_own_id(prepare):
    p = prepare('alignment_prepare', preparation_id='prep_unique_42')
    assert p['stack_mode'] == 'alignment_prepare'
    assert p['preparation_id'] == 'prep_unique_42'
    assert p['plate_alignment_test']
    assert p['workspace_clear'] is False
    assert p['plate_alignment_gripper_position'] == .65
    assert p['scan_min_gripper_z'] == pytest.approx(.820)


@pytest.mark.parametrize('preparation_id', ['', '../bad', 'x'*129])
def test_preparation_cannot_start_without_a_valid_unique_id(prepare, preparation_id):
    with pytest.raises(ValueError, match='preparation_id'):
        prepare('alignment_prepare', preparation_id=preparation_id)


def test_alignment_floor_does_not_follow_the_selected_stack_pocket_support(prepare):
    p = prepare(plate_alignment_test=True, target_cell=9, plate_alignment_tip_clearance=.012)
    assert p['stack_surface_z'] == pytest.approx(.805)
    assert p['scan_min_gripper_z'] == pytest.approx(.822)
    assert p['plate_alignment_cell_ids'] == [1, 4, 24, 21]


@pytest.mark.parametrize('mode', ['physical_plan', 'physical_run'])
def test_physical_stack_routes_to_reviewed_executor_and_preserves_support_heights(prepare, mode):
    p = prepare(mode, target_cell=11, plate_alignment_test=True, empty_xy_scan=True)
    assert prepare.last_node['executable'] == 'physical_stack'
    assert p['stack_mode'] == mode and p['physical_stack_task']
    assert not p['plate_alignment_test'] and not p['empty_xy_scan'] and not p['empty_hand_test']
    assert p['source_surface_z'] == pytest.approx(.805) and p['stack_surface_z'] == pytest.approx(.805)
    assert p['target_has_foundation']
    assert p['recipe_manifest_file'] == p['trajectory_file']
    assert p['workspace_clear'] is False and p['allow_mock_hardware'] is False
    assert p['gripper_open_position'] == .4 and p['gripper_grasp_position'] == .73
    assert 'gripper_pregrasp_position' not in p
    assert p['release_first_position'] == .70 and p['release_clearance_position'] == .60
    assert p['approach_velocity_scaling'] == .05 and p['transfer_velocity_scaling'] == .20
    assert p['approach_acceleration_scaling'] == .05 and p['transfer_acceleration_scaling'] == .20
    assert p['split_stack_speeds'] is True
    assert p['velocity_scaling'] == .20 and p['acceleration_scaling'] == .05
    assert p['vertical_placement_layers'] == 19
    assert p['placement_approach_clearance'] == .12


def test_verified_gripper_profile_cannot_override_the_requested_physical_recipe(prepare, monkeypatch):
    monkeypatch.setattr(module, 'load_calibration_profile', lambda **kwargs:
                        dict(gripper_open_position=.5, gripper_grasp_position=.78))
    p = prepare('physical_plan', gripper_grasp_position=.9, release_first_position=.8, gripper_pregrasp_position=.72)
    assert p['gripper_grasp_position'] == .73 and p['gripper_open_position'] == .4
    assert p['release_first_position'] == .70 and p['release_clearance_position'] == .60
    assert 'gripper_pregrasp_position' not in p


@pytest.mark.parametrize('mode', ['physical_plan', 'physical_run'])
def test_saved_uniform_speed_profile_cannot_override_physical_phase_accelerations(prepare, mode):
    p = prepare(mode, split_stack_speeds=False, stack_acceleration_scaling=.9,
                approach_velocity_scaling=.8, transfer_velocity_scaling=.9,
                approach_acceleration_scaling=.8, transfer_acceleration_scaling=.9)
    assert p['split_stack_speeds'] is True
    assert p['approach_velocity_scaling'] == p['approach_acceleration_scaling'] == .05
    assert p['transfer_velocity_scaling'] == p['transfer_acceleration_scaling'] == .20
    assert p['stack_acceleration_scaling'] == .05


@pytest.mark.parametrize('mode', ['physical_plan', 'physical_run'])
def test_saved_old_transition_and_low_approach_cannot_override_physical_placement(prepare, mode):
    p = prepare(mode, vertical_placement_layers=10, placement_approach_clearance=.04)
    assert p['vertical_placement_layers'] == 19
    assert p['placement_approach_clearance'] == .12


@pytest.mark.parametrize('mode', ['scene', 'plan', 'reach', 'reset', 'dry_plan', 'dry_run', 'alignment_prepare'])
def test_legacy_tasks_cannot_erase_or_ignore_a_stopped_physical_grasp(prepare, mode):
    module.physical_held_path().write_text('{"cause":"stopped grasp"}')
    with pytest.raises(ValueError, match='블록'):
        prepare(mode, preparation_id='prep_recovery')


@pytest.mark.parametrize('mode', ['physical_plan', 'physical_run'])
def test_physical_latch_recovery_is_left_to_sensor_checked_backend(prepare, mode):
    module.physical_held_path().write_text('{"cause":"stopped grasp"}')
    p = prepare(mode)
    assert p['stack_mode'] == mode
    assert prepare.last_node['executable'] == 'physical_stack'


@pytest.mark.parametrize('confirmed', ['false', 'true'])
def test_recovery_routes_held_fault_to_guarded_executor_without_clearing_it(prepare, confirmed):
    held = module.physical_held_path()
    held.write_text('{"cause":"stopped grasp"}')
    p = prepare('physical_recover', preparation_id='recover_unique_42',
                recovery_empty_confirmed=confirmed, empty_hand_test=True,
                plate_alignment_test=True, empty_xy_scan=True)
    assert prepare.last_node['executable'] == 'physical_recovery'
    assert p['stack_mode'] == 'physical_recover'
    assert p['preparation_id'] == 'recover_unique_42'
    assert p['recovery_empty_confirmed'] is (confirmed == 'true')
    assert p['workspace_clear'] is False
    assert not p['empty_hand_test'] and not p['plate_alignment_test'] and not p['empty_xy_scan']
    assert held.read_text() == '{"cause":"stopped grasp"}'


@pytest.mark.parametrize('preparation_id', ['', '../bad', 'x'*129])
def test_recovery_requires_its_own_approval_identity(prepare, preparation_id):
    with pytest.raises(ValueError, match='preparation_id'):
        prepare('physical_recover', preparation_id=preparation_id)


def test_holding_latch_domain_cannot_escape_its_numeric_filename(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '../escape')
    with pytest.raises(ValueError, match='numeric'):
        module.physical_held_path()
