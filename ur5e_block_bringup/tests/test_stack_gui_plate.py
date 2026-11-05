"""Plate GUI geometry and workflow regressions; no ROS node or hardware."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from visualization_msgs.msg import Marker, MarkerArray

from ur5e_block_bringup.plate_geometry import plate_parameters

SOURCE = Path(__file__).resolve().parents[2] / 'ur5e_a3_grid/scripts/stack_gui.py'
SPEC = importlib.util.spec_from_file_location('stack_plate_ui', SOURCE)
ui = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ui)


class Var:
    def __init__(self, value=''):
        self.value = value
    def get(self):
        return self.value
    def set(self, value):
        self.value = value


class Widget:
    def __init__(self):
        self.options = {}
    def configure(self, **values):
        self.options.update(values)


def gui(tmp_path, *, plate=True):
    obj = ui.StackGui.__new__(ui.StackGui)
    obj.plate_enabled = plate
    obj.common = dict(plate_enabled=plate, plate_center_xy=[.4, 0.], plate_yaw_deg=-90.,
                      table_surface_z=.8, reference_frame='world')
    obj.plate_defaults = plate_parameters(obj.common) if plate else {}
    obj.common.update(obj.plate_defaults)
    obj.real_backend = True
    obj.joint_values = {obj.HAND_JOINT: .65}
    obj.simulation_riser_height = 0.0
    obj.home = [.501488, .151445, 1.106263]
    obj.columns, obj.rows = 4, 6
    obj.obstacles, obj.empty, obj.target = set(), set(), 0
    obj.scan_mode = obj.plate_alignment = False
    obj.job = obj.calibration_window = None
    obj.model_error = ''
    obj.stop_pending = obj.user_stopped = obj.close_requested = False
    obj.stop_acknowledged = obj.stop_backend_confirmed = False
    obj.status, obj.summary, obj.corner_text, obj.step_status = Var(), Var(), Var(), Var()
    obj.center_x, obj.center_y = Var('.4'), Var('0')
    obj.obstacle_height, obj.speed, obj.empty_hand_clearance = Var('150'), Var('.05'), Var('50')
    obj.placement_mode = Var(obj.PLACEMENT_LABELS['side'])
    obj.mode = Var('block')
    obj.cells = {number: Widget() for number in range(1, 25)}
    obj.folder = tmp_path
    obj.selection_file = tmp_path/'layout_cube_plate_v1.yaml'
    obj.trajectory_file = tmp_path/'path.cdr'
    obj.plan_manifest_file = tmp_path/'path.gui.json'
    obj.invalidate_plan = Mock()
    obj.refresh_run_button = Mock()
    obj.busy = Mock()
    return obj


def prepare_launch(obj, monkeypatch):
    obj.save_selection()
    obj.scene_client = SimpleNamespace(service_is_ready=lambda: True)
    obj.execution_ready = lambda: True
    obj.clear_step_request = Mock()
    obj.step_seen_requests = set()
    obj.root = object()
    obj.calibration_file, obj.physical_profile = '/tmp/calibration', '/tmp/profile'
    obj.execution_mode = Var(obj.EXECUTION_LABELS['step'])
    obj.plan_valid = True
    obj.plan_matches_selection = lambda: True
    obj.trajectory_file.write_bytes(b'test sealed path')
    metadata = dict(plan_id='current_plan')
    if obj.plate_alignment:
        metadata.update(alignment_policy=ui.ALIGNMENT_POLICY, valid_for_seconds=1800,
                        alignment_stops=[{} for _ in range(8)])
    obj.trajectory_file.with_name(obj.trajectory_file.name+'.meta.json').write_text(json.dumps(metadata))
    node = SimpleNamespace(clients=[])
    obj.node = node
    start = Mock(return_value=SimpleNamespace(stdout=[]))
    monkeypatch.setattr(ui.subprocess, 'Popen', start)
    monkeypatch.setattr(ui.threading, 'Thread', Mock())
    return start


def selection(obj):
    return yaml.safe_load(obj.selection_file.read_text())['/**']['ros__parameters']


def test_save_keeps_a_foundation_block_in_the_selected_stack_pocket(tmp_path):
    obj = gui(tmp_path)
    obj.target = 11
    obj.save_selection()
    value = selection(obj)
    assert value['geometry_profile'] == 'cube_plate_v1'
    assert value['target_cell'] == 11 and value['independent_stack_target'] is False
    assert value['target_has_foundation'] is True
    assert value['stack_target_xy'] == pytest.approx([.4297, .03])
    assert value['source_surface_z'] == pytest.approx(.805)
    assert value['stack_surface_z'] == pytest.approx(.805)
    assert len(value['cell_centers_xy']) == 48
    assert value['cell_centers_xy'][:4] == pytest.approx([.3109, .15, .3703, .15])
    assert value['cell_centers_xy'][-2:] == pytest.approx([.4891, -.15])
    assert value['grasp_height_above_table'] == .018
    assert value['vertical_placement_layers'] == 19
    assert value['placement_approach_clearance'] == .12
    assert value['side_entry_clearance'] == .010
    assert value['physical_approach_clearance'] == .04
    assert value['descent_velocity_scaling'] == value['descent_acceleration_scaling'] == .20


def test_live_translation_moves_pockets_corners_and_stack_together(tmp_path):
    obj = gui(tmp_path)
    obj.target = 11
    before = obj.current_plate_parameters()
    obj.center_x.set('.42')
    obj.center_y.set('.03')
    obj.save_selection()
    after = selection(obj)
    assert after['stack_target_xy'] == pytest.approx([.4497, .06])
    for index, value in enumerate(before['cell_centers_xy']):
        assert after['cell_centers_xy'][index] == pytest.approx(value+(.02 if index%2 == 0 else .03))
    for index, value in enumerate(before['plate_corners_xyz']):
        assert after['plate_corners_xyz'][index] == pytest.approx(value+(.02 if index%3 == 0 else .03 if index%3 == 1 else 0))


def test_alignment_clearance_and_job_kind_survive_save_and_reload(tmp_path):
    obj = gui(tmp_path)
    obj.scan_mode = obj.plate_alignment = True
    obj.empty_hand_clearance.set('60')
    obj.save_selection()
    value = selection(obj)
    assert value['plate_alignment_test'] is True and value['empty_xy_scan'] is True
    assert value['plate_alignment_clearance'] == value['empty_hand_clearance'] == .06
    restored = gui(tmp_path)
    restored.load_selection()
    assert restored.scan_mode and restored.plate_alignment
    assert restored.empty_hand_clearance.get() == '60'


def test_foreign_geometry_layout_is_preserved_and_not_loaded(tmp_path):
    obj = gui(tmp_path)
    legacy = {'/**': {'ros__parameters': dict(target_cell=11, obstacle_cells=[1],
                                             center_xyz=[.9, .4, 1.1])}}
    original = yaml.safe_dump(legacy)
    obj.selection_file.write_text(original)
    obj.load_selection()
    assert obj.selection_file.read_text() == original
    backups = list(tmp_path.glob('*.legacy.*'))
    assert len(backups) == 1 and backups[0].read_text() == original
    assert obj.obstacles == set() and obj.target == 0
    assert obj.center_x.get() == '.4' and obj.center_y.get() == '0'


def test_target_selection_replaces_an_obstacle_with_a_foundation_block(tmp_path):
    obj = gui(tmp_path)
    obj.obstacles = {11}
    obj.mode.set('target')
    obj.select_cell(11)
    assert obj.target == 11 and obj.obstacles == set() and 11 not in obj.empty
    assert '쌓을 곳' in obj.cells[11].options['text']
    assert '바닥 1개' in obj.cells[11].options['text']
    assert '이동 블록 23개' in obj.summary.get()
    obj.save_selection()
    other = gui(tmp_path)
    other.load_selection()
    assert other.target == 11
    assert selection(obj)['target_has_foundation'] is True


def test_selected_foundation_is_excluded_from_sources_and_new_blocks_start_at_layer_two(tmp_path):
    obj = gui(tmp_path)
    obj.target = 11
    obj.redraw()
    assert '이동 블록 23개 + 바닥 블록 1개' in obj.summary.get()
    assert '2층부터' in obj.summary.get()
    assert '완료 시 24층' in obj.summary.get()
    obj.example()
    assert set(range(1, 25))-obj.empty-{obj.target} == {4, 13}
    assert obj.target == 21
    assert '완료 시 3층' in obj.summary.get()
    assert '3층' in obj.status.get()


def test_foundation_alone_counts_as_one_layer_and_full_example_as_twenty_four(tmp_path):
    obj = gui(tmp_path)
    obj.target = 21
    obj.empty = set(range(1, 25)) - {21}
    obj.redraw()
    assert '이동 블록 0개 + 바닥 블록 1개' in obj.summary.get()
    assert '완료 시 1층 / 높이 25mm' in obj.summary.get()
    obj.full_example()
    assert obj.target == 21 and not obj.empty
    assert '이동 블록 23개 + 바닥 블록 1개' in obj.summary.get()
    assert '완료 시 24층 / 높이 600mm' in obj.summary.get()


def test_alignment_plan_needs_no_target_or_obstacle_and_never_approves(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    start = prepare_launch(obj, monkeypatch)
    modal = Mock()
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('plate_align')
    assert obj.obstacles == set() and obj.target == 0
    assert selection(obj)['plate_alignment_test'] is True
    assert selection(obj)['empty_xy_scan'] is True
    assert 'stack_mode:=dry_plan' in start.call_args.args[0]
    assert 'workspace_clear:=true' not in start.call_args.args[0]
    assert obj.job_plan_id is None and obj.node.clients == []
    modal.assert_not_called()


def test_regular_plate_plan_uses_selected_target_and_clears_alignment(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.target = 11
    obj.scan_mode = obj.plate_alignment = True
    start = prepare_launch(obj, monkeypatch)
    obj.launch_task('plan')
    assert start.called and obj.target == 11
    assert not selection(obj)['plate_alignment_test']
    assert not selection(obj)['empty_xy_scan']


def test_regular_plate_plan_requires_operator_selected_pocket(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    start = prepare_launch(obj, monkeypatch)
    obj.launch_task('plan')
    assert not start.called
    assert '쌓을 칸' in obj.status.get()


def test_physical_plan_has_separate_recipe_and_explicit_user_gripper_sequence(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.target = 11
    air_file = obj.trajectory_file
    start = prepare_launch(obj, monkeypatch)
    modal = Mock()
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('physical_plan')
    command = start.call_args.args[0]
    assert 'stack_mode:=physical_plan' in command
    assert f'recipe_manifest_file:={obj.trajectory_file}' in command
    assert obj.plan_kind == 'physical' and obj.trajectory_file != air_file
    assert obj.air_trajectory_file == air_file
    value = selection(obj)
    assert value['physical_stack_task'] and value['target_has_foundation']
    assert not value['plate_alignment_test'] and not value['empty_xy_scan']
    assert value['gripper_grasp_position'] == .73
    assert value['release_first_position'] == .70 and value['release_clearance_position'] == .60
    assert value['approach_velocity_scaling'] == .05 and value['transfer_velocity_scaling'] == 1.
    assert value['gripper_open_position'] == .4
    assert 'workspace_clear:=true' not in command
    assert not obj.visible_layout.get('_air_preview', False)
    assert obj.node.clients == []
    modal.assert_not_called()


@pytest.mark.parametrize('mode', ['step', 'continuous'])
def test_physical_run_requires_its_reviewed_recipe_and_actual_gripper_modal(tmp_path, monkeypatch, mode):
    obj = gui(tmp_path)
    obj.target = 11
    obj.set_plan_kind('physical')
    start = prepare_launch(obj, monkeypatch)
    obj.execution_mode.set(obj.EXECUTION_LABELS[mode])
    obj.trajectory_file.with_name(obj.trajectory_file.name+'.meta.json').write_text(
        '{"plan_id":"physical_recipe","path_kind":"physical_stack"}')
    modal = Mock(return_value=True)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('run')
    command = start.call_args.args[0]
    assert 'stack_mode:=physical_run' in command
    assert 'workspace_clear:=true' in command and f'execution_mode:={mode}' in command
    assert obj.job_plan_id == 'physical_recipe' and obj.node.clients == []
    title, description = modal.call_args.args[:2]
    assert '팔·그리퍼' in title and '실제 블록' in description
    assert all(token in description for token in ('0.73', '0.70', '0.60', '0.05', '1.00', '40mm', '바닥 블록'))
    assert '블록이 없는 상태' not in description


def test_physical_run_rejects_an_air_path_even_if_the_file_fingerprint_matches(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.target = 11
    obj.set_plan_kind('physical')
    start = prepare_launch(obj, monkeypatch)
    modal = Mock(return_value=True)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('run')
    start.assert_not_called()
    modal.assert_not_called()
    assert '계획' in obj.status.get()


def test_physical_recipe_requires_a_selected_destination(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    start = prepare_launch(obj, monkeypatch)
    obj.launch_task('physical_plan')
    start.assert_not_called()
    assert '쌓을 칸' in obj.status.get()


def test_offline_prepare_preserves_the_existing_recipe_and_has_no_motion_permission(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.target = 11
    obj.set_plan_kind('physical')
    start = prepare_launch(obj, monkeypatch)
    before = obj.trajectory_file.read_bytes()
    meta_path = obj.trajectory_file.with_name(obj.trajectory_file.name+'.meta.json')
    metadata = meta_path.read_bytes()
    obj.launch_task('physical_prepare')
    command = start.call_args.args[0]
    assert 'stack_mode:=physical_prepare' in command
    assert 'workspace_clear:=true' not in command
    assert obj.trajectory_file.read_bytes() == before and meta_path.read_bytes() == metadata


def test_restored_physical_selection_keeps_recipe_separate_from_air_file(tmp_path):
    obj = gui(tmp_path)
    obj.target = 11
    obj.set_plan_kind('physical')
    obj.save_selection()
    restored = gui(tmp_path)
    restored.load_selection()
    assert restored.plan_kind == 'physical' and restored.target == 11
    assert restored.trajectory_file == obj.trajectory_file
    assert restored.air_trajectory_file != restored.trajectory_file


def test_alignment_plan_switches_back_to_air_without_overwriting_physical_recipe(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.set_plan_kind('physical')
    start = prepare_launch(obj, monkeypatch)
    physical_file = obj.trajectory_file
    physical_file.write_bytes(b'reviewed physical recipe')
    obj.launch_task('plate_align')
    assert obj.plan_kind == 'air' and obj.trajectory_file == obj.air_trajectory_file
    assert 'stack_mode:=dry_plan' in start.call_args.args[0]
    assert physical_file.read_bytes() == b'reviewed physical recipe'


def test_physical_readiness_requires_the_real_gripper_controller(tmp_path):
    obj = gui(tmp_path)
    obj.plan_kind = 'physical'
    obj.scene_client = SimpleNamespace(service_is_ready=lambda: True)
    obj.fresh_joints = lambda names: True
    obj.controller_state = lambda manager, name: 'active' if manager == '/controller_manager' else 'inactive'
    assert not obj.execution_ready()
    obj.controller_state = lambda manager, name: 'active'
    assert obj.execution_ready()


def test_legacy_regular_plan_still_requires_a_foundation_target(tmp_path, monkeypatch):
    obj = gui(tmp_path, plate=False)
    start = prepare_launch(obj, monkeypatch)
    obj.launch_task('plan')
    assert not start.called
    assert '쌓을 칸' in obj.status.get()


def test_alignment_run_explains_stationary_adjustment_before_later_ok(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.scan_mode = obj.plate_alignment = True
    start = prepare_launch(obj, monkeypatch)
    modal = Mock(return_value=True)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('run')
    assert start.called and 'execution_mode:=step' in start.call_args.args[0]
    description = modal.call_args.args[1]
    assert 'A→B→C→D' in description and '손을 뺀 다음' in description
    assert 'X/Y 두 방향' in description and '90°' in description
    assert obj.node.clients == []


@pytest.mark.parametrize('old_metadata', [
    dict(plan_id='old_plan', alignment_pause_after_segment=4),
    dict(plan_id='old_plan', alignment_policy=ui.ALIGNMENT_POLICY, alignment_stops=[{} for _ in range(8)])])
def test_old_alignment_is_rejected_before_motion_modal(tmp_path, monkeypatch, old_metadata):
    obj = gui(tmp_path)
    obj.scan_mode = obj.plate_alignment = True
    start = prepare_launch(obj, monkeypatch)
    obj.trajectory_file.with_name(obj.trajectory_file.name+'.meta.json').write_text(json.dumps(old_metadata))
    modal = Mock(return_value=True)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('run')
    start.assert_not_called()
    modal.assert_not_called()
    obj.invalidate_plan.assert_called()


@pytest.mark.parametrize('initial_position', [.4, .7])
def test_real_alignment_prepares_point_six_five_only_after_modal_and_explicit_ok(tmp_path, monkeypatch, initial_position):
    obj = gui(tmp_path)
    obj.joint_values[obj.HAND_JOINT] = initial_position
    start = prepare_launch(obj, monkeypatch)
    modal = Mock(return_value=True)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('plate_align')
    command = start.call_args.args[0]
    assert 'stack_mode:=alignment_prepare' in command
    assert 'workspace_clear:=true' in command
    assert 'execution_mode:=step' in command
    assert f'preparation_id:={obj.job_plan_id}' in command
    assert obj.job_plan_id.startswith('prep_')
    assert obj.job_action == 'alignment_gripper' and obj.pending_alignment_plan
    assert obj.node.clients == []  # No automatic approval.
    assert '0.65rad' in modal.call_args.args[1]


def test_declining_real_gripper_preparation_starts_no_task(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.joint_values[obj.HAND_JOINT] = .4
    start = prepare_launch(obj, monkeypatch)
    monkeypatch.setattr(ui.messagebox, 'askokcancel', Mock(return_value=False))
    obj.launch_task('plate_align')
    start.assert_not_called()


def test_simulation_corner_pocket_plan_works_without_target_or_real_hardware(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.real_backend = False
    obj.simulation_riser_height = .016
    start = prepare_launch(obj, monkeypatch)
    modal = Mock()
    monkeypatch.setattr(ui.messagebox, 'askokcancel', modal)
    obj.launch_task('plate_align')
    command = start.call_args.args[0]
    assert command[:4] == ['ros2', 'launch', 'ur5e_a3_grid', 'stack_blocks.launch.py']
    assert 'stack_mode:=plan' in command and 'riser_height:=0.016' in command
    value = selection(obj)
    assert value['plate_alignment_test'] and value['plate_alignment_gripper_position'] == .65
    assert value['plate_alignment_tip_clearance'] == .010
    assert obj.target == 0
    modal.assert_not_called()


def test_corner_markers_use_exact_reference_xyz_and_do_not_duplicate_mesh(tmp_path):
    obj = gui(tmp_path)
    obj.target = 11
    from builtin_interfaces.msg import Time
    obj.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())))
    array = MarkerArray()
    geometry = obj.current_plate_parameters()
    obj.publish_plate_markers(array, geometry)
    corners = [marker for marker in array.markers if marker.ns == 'cube_plate_corners']
    assert len(corners) == 4 and all(marker.type == Marker.CUBE for marker in corners)
    assert [marker.pose.position.x for marker in corners] == pytest.approx(geometry['plate_alignment_points_xyz'][0::3])
    assert [marker.pose.position.y for marker in corners] == pytest.approx(geometry['plate_alignment_points_xyz'][1::3])
    assert [marker.pose.position.z for marker in corners] == pytest.approx([z+.004 for z in geometry['plate_alignment_points_xyz'][2::3]])
    labels = [marker.text for marker in array.markers if marker.ns == 'cube_plate_corner_labels']
    assert len(labels) == 4 and [label[0] for label in labels] == list('ABCD')
    reference_z = geometry['plate_alignment_points_xyz'][2]
    assert labels[0] == f'A(1)\nX310.9\nY150.0\nZ{reference_z*1000:.1f}mm'
    assert all(len(label.splitlines()) == 4 and ' ' not in label for label in labels)
    text_markers = [marker for marker in array.markers if marker.ns == 'cube_plate_corner_labels']
    assert all(marker.scale.z == .020 and marker.pose.position.z == pytest.approx(reference_z+.055)
               for marker in text_markers)
    assert len([marker for marker in array.markers if marker.ns == 'cell_ids']) == 24
    assert not any(marker.type == Marker.MESH_RESOURCE for marker in array.markers)
    target = next(marker for marker in array.markers if marker.ns == 'cube_plate_stack' and marker.id == 1)
    assert target.text == 'STACK 11 · 바닥 1개'
    assert (target.pose.position.x, target.pose.position.y) == pytest.approx([.4297, .03])


def test_no_central_stack_marker_is_drawn_before_target_selection(tmp_path):
    obj = gui(tmp_path)
    from builtin_interfaces.msg import Time
    obj.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())))
    array = MarkerArray()
    obj.publish_plate_markers(array, obj.current_plate_parameters())
    assert not any(marker.ns == 'cube_plate_stack' for marker in array.markers)


def test_new_checked_placement_invalidates_saved_cell_assignment(tmp_path):
    obj = gui(tmp_path)
    obj.obstacles = {1}
    obj.center_x.set('.42')
    obj.save_selection()
    other = gui(tmp_path)
    other.common['plate_center_xy'] = [.45, 0.]
    other.plate_defaults = plate_parameters(other.common)
    other.common.update(other.plate_defaults)
    other.center_x.set('.45')
    other.load_selection()
    assert other.obstacles == set() and other.center_x.get() == '.45'
    assert len(list(tmp_path.glob('*.legacy.*'))) == 1


def test_operator_xy_correction_survives_when_checked_defaults_are_unchanged(tmp_path):
    obj = gui(tmp_path)
    obj.center_x.set('.42')
    obj.center_y.set('.01')
    obj.obstacles = {5}
    obj.save_selection()
    other = gui(tmp_path)
    other.load_selection()
    assert other.obstacles == {5}
    assert float(other.center_x.get()) == .42 and float(other.center_y.get()) == .01
    assert not list(tmp_path.glob('*.legacy.*'))


@pytest.mark.parametrize('height', [0.0, .016, .3])
def test_simulation_riser_accepts_the_same_range_as_the_model(height):
    assert ui.StackGui.checked_simulation_riser_height(height) == height


@pytest.mark.parametrize('height', [-.001, .301, float('nan'), float('inf'), True, None, 'bad'])
def test_simulation_riser_rejects_invalid_geometry_before_starting(height):
    with pytest.raises(ValueError, match='simulation_riser_height'):
        ui.StackGui.checked_simulation_riser_height(height)


@pytest.mark.parametrize('action', ['scene', 'plan', 'reset', 'run'])
def test_simulation_riser_matches_the_child_model_without_raising_the_plate(tmp_path, monkeypatch, action):
    obj = gui(tmp_path)
    obj.target = 11
    obj.real_backend = False
    obj.simulation_riser_height = .016
    start = prepare_launch(obj, monkeypatch)
    obj.launch_task(action)
    command = start.call_args.args[0]
    assert 'stack_blocks.launch.py' in command
    assert 'riser_height:=0.016' in command
    values = selection(obj)
    assert values['robot_riser_height'] == .016
    assert values['plate_bottom_z'] == .8
    assert values['source_surface_z'] == pytest.approx(.805)
    assert values['stack_surface_z'] == pytest.approx(.805)
    assert 'workspace_clear:=true' not in command


def test_real_backend_ignores_simulation_mounting_override(tmp_path, monkeypatch):
    obj = gui(tmp_path)
    obj.target = 11
    obj.simulation_riser_height = .2
    start = prepare_launch(obj, monkeypatch)
    obj.launch_task('plan')
    assert not any(value.startswith('riser_height:=') for value in start.call_args.args[0])
    assert 'robot_riser_height' not in selection(obj)
