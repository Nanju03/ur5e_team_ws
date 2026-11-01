"""STL drawing coordinates and physical support planes, without ROS nodes."""
import pytest
from ur5e_block_bringup.plate_geometry import plate_parameters


def profile(**changes):
    return dict(plate_enabled=True, plate_center_xy=[.4, 0.], plate_yaw_deg=-90.,
                table_surface_z=.8, plate_bottom_z=.8, **changes)


def test_portrait_pockets_use_the_stl_pitch_instead_of_uniform_a3_cells():
    p = plate_parameters(profile())
    assert (p['columns'], p['rows']) == (4, 6)
    cells = list(zip(p['cell_centers_xy'][::2], p['cell_centers_xy'][1::2]))
    assert len(set(cells)) == 24
    assert cells[0] == pytest.approx((.3109, .15))
    assert cells[-1] == pytest.approx((.4891, -.15))
    assert cells[1][0]-cells[0][0] == pytest.approx(.0594)
    assert cells[0][1]-cells[4][1] == pytest.approx(.060)


def test_landscape_orientation_uses_six_columns_and_four_rows():
    base = profile(); base['plate_yaw_deg'] = 0.
    p = plate_parameters(base)
    assert (p['columns'], p['rows']) == (6, 4)
    assert p['cell_centers_xy'][:2] == pytest.approx((.25, .0891))
    assert (p['size_x'], p['size_y']) == pytest.approx((.420, .297))


def test_corner_order_and_mesh_origin_match_the_same_world_rectangle():
    p = plate_parameters(profile())
    assert p['plate_corners_xyz'] == pytest.approx([
        .2515, .210, .810, .5485, .210, .810,
        .5485, -.210, .810, .2515, -.210, .810])
    assert p['plate_origin_xyz'] == pytest.approx([.2515, .210, .8])


def test_selected_pocket_has_a_foundation_block_on_the_same_support_floor():
    p = plate_parameters(profile(target_cell=21))
    assert p['source_surface_z'] == pytest.approx(.805)
    assert p['stack_surface_z'] == pytest.approx(.805)
    assert p['source_surface_z']+p['grasp_height_above_table'] == pytest.approx(.823)
    assert p['target_cell'] == 21
    assert p['stack_target_xy'] == pytest.approx([.3109, -.15])
    assert p['independent_stack_target'] is False
    assert p['target_has_foundation'] is True
    assert p['plate_task_revision'] == 5
    assert p['plate_alignment_policy'] == 'plate_corner_xy_v1'
    # The existing cube is layer 1; the first carried cube is layer 2.
    assert p['stack_surface_z']+.025 == pytest.approx(.830)
    assert 'center_from_home' not in p  # Never replace calibrated FK travel Z.


def test_alignment_references_corner_pocket_centres_not_outer_plate_vertices():
    p = plate_parameters(profile())
    assert p['plate_alignment_cell_ids'] == [1, 4, 24, 21]
    assert p['plate_alignment_points_xyz'] == pytest.approx([
        .3109, .15, .805, .4891, .15, .805,
        .4891, -.15, .805, .3109, -.15, .805])
    assert p['plate_alignment_tip_clearance'] == .010
    assert p['plate_alignment_gripper_position'] == .65
    assert p['target_cell'] == 0  # Alignment does not invent a stack target.


def test_landscape_alignment_pockets_follow_the_grid_numbering():
    p = plate_parameters(dict(profile(), plate_yaw_deg=0., target_cell=24))
    assert p['plate_alignment_cell_ids'] == [1, 6, 24, 19]
    assert p['stack_target_xy'] == p['cell_centers_xy'][-2:]


def test_pose_translation_moves_every_target_and_the_visual_mesh_together():
    first = plate_parameters(profile(target_cell=21))
    moved = plate_parameters(profile(target_cell=21), center_xy=[.45, -.03])
    for key, stride in [('cell_centers_xy', 2), ('plate_corners_xyz', 3), ('plate_alignment_points_xyz', 3)]:
        for i in range(0, len(first[key]), stride):
            assert moved[key][i]-first[key][i] == pytest.approx(.05)
            assert moved[key][i+1]-first[key][i+1] == pytest.approx(-.03)
    assert moved['plate_origin_xyz'] == pytest.approx([.3015, .18, .8])
    assert moved['stack_target_xy'] == pytest.approx([.3609, -.18])


@pytest.mark.parametrize('key,value', [
    ('plate_enabled', 1), ('plate_yaw_deg', 45.), ('plate_yaw_deg', float('nan')),
    ('plate_center_xy', [True, 0.]), ('plate_center_xy', [.4]),
    ('plate_center_xy', [.1, 0.]), ('plate_bottom_z', .816),
    ('plate_alignment_clearance', .01), ('plate_alignment_clearance', float('inf')),
    ('plate_alignment_tip_clearance', 0.), ('plate_alignment_tip_clearance', float('nan')),
    ('plate_alignment_gripper_position', 1.), ('plate_alignment_gripper_position', .7),
    ('target_cell', True), ('target_cell', 25),
])
def test_invalid_geometry_is_rejected_instead_of_moving_a_wrong_layout(key, value):
    base = profile(); base[key] = value
    with pytest.raises(ValueError):
        plate_parameters(base)


def test_legacy_grid_remains_available_when_plate_is_disabled():
    assert plate_parameters({}) == {'plate_enabled': False}
