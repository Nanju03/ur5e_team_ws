"""The measured STL geometry, in metres, shared by the GUI and planner.

The unitless source STL matches the supplied 420 x 297 x 10 mm drawing.
Its origin is a lower corner, its pockets have a 5 mm floor, and its centre
is solid at 10 mm. No robot movement or ROS node is created by this module.
"""
import math


PROFILE = 'cube_plate_v1'
MESH_URI = 'package://ur5e_block_description/meshes/cube_plate.STL'
SIZE = (.420, .297, .010)
ALIGNMENT_GRIPPER_POSITION = .65
ALIGNMENT_POLICY = 'plate_corner_xy_v1'


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def plate_parameters(base, center_xy=None):
    """Return a flat ROS parameter mapping for one posed plate.

    Cell numbering always runs left to right (+X) then top to bottom (-Y)
    in the world view, regardless of the STL's 90-degree mounting rotation.
    A/B/C/D alignment points are the world-view corner pocket centres.
    Outer corners describe the plate outline only. Neither support-plane Z
    is a commanded TCP height; alignment uses the whole gripper tip envelope.
    """
    enabled = base.get('plate_enabled', False)
    if type(enabled) is not bool:
        raise ValueError('plate_enabled must be a boolean')
    if not enabled:
        return {'plate_enabled': False}
    centre = base.get('plate_center_xy', (.4, 0.)) if center_xy is None else center_xy
    if not isinstance(centre, (list, tuple)) or len(centre) != 2:
        raise ValueError('plate_center_xy must contain two coordinates in metres')
    cx, cy = [_number(v, 'plate_center_xy') for v in centre]
    if not .20 <= cx <= .85 or abs(cy) > .60:
        raise ValueError('Plate centre is outside the supported workbench placement region')
    yaw = _number(base.get('plate_yaw_deg', -90.), 'plate_yaw_deg')
    if abs(yaw/90.-round(yaw/90.)) > 1e-8:
        raise ValueError('Use a multiple of 90 degrees for the plate mounting orientation')
    yaw = (yaw+180.) % 360.-180.
    angle = math.radians(yaw)
    c, s = round(math.cos(angle)), round(math.sin(angle))
    bottom = _number(base.get('plate_bottom_z', base.get('table_surface_z', .8)), 'plate_bottom_z')
    table = _number(base.get('table_surface_z', .8), 'table_surface_z')
    if abs(bottom-table) > 1e-8:
        raise ValueError('The plate bottom must rest on the configured physical tabletop')
    clearance = _number(base.get('plate_alignment_clearance', .05), 'plate_alignment_clearance')
    if not .03 <= clearance <= .10:
        raise ValueError('Plate alignment clearance must be between 30 and 100 mm')
    tip_clearance = _number(base.get('plate_alignment_tip_clearance', .010), 'plate_alignment_tip_clearance')
    if not .005 <= tip_clearance <= .050:
        raise ValueError('Plate alignment tip clearance must be between 5 and 50 mm')
    alignment_gripper = _number(base.get('plate_alignment_gripper_position', ALIGNMENT_GRIPPER_POSITION), 'plate_alignment_gripper_position')
    if abs(alignment_gripper-ALIGNMENT_GRIPPER_POSITION) > 1e-9:
        raise ValueError(f'Plate alignment gripper position must remain {ALIGNMENT_GRIPPER_POSITION:g} rad; save the layout again')

    def world(x, y):
        return (round(cx+c*(x-.210)-s*(y-.1485), 12),
                round(cy+s*(x-.210)+c*(y-.1485), 12))

    points = [world(x, y) for y in (.0594, .1188, .1782, .2376)
              for x in (.060, .120, .180, .240, .300, .360)]
    xs = sorted({p[0] for p in points})
    ys = sorted({p[1] for p in points}, reverse=True)
    cells = [(x, y) for y in ys for x in xs]
    target = base.get('target_cell', 0)
    if type(target) is not int or not 0 <= target <= len(cells):
        raise ValueError('Select a valid integer target_cell on the plate')
    target_xy = cells[target-1] if target else (cx, cy)
    alignment_cells = [1, len(xs), len(cells), len(cells)-len(xs)+1]
    corners = [world(x, y) for x, y in ((0., 0.), (.420, 0.), (.420, .297), (0., .297))]
    xmin, xmax = min(p[0] for p in corners), max(p[0] for p in corners)
    ymin, ymax = min(p[1] for p in corners), max(p[1] for p in corners)
    if xmin < -1. or xmax > 1. or ymin < -.5 or ymax > .5:
        raise ValueError('The plate footprint must remain on the modelled 2 x 1 m workbench')
    ordered_corners = ((xmin, ymax), (xmax, ymax), (xmax, ymin), (xmin, ymin))
    origin = world(0., 0.)
    return {
        'geometry_profile': PROFILE, 'plate_enabled': True,
        'plate_center_xy': [cx, cy], 'plate_yaw_deg': yaw,
        'plate_bottom_z': bottom, 'plate_origin_xyz': [*origin, bottom],
        'plate_alignment_clearance': clearance, 'plate_mesh_resource': MESH_URI,
        'plate_alignment_tip_clearance': tip_clearance,
        'plate_alignment_gripper_position': alignment_gripper,
        'plate_task_revision': 5,
        'plate_alignment_policy': ALIGNMENT_POLICY,
        'columns': len(xs), 'rows': len(ys), 'size_x': round(xmax-xmin, 12),
        'size_y': round(ymax-ymin, 12), 'grid_yaw_deg': 0.,
        'cell_centers_xy': [v for p in cells for v in p],
        'plate_corners_xyz': [v for p in ordered_corners for v in (*p, bottom+.010)],
        'plate_alignment_points_xyz': [v for cell in alignment_cells for v in (*cells[cell-1], bottom+.005)],
        'plate_alignment_cell_ids': alignment_cells,
        'source_surface_z': bottom+.005, 'stack_surface_z': bottom+.005,
        'obstacle_surface_z': bottom+.005,
        'target_cell': target, 'stack_target_xy': list(target_xy),
        'independent_stack_target': False, 'target_has_foundation': True,
        # Keep the finger tips above the pocket rim while gripping the exposed
        # upper portion of a 25 mm cube; collision checking still decides validity.
        'grasp_height_above_table': .018,
    }
