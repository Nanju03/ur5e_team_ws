"""Pure analytical geometry and collision-environment identity checks."""
from array import array
import copy
import importlib.util
import math
from pathlib import Path
import struct
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np
import pytest


_SPEC = importlib.util.spec_from_file_location(
    'air_scene_under_test', Path(__file__).resolve().parents[1]
    / 'ur5e_block_bringup' / 'air_scene.py')
air = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(air)


def header(frame='world'):
    return {'frame_id': frame, 'stamp': {'sec': 12, 'nanosec': 34}}


def collision_object(name):
    return {'id': name, 'header': header(), 'operation': 0,
            'primitives': [{'type': 1, 'dimensions': [0.1, 0.2, 0.3]}],
            'primitive_poses': [{'position': {'x': 0.1, 'y': 0.0, 'z': 0.8},
                                 'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}]}


@pytest.fixture
def scene():
    result = {
        'name': 'irrelevant scene label',
        'robot_state': {
            'joint_state': {'header': header(), 'name': ['joint'], 'position': [0.0]},
            'attached_collision_objects': [
                {'link_name': 'grasp_link', 'object': collision_object('held'),
                 'touch_links': ['right_finger', 'left_finger'],
                 'weight': 1.0, 'detach_posture': {'positions': [0.0]}}]},
        'world': {'collision_objects': [collision_object('table'), collision_object('wall')],
                  'octomap': {'header': header(), 'origin': {'position': {'x': 0.0}},
                              'octomap': {'header': header(), 'id': 'OcTree', 'binary': True,
                                          'resolution': 0.01, 'data': array('b', [-1, 0, 1])}}},
        'allowed_collision_matrix': {
            'entry_names': ['table', 'wrist', 'held'],
            'entry_values': [{'enabled': [False, False, True]},
                             {'enabled': [False, False, False]},
                             {'enabled': [True, False, False]}],
            'default_entry_names': ['wall', 'floor'], 'default_entry_values': [False, True]},
        'link_padding': [{'link_name': 'wrist', 'padding': 0.002},
                         {'link_name': 'finger', 'padding': 0.003}],
        'link_scale': [{'link_name': 'wrist', 'scale': 1.0},
                       {'link_name': 'finger', 'scale': 1.1}],
        'fixed_frame_transforms': [
            {'header': header(name), 'child_frame_id': 'world',
             'transform': {'translation': {'x': 0.1, 'y': 0., 'z': 0.},
                           'rotation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}
            for name in ('camera', 'fixture')],
        'object_colors': [{'id': 'table', 'color': {'r': 1.0}}],
    }
    result['world']['collision_objects'][0]['header']['frame_id'] = 'camera'
    result['world']['collision_objects'][1]['header']['frame_id'] = 'fixture'
    return result


def test_scene_hash_ignores_motion_stamps_colors_and_collection_order(scene):
    expected = air.scene_fingerprint(scene)
    changed = copy.deepcopy(scene)
    changed['name'] = 'renamed'
    changed['robot_state']['joint_state']['position'] = [float('nan')]
    changed['object_colors'] = []
    changed['robot_state']['attached_collision_objects'][0]['weight'] = 9.0
    changed['robot_state']['attached_collision_objects'][0]['detach_posture'] = {}
    changed['robot_state']['attached_collision_objects'][0]['touch_links'].reverse()
    for key in ('link_padding', 'link_scale', 'fixed_frame_transforms'):
        changed[key].reverse()
    changed['world']['collision_objects'].reverse()
    matrix = changed['allowed_collision_matrix']
    order = [2, 0, 1]
    old_names, old_values = matrix['entry_names'], matrix['entry_values']
    matrix['entry_names'] = [old_names[index] for index in order]
    matrix['entry_values'] = [
        {'enabled': [old_values[left]['enabled'][right] for right in order]} for left in order]
    matrix['default_entry_names'].reverse()
    matrix['default_entry_values'].reverse()

    def change_stamps(value):
        if isinstance(value, dict):
            if 'header' in value:
                value['header']['stamp'] = {'sec': 999, 'nanosec': 888}
            for child in value.values():
                change_stamps(child)
        elif isinstance(value, list):
            for child in value:
                change_stamps(child)

    change_stamps(changed)
    assert air.scene_fingerprint(changed) == expected
    assert len(expected) == 64


@pytest.mark.parametrize('change', [
    'object_pose', 'object_size', 'octomap_bytes', 'octomap_resolution', 'attached_link',
    'touch_links', 'acm_pair', 'acm_default', 'padding', 'scale', 'fixed_transform', 'frame',
])
def test_scene_hash_detects_collision_environment_changes(scene, change):
    original = air.scene_fingerprint(scene)
    if change == 'object_pose':
        scene['world']['collision_objects'][0]['primitive_poses'][0]['position']['z'] += 0.001
    elif change == 'object_size':
        scene['world']['collision_objects'][0]['primitives'][0]['dimensions'][0] += 0.001
    elif change == 'octomap_bytes':
        scene['world']['octomap']['octomap']['data'][0] = -2
    elif change == 'octomap_resolution':
        scene['world']['octomap']['octomap']['resolution'] *= 2
    elif change == 'attached_link':
        scene['robot_state']['attached_collision_objects'][0]['link_name'] = 'tool0'
    elif change == 'touch_links':
        scene['robot_state']['attached_collision_objects'][0]['touch_links'].append('palm')
    elif change == 'acm_pair':
        matrix = scene['allowed_collision_matrix']['entry_values']
        matrix[0]['enabled'][1] = matrix[1]['enabled'][0] = True
    elif change == 'acm_default':
        scene['allowed_collision_matrix']['default_entry_values'][0] = True
    elif change == 'padding':
        scene['link_padding'][0]['padding'] += 0.001
    elif change == 'scale':
        scene['link_scale'][0]['scale'] += 0.01
    elif change == 'fixed_transform':
        scene['fixed_frame_transforms'][0]['transform']['translation']['x'] += 0.001
    elif change == 'frame':
        scene['world']['collision_objects'][0]['header']['frame_id'] = 'base'
    assert air.scene_fingerprint(scene) != original


@pytest.mark.parametrize('fault', ['nan', 'duplicate_object', 'acm_shape', 'acm_asymmetric'])
def test_scene_hash_rejects_invalid_collision_state(scene, fault):
    if fault == 'nan':
        scene['link_padding'][0]['padding'] = float('nan')
    elif fault == 'duplicate_object':
        scene['world']['collision_objects'].append(copy.deepcopy(scene['world']['collision_objects'][0]))
    elif fault == 'acm_shape':
        scene['allowed_collision_matrix']['entry_values'].pop()
    elif fault == 'acm_asymmetric':
        scene['allowed_collision_matrix']['entry_values'][0]['enabled'][1] = True
    with pytest.raises(ValueError):
        air.scene_fingerprint(scene)


def test_scene_hash_accepts_ros_message_conversion(scene, monkeypatch):
    convert = types.ModuleType('rosidl_runtime_py.convert')
    sentinel = object()

    def to_ordered(message):
        assert message is sentinel
        return scene

    convert.message_to_ordereddict = to_ordered
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py', types.ModuleType('rosidl_runtime_py'))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.convert', convert)
    assert air.scene_fingerprint(sentinel) == air.scene_fingerprint(scene)


def fixed_transform(source, target='world', x=0.):
    return {'header': header(source), 'child_frame_id': target,
            'transform': {'translation': {'x': x, 'y': 0., 'z': 0.},
                          'rotation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}


def test_scene_hash_accepts_moveit_common_target_and_identity_entry(scene):
    # Real MoveIt copyTransforms() keys entries by their source header, not by
    # their repeated child/target name. Its planning-frame identity is valid.
    scene['fixed_frame_transforms'].append(fixed_transform('world'))
    expected = air.scene_fingerprint(scene)
    scene['fixed_frame_transforms'].reverse()
    assert air.scene_fingerprint(scene) == expected


def test_scene_hash_ignores_unused_moving_controller_frame(scene):
    expected = air.scene_fingerprint(scene)
    scene['fixed_frame_transforms'].append(fixed_transform('tool0_controller', x=1.295))
    assert air.scene_fingerprint(scene) == expected
    scene['fixed_frame_transforms'][-1]['transform']['translation']['x'] = 1.30
    scene['fixed_frame_transforms'][-1]['transform']['rotation'] = {
        'x': 0., 'y': 0., 'z': math.sin(.1), 'w': math.cos(.1)}
    assert air.scene_fingerprint(scene) == expected


@pytest.mark.parametrize('geometry', ['world_object', 'attached_object', 'octomap'])
def test_scene_hash_tracks_transforms_referenced_by_geometry(scene, geometry):
    scene['fixed_frame_transforms'].append(fixed_transform('sensor', x=.25))
    if geometry == 'world_object':
        scene['world']['collision_objects'][0]['header'] = header('sensor')
    elif geometry == 'attached_object':
        scene['robot_state']['attached_collision_objects'][0]['object']['header'] = header('sensor')
    else:
        scene['world']['octomap']['octomap']['header'] = header('sensor')
    expected = air.scene_fingerprint(scene)
    scene['fixed_frame_transforms'][-1]['transform']['translation']['x'] += .001
    assert air.scene_fingerprint(scene) != expected


def test_scene_hash_deduplicates_identical_transform_sources(scene):
    expected = air.scene_fingerprint(scene)
    same = copy.deepcopy(scene['fixed_frame_transforms'][0])
    same['header']['stamp'] = {'sec': 999, 'nanosec': 123}
    same['transform']['rotation']['w'] = -1.
    scene['fixed_frame_transforms'].append(same)
    assert air.scene_fingerprint(scene) == expected


@pytest.mark.parametrize('referenced', [False, True])
def test_scene_hash_rejects_conflicting_duplicate_source_even_when_unused(scene, referenced):
    source = 'camera' if referenced else 'unused'
    scene['fixed_frame_transforms'].extend([
        fixed_transform(source, x=.2), fixed_transform(source, x=.3)])
    with pytest.raises(ValueError, match='conflict for source frame'):
        air.scene_fingerprint(scene)


@pytest.mark.parametrize('fault', ['empty_source', 'empty_target', 'different_target', 'zero_quaternion', 'nan'])
def test_scene_hash_rejects_invalid_fixed_frame_transforms(scene, fault):
    item = fixed_transform('sensor')
    if fault == 'empty_source':
        item['header']['frame_id'] = ''
    elif fault == 'empty_target':
        item['child_frame_id'] = ''
    elif fault == 'different_target':
        item['child_frame_id'] = 'other_planning_frame'
    elif fault == 'zero_quaternion':
        item['transform']['rotation']['w'] = 0.
    else:
        item['transform']['translation']['x'] = float('nan')
    scene['fixed_frame_transforms'].append(item)
    with pytest.raises(ValueError):
        air.scene_fingerprint(scene)


@pytest.fixture
def robot():
    root = ET.Element('robot', name='analytical_six_joint_chain')
    for name in ['world', 'base', *[f'link{index}' for index in range(6)], 'grasp_link']:
        ET.SubElement(root, 'link', name=name)

    def joint(name, kind, parent, child, xyz, axis=None):
        item = ET.SubElement(root, 'joint', name=name, type=kind)
        ET.SubElement(item, 'parent', link=parent)
        ET.SubElement(item, 'child', link=child)
        ET.SubElement(item, 'origin', xyz=xyz, rpy='0 0 0')
        if axis:
            ET.SubElement(item, 'axis', xyz=axis)

    joint('mount', 'fixed', 'world', 'base', '0 0 0.8')
    for index in range(6):
        joint(f'q{index}', 'revolute', 'base' if index == 0 else f'link{index-1}',
              f'link{index}', '1 0 0', '0 0 1')
    joint('tcp_mount', 'fixed', 'link5', 'grasp_link', '0 0 0.19')
    return ET.tostring(root, encoding='unicode')


def positions(first=0.0):
    return dict(zip([f'q{index}' for index in range(6)], [first, 0., 0., 0., 0., 0.]))


def test_fk_matches_analytical_rotating_chain(robot):
    kinematics = air.RobotKinematics(robot)
    initial = kinematics.tcp(positions())
    rotated = kinematics.tcp(positions(math.pi / 2))
    assert initial[:3, 3] == pytest.approx([6.0, 0.0, 0.99])
    assert rotated[:3, 3] == pytest.approx([1.0, 5.0, 0.99])
    assert rotated[:3, :3] == pytest.approx(np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]))
    assert rotated[3] == pytest.approx([0., 0., 0., 1.])


def test_fk_composes_origin_rpy_before_non_z_joint_rotation():
    xml = '''<robot name="axes">
      <link name="world"/><link name="arm"/><link name="grasp_link"/>
      <joint name="turn" type="revolute"><parent link="world"/><child link="arm"/>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="0 2 0"/></joint>
      <joint name="tcp" type="fixed"><parent link="arm"/><child link="grasp_link"/>
        <origin xyz="1 0 0"/></joint></robot>'''
    result = air.RobotKinematics(xml).tcp({'turn': math.pi / 2})
    assert result[:3, 3] == pytest.approx([1., 2., 2.])


@pytest.mark.parametrize('fault', ['loop', 'disconnected', 'parent', 'axis', 'origin', 'duplicate_parent'])
def test_fk_rejects_invalid_frame_trees(robot, fault):
    root = ET.fromstring(robot)
    first = root.find("joint[@name='q0']")
    if fault == 'loop':
        root.find("joint[@name='mount']/parent").set('link', 'grasp_link')
    elif fault == 'disconnected':
        ET.SubElement(root, 'link', name='unconnected')
    elif fault == 'parent':
        first.find('parent').set('link', 'unknown')
    elif fault == 'axis':
        first.find('axis').set('xyz', '0 0 0')
    elif fault == 'origin':
        first.find('origin').set('xyz', '0 nan 0')
    elif fault == 'duplicate_parent':
        first.find('child').set('link', 'base')
    with pytest.raises(ValueError):
        air.RobotKinematics(ET.tostring(root, encoding='unicode'))


@pytest.mark.parametrize('fault', ['missing', 'unknown', 'nan', 'bool'])
def test_fk_rejects_invalid_joint_positions(robot, fault):
    values = positions()
    if fault == 'missing':
        values.pop('q5')
    elif fault == 'unknown':
        values['typo'] = values.pop('q5')
    else:
        values['q0'] = float('nan') if fault == 'nan' else True
    with pytest.raises(ValueError):
        air.RobotKinematics(robot).tcp(values)


def test_cartesian_metrics_and_uniform_slowdown(robot):
    names = list(positions())
    samples = [(0., list(positions().values())), (2., list(positions(math.pi / 2).values()))]
    result = air.cartesian_metrics(samples, robot, names)
    assert result['max_tcp_step'] == pytest.approx(math.sqrt(50))
    assert result['max_tcp_speed'] == pytest.approx(math.sqrt(50) / 2)
    assert result['bounds_xyz']['min'] == pytest.approx([1., 0., 0.99])
    assert result['bounds_xyz']['max'] == pytest.approx([6., 5., 0.99])
    slowed = air.cartesian_metrics([(time * 2, joints) for time, joints in samples], robot, names)
    assert slowed['max_tcp_speed'] == pytest.approx(result['max_tcp_speed'] / 2)
    assert slowed['max_tcp_step'] == result['max_tcp_step']
    assert slowed['bounds_xyz'] == result['bounds_xyz']


def test_single_sample_metrics_have_no_motion(robot):
    result = air.cartesian_metrics([(0., [0.] * 6)], robot, list(positions()))
    assert result['max_tcp_speed'] == result['max_tcp_step'] == 0.0
    assert result['bounds_xyz']['min'] == result['bounds_xyz']['max']


@pytest.mark.parametrize('samples', [
    [], [(0., [0.] * 6), (0., [0.] * 6)], [(1., [0.] * 6), (0., [0.] * 6)],
    [(float('inf'), [0.] * 6)], [(-1., [0.] * 6)], [(0., [float('nan')] * 6)],
    [(0., [0.] * 5)], [(0., [True] * 6)],
])
def test_cartesian_metrics_reject_invalid_samples(robot, samples):
    with pytest.raises(ValueError):
        air.cartesian_metrics(samples, robot, list(positions()))


@pytest.fixture
def gripper_robot(robot):
    root = ET.fromstring(robot)
    root.find("joint[@name='q0']/axis").set('xyz', '0 1 0')
    ET.SubElement(root, 'link', name='ag95_base_link')
    mount = ET.SubElement(root, 'joint', name='gripper_mount', type='fixed')
    ET.SubElement(mount, 'parent', link='link5')
    ET.SubElement(mount, 'child', link='ag95_base_link')
    root.find("joint[@name='tcp_mount']/parent").set('link', 'ag95_base_link')
    # This arm collision and AG95 visual geometry must never enter the bound.
    arm = root.find("link[@name='link5']")
    collision = ET.SubElement(arm, 'collision')
    ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size='1 1 100')
    base = root.find("link[@name='ag95_base_link']")
    visual = ET.SubElement(base, 'visual')
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size='1 1 100')

    def finger(name, child, centre, multiplier=None, offset=0.):
        link = ET.SubElement(root, 'link', name=child)
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', xyz=f'{centre} 0 0', rpy='0 0 0')
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size='0.02 0.02 0.02')
        joint = ET.SubElement(root, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent', link='ag95_base_link')
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'axis', xyz='0 1 0')
        ET.SubElement(joint, 'limit', lower='0' if multiplier is None else '-2',
                      upper='0.93' if multiplier is None else '2')
        if multiplier is not None:
            ET.SubElement(joint, 'mimic', joint='left_outer_knuckle_joint',
                          multiplier=str(multiplier), offset=str(offset))

    finger('left_outer_knuckle_joint', 'master_finger', .2)
    for index, (multiplier, offset) in enumerate([(1., 0.), (-1., .1), (.5, -.05), (1.5, .05), (.25, .01)]):
        finger(f'mimic{index}', f'finger{index}', .3 + index * .02, multiplier, offset)
    return ET.tostring(root, encoding='unicode')


def _finger_minima(arm_angle, hand):
    base_z = .8 - 5 * math.sin(arm_angle)
    fingers = [(.2, hand)] + [
        (.3 + index * .02, multiplier * hand + offset)
        for index, (multiplier, offset) in enumerate([(1., 0.), (-1., .1), (.5, -.05), (1.5, .05), (.25, .01)])]
    return [base_z - centre * math.sin(arm_angle + angle)
            - .01 * (abs(math.sin(arm_angle + angle)) + abs(math.cos(arm_angle + angle)))
            for centre, angle in fingers]


def test_gripper_bound_uses_all_five_mimic_fingers_and_cache(gripper_robot):
    kinematics = air.RobotKinematics(gripper_robot)
    value = kinematics.gripper_min_z(positions(), .5)
    assert value == pytest.approx(min(_finger_minima(0., .5)), abs=2e-9)
    assert value < _finger_minima(0., .5)[0]  # A mimic finger extends below the master.
    assert len(kinematics._gripper_cache) == 1
    again = kinematics.gripper_min_z(positions(.3), .5)
    assert again <= min(_finger_minima(.3, .5))
    assert len(kinematics._gripper_cache) == 1  # Arm pose is outside the posture cache.
    assert math.isfinite(again)


@pytest.mark.parametrize('arm,hand', [(0., 0.), (.4, .2), (-.5, .7), (.8, .93)])
def test_gripper_bound_is_conservative_after_world_rotation(gripper_robot, arm, hand):
    result = air.RobotKinematics(gripper_robot).gripper_min_z(positions(arm), hand)
    assert result <= min(_finger_minima(arm, hand))


def _base_geometry_only(gripper_robot, shape, origin='0 0 0', rpy='0 0 0'):
    root = ET.fromstring(gripper_robot)
    for link in root.findall('link'):
        if link.get('name') in {'ag95_base_link', 'master_finger', *[f'finger{i}' for i in range(5)]}:
            for collision in list(link.findall('collision')):
                link.remove(collision)
    base = root.find("link[@name='ag95_base_link']")
    collision = ET.SubElement(base, 'collision')
    ET.SubElement(collision, 'origin', xyz=origin, rpy=rpy)
    geometry = ET.SubElement(collision, 'geometry')
    geometry.append(shape)
    return ET.tostring(root, encoding='unicode')


@pytest.mark.parametrize('tag,attributes,half_z', [
    ('box', {'size': '.02 .04 .06'}, .03),
    ('cylinder', {'radius': '.05', 'length': '.2'}, .1),
    ('sphere', {'radius': '.05'}, .05),
])
def test_gripper_primitive_bounds_ignore_arm_and_visuals(gripper_robot, tag, attributes, half_z):
    xml = _base_geometry_only(gripper_robot, ET.Element(tag, attributes))
    result = air.RobotKinematics(xml).gripper_min_z(positions(), .5)
    assert result == pytest.approx(.8 - half_z, abs=2e-9)


def _write_triangle(path, binary):
    vertices = [(0., 0., 0.), (10., 0., -40.), (0., 20., 10.)]
    if binary:
        # Some binary exporters start their header with "solid" too.
        data = b'solid binary triangle'.ljust(80, b' ') + struct.pack('<I', 1)
        data += struct.pack('<12fH', 0., 0., 1., *[v for point in vertices for v in point], 0)
        path.write_bytes(data)
    else:
        path.write_text('solid triangle\n facet normal 0 0 1\n outer loop\n'
                        + ''.join(f' vertex {x} {y} {z}\n' for x, y, z in vertices)
                        + ' endloop\n endfacet\nendsolid triangle\n')


@pytest.mark.parametrize('binary', [False, True])
def test_gripper_stl_file_uri_scale_and_collision_origin(gripper_robot, tmp_path, binary):
    path = tmp_path / 'triangle mesh.stl'
    _write_triangle(path, binary)
    shape = ET.Element('mesh', filename=path.as_uri(), scale='.001 .001 .001')
    xml = _base_geometry_only(gripper_robot, shape, origin='0 0 -.02')
    kinematics = air.RobotKinematics(xml)
    assert kinematics.gripper_min_z(positions(), .5) == pytest.approx(.74, abs=2e-9)
    path.unlink()
    # Geometry is loaded once, with separate enclosures for measured postures.
    assert kinematics.gripper_min_z(positions(), .2) == pytest.approx(.74, abs=2e-9)


def test_gripper_package_mesh_and_collision_rotation(gripper_robot, tmp_path, monkeypatch):
    path = tmp_path / 'triangle.stl'
    _write_triangle(path, False)
    packages = types.ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda name: str(tmp_path) if name == 'test_gripper' else None
    monkeypatch.setitem(sys.modules, 'ament_index_python', types.ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    shape = ET.Element('mesh', filename='package://test_gripper/triangle.stl', scale='.001 .001 .001')
    xml = _base_geometry_only(gripper_robot, shape, origin='0 0 -.02', rpy=f'{math.pi/2} 0 0')
    assert air.RobotKinematics(xml).gripper_min_z(positions(), .5) == pytest.approx(.78, abs=2e-9)


@pytest.mark.parametrize('symlink_kind', ['directory', 'file'])
def test_gripper_package_mesh_supports_colcon_symlink_install(
        gripper_robot, tmp_path, monkeypatch, symlink_kind):
    source = tmp_path / 'src' / 'test_gripper' / 'meshes'
    source.mkdir(parents=True)
    _write_triangle(source / 'triangle.stl', True)
    share = tmp_path / 'install' / 'test_gripper' / 'share' / 'test_gripper'
    share.mkdir(parents=True)
    if symlink_kind == 'directory':
        (share / 'meshes').symlink_to(source, target_is_directory=True)
    else:
        (share / 'meshes').mkdir()
        (share / 'meshes' / 'triangle.stl').symlink_to(source / 'triangle.stl')
    assert not (share / 'meshes' / 'triangle.stl').resolve().is_relative_to(share)
    packages = types.ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda _: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python', types.ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    shape = ET.Element('mesh', filename='package://test_gripper/meshes/triangle.stl',
                       scale='.001 .001 .001')
    xml = _base_geometry_only(gripper_robot, shape)
    assert air.RobotKinematics(xml).gripper_min_z(positions(), .5) == pytest.approx(.76, abs=2e-9)


@pytest.mark.parametrize('relative', [
    '../outside.stl', 'meshes/../../outside.stl', '%2e%2e/outside.stl',
    '/absolute.stl', '%2Fabsolute.stl', 'meshes/..%2Foutside.stl',
])
def test_package_mesh_still_rejects_lexical_traversal_and_absolute_paths(
        tmp_path, monkeypatch, relative):
    packages = types.ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda _: str(tmp_path)
    monkeypatch.setitem(sys.modules, 'ament_index_python', types.ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    with pytest.raises(ValueError, match='package-relative'):
        air._mesh_path('package://test_gripper/' + relative)


@pytest.mark.parametrize('hand', [float('nan'), float('inf'), True, -.01, .94])
def test_gripper_bound_rejects_unmeasured_or_out_of_range_position(gripper_robot, hand):
    with pytest.raises(ValueError):
        air.RobotKinematics(gripper_robot).gripper_min_z(positions(), hand)


@pytest.mark.parametrize('fault', ['mimic_loop', 'derived_limit', 'independent_joint'])
def test_gripper_bound_rejects_invalid_mimic_models(gripper_robot, fault):
    root = ET.fromstring(gripper_robot)
    first = root.find("joint[@name='mimic0']")
    if fault == 'mimic_loop':
        first.find('mimic').set('joint', 'mimic1')
        root.find("joint[@name='mimic1']/mimic").set('joint', 'mimic0')
    elif fault == 'derived_limit':
        first.find('limit').set('upper', '.1')
    elif fault == 'independent_joint':
        first.remove(first.find('mimic'))
    with pytest.raises(ValueError):
        air.RobotKinematics(ET.tostring(root, encoding='unicode')).gripper_min_z(positions(), .5)


def test_gripper_bound_loads_actual_ag95_collision_assets_offline(monkeypatch):
    workspace = Path(__file__).resolve().parents[2]
    native_description = workspace / 'dh_ag95_gripper_ros2' / 'dh_ag95_description'
    packages = types.ModuleType('ament_index_python.packages')

    def share(name):
        assert name == 'dh_ag95_description'
        return str(native_description)

    packages.get_package_share_directory = share
    monkeypatch.setitem(sys.modules, 'ament_index_python', types.ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    xml = (workspace / 'ur5e_block_description' / 'urdf' / 'ur5e_on_box.urdf').read_text()
    kinematics = air.RobotKinematics(xml)
    joints = dict.fromkeys(['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], 0.)
    for hand in (0., .93):
        result = kinematics.gripper_min_z(joints, hand)
        assert math.isfinite(result)
        assert result < kinematics.tcp(joints)[2, 3] + .19
