"""Validate calibration against the project's actual plain URDF, without ROS."""
import copy
import importlib.util
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

import pytest
import yaml


_PACKAGE = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    'block_model_under_test', _PACKAGE / 'ur5e_block_bringup' / 'model.py')
model = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(model)
_PARTS = {
    'shoulder_pan_joint': 'shoulder',
    'shoulder_lift_joint': 'upper_arm',
    'elbow_joint': 'forearm',
    'wrist_1_joint': 'wrist_1',
    'wrist_2_joint': 'wrist_2',
    'wrist_3_joint': 'wrist_3',
}


@pytest.fixture
def source(tmp_path):
    original = _PACKAGE.parent / 'ur5e_block_description' / 'urdf' / 'ur5e_on_box.urdf'
    path = tmp_path / 'nominal.urdf'
    path.write_text(original.read_text(encoding='utf-8'), encoding='utf-8')
    return path


@pytest.fixture
def calibration(source, tmp_path):
    kinematics = {'hash': 'calib_test_fixture'}
    root = ET.fromstring(source.read_text())
    for index, (joint, part) in enumerate(_PARTS.items(), 1):
        origin = root.find(f"joint[@name='{joint}']/origin")
        values = [float(value) for value in
                  (origin.get('xyz') + ' ' + origin.get('rpy')).split()]
        kinematics[part] = dict(zip(
            ('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
            (value + index * 0.00001 for value in values)))
    document = {'kinematics': kinematics}
    path = tmp_path / 'factory.yaml'
    path.write_text(yaml.safe_dump(document))
    return path, document


def test_replaces_six_origins_preserves_the_rest_and_is_deterministic(source, calibration):
    path, document = calibration
    original = source.read_text()
    result = model.load_calibrated_description(source, path)
    assert result == model.load_calibrated_description(source, path)
    assert source.read_text() == original
    before, after = ET.fromstring(original), ET.fromstring(result)
    for joint, part in _PARTS.items():
        old = before.find(f"joint[@name='{joint}']/origin")
        new = after.find(f"joint[@name='{joint}']/origin")
        values = document['kinematics'][part]
        assert [float(value) for value in new.get('xyz').split()] == [
            values[name] for name in ('x', 'y', 'z')]
        assert [float(value) for value in new.get('rpy').split()] == [
            values[name] for name in ('roll', 'pitch', 'yaw')]
        # Removing only the permitted differences leaves the full source model,
        # including the workbench, meshes, limits and gripper mimic relations.
        new.attrib.clear()
        new.attrib.update(old.attrib)
    assert ET.tostring(after) == ET.tostring(before)


@pytest.mark.parametrize('fault', [
    'missing_hash', 'wrong_hash', 'empty_hash', 'missing_part', 'missing_field',
    'nan', 'infinity', 'bool', 'numeric_string', 'mapping', 'yaml_syntax',
])
def test_rejects_invalid_calibration(source, calibration, fault):
    path, document = calibration
    document = copy.deepcopy(document)
    kinematics = document['kinematics']
    if fault == 'missing_hash':
        del kinematics['hash']
    elif fault == 'wrong_hash':
        kinematics['hash'] = 'another_robot'
    elif fault == 'empty_hash':
        kinematics['hash'] = 'calib_'
    elif fault == 'missing_part':
        del kinematics['wrist_3']
    elif fault == 'missing_field':
        del kinematics['forearm']['yaw']
    elif fault == 'mapping':
        document['kinematics'] = []
    elif fault != 'yaml_syntax':
        kinematics['shoulder']['z'] = {
            'nan': float('nan'), 'infinity': float('inf'),
            'bool': True, 'numeric_string': '0.1625',
        }[fault]
    path.write_text('kinematics: [\n' if fault == 'yaml_syntax' else yaml.safe_dump(document))
    with pytest.raises(ValueError):
        model.load_calibrated_description(source, path)


@pytest.mark.parametrize('fault', [
    'axis', 'parent', 'child', 'missing_joint', 'duplicate_joint',
    'non_nominal', 'hardware',
])
def test_rejects_incompatible_source_model(source, calibration, fault):
    root = ET.fromstring(source.read_text())
    joint = root.find("joint[@name='shoulder_pan_joint']")
    if fault == 'axis':
        joint.find('axis').set('xyz', '0 0 -1')
    elif fault in ('parent', 'child'):
        joint.find(fault).set('link', 'workbench')
    elif fault == 'missing_joint':
        root.remove(joint)
    elif fault == 'duplicate_joint':
        root.append(copy.deepcopy(joint))
    elif fault == 'non_nominal':
        joint.find('origin').set('xyz', '0 0 0.1725')
    elif fault == 'hardware':
        ET.SubElement(root, 'ros2_control', name='FakeSystem', type='system')
    source.write_text(ET.tostring(root, encoding='unicode'))
    with pytest.raises(ValueError):
        model.load_calibrated_description(source, calibration[0])


@pytest.mark.parametrize('missing', ['urdf', 'calibration'])
def test_rejects_missing_paths(source, calibration, tmp_path, missing):
    urdf, factory = source, calibration[0]
    if missing == 'urdf':
        urdf = tmp_path / 'absent.urdf'
    else:
        factory = tmp_path / 'absent.yaml'
    with pytest.raises(FileNotFoundError):
        model.load_calibrated_description(urdf, factory)


@pytest.mark.parametrize('use_profile', [False, True])
def test_moveit_settings_reuses_exact_calibrated_xml_and_protects_table(
        source, calibration, monkeypatch, tmp_path, use_profile):
    calls = []
    packages = types.ModuleType('ament_index_python.packages')

    def share_directory(name):
        assert name == 'ur5e_block_description'
        return str(_PACKAGE.parent / name)

    packages.get_package_share_directory = share_directory
    monkeypatch.setitem(sys.modules, 'ament_index_python', types.ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)

    semantic_path = _PACKAGE.parent / 'ur5e_ag95_moveit_config' / 'config' / 'ur5e_on_box.srdf'
    original_semantic = semantic_path.read_text()

    class FakeModel:
        robot_description = {'robot_description': 'nominal wrapper'}

        def to_dict(self):
            return {'robot_description_kinematics': {'ur5e_arm': 'retained'},
                    'robot_description_semantic': original_semantic,
                    'robot_description_planning': {
                        'default_velocity_scaling_factor': 0.1,
                        'joint_limits': {
                            **{name: {'has_velocity_limits': True, 'max_velocity': 3.14159,
                                      'has_acceleration_limits': False}
                               for name in _PARTS},
                            'left_outer_knuckle_joint': {'max_velocity': 2.1,
                                                        'has_acceleration_limits': False},
                        }},
                    **self.robot_description}

    class FakeBuilder:
        def __init__(self, name, *, package_name):
            calls.append((name, package_name))

        def to_moveit_configs(self):
            return FakeModel()

    configs = types.ModuleType('moveit_configs_utils')
    configs.MoveItConfigsBuilder = FakeBuilder
    monkeypatch.setitem(sys.modules, 'moveit_configs_utils', configs)
    mounting = {'base_riser_height': 0.016, 'gripper_adapter_height': 0.0} if use_profile else {}
    profile = tmp_path / 'physical.yaml' if use_profile else None
    if use_profile:
        profile.write_text(yaml.safe_dump(mounting))
    settings = model.load_moveit_settings(calibration[0], profile)
    expected = model.load_calibrated_description(source, calibration[0], **mounting)
    assert settings['robot_description'] == expected
    assert 'ros2_control' not in settings['robot_description']
    assert settings['robot_description_kinematics'] == {'ur5e_arm': 'retained'}
    assert calls == [('ur5e_on_box', 'ur5e_ag95_moveit_config')]
    assert semantic_path.read_text() == original_semantic
    before = ET.fromstring(original_semantic)
    after = ET.fromstring(settings['robot_description_semantic'])
    retained = {(pair.get('link1'), pair.get('link2')) for pair in after.findall('disable_collisions')}
    for pair in before.findall('disable_collisions'):
        names = (pair.get('link1'), pair.get('link2'))
        if 'workbench' in names and 'base_link_inertia' not in names:
            assert names not in retained
        else:
            assert names in retained
    riser_contacts = {frozenset(names) for names in retained if 'base_riser' in names}
    assert riser_contacts == ({frozenset(('base_riser', 'workbench')),
                               frozenset(('base_riser', 'base_link_inertia'))}
                              if use_profile else set())
    planning = settings['robot_description_planning']
    assert planning['default_velocity_scaling_factor'] == 0.1
    for name in _PARTS:
        assert planning['joint_limits'][name] == {
            'has_velocity_limits': True, 'max_velocity': 3.14159,
            'has_acceleration_limits': True, 'max_acceleration': 1.0,
        }
    assert planning['joint_limits']['left_outer_knuckle_joint'] == {
        'max_velocity': 2.1, 'has_acceleration_limits': False}


@pytest.mark.parametrize('riser,adapter', [(0.016, 0.0), (0.0, 0.025), (0.3, 0.3)])
def test_mounting_extends_fixed_z_and_adds_only_a_local_support(source, calibration, riser, adapter):
    original = source.read_text()
    nominal = ET.fromstring(model.load_calibrated_description(source, calibration[0]))
    mounted = ET.fromstring(model.load_calibrated_description(
        source, calibration[0], riser, adapter))
    for name, expected in [('base_joint', (0.0, 0.0, 0.4 + riser)),
                           ('gripper_base_joint', (0.0, 0.0, adapter))]:
        before = nominal.find(f"joint[@name='{name}']/origin")
        after = mounted.find(f"joint[@name='{name}']/origin")
        assert tuple(map(float, after.get('xyz').split())) == pytest.approx(expected)
        assert after.get('rpy') == before.get('rpy')
        after.attrib.clear()
        after.attrib.update(before.attrib)
    before_table = nominal.find("link[@name='workbench']")
    after_table = mounted.find("link[@name='workbench']")
    # A 10 mm plate at a remote location must not be buried inside a
    # collision-only 16 mm extension of the whole table.
    assert ET.tostring(after_table) == ET.tostring(before_table)
    table_size = tuple(map(float, after_table.find('collision/geometry/box').get('size').split()))
    table_centre = tuple(map(float, after_table.find('collision/origin').get('xyz').split()))
    assert 0.4 + table_centre[2] + table_size[2] / 2.0 == pytest.approx(0.8)
    support = mounted.find("link[@name='base_riser']")
    support_joint = mounted.find("joint[@name='workbench_to_base_riser']")
    if riser:
        assert support is not None and support_joint is not None
        assert support_joint.get('type') == 'fixed'
        assert support_joint.find('parent').get('link') == 'workbench'
        assert support_joint.find('child').get('link') == 'base_riser'
        centre = tuple(map(float, support_joint.find('origin').get('xyz').split()))
        assert centre == pytest.approx((0.0, 0.0, 0.4 + riser / 2.0))
        visual = support.find('visual')
        collision = support.find('collision')
        visual_size = tuple(map(float, visual.find('geometry/box').get('size').split()))
        size = tuple(map(float, collision.find('geometry/box').get('size').split()))
        assert visual_size == size == pytest.approx((0.2, 0.2, riser))
        assert collision.find('origin').get('xyz') == visual.find('origin').get('xyz') == '0 0 0'
        assert visual.find('material/color').get('rgba').split()[-1] == '1'
        # Bottom touches the real table, top touches the unchanged raised base.
        assert 0.4 + centre[2] - size[2] / 2.0 == pytest.approx(0.8)
        assert 0.4 + centre[2] + size[2] / 2.0 == pytest.approx(0.8 + riser)
        # A plate pocket at x=.4 and z=.805 stays outside this local support.
        assert 0.4 > size[0] / 2.0
        mounted.remove(support)
        mounted.remove(support_joint)
    else:
        assert support is None and support_joint is None
    assert ET.tostring(mounted) == ET.tostring(nominal)
    assert source.read_text() == original


def test_riser_never_disables_collisions_with_a_moving_link(source, calibration):
    description = model.load_calibrated_description(source, calibration[0], 0.016)
    semantic = '''<robot name="ur5e_on_box">
      <disable_collisions link1="base_riser" link2="forearm_link" reason="User"/>
      <disable_collisions link1="left_finger_pad" link2="base_riser" reason="User"/>
      <disable_collisions link1="workbench" link2="wrist_3_link" reason="User"/>
      <disable_collisions link1="base_riser" link2="base_link_inertia" reason="Fixed"/>
    </robot>'''
    protected = ET.fromstring(model._protect_workbench(description, semantic))
    pairs = [frozenset((pair.get('link1'), pair.get('link2')))
             for pair in protected.findall('disable_collisions')]
    assert set(pairs) == {frozenset(('base_riser', 'workbench')),
                          frozenset(('base_riser', 'base_link_inertia'))}
    assert len(pairs) == len(set(pairs))  # Existing fixed contacts are not duplicated.


@pytest.mark.parametrize('tag,name', [('link', 'base_riser'),
                                     ('joint', 'workbench_to_base_riser')])
def test_rejects_a_source_that_already_contains_the_generated_support(source, calibration, tag, name):
    root = ET.fromstring(source.read_text())
    ET.SubElement(root, tag, name=name)
    source.write_text(ET.tostring(root, encoding='unicode'))
    with pytest.raises(ValueError, match='generated base riser'):
        model.load_calibrated_description(source, calibration[0], 0.016)


@pytest.mark.parametrize('joint,attribute,value', [
    ('base_joint', 'rpy', '0 0 0.1'),
    ('gripper_base_joint', 'rpy', '0 0.1 0'),
    ('base_joint', 'xyz', '0.1 0 0.4'),
    ('gripper_base_joint', 'xyz', '0 0 0.02'),
])
def test_rejects_incompatible_mounting_axes(source, calibration, joint, attribute, value):
    root = ET.fromstring(source.read_text())
    root.find(f"joint[@name='{joint}']/origin").set(attribute, value)
    source.write_text(ET.tostring(root, encoding='unicode'))
    with pytest.raises(ValueError):
        model.load_calibrated_description(source, calibration[0], 0.016, 0.0)


@pytest.mark.parametrize('value', [-0.001, 0.301, float('nan'), float('inf'), True, '0.016'])
def test_rejects_invalid_mounting_distances_in_direct_api(source, calibration, value):
    with pytest.raises(ValueError):
        model.load_calibrated_description(source, calibration[0], base_riser_height=value)


@pytest.mark.parametrize('document', [
    {'base_riser_height': 0.016, 'gripper_adapter_height': 0.0},
    {'base_riser_height': 0, 'gripper_adapter_height': 0.3},
])
def test_load_physical_profile_uses_explicit_metres(tmp_path, document):
    path = tmp_path / 'physical.yaml'
    path.write_text(yaml.safe_dump(document))
    assert model.load_physical_profile(path) == document


@pytest.mark.parametrize('document', [
    [], None, {'base_riser_height': 0.016},
    {'base_riser_height': 0.016, 'gripper_adapter_height': 0.0, 'unknown': 1},
    {'base_riser_height': -0.001, 'gripper_adapter_height': 0.0},
    {'base_riser_height': 0.016, 'gripper_adapter_height': 0.301},
    {'base_riser_height': float('nan'), 'gripper_adapter_height': 0.0},
    {'base_riser_height': 0.016, 'gripper_adapter_height': float('inf')},
    {'base_riser_height': True, 'gripper_adapter_height': 0.0},
    {'base_riser_height': 0.016, 'gripper_adapter_height': '0.0'},
])
def test_rejects_invalid_physical_profile(tmp_path, document):
    path = tmp_path / 'physical.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        model.load_physical_profile(path)


def test_rejects_duplicate_profile_keys_and_missing_file(tmp_path):
    path = tmp_path / 'physical.yaml'
    with pytest.raises(FileNotFoundError):
        model.load_physical_profile(path)
    path.write_text('base_riser_height: 0.1\nbase_riser_height: 0.016\ngripper_adapter_height: 0.0\n')
    with pytest.raises(ValueError, match='duplicate'):
        model.load_physical_profile(path)
