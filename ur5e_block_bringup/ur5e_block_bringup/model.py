"""Apply the robot's factory kinematics to the existing workbench/AG95 model."""
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


# The six origins below are the nominal UR5e chain in the supplied plain URDF.
# Their YAML names are the names consumed by Universal Robots' ur_macro.xacro.
_ARM_CHAIN = (
    ('shoulder_pan_joint', 'shoulder', 'base_link_inertia', 'shoulder_link',
     (0.0, 0.0, 0.1625), (0.0, 0.0, 0.0)),
    ('shoulder_lift_joint', 'upper_arm', 'shoulder_link', 'upper_arm_link',
     (0.0, 0.0, 0.0), (1.570796327, 0.0, 0.0)),
    ('elbow_joint', 'forearm', 'upper_arm_link', 'forearm_link',
     (-0.425, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ('wrist_1_joint', 'wrist_1', 'forearm_link', 'wrist_1_link',
     (-0.3922, 0.0, 0.1333), (0.0, 0.0, 0.0)),
    ('wrist_2_joint', 'wrist_2', 'wrist_1_link', 'wrist_2_link',
     (0.0, -0.0997, -2.044881182297852e-11), (1.570796327, 0.0, 0.0)),
    ('wrist_3_joint', 'wrist_3', 'wrist_2_link', 'wrist_3_link',
     (0.0, 0.0996, -2.042830148012698e-11),
     (1.570796326589793, math.pi, math.pi)),
)
_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
_PHYSICAL_FIELDS = ('base_riser_height', 'gripper_adapter_height')
# Only the height has been measured. This square mounting proxy covers the
# supplied base collision mesh's ~151 x 151 mm footprint with an extra margin;
# 200 x 200 mm is a model assumption, not a measurement of the real fixture.
_RISER_FOOTPRINT = (0.2, 0.2)
_RISER_LINK = 'base_riser'
_RISER_JOINT = 'workbench_to_base_riser'


def _read_file(path, purpose):
    path = Path(path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f'{purpose} file does not exist: {path}')
    return path, path.read_text(encoding='utf-8')


def _calibration(path):
    path, text = _read_file(path, 'Calibration')
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError(f'Invalid calibration YAML: {path}') from exc
    if not isinstance(document, dict) or not isinstance(document.get('kinematics'), dict):
        raise ValueError(f'Calibration must contain a kinematics mapping: {path}')
    kinematics = document['kinematics']
    calibration_hash = kinematics.get('hash')
    if not isinstance(calibration_hash, str) or not re.fullmatch(
            r'calib_[A-Za-z0-9_-]+', calibration_hash):
        raise ValueError('Calibration kinematics.hash must be a nonempty calib_ identifier')
    result = {}
    for _, part, *_ in _ARM_CHAIN:
        values = kinematics.get(part)
        if not isinstance(values, dict):
            raise ValueError(f'Calibration is missing kinematics.{part}')
        converted = []
        for field in _FIELDS:
            value = values.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'Calibration kinematics.{part}.{field} must be a finite number')
            try:
                number = float(value)
            except (ValueError, OverflowError) as exc:
                raise ValueError(f'Calibration kinematics.{part}.{field} is invalid') from exc
            if not math.isfinite(number):
                raise ValueError(f'Calibration kinematics.{part}.{field} must be finite')
            converted.append(number)
        result[part] = tuple(converted)
    return result


def _physical_distance(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number of metres in 0..0.3')
    try:
        number = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must be a finite number of metres in 0..0.3') from exc
    if not math.isfinite(number) or not 0.0 <= number <= 0.3:
        raise ValueError(f'{name} must be a finite number of metres in 0..0.3')
    return number


def load_physical_profile(profile_path):
    """Read the two explicit mounting distances in metres; reject extra keys."""
    path, text = _read_file(profile_path, 'Physical profile')
    try:
        document = yaml.safe_load(text)
        # Duplicate mounting keys are ambiguous even if their last value is valid.
        node = yaml.compose(text)
    except yaml.YAMLError as exc:
        raise ValueError(f'Invalid physical profile YAML: {path}') from exc
    if not isinstance(document, dict) or set(document) != set(_PHYSICAL_FIELDS):
        raise ValueError('Physical profile must contain only base_riser_height and gripper_adapter_height')
    if len(node.value) != len(document):
        raise ValueError('Physical profile contains duplicate keys')
    return {name: _physical_distance(document[name], name) for name in _PHYSICAL_FIELDS}


def _one_child(joint, tag):
    matches = joint.findall(tag)
    if len(matches) != 1:
        raise ValueError(f'URDF joint {joint.get("name")} must have exactly one {tag}')
    return matches[0]


def _vector(text, label):
    try:
        values = tuple(float(value) for value in text.split())
    except (AttributeError, ValueError, OverflowError) as exc:
        raise ValueError(f'Invalid URDF vector: {label}') from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f'URDF {label} must contain three finite numbers')
    return values


def _matches(actual, expected):
    return all(math.isclose(left, right, rel_tol=0.0, abs_tol=1e-8)
               for left, right in zip(actual, expected))


def _numbers(values):
    return ' '.join(format(0.0 if value == 0.0 else value, '.17g') for value in values)


def _riser_mount(root, riser):
    """Validate the nominal table and return its local top mounting plane."""
    if not riser:
        return None
    if (root.find(f"link[@name='{_RISER_LINK}']") is not None
            or root.find(f"joint[@name='{_RISER_JOINT}']") is not None):
        raise ValueError('The source URDF already contains the generated base riser')
    links = [link for link in root.findall('link') if link.get('name') == 'workbench']
    if len(links) != 1:
        raise ValueError('A raised mounting requires exactly one workbench link')
    collision = _one_child(links[0], 'collision')
    origin = _one_child(collision, 'origin')
    xyz = _vector(origin.get('xyz'), 'workbench collision origin xyz')
    rpy = _vector(origin.get('rpy'), 'workbench collision origin rpy')
    geometry = _one_child(collision, 'geometry')
    if len(geometry) != 1 or geometry[0].tag != 'box':
        raise ValueError('A raised mounting requires one workbench collision box')
    box = geometry[0]
    size = _vector(box.get('size'), 'workbench collision box size')
    if (not _matches(xyz, (0.0, 0.0, 0.0)) or not _matches(rpy, (0.0, 0.0, 0.0))
            or size[0] <= 0.0 or size[1] <= 0.0
            or not math.isclose(size[2], 0.8, rel_tol=0.0, abs_tol=1e-8)):
        raise ValueError('The workbench collision box must have its nominal centre, axes and 0.8 m height')
    return xyz[2] + size[2] / 2.0


def _add_riser(root, height, mount_z):
    """Fill only the physical base gap with a visible fixed square support."""
    link = ET.SubElement(root, 'link', name=_RISER_LINK)
    for tag in ('visual', 'collision'):
        element = ET.SubElement(link, tag)
        ET.SubElement(element, 'origin', xyz='0 0 0', rpy='0 0 0')
        geometry = ET.SubElement(element, 'geometry')
        ET.SubElement(geometry, 'box', size=_numbers((*_RISER_FOOTPRINT, height)))
        if tag == 'visual':
            material = ET.SubElement(element, 'material', name='base_riser_gray')
            ET.SubElement(material, 'color', rgba='0.30 0.32 0.35 1')
    joint = ET.SubElement(root, 'joint', name=_RISER_JOINT, type='fixed')
    ET.SubElement(joint, 'parent', link='workbench')
    ET.SubElement(joint, 'child', link=_RISER_LINK)
    ET.SubElement(joint, 'origin', xyz=_numbers((0.0, 0.0, mount_z + height / 2.0)),
                  rpy='0 0 0')


def load_calibrated_description(urdf_path, calibration_path,
                                base_riser_height=0.0, gripper_adapter_height=0.0):
    """Return one calibrated XML string; neither input file is modified.

    Calibration values replace the nominal origins, rather than being offsets.
    Reject hardware wrappers or a different/already calibrated source chain.
    Mounting distances extend the fixed joints along their unchanged local +Z.
    A raised base adds a local square support without raising the tabletop.
    """
    base_riser_height = _physical_distance(base_riser_height, 'base_riser_height')
    gripper_adapter_height = _physical_distance(gripper_adapter_height, 'gripper_adapter_height')
    _, text = _read_file(urdf_path, 'URDF')
    calibration = _calibration(calibration_path)
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise ValueError(f'Invalid URDF XML: {urdf_path}') from exc
    if root.tag != 'robot':
        raise ValueError('The source URDF root must be robot')
    if any(element.tag.rsplit('}', 1)[-1] == 'ros2_control' for element in root.iter()):
        raise ValueError('The source URDF must be plain, without ros2_control hardware')
    if any(element.tag.startswith('{http://www.ros.org/wiki/xacro}')
           for element in root.iter()):
        raise ValueError('The source URDF must already be expanded, without xacro elements')

    links = {link.get('name') for link in root.findall('link')}
    origins = []
    for name, part, parent, child, nominal_xyz, nominal_rpy in _ARM_CHAIN:
        joints = [joint for joint in root.findall('joint') if joint.get('name') == name]
        if len(joints) != 1:
            raise ValueError(f'The source URDF must contain exactly one joint {name}')
        joint = joints[0]
        if joint.get('type') != 'revolute':
            raise ValueError(f'The source URDF joint {name} must be revolute')
        if (_one_child(joint, 'parent').get('link') != parent
                or _one_child(joint, 'child').get('link') != child
                or parent not in links or child not in links):
            raise ValueError(f'The source URDF joint {name} has an unexpected parent/child chain')
        axis = _vector(_one_child(joint, 'axis').get('xyz'), f'{name} axis')
        if not _matches(axis, (0.0, 0.0, 1.0)):
            raise ValueError(f'The source URDF joint {name} must rotate around +Z')
        origin = _one_child(joint, 'origin')
        xyz = _vector(origin.get('xyz'), f'{name} origin xyz')
        rpy = _vector(origin.get('rpy'), f'{name} origin rpy')
        if not _matches(xyz, nominal_xyz) or not _matches(rpy, nominal_rpy):
            raise ValueError(f'The source URDF joint {name} does not have the nominal UR5e origin')
        origins.append((origin, calibration[part]))

    mounts = []
    for name, parent, child, nominal_z, extension in (
            ('base_joint', 'workbench', 'base_link', 0.4, base_riser_height),
            ('gripper_base_joint', 'tool0', 'ag95_base_link', 0.0, gripper_adapter_height)):
        joints = [joint for joint in root.findall('joint') if joint.get('name') == name]
        if len(joints) != 1 or joints[0].get('type') != 'fixed':
            raise ValueError(f'The source URDF must contain exactly one fixed mounting joint {name}')
        joint = joints[0]
        if (_one_child(joint, 'parent').get('link') != parent
                or _one_child(joint, 'child').get('link') != child
                or parent not in links or child not in links):
            raise ValueError(f'The source URDF mounting joint {name} has an unexpected parent/child chain')
        origin = _one_child(joint, 'origin')
        xyz = _vector(origin.get('xyz'), f'{name} origin xyz')
        rpy = _vector(origin.get('rpy'), f'{name} origin rpy')
        if not _matches(xyz, (0.0, 0.0, nominal_z)) or not _matches(rpy, (0.0, 0.0, 0.0)):
            raise ValueError(f'The source URDF mounting joint {name} must have its nominal translation and identity rotation')
        if joint.find('axis') is not None:
            axis = _vector(_one_child(joint, 'axis').get('xyz'), f'{name} axis')
            if not _matches(axis, (0.0, 0.0, 1.0)):
                raise ValueError(f'The source URDF mounting joint {name} must use +Z')
        mounts.append((origin, xyz, extension))

    riser_mount = _riser_mount(root, base_riser_height)
    # Validate the whole source before changing any of its in-memory elements.
    for origin, values in origins:
        origin.set('xyz', _numbers(values[:3]))
        origin.set('rpy', _numbers(values[3:]))
    for origin, xyz, extension in mounts:
        if extension:
            origin.set('xyz', _numbers((xyz[0], xyz[1], xyz[2] + extension)))
    if riser_mount is not None:
        _add_riser(root, base_riser_height, riser_mount)
    # Share this exact result with RSP, MoveIt and the block client. Their guard
    # compares description bytes, including serialization and hardware wrappers.
    return ET.tostring(root, encoding='unicode', short_empty_elements=True)


def _protect_workbench(description, semantic):
    """Check moving links against the tabletop and local mounting support."""
    robot = ET.fromstring(description)
    try:
        srdf = ET.fromstring(semantic)
    except (ET.ParseError, TypeError) as exc:
        raise ValueError('Invalid MoveIt semantic description') from exc
    if srdf.tag != 'robot':
        raise ValueError('The semantic description root must be robot')
    descendants = {}
    fixed_neighbors = {}
    moving = set()
    for joint in robot.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        descendants.setdefault(parent, set()).add(child)
        if joint.get('type') != 'fixed':
            moving.add(child)
        else:
            fixed_neighbors.setdefault(parent, set()).add(child)
            fixed_neighbors.setdefault(child, set()).add(parent)
    pending = list(moving)
    while pending:
        for child in descendants.get(pending.pop(), ()):
            if child not in moving:
                moving.add(child)
                pending.append(child)
    fixtures = {'workbench', _RISER_LINK}
    for pair in list(srdf.findall('disable_collisions')):
        left, right = pair.get('link1'), pair.get('link2')
        if (left in fixtures and right in moving
                or right in fixtures and left in moving):
            srdf.remove(pair)
    if robot.find(f"link[@name='{_RISER_LINK}']") is not None:
        # The support touches the tabletop and the fixed base mesh. Suppress
        # only these fixed mounting contacts, never contacts with a moving arm
        # or gripper link. Fixed aliases with no geometry need no exception.
        fixed = {_RISER_LINK}
        pending = [_RISER_LINK]
        while pending:
            for neighbor in fixed_neighbors.get(pending.pop(), ()):
                if neighbor not in fixed:
                    fixed.add(neighbor)
                    pending.append(neighbor)
        existing = {frozenset((pair.get('link1'), pair.get('link2')))
                    for pair in srdf.findall('disable_collisions')}
        for name in sorted(fixed - {_RISER_LINK}):
            link = robot.find(f"link[@name='{name}']")
            if link is not None and link.find('collision') is not None:
                pair = frozenset((_RISER_LINK, name))
                if pair not in existing:
                    ET.SubElement(srdf, 'disable_collisions', link1=_RISER_LINK,
                                  link2=name, reason='Fixed mounting contact')
    return ET.tostring(srdf, encoding='unicode', short_empty_elements=True)


def load_moveit_settings(calibration_path, physical_profile=None):
    """Load existing planning settings with the same calibrated plain URDF.

    The caller supplies the real arm/gripper controller mappings separately.
    Imports are delayed so calibration validation can be tested without ROS.
    """
    from ament_index_python.packages import get_package_share_directory
    from moveit_configs_utils import MoveItConfigsBuilder

    share = Path(get_package_share_directory('ur5e_block_description'))
    mounting = load_physical_profile(physical_profile) if physical_profile is not None else {}
    description = load_calibrated_description(
        share / 'urdf' / 'ur5e_on_box.urdf', calibration_path, **mounting)
    model = MoveItConfigsBuilder(
        'ur5e_on_box', package_name='ur5e_ag95_moveit_config').to_moveit_configs()
    model.robot_description = {'robot_description': description}
    settings = model.to_dict()
    settings['robot_description_semantic'] = _protect_workbench(
        description, settings.get('robot_description_semantic'))
    limits = settings.setdefault('robot_description_planning', {}).setdefault('joint_limits', {})
    for joint, *_ in _ARM_CHAIN:
        limits.setdefault(joint, {}).update(has_acceleration_limits=True, max_acceleration=1.0)
    return settings
