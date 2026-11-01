"""Pure collision-scene hashing and URDF TCP measurements for air-path review."""
from collections.abc import Mapping, Sequence
import hashlib
import json
import math
from numbers import Integral, Real
from pathlib import Path
import struct
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import numpy as np


def _number(value, label):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f'{label} must be a finite number')
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f'{label} must be a finite number') from exc
    if not math.isfinite(result):
        raise ValueError(f'{label} must be a finite number')
    return result


def _mapping(value, label):
    if not isinstance(value, Mapping):
        raise ValueError(f'{label} must be a mapping')
    return value


def _canonical(value):
    """Normalize message containers and ignore header stamps, retaining frames."""
    if isinstance(value, Mapping):
        result = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError('Scene field names must be strings')
            if key == 'header':
                item = {name: entry for name, entry in _mapping(item, 'header').items()
                        if name != 'stamp'}
            result[key] = _canonical(item)
        return result
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        value = _number(value, 'Scene number')
        return 0.0 if value == 0.0 else value
    if value is None or isinstance(value, str):
        return value
    if isinstance(value, (Sequence, bytes, bytearray)):
        return [_canonical(item) for item in value]
    if isinstance(value, np.ndarray):
        return _canonical(value.tolist())
    raise ValueError(f'Unsupported scene value type: {type(value).__name__}')


def _named_entries(entries, key, label):
    result, seen = [], set()
    for entry in entries:
        entry = _mapping(entry, label)
        name = entry.get(key)
        if not isinstance(name, str) or not name or name in seen:
            raise ValueError(f'{label} requires unique nonempty {key} names')
        seen.add(name)
        result.append(_canonical(entry))
    return sorted(result, key=lambda entry: entry[key])


def _collision_objects(objects):
    return _named_entries(objects, 'id', 'Collision objects')


def _referenced_frames(value):
    """Collect the frames in which actual collision geometry is expressed."""
    frames = set()
    if isinstance(value, Mapping):
        for name, child in value.items():
            if name == 'header':
                frame = _mapping(child, 'Collision geometry header').get('frame_id', '')
                if not isinstance(frame, str):
                    raise ValueError('Collision geometry frame IDs must be strings')
                if frame:
                    frames.add(frame)
            else:
                frames.update(_referenced_frames(child))
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        for child in value:
            frames.update(_referenced_frames(child))
    return frames


def _fixed_frame_transforms(entries, referenced_frames):
    """Normalize MoveIt's source-to-planning-frame transform map.

    In PlanningScene, header.frame_id is the map's source key and every
    child_frame_id is the common target/planning frame. This differs from
    identifying a TF tree entry by its child. MoveIt also includes transforms
    for unused robot-controller frames here; their changing poses do not move
    collision geometry and must not invalidate an otherwise unchanged scene.
    """
    by_source = {}
    targets = set()
    for item in entries:
        item = _mapping(item, 'Fixed frame transform')
        source = _mapping(item.get('header', {}), 'Fixed frame header').get('frame_id')
        target = item.get('child_frame_id')
        if (not isinstance(source, str) or not source
                or not isinstance(target, str) or not target):
            raise ValueError('Fixed frame transforms require nonempty source and target frame names')
        transform = _mapping(item.get('transform'), 'Fixed frame transform value')
        translation = _mapping(transform.get('translation'), 'Fixed frame translation')
        rotation = _mapping(transform.get('rotation'), 'Fixed frame rotation')
        xyz = {axis: _number(translation.get(axis), 'Fixed frame translation') for axis in ('x', 'y', 'z')}
        quaternion = np.array([_number(rotation.get(axis), 'Fixed frame quaternion')
                               for axis in ('x', 'y', 'z', 'w')])
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError('Fixed frame quaternion must have a nonzero finite norm')
        quaternion /= norm  # MoveIt normalizes incoming quaternions as well.
        # q and -q express the same rotation. Keep one canonical sign.
        first = next(value for value in quaternion[::-1] if value != 0.)
        if first < 0.:
            quaternion *= -1.
        normalized = _canonical({
            'header': {'frame_id': source}, 'child_frame_id': target,
            'transform': {'translation': xyz,
                          'rotation': dict(zip(('x', 'y', 'z', 'w'), quaternion))},
        })
        if source in by_source and by_source[source] != normalized:
            raise ValueError(f'Fixed frame transforms conflict for source frame {source}')
        by_source[source] = normalized
        targets.add(target)
    if len(targets) > 1:
        raise ValueError('Fixed frame transforms must share one planning target frame')
    return [by_source[source] for source in sorted(by_source) if source in referenced_frames]


def _allowed_collisions(matrix):
    matrix = _mapping(matrix, 'Allowed collision matrix')
    names = list(matrix.get('entry_names', []))
    rows = list(matrix.get('entry_values', []))
    if (any(not isinstance(name, str) or not name for name in names)
            or len(names) != len(set(names)) or len(rows) != len(names)):
        raise ValueError('Allowed collision matrix has invalid names or dimensions')
    values = []
    for row in rows:
        row = list(_mapping(row, 'Allowed collision row').get('enabled', []))
        if len(row) != len(names) or any(not isinstance(item, (bool, np.bool_)) for item in row):
            raise ValueError('Allowed collision matrix must be a square boolean matrix')
        values.append([bool(item) for item in row])
    pairs = []
    for left in range(len(names)):
        for right in range(left, len(names)):
            if values[left][right] != values[right][left]:
                raise ValueError('Allowed collision matrix must be symmetric')
            first, second = sorted((names[left], names[right]))
            pairs.append([first, second, values[left][right]])
    default_names = list(matrix.get('default_entry_names', []))
    defaults = list(matrix.get('default_entry_values', []))
    if (len(default_names) != len(defaults)
            or any(not isinstance(name, str) or not name for name in default_names)
            or len(default_names) != len(set(default_names))
            or any(not isinstance(item, (bool, np.bool_)) for item in defaults)):
        raise ValueError('Allowed collision defaults must have unique names and boolean values')
    return {'pairs': sorted(pairs),
            'defaults': sorted([name, bool(value)] for name, value in zip(default_names, defaults))}


def scene_fingerprint(scene):
    """Hash collision-relevant PlanningScene fields, excluding robot joint states.

    Accept a ROS PlanningScene or its message_to_ordereddict representation.
    Object colors, scene names, message stamps and attached detach-postures do
    not describe collision geometry and are omitted. Octomap bytes are included.
    """
    if not isinstance(scene, Mapping):
        from rosidl_runtime_py.convert import message_to_ordereddict
        scene = message_to_ordereddict(scene)
    scene = _mapping(scene, 'Planning scene')
    world = dict(_mapping(scene.get('world', {}), 'World'))
    world['collision_objects'] = _collision_objects(world.get('collision_objects', []))
    robot = _mapping(scene.get('robot_state', {}), 'Robot state')
    attached, seen = [], set()
    for body in robot.get('attached_collision_objects', []):
        body = _mapping(body, 'Attached object')
        objects = _collision_objects([body.get('object', {})])
        name = objects[0]['id']
        link = body.get('link_name')
        if not isinstance(link, str) or not link or name in seen:
            raise ValueError('Attached objects require link names and unique object IDs')
        seen.add(name)
        touch = list(body.get('touch_links', []))
        if any(not isinstance(item, str) or not item for item in touch):
            raise ValueError('Attached object touch links must be nonempty names')
        attached.append({'link_name': link, 'object': objects[0], 'touch_links': sorted(set(touch))})
    references = _referenced_frames(world)
    for body in attached:
        references.update(_referenced_frames(body['object']))
    transforms = _fixed_frame_transforms(scene.get('fixed_frame_transforms', []), references)
    collision_state = {
        'world': world,
        'attached': sorted(attached, key=lambda body: body['object']['id']),
        'allowed_collisions': _allowed_collisions(scene.get('allowed_collision_matrix', {})),
        'link_padding': _named_entries(scene.get('link_padding', []), 'link_name', 'Link padding'),
        'link_scale': _named_entries(scene.get('link_scale', []), 'link_name', 'Link scale'),
        'fixed_frame_transforms': transforms,
    }
    encoded = json.dumps(_canonical(collision_state), sort_keys=True, ensure_ascii=False,
                         separators=(',', ':'), allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _vector(text, default, label):
    if text is None:
        return np.array(default, dtype=float)
    try:
        values = [float(value) for value in text.split()]
    except (AttributeError, ValueError, OverflowError) as exc:
        raise ValueError(f'Invalid URDF {label}') from exc
    if len(values) != 3 or not np.isfinite(values).all():
        raise ValueError(f'URDF {label} requires three finite values')
    return np.array(values, dtype=float)


def _origin(joint):
    origins = joint.findall('origin')
    if len(origins) > 1:
        raise ValueError(f'Duplicate URDF origin for {joint.get("name")}')
    origin = origins[0] if origins else None
    xyz = _vector(None if origin is None else origin.get('xyz'), (0, 0, 0), 'origin xyz')
    roll, pitch, yaw = _vector(
        None if origin is None else origin.get('rpy'), (0, 0, 0), 'origin rpy')
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
        math.cos(yaw), math.sin(yaw))
    transform = np.eye(4)
    transform[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    transform[:3, 3] = xyz
    return transform


def _revolute_motion(axis, angle):
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    sine, cosine = math.sin(angle), math.cos(angle)
    motion = np.eye(4)
    motion[:3, :3] = cosine * np.eye(3) + (1.0 - cosine) * np.outer(axis, axis) + sine * skew
    return motion


def _corners(minimum, maximum):
    return np.array([[x, y, z] for x in (minimum[0], maximum[0])
                     for y in (minimum[1], maximum[1])
                     for z in (minimum[2], maximum[2])], dtype=float)


def _transform_vertices(transform, vertices):
    with np.errstate(over='ignore', invalid='ignore'):
        result = vertices @ transform[:3, :3].T + transform[:3, 3]
    if not np.isfinite(result).all():
        raise ValueError('Collision geometry produced nonfinite vertices')
    return result


def _mesh_path(uri):
    if not isinstance(uri, str) or not uri:
        raise ValueError('Collision mesh requires a filename URI')
    parsed = urlsplit(uri)
    if parsed.query or parsed.fragment:
        raise ValueError('Collision mesh URI must not contain a query or fragment')
    if parsed.scheme == 'package':
        from ament_index_python.packages import get_package_share_directory
        if not parsed.netloc or not parsed.path.startswith('/') or not parsed.path[1:]:
            raise ValueError('package:// mesh URI requires a package and relative path')
        relative = unquote(parsed.path[1:])
        if (Path(relative).is_absolute() or '\\' in relative or '\x00' in relative
                or '..' in relative.split('/')):
            raise ValueError('Collision mesh URI must use a package-relative path without parent traversal')
        share = Path(get_package_share_directory(parsed.netloc))
        # colcon --symlink-install legitimately points resource directories or
        # individual meshes back into src. Validate the lexical URI above,
        # rather than requiring the resolved target to remain inside install.
        path = share / relative
    elif parsed.scheme == 'file':
        if parsed.netloc not in ('', 'localhost'):
            raise ValueError('Only local file:// collision meshes are supported')
        path = Path(unquote(parsed.path))
        if not path.is_absolute():
            raise ValueError('file:// collision mesh path must be absolute')
    elif not parsed.scheme and Path(uri).is_absolute():
        path = Path(uri)
    else:
        raise ValueError('Collision mesh must use package:// or local file://')
    if path.suffix.lower() != '.stl':
        raise ValueError('Only STL collision meshes are supported')
    if not path.is_file():
        raise FileNotFoundError(f'Collision mesh does not exist: {path}')
    return path


def _stl_vertices(uri):
    path = _mesh_path(uri)
    if path.stat().st_size > 64 * 1024 * 1024:
        raise ValueError('Collision STL exceeds the 64 MiB parsing limit')
    data = path.read_bytes()
    vertices = None
    if len(data) >= 84:
        triangles = struct.unpack_from('<I', data, 80)[0]
        expected = 84 + 50 * triangles
        if expected <= len(data) and not data[expected:].strip(b'\x00 \t\r\n'):
            if not triangles:
                raise ValueError('Collision STL has no triangles')
            dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
            vertices = np.frombuffer(data, dtype=dtype, count=triangles, offset=84)['vertices']
            vertices = vertices.reshape(-1, 3).astype(float)
    if vertices is None:
        try:
            text = data.decode('ascii')
            if not text.lstrip().lower().startswith('solid'):
                raise ValueError('ASCII STL must begin with solid')
            points = []
            for line in text.splitlines():
                tokens = line.split()
                if tokens and tokens[0].lower() == 'vertex':
                    if len(tokens) != 4:
                        raise ValueError('Malformed ASCII STL vertex')
                    points.append([float(value) for value in tokens[1:]])
            if not points or len(points) % 3:
                raise ValueError('ASCII STL must contain complete triangles')
            vertices = np.array(points, dtype=float)
        except (UnicodeDecodeError, ValueError, OverflowError) as exc:
            raise ValueError(f'Invalid collision STL: {path}') from exc
    if not np.isfinite(vertices).all():
        raise ValueError('Collision STL contains nonfinite vertices')
    return vertices


def _xml_number(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'Invalid URDF {label}') from exc
    if not math.isfinite(number):
        raise ValueError(f'URDF {label} must be finite')
    return number


def _link_collision_vertices(link):
    points = []
    for collision in link.findall('collision'):
        geometries = collision.findall('geometry')
        if len(geometries) != 1 or len(geometries[0]) != 1:
            raise ValueError('Each gripper collision requires exactly one shape')
        shape = geometries[0][0]
        if shape.tag == 'mesh':
            vertices = _stl_vertices(shape.get('filename'))
            scale = _vector(shape.get('scale'), (1, 1, 1), 'mesh scale')
            if np.any(scale <= 0):
                raise ValueError('Gripper collision mesh scale must be positive')
            vertices = vertices * scale
        elif shape.tag == 'box':
            size = _vector(shape.get('size'), (0, 0, 0), 'collision box size')
            if np.any(size <= 0):
                raise ValueError('Gripper collision box dimensions must be positive')
            vertices = _corners(-size / 2, size / 2)
        elif shape.tag in ('cylinder', 'sphere'):
            radius = _xml_number(shape.get('radius'), 'collision radius')
            length = (_xml_number(shape.get('length'), 'collision length')
                      if shape.tag == 'cylinder' else 2 * radius)
            if radius <= 0 or length <= 0:
                raise ValueError('Gripper collision radius/length must be positive')
            half = np.array([radius, radius, length / 2])
            vertices = _corners(-half, half)
        else:
            raise ValueError(f'Unsupported gripper collision geometry: {shape.tag}')
        points.append(_transform_vertices(_origin(collision), vertices))
    return np.concatenate(points) if points else None


class RobotKinematics:
    """Evaluate the validated world-to-grasp_link URDF tree without ROS."""
    def __init__(self, urdf_xml):
        if not isinstance(urdf_xml, str):
            raise ValueError('URDF must be an XML string')
        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError as exc:
            raise ValueError('Invalid URDF XML') from exc
        if root.tag != 'robot':
            raise ValueError('URDF root must be robot')
        names = [link.get('name') for link in root.findall('link')]
        if (any(not name for name in names) or len(names) != len(set(names))
                or 'world' not in names or 'grasp_link' not in names):
            raise ValueError('URDF requires unique link names, world and grasp_link')
        links, parents, children, joints = set(names), {}, {}, {}
        self._actuated = set()
        for joint in root.findall('joint'):
            name, kind = joint.get('name'), joint.get('type')
            if not name or name in joints or kind not in ('fixed', 'revolute', 'continuous'):
                raise ValueError('URDF has a duplicate/invalid joint or an unsupported joint type')
            parent_tags, child_tags = joint.findall('parent'), joint.findall('child')
            if len(parent_tags) != 1 or len(child_tags) != 1:
                raise ValueError(f'URDF joint {name} requires one parent and one child')
            parent, child = parent_tags[0].get('link'), child_tags[0].get('link')
            if parent not in links or child not in links or child in parents or parent == child:
                raise ValueError(f'URDF joint {name} has an invalid parent/child tree')
            axis_tags = joint.findall('axis')
            if len(axis_tags) > 1:
                raise ValueError(f'Duplicate URDF axis for {name}')
            axis = _vector(axis_tags[0].get('xyz') if axis_tags else None,
                           (1, 0, 0), 'joint axis')
            norm = float(np.linalg.norm(axis))
            if not math.isfinite(norm) or norm <= 0.0:
                raise ValueError(f'URDF joint {name} requires a nonzero finite axis')
            record = (name, kind, _origin(joint), axis / norm)
            joints[name] = record
            parents[child] = (parent, record)
            children.setdefault(parent, []).append(child)
            if kind != 'fixed':
                self._actuated.add(name)

        visited, visiting = set(), set()

        def visit(link):
            if link in visiting:
                raise ValueError('URDF parent/child tree contains a loop')
            if link in visited:
                return
            visiting.add(link)
            for child in children.get(link, []):
                visit(child)
            visiting.remove(link)
            visited.add(link)

        for link in links:
            visit(link)
        if links - set(parents) != {'world'}:
            raise ValueError('URDF must be one connected tree rooted at world')
        self._path = []
        child = 'grasp_link'
        while child != 'world':
            parent, record = parents[child]
            self._path.append(record)
            child = parent
        self._path.reverse()
        self._parents = parents
        self._children = children
        self._joint_xml = {joint.get('name'): joint for joint in root.findall('joint')}
        self._link_xml = {link.get('name'): link for link in root.findall('link')}
        self._gripper_geometry = None
        self._gripper_cache = {}

    def tcp(self, joints):
        """Return world_T_grasp_link for finite named joint positions in radians."""
        joints = _mapping(joints, 'Joint positions')
        if any(name not in self._actuated for name in joints):
            raise ValueError('Joint positions contain an unknown or fixed joint name')
        positions = {name: _number(value, f'Joint {name}') for name, value in joints.items()}
        transform = np.eye(4)
        for name, kind, origin, axis in self._path:
            transform = transform @ origin
            if kind != 'fixed':
                if name not in positions:
                    raise ValueError(f'Missing TCP-chain joint position: {name}')
                transform = transform @ _revolute_motion(axis, positions[name])
        if not np.isfinite(transform).all():
            raise ValueError('TCP transformation is not finite')
        return transform

    def _prepare_gripper_geometry(self):
        base, hand = 'ag95_base_link', 'left_outer_knuckle_joint'
        if base not in self._link_xml:
            raise ValueError('URDF has no AG95 base link')
        subtree, pending = {base}, [base]
        while pending:
            for child in self._children.get(pending.pop(), []):
                subtree.add(child)
                pending.append(child)
        if 'grasp_link' not in subtree or hand not in self._joint_xml:
            raise ValueError('AG95 geometry requires grasp_link and its master gripper joint')
        link = 'grasp_link'
        while link != base:
            link, record = self._parents[link]
            if record[1] != 'fixed':
                raise ValueError('AG95 grasp_link must be fixed relative to its base')
        moving = {self._parents[name][1][0] for name in subtree - {base}
                  if self._parents[name][1][1] != 'fixed'}
        if hand not in moving:
            raise ValueError('AG95 master joint must belong to its gripper subtree')
        mimics, limits = {}, {}
        for name in moving:
            joint = self._joint_xml[name]
            bounds, mimic = joint.findall('limit'), joint.findall('mimic')
            if joint.get('type') != 'revolute' or len(bounds) != 1 or len(mimic) > 1:
                raise ValueError('AG95 moving joints require finite revolute limits and at most one mimic')
            lower = _xml_number(bounds[0].get('lower'), f'{name} lower limit')
            upper = _xml_number(bounds[0].get('upper'), f'{name} upper limit')
            if lower > upper:
                raise ValueError('AG95 joint limits are reversed')
            limits[name] = (lower, upper)
            if name == hand:
                if mimic:
                    raise ValueError('AG95 measured master joint must not itself be a mimic')
            elif mimic:
                source = mimic[0].get('joint')
                if source not in moving:
                    raise ValueError('AG95 mimic source must be another gripper joint')
                mimics[name] = (
                    source, _xml_number(mimic[0].get('multiplier', '1'), 'mimic multiplier'),
                    _xml_number(mimic[0].get('offset', '0'), 'mimic offset'))
            else:
                raise ValueError('AG95 has an unmeasured independent moving joint')
        geometry = {name: points for name in sorted(subtree)
                    if (points := _link_collision_vertices(self._link_xml[name])) is not None}
        if not geometry:
            raise ValueError('AG95 subtree has no collision geometry')
        self._gripper_subtree = subtree
        self._gripper_mimics, self._gripper_limits = mimics, limits
        self._gripper_geometry = geometry

    def _gripper_corners(self, hand_position):
        if self._gripper_geometry is None:
            self._prepare_gripper_geometry()
        hand = 'left_outer_knuckle_joint'
        lower, upper = self._gripper_limits[hand]
        if not lower <= hand_position <= upper:
            raise ValueError('AG95 measured position exceeds its URDF limits')
        if hand_position in self._gripper_cache:
            return self._gripper_cache[hand_position]
        positions, resolving = {}, set()

        def position(name):
            if name in positions:
                return positions[name]
            if name in resolving:
                raise ValueError('AG95 mimic relations contain a loop')
            resolving.add(name)
            if name == hand:
                value = hand_position
            else:
                source, multiplier, offset = self._gripper_mimics[name]
                value = position(source) * multiplier + offset
            if not math.isfinite(value) or not self._gripper_limits[name][0] <= value <= self._gripper_limits[name][1]:
                raise ValueError(f'AG95 derived joint {name} exceeds its finite URDF limits')
            resolving.remove(name)
            positions[name] = value
            return value

        for name in self._gripper_limits:
            position(name)
        poses, pending = {'ag95_base_link': np.eye(4)}, ['ag95_base_link']
        while pending:
            parent = pending.pop()
            for child in self._children.get(parent, []):
                name, kind, origin, axis = self._parents[child][1]
                transform = poses[parent] @ origin
                if kind != 'fixed':
                    transform = transform @ _revolute_motion(axis, positions[name])
                poses[child] = transform
                pending.append(child)
        tcp_from_base = np.linalg.inv(poses['grasp_link'])
        points = np.concatenate([
            _transform_vertices(tcp_from_base @ poses[name], vertices)
            for name, vertices in self._gripper_geometry.items()])
        # A TCP-aligned box containing all primitive bounds and mesh vertices
        # remains a conservative enclosure after any rigid world rotation.
        minimum = np.nextafter(points.min(axis=0), -np.inf)
        maximum = np.nextafter(points.max(axis=0), np.inf)
        corners = _corners(minimum, maximum)
        corners.setflags(write=False)
        if len(self._gripper_cache) >= 32:
            self._gripper_cache.pop(next(iter(self._gripper_cache)))
        self._gripper_cache[hand_position] = corners
        return corners

    def gripper_min_z(self, joints, hand_position):
        """Conservative world lower Z of AG95 collision geometry, in metres.

        Uses the measured master position, deriving all mimic fingers. The
        TCP-frame collision enclosure is cached per hand value; visual meshes
        and arm links are deliberately excluded. No ROS nodes/devices are used.
        """
        joints = _mapping(joints, 'Joint positions')
        hand_position = _number(hand_position, 'AG95 measured position')
        if 'left_outer_knuckle_joint' in joints and not math.isclose(
                _number(joints['left_outer_knuckle_joint'], 'AG95 joint position'),
                hand_position, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError('AG95 position arguments disagree')
        world_from_tcp = self.tcp(joints)
        points = _transform_vertices(world_from_tcp, self._gripper_corners(hand_position))
        # Round outward, including a nanometre of arithmetic allowance.
        result = float(np.nextafter(points[:, 2].min() - 1e-9, -np.inf))
        if not math.isfinite(result):
            raise ValueError('AG95 world collision bound is nonfinite')
        return result


def cartesian_metrics(samples, urdf_xml, joint_names):
    """Measure sampled grasp-link translations; units are metres and seconds.

    Speeds and steps are between the supplied samples, rather than guarantees
    about unprovided points on a controller spline. bounds_xyz has min/max XYZ.
    """
    names = list(joint_names)
    if (len(names) != 6 or len(set(names)) != 6
            or any(not isinstance(name, str) or not name for name in names)):
        raise ValueError('Exactly six unique arm joint names are required')
    kinematics = RobotKinematics(urdf_xml)
    minimum = maximum = previous_position = previous_time = None
    max_speed = max_step = 0.0
    count = 0
    for sample in samples:
        if len(sample) != 2:
            raise ValueError('Each sample must contain time and six joint positions')
        stamp = _number(sample[0], 'Sample time')
        if stamp < 0.0 or previous_time is not None and stamp <= previous_time:
            raise ValueError('Sample times must be nonnegative and strictly ascending')
        values = list(sample[1])
        if len(values) != 6:
            raise ValueError('Each sample must contain six joint positions')
        position = kinematics.tcp(dict(zip(names, values)))[:3, 3]
        if count:
            step = float(np.linalg.norm(position - previous_position))
            speed = step / (stamp - previous_time)
            if not math.isfinite(step) or not math.isfinite(speed):
                raise ValueError('TCP step or speed is not finite')
            max_step, max_speed = max(max_step, step), max(max_speed, speed)
            minimum, maximum = np.minimum(minimum, position), np.maximum(maximum, position)
        else:
            minimum, maximum = position.copy(), position.copy()
        previous_position, previous_time = position, stamp
        count += 1
    if not count:
        raise ValueError('At least one timed joint sample is required')
    return {'max_tcp_speed': max_speed, 'max_tcp_step': max_step,
            'bounds_xyz': {'min': minimum.tolist(), 'max': maximum.tolist()}}
