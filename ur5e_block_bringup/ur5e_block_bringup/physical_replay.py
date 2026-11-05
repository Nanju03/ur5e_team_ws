"""Sealed, self-contained physical path packages; no ROS clients or actuators.

Collision certificates are produced once by the planning/offline preparation
process. Playback checks their context and measured finger range, not FCL again.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import uuid

POLICY = 'physical_prevalidated_finger_grid_v2'
HOLDING_LOWER = .70
HOLDING_UPPER = .733
FINGER_STEP = .005
EMPTY_START_TOLERANCE = .015
EMPTY_MOTION_TOLERANCE = .003
MAX_PACKAGE_BYTES = 256 * 1024 * 1024
ARTIFACT_FIELDS = ('file', 'scene_before_file', 'scene_after_file')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def finger_range(step, holding, bounds=None):
    """Cover accepted measured starts plus the existing moving hand watchdog.

    Held paths retain the original measured-anchor watchdog as well. An anchor
    whose allowed interval extends below .70 requires a different prepared path.
    Finger commands cover the entire opening/closing sweep, including endpoint
    error. These are discrete .005rad geometry samples, as in live validation.
    """
    start = step['start_joints'][-1]
    if holding:
        low, high = HOLDING_LOWER, HOLDING_UPPER
    else:
        # Before an empty-hand arm move the preceding command already required
        # <=.003rad endpoint error. Cover that plus the .003rad moving watchdog.
        # Stationary finger commands retain the original .015rad start range.
        margin = (EMPTY_START_TOLERANCE + EMPTY_MOTION_TOLERANCE
                  if step['kind'] == 'hand' else 2*EMPTY_MOTION_TOLERANCE)
        low, high = start-margin, start+margin
    if step['kind'] == 'hand':
        target = step['hand_target']
        low = min(low, target-EMPTY_MOTION_TOLERANCE)
        high = max(high, target+EMPTY_MOTION_TOLERANCE)
    if bounds is not None:
        low, high = max(low, bounds[0]), min(high, bounds[1])
    return low, high


def finger_shapes(step, holding, bounds):
    low, high = finger_range(step, holding, bounds)
    if not bounds[0] <= low <= high <= bounds[1]:
        raise ValueError('Prevalidated finger range exceeds the gripper joint limit')
    count = max(1, math.ceil((high-low)/FINGER_STEP))
    return sorted(set([low+(high-low)*i/count for i in range(count+1)] +
                      [step['start_joints'][-1], step['end_joints'][-1]]))


def motion_context(step):
    return sha(json.dumps({key: step.get(key) for key in
        ('index', 'kind', 'phase', 'start_joints', 'end_joints', 'hand_target',
         'velocity_scaling', 'acceleration_scaling')},
        sort_keys=True, allow_nan=False).encode())


def holding_steps(steps):
    holding = False
    for step in steps:
        yield step, holding
        if step['phase'] == 'attach':
            holding = True
        elif step['phase'] == 'place':
            holding = False


def check_certificates(recipe, metadata):
    if recipe.get('replay_policy') != POLICY or metadata.get('replay_policy') != POLICY:
        raise ValueError('Saved path needs one offline preparation before fast replay')
    steps = recipe['steps']
    bounds = metadata.get('finger_joint_bounds')
    if (not isinstance(bounds, list) or len(bounds) != 2
            or not all(finite_number(value) for value in bounds) or bounds[0] >= bounds[1]):
        raise ValueError('Saved path finger bounds are missing')
    required = {recipe.get('initial_scene_file'), recipe.get('initial_state_file')}
    required.update(step[key] for step in steps for key in ARTIFACT_FIELDS if step.get(key))
    required.discard(None)
    if (any(not isinstance(name, str) or Path(name).name != name or name in ('', '.', '..') for name in required)
            or not required <= metadata['artifact_sha256'].keys()):
        raise ValueError('Saved path references an unsealed artifact')
    expected_scene = metadata['scene_sha256']
    for step, holding in holding_steps(steps):
        if step.get('scene_before_sha256') != expected_scene:
            raise ValueError('Saved path scene chain is discontinuous')
        expected_scene = step.get('scene_after_sha256')
        if not isinstance(expected_scene, str) or not re.fullmatch('[0-9a-f]{64}', expected_scene):
            raise ValueError('Saved path has no sealed scene fingerprint')
        if step['kind'] == 'scene':
            continue
        if expected_scene != step['scene_before_sha256']:
            raise ValueError('A saved motion unexpectedly changes collision geometry')
        cert = step.get('collision_certificate', {})
        if (cert.get('policy') != POLICY or cert.get('context_sha256') != motion_context(step)
                or cert.get('scene_sha256') != step['scene_before_sha256']
                or cert.get('trajectory_sha256') != metadata['artifact_sha256'].get(step['file'])
                or not isinstance(cert.get('trajectory_sha256'), str)
                or not re.fullmatch('[0-9a-f]{64}', cert['trajectory_sha256'])
                or not isinstance(cert.get('trajectory_content_sha256'), str)
                or not re.fullmatch('[0-9a-f]{64}', cert['trajectory_content_sha256'])
                or cert.get('holding') is not holding
                or cert.get('finger_range') != list(finger_range(step, holding, metadata['finger_joint_bounds']))
                or cert.get('finger_step') != FINGER_STEP
                or type(cert.get('checked_samples')) is not int
                or not 1 <= cert['checked_samples'] <= 1_000_000):
            raise ValueError('Saved motion collision certificate is missing or changed')


def initial_hand_matches(recipe, actual, bounds):
    if not finite_number(actual):
        return False
    first = recipe['steps'][0]
    if first['kind'] == 'hand' and first['phase'] == 'open':
        low, high = first['collision_certificate']['finger_range']
        return low <= actual <= high
    if first['kind'] == 'arm':
        low, high = first['collision_certificate']['finger_range']
        return (low <= max(actual-EMPTY_MOTION_TOLERANCE, bounds[0])
                <= min(actual+EMPTY_MOTION_TOLERANCE, bounds[1]) <= high)
    return abs(actual-recipe['initial_joints'][-1]) <= EMPTY_START_TOLERANCE


def context_key(metadata):
    return sha(json.dumps({key: metadata.get(key) for key in
        ('input_sha256', 'model_sha256', 'semantic_sha256', 'hardware_class',
         'scene_hash_policy', 'recipe_policy', 'replay_policy', 'grasp_policy',
         'arm_execution_profile', 'finger_joint_bounds')}, sort_keys=True, allow_nan=False).encode())


def read_package(directory, recipe_name='recipe.json'):
    """Verify all bytes once and pin them in memory for this invocation."""
    directory = Path(directory)
    recipe_path = directory/recipe_name
    meta_path = directory/(recipe_name+'.meta.json')
    for path in (recipe_path, meta_path):
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 16 * 1024 * 1024:
            raise ValueError('Invalid saved path manifest')
    encoded, meta_encoded = recipe_path.read_bytes(), meta_path.read_bytes()
    metadata = json.loads(meta_encoded)
    unsigned = {key: value for key, value in metadata.items() if key != 'plan_id'}
    if (metadata.get('plan_id') != sha(json.dumps(unsigned, sort_keys=True).encode())[:16]
            or metadata.get('recipe_sha256') != sha(encoded)):
        raise ValueError('Saved path manifest checksum differs')
    artifacts = metadata.get('artifact_sha256')
    if not isinstance(artifacts, dict) or not artifacts or len(artifacts) > 40000:
        raise ValueError('Saved path artifact manifest is missing or oversized')
    payloads, size = {}, len(encoded)+len(meta_encoded)
    for name, expected in artifacts.items():
        if (not isinstance(name, str) or Path(name).name != name or name in ('', '.', '..')
                or name in (recipe_name, recipe_name+'.meta.json')):
            raise ValueError('Invalid saved path artifact basename')
        path = directory/name
        if path.is_symlink() or not path.is_file():
            raise ValueError('Saved path artifact is missing or a symlink')
        size += path.stat().st_size
        if size > MAX_PACKAGE_BYTES:
            raise ValueError('Saved path package exceeds the memory bound')
        payload = path.read_bytes()
        if len(payload) != path.stat().st_size:
            raise ValueError('Saved path artifact changed while loading')
        if sha(payload) != expected:
            raise ValueError('Saved path artifact checksum differs')
        payloads[name] = payload
    return metadata, json.loads(encoded), payloads


def save_package(library, metadata, recipe, payloads):
    """Publish only a complete, validated package via an atomic directory rename."""
    check_certificates(recipe, metadata)
    if sha((json.dumps(recipe, sort_keys=True, allow_nan=False)+'\n').encode()) != metadata['recipe_sha256']:
        raise ValueError('Package recipe differs from the sealed metadata')
    if set(payloads) != set(metadata['artifact_sha256']):
        raise ValueError('Package artifacts are incomplete')
    parent = Path(library)/context_key(metadata)
    parent.mkdir(parents=True, exist_ok=True)
    destination = parent/metadata['plan_id']
    temporary = parent/('.preparing_'+uuid.uuid4().hex)
    temporary.mkdir()
    try:
        for name, payload in payloads.items():
            if Path(name).name != name or sha(payload) != metadata['artifact_sha256'][name]:
                raise ValueError('Package artifact differs from the sealed metadata')
            (temporary/name).write_bytes(payload)
        (temporary/'recipe.json').write_text(json.dumps(recipe, sort_keys=True, allow_nan=False)+'\n')
        (temporary/'recipe.json.meta.json').write_text(json.dumps(metadata, indent=2)+'\n')
        if destination.exists():
            read_package(destination)  # Never silently replace a damaged package.
        else:
            temporary.rename(destination)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return destination


def rename_artifacts(recipe, payloads, prefix):
    """Give each restored session its own filenames, preserving archived bytes."""
    recipe = copy.deepcopy(recipe)
    names = {name: prefix+str(index)+Path(name).suffix for index, name in enumerate(payloads)}
    for key in ('initial_scene_file', 'initial_state_file'):
        if recipe.get(key):
            recipe[key] = names[recipe[key]]
    for step in recipe['steps']:
        for key in ARTIFACT_FIELDS:
            if step.get(key):
                step[key] = names[step[key]]
    return recipe, {names[name]: payload for name, payload in payloads.items()}
