"""Only effective, complete offline-plan parameters may bypass the live lock."""
import fcntl
import os
from pathlib import Path
import subprocess
import uuid

from ament_index_python.packages import get_package_prefix
import pytest
import yaml


def offline_parameters():
    return dict(stack_mode='plan', physical_stack_task=True, incremental_stack=True,
                offline_planning_scene_file='/unused/scene.cdr',
                offline_robot_state_file='/unused/state.cdr',
                recipe_manifest_file='/unused/recipe.json')


@pytest.mark.parametrize('scope,changes,arguments', [
    ('/**', {'offline_robot_state_file': ''}, []),
    ('/**', {'stack_mode': 'run'}, []),
    ('/**', {'physical_stack_task': False}, []),
    ('/**', {'incremental_stack': False}, []),
    ('/other_node', {}, []),
    ('/**', {}, ['-p', 'offline_planning_scene_file:=""']),
    ('/**', {}, ['-p', 'stack_mode:=scene']),
    ('/ur5e_a3_stack', {}, ['-r', '__node:=private_planner']),
])
def test_non_offline_effective_parameters_keep_pre_ros_duplicate_guard(tmp_path, scope, changes, arguments):
    params = offline_parameters()
    params.update(changes)
    file = tmp_path/'parameters.yaml'
    file.write_text(yaml.safe_dump({scope: {'ros__parameters': params}}))
    executable = Path(get_package_prefix('ur5e_a3_grid'))/'lib/ur5e_a3_grid/stack_blocks'
    assert executable.is_file(), 'Build ur5e_a3_grid before testing the native lock'
    # Any accidental DDS initialization aborts on this deliberately invalid
    # domain. A duplicate must return its bounded lock error before that point.
    domain = 'offline_lock_test_'+uuid.uuid4().hex
    lock = Path('/tmp')/f'ur5e_stack_{domain}.lock'
    descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = subprocess.run(
            [str(executable), '--ros-args', '--params-file', str(file), *arguments],
            env=dict(os.environ, ROS_DOMAIN_ID=domain), capture_output=True,
            text=True, timeout=3.)
        assert result.returncode == 1, result.stderr
        assert 'STACK_ERROR Another stacking task is running in this ROS domain' in result.stderr
        assert not result.stdout
    finally:
        os.close(descriptor)
        lock.unlink()
