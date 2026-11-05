"""Real cross-process flock handoff; no ROS node, driver or robot goal."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

from ur5e_block_bringup import physical_stack as physical
from test_physical_stack import recipe


class ValidationReached(Exception):
    pass


PROBE = '''
import fcntl, os, sys
descriptor = os.open(sys.argv[1], os.O_CREAT | os.O_RDWR, 0o600)
try:
    fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit(43)
finally:
    os.close(descriptor)
'''


def probe(path):
    return subprocess.run([sys.executable, '-c', PROBE, str(path)],
                          capture_output=True, timeout=5.).returncode


@pytest.fixture
def task(tmp_path, monkeypatch):
    # Redirect only this test's stack lock. Never touch real domain 0's lock.
    lock = tmp_path/'stack.lock'
    original_open = os.open
    monkeypatch.setenv('ROS_DOMAIN_ID', '219')
    monkeypatch.setattr(physical.os, 'open', lambda path, *args, **kwargs:
                        original_open(lock if str(path) == '/tmp/ur5e_stack_219.lock' else path,
                                      *args, **kwargs))
    value = SimpleNamespace(
        stack_lock=None, path=tmp_path/'recipe.json', meta_path=tmp_path/'recipe.json.meta.json',
        library=tmp_path/'library', spin_until=Mock(), fresh=lambda: True, check_model=Mock(),
        hardware=lambda **_: 'hardware', gripper_runtime=Mock(), measured_stop=lambda: True,
        sensor_fresh=lambda: True, mock=True, input_hashes=lambda: {},
        acquire_stack_lock=None, release_stack_lock=None, model='<robot/>', semantic='<semantic/>')
    for name in ('acquire_stack_lock', 'release_stack_lock'):
        setattr(value, name, MethodType(getattr(physical.PhysicalStackTask, name), value))
    monkeypatch.setattr(physical, 'read_held_fault', lambda: None)
    monkeypatch.setattr(physical, 'clear_held_fault', Mock())
    value.lock_path = lock
    try:
        yield value
    finally:
        value.release_stack_lock()


def test_cache_miss_child_can_lock_then_parent_relocks_before_validation(task):
    events = []
    def cache(*_):
        assert task.stack_lock is not None
        assert probe(task.lock_path) == 43
        events.append('cache_checked_under_parent_lock')
        return False
    def planner(output):
        assert task.stack_lock is None
        assert probe(task.lock_path) == 0  # Same flock/open protocol as C++ Lock.
        output.write_text(json.dumps(recipe()))
        events.append('child_planner_acquired_its_lock')
    def read_message(*_):
        assert task.stack_lock is not None
        assert probe(task.lock_path) == 43
        events.append('parent_relocked_before_validation')
        raise ValidationReached()
    task.restore_saved_plan, task.planner, task.read_message = cache, planner, read_message
    with pytest.raises(ValidationReached):
        physical.PhysicalStackTask.plan(task)
    assert events == ['cache_checked_under_parent_lock', 'child_planner_acquired_its_lock',
                      'parent_relocked_before_validation']


def test_cache_hit_retains_parent_lock_and_never_starts_planner(task):
    def cache(*_):
        assert probe(task.lock_path) == 43
        return True
    task.restore_saved_plan = cache
    task.planner = Mock(side_effect=AssertionError('Cache hit must not replan'))
    physical.PhysicalStackTask.plan(task)
    assert task.stack_lock is not None and probe(task.lock_path) == 43
    task.planner.assert_not_called()


def test_child_failure_leaves_no_parent_lock_or_validation(task):
    task.restore_saved_plan = lambda *_: False
    task.planner = Mock(side_effect=RuntimeError('planner failed'))
    task.read_message = Mock()
    with pytest.raises(RuntimeError, match='planner failed'):
        physical.PhysicalStackTask.plan(task)
    assert task.stack_lock is None and probe(task.lock_path) == 0
    task.read_message.assert_not_called()


def test_real_competing_task_still_blocks_planning_and_failed_acquire_can_retry(task):
    descriptor = os.open(task.lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        task.restore_saved_plan, task.planner = Mock(), Mock()
        with pytest.raises(BlockingIOError):
            physical.PhysicalStackTask.plan(task)
        assert task.stack_lock is None
        task.restore_saved_plan.assert_not_called()
        task.planner.assert_not_called()
    finally:
        os.close(descriptor)
    task.acquire_stack_lock()
    assert task.stack_lock is not None and probe(task.lock_path) == 43


def test_competitor_after_child_exit_prevents_any_scene_validation(task):
    competitor = []
    task.restore_saved_plan = lambda *_: False
    def planner(output):
        assert probe(task.lock_path) == 0
        output.write_text(json.dumps(recipe()))
        descriptor = os.open(task.lock_path, os.O_RDWR)
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        competitor.append(descriptor)
    task.planner, task.read_message = planner, Mock()
    try:
        with pytest.raises(BlockingIOError):
            physical.PhysicalStackTask.plan(task)
        assert task.stack_lock is None
        task.read_message.assert_not_called()
    finally:
        for descriptor in competitor:
            os.close(descriptor)


def test_failed_flock_closes_unowned_descriptor_and_release_is_idempotent(task, monkeypatch):
    attempted = []
    def denied(descriptor, mode):
        attempted.append(descriptor)
        raise BlockingIOError('busy')
    monkeypatch.setattr(physical.fcntl, 'flock', denied)
    with pytest.raises(BlockingIOError):
        task.acquire_stack_lock()
    assert task.stack_lock is None
    with pytest.raises(OSError):
        os.fstat(attempted[0])
    task.release_stack_lock()
    task.release_stack_lock()
