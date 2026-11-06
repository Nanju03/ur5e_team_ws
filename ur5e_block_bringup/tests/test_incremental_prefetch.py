"""Offline prefetch process ownership and result contracts; no ROS or robot."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from ur5e_block_bringup import incremental_prefetch as prefetch


def start(tmp_path, source, request=None, name='job'):
    directory = tmp_path/name
    return prefetch.PrefetchJob(directory, request or {'context': 'unchanged'},
        [sys.executable, '-c', source, str(directory)])


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            pytest.fail('Owned test process did not reach its expected state')
        time.sleep(.01)


def process_alive(pid):
    try:
        # A descendant reparented to init may briefly remain a reaped zombie.
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1][0] != 'Z'
    except FileNotFoundError:
        return False


def test_job_uses_private_request_and_isolates_ros_and_thread_environment(tmp_path):
    request = {'context': {'model': 'same'}, 'source': 4}
    code = '''
import json, os, pathlib, sys
p = pathlib.Path(sys.argv[1])
keys = ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')
result = dict(status='ready', request=json.loads((p/'request.json').read_text()),
              environment={key: os.environ[key] for key in keys},
              pid=os.getpid(), group=os.getpgrp(), session=os.getsid(0))
(p/'result.json').write_text(json.dumps(result))
print('offline child complete', flush=True)
'''
    job = start(tmp_path, code, request)
    try:
        wait_until(lambda: not job.running())
        result = job.result()
        assert result['request'] == dict(request, output_dir=str(job.directory))
        assert 'output_dir' not in request
        request['context']['model'] = 'changed after submission'
        assert job.request['context']['model'] == 'same'
        assert result['environment'] == dict(ROS_DOMAIN_ID='46', ROS_LOCALHOST_ONLY='1',
            OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
        assert result['pid'] == result['group'] == result['session'] == job.process.pid
        assert 'offline child complete' in (job.directory/'worker.log').read_text()
    finally:
        job.close()
    assert job.log.closed


def test_result_is_pending_until_child_finishes_even_if_file_already_exists(tmp_path):
    job = start(tmp_path, "import pathlib,sys,time; p=pathlib.Path(sys.argv[1]); "
        "(p/'result.json').write_text('{\"status\":\"ready\"}'); time.sleep(30)")
    try:
        wait_until(lambda: (job.directory/'result.json').is_file())
        assert job.running()
        assert job.result() is None
    finally:
        job.close()
    assert not job.running()


@pytest.mark.parametrize('status', ['ready', 'finished'])
def test_only_successful_complete_child_returns_accepted_status(tmp_path, status):
    job = start(tmp_path, "import pathlib,sys; "
        f"(pathlib.Path(sys.argv[1])/'result.json').write_text({json.dumps(json.dumps({'status': status}))})")
    try:
        wait_until(lambda: not job.running())
        assert job.result() == {'status': status}
    finally:
        job.close()


@pytest.mark.parametrize('content,expected', [
    (None, 'missing or oversized'), ('x'*65537, 'missing or oversized'),
    ('[]', 'Invalid offline'), ('{}', 'Invalid offline'),
    ('{"status":"failed"}', 'Invalid offline'),
])
def test_success_exit_does_not_publish_missing_or_invalid_result(tmp_path, content, expected):
    code = 'pass' if content is None else (
        "import pathlib,sys; (pathlib.Path(sys.argv[1])/'result.json').write_text("+repr(content)+')')
    job = start(tmp_path, code)
    try:
        wait_until(lambda: not job.running())
        with pytest.raises(RuntimeError, match=expected):
            job.result()
    finally:
        job.close()


def test_nonzero_child_cannot_publish_a_ready_result(tmp_path):
    job = start(tmp_path, "import pathlib,sys; "
        "(pathlib.Path(sys.argv[1])/'result.json').write_text('{\"status\":\"ready\"}'); "
        "print('planner failed', flush=True); sys.exit(7)")
    try:
        wait_until(lambda: not job.running())
        with pytest.raises(RuntimeError, match='Offline planning failed'):
            job.result()
        assert 'planner failed' in (job.directory/'worker.log').read_text()
    finally:
        job.close()


def test_spawn_failure_closes_log_and_leaves_auditable_request(tmp_path):
    directory = tmp_path/'bad_spawn'
    with pytest.raises(FileNotFoundError):
        prefetch.PrefetchJob(directory, {'source': 3}, [str(tmp_path/'nonexistent_executable')])
    assert json.loads((directory/'request.json').read_text())['source'] == 3
    # A retained log is useful for reporting the failed job; it must not be held.
    with (directory/'worker.log').open('ab') as stream:
        stream.write(b'checked')


def test_job_cannot_overwrite_an_existing_job_directory(tmp_path):
    directory = tmp_path/'existing'
    directory.mkdir()
    marker = directory/'result.json'
    marker.write_text('existing result')
    with pytest.raises(FileExistsError):
        prefetch.PrefetchJob(directory, {}, [sys.executable, '-c', 'pass'])
    assert marker.read_text() == 'existing result'


def test_close_kills_owned_descendant_after_group_leader_has_exited(tmp_path, monkeypatch):
    code = '''
import pathlib, subprocess, sys, time
p = pathlib.Path(sys.argv[1])
child = subprocess.Popen([sys.executable, '-c',
    'import os,pathlib,signal,sys,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
    'pathlib.Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(30)', str(p/'child.pid')])
while not (p/'child.pid').exists(): time.sleep(.01)
'''
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    job = start(tmp_path, code)
    child_pid = None
    signals = []
    real_killpg = os.killpg

    def owned_killpg(group, signum):
        signals.append((group, signum))
        return real_killpg(group, signum)

    monkeypatch.setattr(prefetch.os, 'killpg', owned_killpg)
    try:
        wait_until(lambda: not job.running())
        child_pid = int((job.directory/'child.pid').read_text())
        assert process_alive(child_pid)
        job.close()
        wait_until(lambda: not process_alive(child_pid))
        assert unrelated.poll() is None
        assert {group for group, _ in signals} == {job.process.pid}
        assert [signum for _, signum in signals] == [signal.SIGTERM, signal.SIGKILL]
        count = len(signals)
        job.close()
        assert len(signals) == count
        assert job.log.closed
    finally:
        job.close()
        unrelated.terminate()
        unrelated.wait(timeout=2)
        if child_pid is not None and process_alive(child_pid):
            os.kill(child_pid, signal.SIGKILL)
