"""Fatal/Stop subprocess shutdown is bounded, including inherited output pipes."""
import fcntl
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
from types import MethodType, SimpleNamespace
import uuid

import pytest
from ament_index_python.packages import get_package_prefix

from ur5e_block_bringup import physical_stack as physical


@pytest.fixture
def runner(tmp_path, monkeypatch):
    actual_popen = subprocess.Popen
    processes = []
    task = SimpleNamespace(child=None, stopped=False, get_parameters_by_prefix=lambda _: {})
    task.signal_planner = physical.PhysicalStackTask.signal_planner
    task.finish_planner = MethodType(physical.PhysicalStackTask.finish_planner, task)
    def check_stop():
        if task.stopped:
            raise RuntimeError('operator stopped planning')
    task.check_stop = check_stop
    monkeypatch.setattr(physical.rclpy, 'spin_once', lambda *_, **__: time.sleep(.005))
    for name in ('PLANNER_INTERRUPT_GRACE', 'PLANNER_TERMINATE_GRACE', 'PLANNER_KILL_GRACE'):
        monkeypatch.setattr(physical, name, .06)
    output = tmp_path/'plan.json'

    def run(script):
        def spawn(_command, **options):
            assert options['start_new_session'] is True
            process = actual_popen([sys.executable, '-u', '-c', script], **options)
            processes.append(process)
            return process
        monkeypatch.setattr(physical.subprocess, 'Popen', spawn)
        physical.PhysicalStackTask.planner(task, output)
    try:
        yield task, run, output, processes
    finally:
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=2.)


def ignore_interrupts():
    return ('import signal,time\n'
            'signal.signal(signal.SIGINT,signal.SIG_IGN)\n'
            'signal.signal(signal.SIGTERM,signal.SIG_IGN)\n')


def assert_clean(task, output, processes):
    assert task.child is None
    assert not output.with_name(output.name+'.planner.yaml').exists()
    assert processes and all(process.poll() is not None for process in processes)
    assert all(process.stdout.closed for process in processes)


def test_fatal_message_cannot_keep_the_parent_and_lock_alive(runner, capsys):
    task, run, output, processes = runner
    began = time.monotonic()
    with pytest.raises(RuntimeError, match='synthetic planner failure'):
        run(ignore_interrupts()+"print('STACK_ERROR synthetic planner failure',flush=True)\nwhile True:time.sleep(.02)\n")
    assert time.monotonic()-began < 3.
    assert processes[0].returncode == -signal.SIGKILL
    log = capsys.readouterr().out
    assert all(stage in log for stage in ('SIGINT', 'SIGTERM', 'SIGKILL', 'planner_error'))
    assert_clean(task, output, processes)


def test_operator_stop_interrupts_the_whole_planner_session_and_finishes(runner, monkeypatch):
    task, run, output, processes = runner
    ready = output.parent/'ready'
    def spin(*_, **__):
        time.sleep(.005)
        if ready.exists():
            task.stopped = True
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    with pytest.raises(RuntimeError, match='operator stopped'):
        run(ignore_interrupts()+f"from pathlib import Path\nPath({str(ready)!r}).touch()\nwhile True:time.sleep(.02)\n")
    assert_clean(task, output, processes)


def test_cleanup_is_bounded_when_the_parent_gets_an_unexpected_exception(runner, monkeypatch):
    task, run, output, processes = runner
    ready = output.parent/'ready'
    def spin(*_, **__):
        time.sleep(.005)
        if ready.exists():
            raise ValueError('synthetic callback exception')
    monkeypatch.setattr(physical.rclpy, 'spin_once', spin)
    with pytest.raises(ValueError, match='callback exception'):
        run(ignore_interrupts()+f"from pathlib import Path\nPath({str(ready)!r}).touch()\nwhile True:time.sleep(.02)\n")
    assert_clean(task, output, processes)


def test_fatal_message_is_failure_even_with_zero_exit_and_a_partial_artifact(runner):
    task, run, output, processes = runner
    with pytest.raises(RuntimeError, match='partial export'):
        run(f"from pathlib import Path\nPath({str(output)!r}).write_text('{{}}')\nprint('STACK_ERROR partial export',flush=True)\n")
    assert_clean(task, output, processes)


def test_success_keeps_the_output_and_sends_no_stop_signal(runner, monkeypatch):
    task, run, output, processes = runner
    task.signal_planner = lambda *_: pytest.fail('Successful planner was interrupted')
    run(f"from pathlib import Path\nPath({str(output)!r}).write_text('{{}}')\nprint('STACK_PLAN_READY',flush=True)\n")
    assert output.read_text() == '{}'
    assert processes[0].returncode == 0
    assert_clean(task, output, processes)


def test_descendant_holding_the_output_pipe_is_stopped_with_its_process_group(runner):
    task, run, output, processes = runner
    pid_file = output.parent/'descendant.pid'
    descendant = ignore_interrupts()+"while True:time.sleep(.02)\n"
    script = (ignore_interrupts()+
        f"import subprocess,sys\nfrom pathlib import Path\n"
        f"child=subprocess.Popen([sys.executable,'-u','-c',{descendant!r}])\n"
        f"Path({str(pid_file)!r}).write_text(str(child.pid))\n"
        "print('STACK_ERROR descendant test',flush=True)\nwhile True:time.sleep(.02)\n")
    with pytest.raises(RuntimeError, match='descendant test'):
        run(script)
    child_pid = int(pid_file.read_text())
    try:
        descriptor = os.pidfd_open(child_pid)
    except ProcessLookupError:
        pass
    else:
        try:
            assert select.select([descriptor], [], [], 1.)[0]
        finally:
            os.close(descriptor)
    assert_clean(task, output, processes)


def test_duplicate_cpp_task_exits_before_ros_and_never_leaves_a_spinner():
    executable = Path(get_package_prefix('ur5e_a3_grid'))/'lib/ur5e_a3_grid/stack_blocks'
    assert executable.is_file(), 'Build ur5e_a3_grid before running the duplicate-lock integration test'
    # This intentionally is not a DDS domain: a rejected duplicate must never
    # initialize ROS. The unique lock is only for this test, not domain 0.
    domain = 'test_'+uuid.uuid4().hex
    lock = Path('/tmp')/f'ur5e_stack_{domain}.lock'
    descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        env = dict(os.environ, ROS_DOMAIN_ID=domain)
        for _ in range(20):
            result = subprocess.run([str(executable), '--ros-args', '-p', 'stack_mode:=scene'],
                                    env=env, capture_output=True, text=True, timeout=3.)
            assert result.returncode == 1
            assert 'STACK_ERROR Another stacking task is running in this ROS domain' in result.stderr
            assert not result.stdout
    finally:
        os.close(descriptor)
        lock.unlink()
