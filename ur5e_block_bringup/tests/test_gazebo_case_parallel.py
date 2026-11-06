"""Concurrent case ownership, isolated endpoints, cancellation and resume.

Uses the production Runner/archive/report path with serialized fixture motions;
does not start a simulator, robot controller, action server or hardware driver.
"""
import copy
import json
import os
from pathlib import Path
import threading
import time

import pytest

from ur5e_block_bringup import gazebo_batch as batch
from ur5e_block_bringup.gazebo_parallel import CasePool, cpu_partitions, lease
from ur5e_block_bringup.gazebo_session import (GazeboSession, MAX_CASE_JOBS,
    session_from_environment, worker_session)
from test_gazebo_batch import FakeProcess, Harness, MODEL_HASH, parameters


def test_worker_endpoints_are_unique_and_exclude_physical_domains():
    sessions = [worker_session(slot) for slot in range(MAX_CASE_JOBS)]
    assert len({s.domain for s in sessions}) == len({s.port for s in sessions}) == 8
    assert {s.domain for s in sessions}.isdisjoint({0, 42, 44, 46})
    for session in sessions+[GazeboSession()]:
        assert session_from_environment(session.environment()) == session


@pytest.mark.parametrize('patch', [
    {'ROS_DOMAIN_ID': '0'}, {'ROS_LOCALHOST_ONLY': '0'},
    {'GAZEBO_MASTER_URI': 'http://192.168.0.2:11370'},
    {'UR5E_GAZEBO_WORKER_SLOT': '8'}, {'UR5E_GAZEBO_WORKER_SLOT': '-1'},
    {'UR5E_GAZEBO_WORKER_SLOT': '01'}, {'ROS_DOMAIN_ID': '61'},
    {'GAZEBO_MASTER_URI': 'http://127.0.0.1:11355'}])
def test_worker_guard_rejects_wrong_domain_port_or_nonlocal_host(patch):
    with pytest.raises(ValueError):
        session_from_environment({**worker_session(0).environment(), **patch})


def test_worker_domains_require_explicit_slot_marker():
    env = worker_session(0).environment()
    del env['UR5E_GAZEBO_WORKER_SLOT']
    with pytest.raises(ValueError):
        session_from_environment(env)


@pytest.mark.parametrize('jobs', [1, 2, 4])
def test_parallel_cpu_allocations_are_nonempty_and_disjoint(jobs):
    result = cpu_partitions(jobs)
    assert len(result) == jobs and all(result)
    flattened = [cpu for cpus in result for cpu in cpus]
    assert len(flattened) == len(set(flattened))
    assert set(flattened) <= set(os.sched_getaffinity(0))


def test_job_cli_uses_small_per_case_budget_and_separate_report_folder(monkeypatch, tmp_path):
    monkeypatch.setenv('UR5E_WS', str(tmp_path))
    monkeypatch.setattr(batch, 'available_cpus', lambda: 20)
    args = batch.parse_args(['--jobs', '4'])
    assert args.case_parallel and args.jobs == 4 and args.workers == 2
    assert not args.pin_cpus
    assert args.physics_threads == 1 and len(args.cases) == 60
    assert args.output_dir == tmp_path/'log/gazebo_diagonal_case_parallel'
    assert not list(tmp_path.iterdir())
    explicit = batch.parse_args(['--jobs', '1'])
    assert explicit.case_parallel
    for jobs in ('0', '9', '-1'):
        with pytest.raises(SystemExit):
            batch.parse_args(['--jobs', jobs])


def test_domain_lease_excludes_other_campaign_and_can_be_released(tmp_path):
    path = tmp_path/'lease'
    with lease(path):
        with pytest.raises(ValueError, match='다른 시험'):
            lease(path)
    with lease(path):
        assert path.is_file()


class FixturePool:
    def __init__(self, args, outcomes, stop_after_started=None, busy_slot=None):
        self.args, self.outcomes = args, outcomes
        self.stop_after_started, self.busy_slot = stop_after_started, busy_slot
        self.lock = threading.Lock()
        self.active, self.peak, self.assigned, self.monitors, self.environments = set(), 0, [], [], []
        self.cold_active, self.cold_peak = set(), 0
        self.pool = None

    def monitor(self, urdf, home, cancelled, session):
        owner = self
        class Monitor(Harness):
            def __init__(self):
                super().__init__(owner.args, [])
                self.session, self.cancelled, self.closed = session, cancelled, False
            def wait_idle(self, seconds=None):
                if session.slot == owner.busy_slot:
                    raise batch.TrialError('existing user simulator', 'environment_error')
                super().wait_idle()
            def close(self):
                self.closed = True
        monitor = Monitor()
        with self.lock:
            self.monitors.append(monitor)
        return monitor

    def runner(self, args, campaign, settings, monitor):
        owner = self
        monitor.modes.append(self.outcomes.get(campaign.pair, 'succeeded'))
        with self.lock:
            self.assigned.append((campaign.pair, monitor.session.slot))
        class Process(FakeProcess):
            def __init__(self, harness, command, logfile, env):
                assert session_from_environment(env) == monitor.session
                super().__init__(harness, command, logfile, env)
                with owner.lock:
                    owner.environments.append(copy.deepcopy(env))
                    if self.stage == 'server':
                        owner.active.add(monitor.session.slot)
                        owner.cold_active.add(monitor.session.slot)
                        owner.cold_peak = max(owner.cold_peak, len(owner.cold_active))
                        owner.peak = max(owner.peak, len(owner.active))
                        if owner.stop_after_started and len(owner.active) >= owner.stop_after_started:
                            owner.pool.request_stop()
                if self.stage == 'run':
                    context = batch.read_json(self.archive/'context.json')
                    context['ros_domain'] = env['ROS_DOMAIN_ID']  # Fixture records its emulated child env.
                    batch.write_json(self.archive/'context.json', context)
            def stop(self):
                if self.stage in ('scene', 'server'):
                    with owner.lock:
                        owner.cold_active.discard(monitor.session.slot)
                if self.stage == 'server':
                    with owner.lock:
                        owner.active.discard(monitor.session.slot)
                super().stop()
        return batch.Runner(args, campaign, settings, monitor,
            process_factory=lambda command, logfile, env: Process(monitor, command, logfile, env))


def setup_pool(tmp_path, pairs, outcomes=None, jobs=2, stop_after_started=None, busy_slot=None, retry=False):
    args = batch.parse_args(['--jobs', str(jobs), '--output-dir', str(tmp_path/'reports'),
        '--archive-root', str(tmp_path/'archives'), '--planning-timeout', '10.'])
    args.retry_failed = retry
    campaign = batch.Campaign(args.output_dir, 'fixed-model', {'model_sha256': MODEL_HASH}, cases=pairs)
    fixture = FixturePool(args, outcomes or {}, stop_after_started, busy_slot)
    pool = CasePool(args, campaign, parameters(), '', {}, [[i] for i in range(jobs)],
        monitor_factory=fixture.monitor, runner_factory=fixture.runner, pin_cpus=False)
    fixture.pool = pool
    return campaign, pool, fixture


def test_cases_overlap_without_duplicates_and_preserve_real_archive_checks(tmp_path):
    pairs = batch.adjacent_diagonals()[:6]
    original_env = dict(os.environ)
    campaign, pool, fixture = setup_pool(tmp_path, pairs, {pairs[1]: 'failed'})
    try:
        assert pool.run() == 1
        assert fixture.peak == 2 and not fixture.active
        assert fixture.cold_peak == 1 and not fixture.cold_active
        assert len(fixture.assigned) == 6 and {p for p, _ in fixture.assigned} == set(pairs)
        assert all(m.closed for m in fixture.monitors)
        assert dict(os.environ) == original_env
        state = batch.read_json(campaign.folder/'state.json')
        assert len(state['results']) == 6
        assert batch.read_json(campaign.folder/'summary.json')['counts']['succeeded'] == 5
        assert batch.read_json(campaign.folder/'failed_cases.json')['count'] == 1
        for row in state['results'].values():
            assert row['ros_domain'] == 59+row['worker_id']
            if row['status'] == 'succeeded':
                assert Path(row['successful_path']).is_file()
        assert '작업자 ' in (campaign.folder/'report.html').read_text()
    finally:
        campaign.close()


def test_interrupt_stops_all_owned_simulators_and_leaves_unassigned_cases_pending(tmp_path):
    pairs = batch.adjacent_diagonals()[:6]
    campaign, pool, fixture = setup_pool(tmp_path, pairs, {p: 'timeout' for p in pairs}, stop_after_started=2)
    try:
        assert pool.run() == 130
        assert len(fixture.assigned) == 2 and not fixture.active
        assert all(m.closed and all(p.stopped for p in m.servers) for m in fixture.monitors)
        summary = batch.read_json(campaign.folder/'summary.json')
        assert summary['counts']['interrupted'] == 2
        assert summary['counts']['pending'] == 4
        assert batch.read_json(campaign.folder/'failed_cases.json')['count'] == 0
    finally:
        campaign.close()


def test_busy_worker_domain_stops_before_any_case_or_simulator_is_started(tmp_path):
    pairs = batch.adjacent_diagonals()[:4]
    campaign, pool, fixture = setup_pool(tmp_path, pairs, busy_slot=1)
    try:
        assert pool.run() == 2
        assert not fixture.assigned and fixture.peak == 0
        assert not campaign.state['results']
        assert all(m.closed for m in fixture.monitors)
    finally:
        campaign.close()


def test_interrupt_during_worker_preflight_aborts_the_start_barrier(tmp_path):
    pairs = batch.adjacent_diagonals()[:4]
    campaign, pool, fixture = setup_pool(tmp_path, pairs)
    original = fixture.monitor
    def monitor_factory(urdf, home, cancelled, session):
        monitor = original(urdf, home, cancelled, session)
        if session.slot == 1:
            def wait_idle(seconds=None):
                while not cancelled():
                    time.sleep(.005)
                raise batch.Interrupted()
            monitor.wait_idle = wait_idle
        return monitor
    pool.monitor_factory = monitor_factory
    timer = threading.Timer(.15, pool.request_stop)
    began = time.monotonic()
    try:
        timer.start()
        assert pool.run() == 130
        assert time.monotonic()-began < 2.
        assert not fixture.assigned
        assert all(m.closed for m in fixture.monitors)
    finally:
        timer.cancel()
        campaign.close()


def test_retry_failed_only_keeps_successful_zip_bytes_and_adds_attempt(tmp_path):
    pairs = batch.adjacent_diagonals()[:3]
    campaign, pool, fixture = setup_pool(tmp_path, pairs, {pairs[0]: 'failed'})
    try:
        assert pool.run() == 1
        successes = {r['successful_path']: Path(r['successful_path']).read_bytes()
            for r in campaign.state['results'].values() if r['status'] == 'succeeded'}
    finally:
        campaign.close()
    campaign, pool, fixture = setup_pool(tmp_path, pairs, jobs=1, retry=True)
    try:
        assert pool.run() == 0
        assert [p for p, _ in fixture.assigned] == [pairs[0]]
        assert campaign.state['results'][batch.key(pairs[0])]['attempts'] == 2
        assert all(Path(path).read_bytes() == data for path, data in successes.items())
        assert batch.read_json(campaign.folder/'failed_cases.json')['count'] == 0
    finally:
        campaign.close()


def test_approval_rejects_archive_from_a_different_domain(tmp_path):
    from test_gazebo_batch import data_archive, ready_event
    folder = data_archive(tmp_path)
    context = batch.read_json(folder/'context.json')
    context['ros_domain'] = '60'
    batch.write_json(folder/'context.json', context)
    event = ready_event(folder)
    with pytest.raises(batch.TrialError):
        event.approval_service((11, 14), tmp_path, MODEL_HASH, expected_domain=61)
    assert not event.bound
