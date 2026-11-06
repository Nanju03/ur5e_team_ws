"""A bounded queue of independent Gazebo cases; each worker owns one ROS context.

Simulation processes remain owned by the existing Runner, including its Stop,
final tower observations and archive checks. Only this coordinator writes the
shared reports. No global ROS/Gazebo environment is mutated from worker threads.
"""
import copy
from contextlib import ExitStack
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import shutil
import signal
import threading
import time

import yaml

from . import gazebo_batch as batch
from .gazebo_session import worker_session, MAX_CASE_JOBS


def cpu_partitions(jobs):
    """Keep SMT siblings together where topology is available; leave one core free."""
    cpus = sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else list(range(os.cpu_count() or 1))
    groups = {}
    for cpu in cpus:
        topology = Path(f'/sys/devices/system/cpu/cpu{cpu}/topology')
        try:
            key = ((topology/'physical_package_id').read_text().strip(),
                   (topology/'core_id').read_text().strip())
        except OSError:
            key = ('cpu', str(cpu))
        groups.setdefault(key, []).append(cpu)
    bundles = sorted(groups.values(), key=lambda values: (-len(values), values[0]))
    if len(bundles) > jobs:
        bundles = bundles[1:]  # Reserve a physical core for the desktop/coordinator.
    if len(bundles) < jobs:
        # A small CPU affinity may expose fewer physical than requested workers.
        bundles = [[cpu] for bundle in bundles for cpu in bundle]
    if len(bundles) < jobs:
        raise ValueError(f'사용 가능한 CPU보다 작업자가 많습니다: jobs={jobs}, CPUs={len(cpus)}')
    result = [[] for _ in range(jobs)]
    for bundle in bundles:
        target = min(range(jobs), key=lambda index: (len(result[index]), index))
        result[target].extend(bundle)
    return [sorted(values) for values in result]


def lease(path):
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    stream = os.fdopen(fd, 'a+')
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BaseException:
        stream.close()
        raise ValueError('다른 시험이 병렬 Gazebo 작업자를 사용 중입니다: '+str(path)) from None
    return stream


class CaseCampaign:
    """A worker-local row; publish snapshots under the coordinator's write lock."""
    def __init__(self, owner, pair, slot):
        self.owner, self.pair, self.slot = owner, pair, slot
        self.folder = owner.campaign.folder
        with owner.lock:
            previous = copy.deepcopy(owner.campaign.state['results'].get(batch.key(pair)))
        self.state = dict(metadata=owner.campaign.state['metadata'], signature=owner.campaign.state['signature'],
                          cases=[list(pair)], results={})
        if previous is not None:
            self.state['results'][batch.key(pair)] = previous

    def checkpoint(self):
        row = self.state['results'].get(batch.key(self.pair))
        if row is not None:
            self.owner.publish(self.slot, self.pair, row)


class CasePool:
    def __init__(self, args, campaign, settings, urdf, home, partitions,
                 monitor_factory=batch.RosMonitor, runner_factory=batch.Runner, pin_cpus=True):
        self.args, self.campaign, self.settings = args, campaign, settings
        self.urdf, self.home, self.partitions = urdf, home, partitions
        self.monitor_factory, self.runner_factory, self.pin_cpus = monitor_factory, runner_factory, pin_cpus
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        # Cold model/controller/mesh startup is deliberately staggered. Path
        # planning and the long physics executions run concurrently afterward.
        self.startup_gate = threading.Semaphore(1)
        self.stop_reason = self.stop_kind = None
        self.environments = 0
        self.jobs = len(partitions)
        self.barrier = threading.Barrier(self.jobs) if self.jobs else None
        self.pending = queue.Queue()
        pairs = batch.pending_pairs(campaign.state, args.retry_failed)
        if args.max_cases:
            pairs = pairs[:args.max_cases]
        for pair in pairs:
            self.pending.put(pair)
        self.workers = {str(slot+1): dict(worker_id=slot+1, ros_domain=worker_session(slot).domain,
            master_uri=worker_session(slot).master_uri, cpu_affinity=cpus, stage='starting', case=None)
            for slot, cpus in enumerate(partitions)}

    def request_stop(self, reason='사용자가 병렬 시험을 중단했습니다.', kind='user'):
        # Also called by the main-thread signal handler: no locks or ROS calls.
        if self.stop_reason is None:
            self.stop_reason, self.stop_kind = reason, kind
        self.cancelled.set()

    def checkpoint(self):
        with self.lock:
            self.campaign.state['workers'] = copy.deepcopy(self.workers)
            self.campaign.state['runtime'] = dict(jobs=self.jobs, requested_jobs=self.args.jobs,
                planner_workers_per_case=self.args.workers, physics_threads_per_case=self.args.physics_threads,
                cpu_pinning=self.pin_cpus)
            if self.cancelled.is_set():
                self.campaign.state['message'] = self.stop_reason
            else:
                running = sum(row.get('status') == 'running' for row in self.campaign.state['results'].values())
                self.campaign.state['message'] = f'케이스별 병렬 시험 · 작업자 {self.jobs}개 · 진행 {running}개 · 대기 {self.pending.qsize()}개'
            self.campaign.checkpoint()

    def publish(self, slot, pair, row):
        with self.lock:
            self.campaign.state['results'][batch.key(pair)] = copy.deepcopy(row)
            self.workers[str(slot+1)].update(case=list(pair), stage=row.get('stage'),
                status=row.get('status'), moved=row.get('moved', 0))
            self.checkpoint()

    def note_result(self, row):
        with self.lock:
            self.environments = self.environments+1 if row['status'] == 'environment_error' else 0
            if row.get('cleanup_errors'):
                self.request_stop('시험 프로세스 종료를 확인할 수 없어 전체 병렬 시험을 중단했습니다.', 'environment')
            elif self.environments >= 3:
                self.request_stop('환경 오류 3건 연속으로 전체 병렬 시험을 중단했습니다.', 'environment')

    def worker(self, slot):
        session, monitor, runner, current = worker_session(slot), None, None, None
        cpus = self.partitions[slot]
        try:
            if self.pin_cpus and hasattr(os, 'sched_setaffinity'):
                # Pin this worker thread only. Its DDS threads and launched
                # processes inherit this CPU set; the main thread is unchanged.
                os.sched_setaffinity(threading.get_native_id(), cpus)
            monitor = self.monitor_factory(self.urdf, self.home, self.cancelled.is_set, session=session)
            monitor.wait_idle(seconds=5.)
            self.barrier.wait(timeout=30.)
            while not self.cancelled.is_set():
                if shutil.disk_usage(self.campaign.folder).free < self.args.min_free_gb*1024**3:
                    self.request_stop('저장 공간 부족으로 전체 병렬 시험을 중단했습니다.', 'environment')
                    break
                try:
                    current = self.pending.get_nowait()
                except queue.Empty:
                    break
                local = CaseCampaign(self, current, slot)
                args = copy.copy(self.args)
                args.worker_id, args.cpu_affinity = slot+1, cpus
                runner = self.runner_factory(args, local, self.settings, monitor)
                runner.external_cancelled = self.cancelled.is_set
                runner.startup_gate = self.startup_gate
                ros_log = self.campaign.folder/'workers'/f'worker_{slot+1}'/'ros'
                ros_log.mkdir(parents=True, exist_ok=True)
                runner.env.update(session.environment(), ROS_LOG_DIR=str(ros_log))
                print(f'POOL_ASSIGN worker={slot+1} obstacle={current[0]} target={current[1]} '
                      f'ros_domain={session.domain} port={session.port} cpus={cpus}', flush=True)
                row = runner.attempt(current)
                self.note_result(row)
                if runner.stopping:
                    self.request_stop('작업자 중지 또는 프로세스 정리 오류로 병렬 시험을 중단했습니다.', 'environment')
                self.pending.task_done()
                current, runner = None, None
        except (batch.Interrupted, threading.BrokenBarrierError):
            if not self.cancelled.is_set():
                self.request_stop('병렬 작업자 준비가 완료되지 않아 중단했습니다.', 'environment')
            if self.barrier is not None:
                self.barrier.abort()
        except Exception as error:
            self.request_stop(f'작업자 {slot+1} 오류: {type(error).__name__}: {error}', 'environment')
            if self.barrier is not None:
                self.barrier.abort()
        finally:
            # Normally Runner.attempt has already stopped both owned groups.
            # This fallback covers exceptions in report I/O/monitor setup.
            if runner is not None:
                runner.stopping = True
                for child in (runner.task, runner.server):
                    if child is not None:
                        try:
                            child.stop()
                        except Exception as error:
                            self.request_stop('작업자 프로세스 정리 오류: '+str(error), 'environment')
            if monitor is not None:
                try:
                    monitor.close()
                except Exception as error:
                    self.request_stop('ROS 작업자 종료 오류: '+str(error), 'environment')
            with self.lock:
                self.workers[str(slot+1)].update(stage='stopped', case=None)
                try:
                    self.checkpoint()
                except Exception as error:
                    self.request_stop('통합 보고서 저장 오류: '+str(error), 'environment')

    def run(self):
        self.campaign.state['status'] = 'running'
        self.checkpoint()
        print(f'POOL_START jobs={self.jobs} requested_jobs={self.args.jobs} queued={self.pending.qsize()} '
              f'planner_workers_per_case={self.args.workers} physics_threads_per_case={self.args.physics_threads}', flush=True)
        threads = [threading.Thread(target=self.worker, args=(slot,), name=f'gazebo-case-{slot+1}')
                   for slot in range(self.jobs)]
        started = []
        try:
            for thread in threads:
                thread.start()
                started.append(thread)
            while any(thread.is_alive() for thread in started):
                for thread in started:
                    thread.join(timeout=.2)
        except BaseException:
            self.request_stop('병렬 조정기 오류로 시험을 중단했습니다.', 'environment')
            if self.barrier is not None:
                self.barrier.abort()
            raise
        finally:
            while any(thread.is_alive() for thread in started):
                for thread in started:
                    thread.join(timeout=.2)
        with self.lock:
            # Unexpected report/cleanup exceptions may leave an assigned row
            # running. Persist interruption rather than fabricate a failure.
            for row in self.campaign.state['results'].values():
                if row.get('status') == 'running':
                    row.update(status='interrupted', reason=self.stop_reason or '작업자가 완료 기록 없이 종료됨')
            remaining = batch.pending_pairs(self.campaign.state)
            failures = [row for row in self.campaign.state['results'].values() if row['status'] in batch.FAILED]
            self.campaign.state['status'] = 'interrupted' if self.cancelled.is_set() else 'partial' if remaining else 'completed'
            self.checkpoint()
        print(f"POOL_FINISHED status={self.campaign.state['status']} failures={len(failures)} remaining={len(remaining)}", flush=True)
        print('전체 결과: '+str(self.campaign.folder/'report.html'), flush=True)
        print('실패 목록: '+str(self.campaign.folder/'failed_cases.md'), flush=True)
        return (130 if self.stop_kind == 'user' else 2) if self.cancelled.is_set() else 1 if failures else 0


def main(args):
    campaign = pool = None
    old_signals = {}
    try:
        with ExitStack() as leases:
            leases.enter_context(lease('/tmp/ur5e_gazebo_case_pool.lock'))
            settings, urdf, srdf, home, _, metadata = batch.load_configuration(args.workspace, args.riser_height)
            from ament_index_python.packages import get_package_share_directory
            source = (Path(get_package_share_directory('ur5e_a3_grid'))/'worlds/stack.world').read_text()
            world = batch.parallel_world(source, args.physics_threads)
            # Worker count/CPU allocation may change on resume. Model, case
            # scope, planner settings and archive checks must still match.
            metadata.update(scope='adjacent_diagonal_case_pool_v1', ros_domain=None, master_uri=None,
                cases=[list(pair) for pair in args.cases], planner_workers=args.workers,
                physics_threads=args.physics_threads, world_sha256=hashlib.sha256(world.encode()).hexdigest(),
                worker_endpoints=[worker_session(slot).environment() for slot in range(MAX_CASE_JOBS)])
            signature = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode()).hexdigest()
            campaign = batch.Campaign(args.output_dir, signature, metadata, cases=args.cases)
            pairs = batch.pending_pairs(campaign.state, args.retry_failed)
            if args.max_cases:
                pairs = pairs[:args.max_cases]
            count = min(args.jobs, len(pairs))
            if args.pin_cpus:
                partitions = cpu_partitions(count) if count else []
            else:
                allowed = sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else list(range(os.cpu_count() or 1))
                partitions = [allowed[:] for _ in range(count)]
            for slot in range(count):
                leases.enter_context(lease(f'/tmp/ur5e_gazebo_diagonal_batch_{worker_session(slot).domain}.lock'))
            args.world_file = campaign.folder/'parallel.world'
            batch.atomic_text(args.world_file, world)
            settings['planner_workers'] = args.workers
            batch.atomic_text(campaign.folder/'robot.urdf', urdf)
            batch.atomic_text(campaign.folder/'robot.srdf', srdf)
            batch.atomic_text(campaign.folder/'base_parameters.yaml', yaml.safe_dump(settings))
            pool = CasePool(args, campaign, settings, urdf, home, partitions, pin_cpus=args.pin_cpus)
            for signum in (signal.SIGINT, signal.SIGTERM):
                old_signals[signum] = signal.signal(signum, lambda *_: pool.request_stop())
            return pool.run()
    except (OSError, ValueError, batch.TrialError) as error:
        print('POOL_ERROR '+str(error), flush=True)
        return 2
    finally:
        for signum, handler in old_signals.items():
            signal.signal(signum, handler)
        if campaign is not None:
            campaign.close()
