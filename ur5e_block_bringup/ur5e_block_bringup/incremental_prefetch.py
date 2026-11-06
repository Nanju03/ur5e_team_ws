"""One bounded, disposable offline planning job; no ROS interfaces."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


class PrefetchJob:
    def __init__(self, directory, request, command=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.request = copy.deepcopy(request)
        self.started = time.monotonic()
        self.process = None
        self.log = None
        self.closed = False
        request = dict(request, output_dir=str(self.directory))
        path = self.directory/'request.json'
        path.write_text(json.dumps(request, allow_nan=False))
        env = os.environ.copy()
        env.update(ROS_DOMAIN_ID='46', ROS_LOCALHOST_ONLY='1', OMP_NUM_THREADS='1',
                   OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
        if command is None:
            command = ['/usr/bin/nice', '-n', '10', sys.executable, '-m',
                       'ur5e_block_bringup.incremental_prefetch_worker', str(path)]
        try:
            self.log = (self.directory/'worker.log').open('wb')
            self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                            env=env, start_new_session=True)
        except BaseException:
            self.close()
            raise

    def running(self):
        return self.process is not None and self.process.poll() is None

    def result(self):
        if self.running():
            return None
        if self.process is None or self.process.returncode != 0:
            raise RuntimeError(f'Offline planning failed; see {self.directory}/worker.log')
        path = self.directory/'result.json'
        if not path.is_file() or path.stat().st_size > 65536:
            raise RuntimeError('Offline planning result is missing or oversized')
        result = json.loads(path.read_text())
        if not isinstance(result, dict) or result.get('status') not in ('ready', 'finished'):
            raise RuntimeError('Invalid offline planning result')
        return result

    def close(self):
        if self.closed:
            return
        self.closed = True
        process = self.process
        if process is not None:
            # Terminate the owned group even when its leader already exited:
            # a failed planner must never leave collision/planning children.
            for signum, timeout in ((signal.SIGTERM, 1.), (signal.SIGKILL, 1.)):
                try:
                    os.killpg(process.pid, signum)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
                # A terminated leader does not prove its children have exited.
                if signum == signal.SIGTERM:
                    continue
            process.wait(timeout=1.)
        if self.log is not None:
            self.log.close()
