"""Bounded local IPC for planning-time collision checks, with cooperative Stop.

The same Python-generated quintic and wrist-sweep samples are checked by MoveIt
in one persistent C++ process. Saved physical playback uses these certificates
without repeating collision checks; live state and Stop monitors remain active.
"""
from array import array
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import time
import uuid

from ament_index_python.packages import get_package_prefix
from rclpy.serialization import serialize_message

MAX_STATES = 8192
MAX_SCENE_BYTES = 100 * 1024 * 1024


class CollisionWorker:
    def __init__(self, parameters, directory, poll, timeout=30., command=None):
        self.poll, self.timeout = poll, timeout
        self.buffer = bytearray()
        self.process = None
        self.directory = Path(directory)
        identity = uuid.uuid4().hex
        self.parameters_path = self.directory/f'collision_checker_{identity}.params.yaml'
        self.stderr_path = self.directory/f'collision_checker_{identity}.stderr.log'
        if command is None:
            import yaml
            executable = Path(get_package_prefix('ur5e_a3_grid'))/'lib/ur5e_a3_grid/physical_collision_checker'
            if not executable.is_file():
                raise RuntimeError('physical_collision_checker가 없습니다. ur5e_a3_grid를 빌드한 뒤 다시 계획하세요.')
            self.parameters_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
            command = [str(executable), '--ros-args', '--params-file', str(self.parameters_path)]
        env = os.environ.copy()
        # The worker has no robot-facing ROS endpoints. Keep discovery separate
        # too; its model comes entirely from parameters, not the live server.
        env.update(ROS_DOMAIN_ID='46', ROS_LOCALHOST_ONLY='1', RCUTILS_LOGGING_USE_STDOUT='0')
        self.stderr = self.stderr_path.open('wb')
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.stderr, env=env, bufsize=0)
            for stream in (self.process.stdin, self.process.stdout):
                os.set_blocking(stream.fileno(), False)
            ready = self._exchange(b'')
            if ready != dict(ready=True, protocol=1):
                raise RuntimeError('Unexpected collision checker handshake')
        except BaseException:
            self.close()
            raise

    def _exchange(self, payload):
        remaining = memoryview(payload)
        deadline = time.monotonic()+self.timeout
        while True:
            self.poll()
            if time.monotonic() > deadline:
                raise TimeoutError('Local physical collision validation timed out')
            if not remaining and b'\n' in self.buffer:
                line, _, rest = self.buffer.partition(b'\n')
                self.buffer = bytearray(rest)
                try:
                    result = json.loads(line)
                except (ValueError, UnicodeError) as error:
                    raise RuntimeError('Malformed collision checker reply') from error
                if not isinstance(result, dict):
                    raise RuntimeError('Malformed collision checker reply')
                return result
            readable, writable, _ = select.select([self.process.stdout],
                                                  [self.process.stdin] if remaining else [], [], .01)
            if writable:
                try:
                    count = os.write(self.process.stdin.fileno(), remaining[:65536])
                except BrokenPipeError as error:
                    raise RuntimeError('Physical collision checker exited; see '+str(self.stderr_path)) from error
                remaining = remaining[count:]
            if readable:
                chunk = os.read(self.process.stdout.fileno(), 4096)
                if not chunk:
                    raise RuntimeError('Physical collision checker exited; see '+str(self.stderr_path))
                self.buffer.extend(chunk)
                if len(self.buffer) > 16384:
                    raise RuntimeError('Oversized collision checker reply')

    def validate(self, scene, samples):
        if scene.is_diff or scene.robot_state.is_diff:
            raise ValueError('Local collision validation requires a full snapshot')
        encoded = serialize_message(scene)
        if not 0 < len(encoded) <= MAX_SCENE_BYTES:
            raise ValueError('Collision scene exceeds the bounded request size')
        total = 0
        for start in range(0, len(samples), MAX_STATES):
            batch = samples[start:start+MAX_STATES]
            values = array('d')
            for _, joints in batch:
                if len(joints) != 7 or any(not math.isfinite(value) for value in joints):
                    raise ValueError('Collision samples require seven finite joints')
                values.extend(joints)
            if sys.byteorder != 'little':
                values.byteswap()
            packet = struct.pack('<II', len(encoded), len(batch))+encoded+values.tobytes()
            result = self._exchange(packet)
            if result.get('ok') is not True:
                raise RuntimeError('집기·쌓기 경로의 로컬 충돌 검사 실패: '+json.dumps(result, ensure_ascii=False))
            if type(result.get('checked')) is not int or result['checked'] != len(batch):
                raise RuntimeError('Collision checker did not acknowledge every sample')
            total += result['checked']
        return total

    def close(self):
        process = self.process
        if process is not None:
            if process.stdin:
                process.stdin.close()
            try:
                process.wait(timeout=.25)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.)
            if process.stdout:
                process.stdout.close()
        if hasattr(self, 'stderr'):
            self.stderr.close()
        self.parameters_path.unlink(missing_ok=True)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
