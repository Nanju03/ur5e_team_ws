"""Reject a second local hardware stack before launch creates any child nodes.

Inspect native process arguments rather than a possibly stale ROS daemon graph.
The domain lock also covers the interval before the first controller process is
spawned. It belongs to this launch process and is released by the OS on exit.
"""
import fcntl
import os
from pathlib import Path


_LAUNCH_LOCKS = []


def _domain(value):
    try:
        number = int(value)
    except (TypeError, ValueError):
        raise RuntimeError('ROS_DOMAIN_ID must be a nonnegative integer') from None
    if number < 0:
        raise RuntimeError('ROS_DOMAIN_ID must be a nonnegative integer')
    return str(number)


def controller_kind(arguments):
    """Identify native UR control, or the DH controller manager namespace."""
    if not arguments:
        return None
    executable = Path(arguments[0]).name
    if executable == 'ur_ros2_control_node':
        return 'UR arm driver'
    if executable == 'ros2_control_node':
        namespaces = [argument.split(':=', 1)[1] for argument in arguments
                      if argument.startswith('__ns:=')]
        if namespaces and namespaces[-1].strip('/') == 'gripper':
            return 'AG95 gripper driver'
    return None


def controller_processes(*, domain=None, proc_root=Path('/proc'), uid=None):
    """Find live same-user controller processes in this launch's ROS domain."""
    expected_domain = _domain(os.environ.get('ROS_DOMAIN_ID', '0') if domain is None else domain)
    uid = os.getuid() if uid is None else uid
    matches = []
    try:
        processes = list(Path(proc_root).iterdir())
    except OSError as error:
        raise RuntimeError(f'Cannot inspect existing hardware processes: {error}') from error
    for process in processes:
        if not process.name.isdigit():
            continue
        try:
            status = dict(line.split(':', 1) for line in (process/'status').read_text().splitlines()
                          if ':' in line)
            if int(status['Uid'].split()[0]) != uid or status['State'].strip().startswith(('Z', 'X')):
                continue
            arguments = (process/'cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
        except (FileNotFoundError, ProcessLookupError):
            continue  # A process may exit between /proc reads.
        except (OSError, KeyError, ValueError):
            continue  # Unrelated or inaccessible processes cannot be identified.
        kind = controller_kind(arguments)
        if kind is None:
            continue
        try:
            environment = dict(item.split(b'=', 1) for item in (process/'environ').read_bytes().split(b'\0')
                               if b'=' in item)
            observed_domain = _domain(environment.get(b'ROS_DOMAIN_ID', b'0').decode())
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (OSError, UnicodeError, RuntimeError) as error:
            raise RuntimeError(f'Cannot verify ROS domain of existing {kind} PID {process.name}; '
                               'no hardware nodes were started') from error
        if observed_domain == expected_domain:
            matches.append(dict(pid=int(process.name), kind=kind, executable=arguments[0], domain=observed_domain))
    return sorted(matches, key=lambda item: item['pid'])


def guard_hardware_launch(*, start_arm, start_gripper, domain=None,
                          proc_root=Path('/proc'), lock_root=Path('/tmp'), uid=None):
    """Keep model-only reuse available; hardware launches require an empty stack."""
    if not (start_arm or start_gripper):
        return
    domain = _domain(os.environ.get('ROS_DOMAIN_ID', '0') if domain is None else domain)
    uid = os.getuid() if uid is None else uid
    lock_path = Path(lock_root)/f'ur5e_real_hardware_launch_{uid}_{domain}.lock'
    descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f'UR5e hardware launch is already running in ROS domain {domain}. '
                               'Reuse its current connection; no additional nodes were started.') from None
        existing = controller_processes(domain=domain, proc_root=proc_root, uid=uid)
        if existing:
            owners = ', '.join(f'{item["kind"]} PID {item["pid"]}' for item in existing)
            raise RuntimeError(f'Existing hardware connection in ROS domain {domain}: {owners}. '
                               'A second launch could initialize the gripper or replace active controllers. '
                               'Reuse the current connection; no additional nodes were started.')
    except BaseException:
        os.close(descriptor)
        raise
    _LAUNCH_LOCKS.append(descriptor)
