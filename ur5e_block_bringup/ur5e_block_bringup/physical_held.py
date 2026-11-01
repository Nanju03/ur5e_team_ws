"""Persistent, per-domain evidence of an unresolved physically held block."""
import json
import os
from pathlib import Path
import time


def held_fault_path(domain=None):
    domain = os.environ.get('ROS_DOMAIN_ID', '0') if domain is None else str(domain)
    if not domain.isdigit():
        raise ValueError('ROS_DOMAIN_ID must be numeric')
    return Path('/tmp')/f'ur5e_physical_held_{domain}.json'


def read_held_fault(domain=None):
    path = held_fault_path(domain)
    if not path.exists() and not path.is_symlink():
        return None
    try:
        if path.is_symlink() or path.stat().st_size > 16384:
            raise ValueError('Invalid held-block fault file')
        value = json.loads(path.read_text())
        if not isinstance(value, dict):
            raise ValueError('Invalid held-block fault record')
        return value
    except (OSError, ValueError):
        return {'cause': 'Unreadable held-block fault record; inspect the physical gripper before recovery'}


def write_held_fault(cause, model_sha256, domain=None):
    path = held_fault_path(domain)
    data = dict(cause=str(cause)[:1000], model_sha256=str(model_sha256), recorded_at=time.time())
    temporary = path.with_name(path.name+f'.tmp.{os.getpid()}')
    temporary.write_text(json.dumps(data, indent=2)+'\n')
    temporary.replace(path)


def clear_held_fault(domain=None):
    """Caller must already have fresh empty-gripper and standstill evidence."""
    held_fault_path(domain).unlink(missing_ok=True)
