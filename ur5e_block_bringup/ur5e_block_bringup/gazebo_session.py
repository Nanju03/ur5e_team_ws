"""Explicit localhost-only endpoints for manual and parallel Gazebo sessions."""
from dataclasses import dataclass
import os

MAX_CASE_JOBS = 8


@dataclass(frozen=True)
class GazeboSession:
    domain: int = 44
    port: int = 11355
    slot: int | None = None

    @property
    def master_uri(self):
        return f'http://127.0.0.1:{self.port}'

    def environment(self):
        result = dict(ROS_DOMAIN_ID=str(self.domain), ROS_LOCALHOST_ONLY='1',
                      GAZEBO_MASTER_URI=self.master_uri, GAZEBO_IP='127.0.0.1',
                      IGN_PARTITION=f'ur5e_gazebo_{self.domain}')
        if self.slot is not None:
            result['UR5E_GAZEBO_WORKER_SLOT'] = str(self.slot)
        return result


def worker_session(slot):
    if type(slot) is not int or not 0 <= slot < MAX_CASE_JOBS:
        raise ValueError('Gazebo worker slot must be 0..7')
    return GazeboSession(domain=60+slot, port=11370+slot, slot=slot)


def session_from_environment(environment=None):
    """Never broaden simulation access to real-robot domain 0 or arbitrary ports."""
    env = os.environ if environment is None else environment
    slot = env.get('UR5E_GAZEBO_WORKER_SLOT')
    if slot is None:
        session = GazeboSession()
    else:
        if not isinstance(slot, str) or slot not in {str(i) for i in range(MAX_CASE_JOBS)}:
            raise ValueError('Invalid UR5E_GAZEBO_WORKER_SLOT')
        session = worker_session(int(slot))
    if (env.get('ROS_LOCALHOST_ONLY') != '1' or env.get('ROS_DOMAIN_ID') != str(session.domain)
            or env.get('GAZEBO_MASTER_URI') != session.master_uri):
        raise ValueError(f'Gazebo requires localhost ROS {session.domain} / {session.master_uri}; '
                         'use the workspace Gazebo launcher')
    return session
