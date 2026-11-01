"""Exercise duplicate detection with synthetic /proc entries; no ROS nodes."""
import importlib.util
from pathlib import Path

import pytest


SOURCE = Path(__file__).resolve().parents[1]/'ur5e_block_bringup/driver_launch_guard.py'


@pytest.fixture
def guard():
    spec = importlib.util.spec_from_file_location('isolated_driver_launch_guard', SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    yield module
    for descriptor in module._LAUNCH_LOCKS:
        module.os.close(descriptor)


def process(root, pid, arguments, *, domain='0', uid=1000, state='S'):
    directory = root/str(pid)
    directory.mkdir()
    (directory/'status').write_text(f'Name:\tcontrol\nState:\t{state} (test)\nUid:\t{uid}\t{uid}\t{uid}\t{uid}\n')
    (directory/'cmdline').write_bytes(('\0'.join(arguments)+'\0').encode())
    (directory/'environ').write_bytes(b'PATH=/usr/bin\0'+
        (f'ROS_DOMAIN_ID={domain}\0'.encode() if domain is not None else b''))
    return directory


@pytest.mark.parametrize('arguments,expected', [
    (['/opt/ros/humble/lib/ur_robot_driver/ur_ros2_control_node'], 'UR arm driver'),
    (['/opt/ros/humble/lib/controller_manager/ros2_control_node', '--ros-args', '-r', '__ns:=/gripper'], 'AG95 gripper driver'),
    (['ros2_control_node', '--ros-args', '--remap', '__ns:=gripper/'], 'AG95 gripper driver'),
    (['ros2_control_node', '--ros-args', '-r', '__ns:=/other', '-r', '__ns:=/gripper'], 'AG95 gripper driver'),
    (['ros2_control_node', '--ros-args', '-r', '__ns:=/gripper', '-r', '__ns:=/other'], None),
    (['ros2_control_node', '--ros-args', '-r', '__ns:=/other'], None),
    (['ros2_control_node'], None),
    (['python3', 'log_parser.py', 'ur_ros2_control_node'], None),
    (['spawner', 'ur_ros2_control_node'], None),
    ([], None),
])
def test_identifies_executable_and_effective_gripper_namespace(guard, arguments, expected):
    assert guard.controller_kind(arguments) == expected


def test_matches_only_live_same_user_same_domain_drivers(guard, tmp_path):
    process(tmp_path, 1, ['ur_ros2_control_node'], domain='07')
    process(tmp_path, 2, ['ros2_control_node', '--ros-args', '-r', '__ns:=/gripper'], domain='7')
    process(tmp_path, 3, ['ur_ros2_control_node'], domain='42')
    process(tmp_path, 4, ['ur_ros2_control_node'], domain='7', uid=2000)
    process(tmp_path, 5, ['ur_ros2_control_node'], domain='7', state='Z')
    process(tmp_path, 6, ['ros2_control_node', '--ros-args', '-r', '__ns:=/other'], domain='7')
    process(tmp_path, 7, ['robot_state_publisher'], domain='7')
    assert [item['pid'] for item in guard.controller_processes(domain='7', proc_root=tmp_path, uid=1000)] == [1, 2]


def test_missing_domain_is_domain_zero_not_an_unknown_or_other_domain(guard, tmp_path):
    process(tmp_path, 9, ['ur_ros2_control_node'], domain=None)
    assert [item['pid'] for item in guard.controller_processes(domain=0, proc_root=tmp_path, uid=1000)] == [9]
    assert guard.controller_processes(domain=103, proc_root=tmp_path, uid=1000) == []


def test_ignores_a_candidate_which_exited_before_environment_read(guard, tmp_path):
    directory = process(tmp_path, 9, ['ur_ros2_control_node'])
    (directory/'environ').unlink()
    assert guard.controller_processes(domain=0, proc_root=tmp_path, uid=1000) == []


def test_unverifiable_candidate_domain_fails_closed(guard, tmp_path):
    directory = process(tmp_path, 9, ['ur_ros2_control_node'])
    (directory/'environ').unlink()
    (directory/'environ').mkdir()
    with pytest.raises(RuntimeError, match='Cannot verify ROS domain.*PID 9'):
        guard.controller_processes(domain=0, proc_root=tmp_path, uid=1000)


@pytest.mark.parametrize('arguments,arm,gripper', [
    (['ur_ros2_control_node'], True, True),
    (['ur_ros2_control_node'], False, True),
    (['ros2_control_node', '--ros-args', '-r', '__ns:=/gripper'], True, False),
])
def test_partial_existing_stack_blocks_all_requested_hardware(guard, tmp_path, arguments, arm, gripper):
    proc = tmp_path/'proc'; proc.mkdir()
    process(proc, 71, arguments, domain='0')
    with pytest.raises(RuntimeError, match='Existing hardware connection.*PID 71'):
        guard.guard_hardware_launch(start_arm=arm, start_gripper=gripper,
                                   domain=0, proc_root=proc, lock_root=tmp_path, uid=1000)
    assert not guard._LAUNCH_LOCKS


def test_model_only_launch_does_not_inspect_or_lock_hardware(guard, tmp_path):
    guard.guard_hardware_launch(start_arm=False, start_gripper=False,
                               domain=0, proc_root=tmp_path/'missing', lock_root=tmp_path/'missing')
    assert not guard._LAUNCH_LOCKS and list(tmp_path.iterdir()) == []


def test_lock_closes_simultaneous_launch_race_and_lasts_until_process_exit(guard, tmp_path):
    proc = tmp_path/'proc'; proc.mkdir()
    kwargs = dict(start_arm=True, start_gripper=True, domain=0, proc_root=proc, lock_root=tmp_path, uid=1000)
    guard.guard_hardware_launch(**kwargs)
    with pytest.raises(RuntimeError, match='hardware launch is already running'):
        guard.guard_hardware_launch(**kwargs)
    assert len(guard._LAUNCH_LOCKS) == 1


def test_other_domain_hardware_and_launch_lock_are_independent(guard, tmp_path):
    proc = tmp_path/'proc'; proc.mkdir()
    process(proc, 71, ['ur_ros2_control_node'], domain='42')
    for domain in (0, 103):
        guard.guard_hardware_launch(start_arm=True, start_gripper=True,
                                   domain=domain, proc_root=proc, lock_root=tmp_path, uid=1000)
    assert len(guard._LAUNCH_LOCKS) == 2


@pytest.mark.parametrize('domain', ['invalid', -1])
def test_rejects_invalid_domain_before_lock_or_proc_inspection(guard, tmp_path, domain):
    with pytest.raises(RuntimeError, match='ROS_DOMAIN_ID'):
        guard.guard_hardware_launch(start_arm=True, start_gripper=True,
                                   domain=domain, proc_root=tmp_path, lock_root=tmp_path)
    assert not list(tmp_path.iterdir())
