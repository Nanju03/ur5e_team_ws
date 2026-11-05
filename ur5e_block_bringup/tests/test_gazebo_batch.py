"""Trial orchestration with serialized data fixtures and owned dummy processes.

No robot action server, hardware driver, or Gazebo is started by this file.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import zipfile

import pytest
from rclpy.serialization import serialize_message
from moveit_msgs.msg import DisplayTrajectory, PlanningScene, RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

from ur5e_block_bringup import gazebo_batch as batch
from ur5e_block_bringup.path_archive import prepare_execution_archive, seal_successful_archive

GRID = Path(__file__).resolve().parents[2]/'ur5e_a3_grid'
URDF = '<robot name="data_contract_only"><ros2_control name="GazeboSystem"><hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware><joint name="arm"><command_interface name="position_pid"/></joint><joint name="left_outer_knuckle_joint"><command_interface name="effort"/></joint></ros2_control></robot>'
MODEL = {'robot_description': URDF, 'robot_description_semantic': '<robot name="data_contract_only"/>'}
MODEL_HASH = hashlib.sha256(URDF.encode()).hexdigest()


def parameters(pair=(11, 14)):
    data = {}
    for name in ('a3_grid.yaml', 'stack_blocks.yaml'):
        data.update(yaml.safe_load((GRID/'config'/name).read_text())['/**']['ros__parameters'])
    data.update(target_cell=pair[1], obstacle_cells=[pair[0]], empty_cells=[],
                stack_mode='run', execution_mode='continuous', simulation_backend='gazebo')
    return data


def data_archive(root, pair=(11, 14)):
    """Small synthetic command file with complete 23-layer placement metadata."""
    folder = prepare_execution_archive(parameters(pair), MODEL, GRID, root=root)
    files = {'initial.cdr': PlanningScene(), 'state.cdr': RobotState(), 'preview.cdr': DisplayTrajectory()}
    path = RobotTrajectory()
    path.joint_trajectory.joint_names = ['left_outer_knuckle_joint']
    point = JointTrajectoryPoint(positions=[.4], velocities=[0.], accelerations=[0.])
    point.time_from_start.sec = 1
    path.joint_trajectory.points = [point]
    steps = [dict(index=1, kind='hand', phase='open', file='planned.cdr', scene_before_file='before.cdr', scene_after_file='after.cdr')]
    files.update({'planned.cdr': path, 'before.cdr': PlanningScene(), 'after.cdr': PlanningScene(),
                  'sent.cdr': path, 'start.cdr': RobotState(), 'end.cdr': RobotState()})
    sources = sorted(set(range(1, 25))-set(pair))
    for completed, block in enumerate(sources, 1):
        file = f'place_{block}.cdr'
        files[file] = PlanningScene()
        steps.append(dict(index=completed+1, kind='scene', phase='place', block_id=block,
                          completed=completed, destination_cell=pair[1], temporary=False,
                          file=file, scene_before_file='before.cdr', scene_after_file='after.cdr'))
    for name, message in files.items():
        (folder/name).write_bytes(serialize_message(message))
    batch.write_json(folder/'plan.recipe.json', dict(target_cell=pair[1], layers=23, moves=22,
        initial_scene_file='initial.cdr', initial_state_file='state.cdr', steps=steps))
    journal = [dict(event='dispatched', index=1, file='sent.cdr', start_state_file='start.cdr'),
               dict(event='completed', index=1, end_state_file='end.cdr')]
    (folder/'commands.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in journal))
    batch.write_json(folder/'execution_result.json', dict(status='planned', backend='gazebo', plan_id='sim_data_contract_only',
        target_cell=pair[1], completed_moves=0, layers=1, completed_commands=0))
    return folder


def ready_event(folder):
    events = batch.RunEvents()
    events.feed('STACK_EXECUTION_ARCHIVE '+json.dumps(dict(directory=str(folder), plan_id='sim_data_contract_only')))
    events.feed('STACK_PREFLIGHT_PASSED moves=22 layers=23 segments=300')
    events.feed('STACK_STEP_READY '+json.dumps(dict(plan_id=events.plan_id, request_id=events.plan_id+'_1',
        index=1, total=1, duration=500., approve_service='/ur5e_a3_stack/approve_next_'+events.plan_id+'_1')))
    return events


def test_exactly_60_ordered_neighbour_diagonals_without_wrap_or_distant_pairs():
    pairs = batch.adjacent_diagonals()
    assert len(pairs) == len(set(pairs)) == 60
    assert pairs[:2] == [(11, 14), (14, 11)]
    for o, t in pairs:
        assert (t, o) in pairs
        r1, c1 = divmod(o-1, 4)
        r2, c2 = divmod(t-1, 4)
        assert abs(r1-r2) == abs(c1-c2) == 1
    assert (4, 9) not in pairs and (1, 11) not in pairs


def test_reports_keep_failures_separate_from_pending_interrupted_and_success(tmp_path):
    state = dict(cases=[[11, 14], [14, 11], [1, 6], [1, 2]], status='partial',
                 results={'obstacle_11_target_14': dict(status='failed', reason='collides <unsafe> | object', stage='planning'),
                          'obstacle_14_target_11': dict(status='interrupted'),
                          'obstacle_1_target_6': dict(status='succeeded')})
    batch.write_reports(tmp_path, state)
    failed = batch.read_json(tmp_path/'failed_cases.json')
    assert failed['count'] == 1 and failed['cases'][0]['obstacle'] == 11
    report = (tmp_path/'report.html').read_text()
    assert '&lt;unsafe&gt;' in report and '<unsafe>' not in report
    assert batch.read_json(tmp_path/'summary.json')['counts']['pending'] == 1
    assert len((tmp_path/'failed_cases.csv').read_text().splitlines()) == 2


def test_resume_recovers_interrupted_ignores_finished_and_retries_only_failures(tmp_path):
    campaign = batch.Campaign(tmp_path, 'same', {})
    try:
        campaign.state['results'] = {batch.key((11, 14)): dict(status='running', attempts=1),
                                    batch.key((14, 11)): dict(status='timeout', attempts=1)}
        campaign.checkpoint()
        with pytest.raises(BlockingIOError):
            batch.Campaign(tmp_path, 'same', {})
    finally:
        campaign.close()
    resumed = batch.Campaign(tmp_path, 'same', {})
    try:
        assert resumed.state['results'][batch.key((11, 14))]['status'] == 'interrupted'
        assert len(resumed.state['results'][batch.key((11, 14))]['history']) == 1
        assert (11, 14) in batch.pending_pairs(resumed.state)
        assert (14, 11) not in batch.pending_pairs(resumed.state)
        assert batch.pending_pairs(resumed.state, retry_failed=True) == [(14, 11)]
    finally:
        resumed.close()
    with pytest.raises(ValueError, match='모델·설정·코드'):
        batch.Campaign(tmp_path, 'changed', {})


@pytest.mark.parametrize('change', ['target', 'backend', 'model', 'mode', 'empty', 'plan_id', 'service', 'extra_gate', 'preflight', 'archive_root'])
def test_approval_is_bound_to_this_full_gazebo_case_and_plan(tmp_path, change):
    folder = data_archive(tmp_path)
    events = ready_event(folder)
    filename = 'context.json'
    value = batch.read_json(folder/filename)
    if change == 'target':
        value['target_cell'] = 10
    elif change == 'backend':
        value['backend'] = 'real'
    elif change == 'model':
        value['model_sha256'] = 'changed'
    elif change in ('mode', 'empty'):
        filename = 'selection.yaml'
        value = yaml.safe_load((folder/filename).read_text())
        value['/**']['ros__parameters']['execution_mode' if change == 'mode' else 'empty_cells'] = 'step' if change == 'mode' else [1]
    elif change == 'plan_id':
        events.ready['plan_id'] = 'other'
    elif change == 'service':
        events.ready['approve_service'] = '/unrelated_trigger'
    elif change == 'extra_gate':
        events.ready['total'] = 2
    elif change == 'preflight':
        events.preflight = False
    elif change == 'archive_root':
        events.archive = folder.parent
    if filename.endswith('.yaml'):
        (folder/filename).write_text(yaml.safe_dump(value))
    else:
        batch.write_json(folder/filename, value)
    with pytest.raises(batch.TrialError):
        events.approval_service((11, 14), tmp_path, MODEL_HASH)
    assert not events.bound


def test_correct_gate_and_all_23_final_block_centres_include_foundation(tmp_path):
    folder = data_archive(tmp_path)
    events = ready_event(folder)
    assert events.approval_service((11, 14), tmp_path, MODEL_HASH).endswith('sim_data_contract_only_1')
    expected = batch.final_tower_positions(folder, (11, 14), parameters())
    assert len(expected) == 23 and 'ur5e_stack_block_11' not in expected
    assert expected['ur5e_stack_block_14'][2] == pytest.approx(.8175)
    assert max(p[2] for p in expected.values()) == pytest.approx(1.3675)


@pytest.mark.parametrize('status', ['planned', 'running'])
def test_gate_accepts_current_cpp_preapproval_lifecycle_but_no_completed_commands(tmp_path, status):
    folder = data_archive(tmp_path)
    result = batch.read_json(folder/'execution_result.json')
    result['status'] = status
    batch.write_json(folder/'execution_result.json', result)
    assert ready_event(folder).approval_service((11, 14), tmp_path, MODEL_HASH)
    result['completed_commands'] = 1
    batch.write_json(folder/'execution_result.json', result)
    with pytest.raises(batch.TrialError):
        ready_event(folder).approval_service((11, 14), tmp_path, MODEL_HASH)


@pytest.mark.parametrize('fault', ['missing', 'duplicate', 'wrong_target'])
def test_incomplete_final_placement_order_is_not_accepted(tmp_path, fault):
    folder = data_archive(tmp_path)
    recipe = batch.read_json(folder/'plan.recipe.json')
    if fault == 'missing':
        recipe['steps'].pop()
    elif fault == 'duplicate':
        recipe['steps'][-1]['block_id'] = recipe['steps'][-2]['block_id']
    else:
        recipe['steps'][-1]['destination_cell'] = 10
    batch.write_json(folder/'plan.recipe.json', recipe)
    with pytest.raises(batch.TrialError):
        batch.final_tower_positions(folder, (11, 14), parameters())


@pytest.mark.parametrize('fault', ['real_model', 'real_component', 'unknown_imported', 'unclaimed', 'inactive'])
def test_controller_gate_rejects_physical_or_unready_hardware(fault):
    urdf = URDF
    interfaces = [NS(name='arm/position', is_available=True, is_claimed=True),
                  NS(name='left_outer_knuckle_joint/effort', is_available=True, is_claimed=True)]
    component = NS(name='GazeboSystem', class_type='', state=NS(id=3), command_interfaces=interfaces)
    controllers = [NS(name=n, state='active') for n in ('joint_state_broadcaster', 'ur5e_arm_controller', 'dh_ag95_gripper_controller')]
    batch.validate_hardware(urdf, [component], controllers)
    if fault == 'real_model':
        urdf = urdf.replace('gazebo_ros2_control/GazeboSystem', 'ur_robot_driver/URPositionHardwareInterface')
    elif fault == 'real_component':
        component.class_type = 'ur_robot_driver/URPositionHardwareInterface'
    elif fault == 'unknown_imported':
        component.name = 'unknown'
    elif fault == 'unclaimed':
        interfaces[0].is_claimed = False
    else:
        controllers[0].state = 'inactive'
    with pytest.raises(batch.TrialError):
        batch.validate_hardware(urdf, [component], controllers)


def test_whole_tower_check_waits_two_sim_seconds_and_detects_older_fallen_block(tmp_path):
    monitor = object.__new__(batch.RosMonitor)
    monitor.clock, monitor.bodies_at = 10., time.monotonic()
    monitor.bodies = {'ur5e_stack_block_1': (.4, 0., .85), 'ur5e_stack_block_14': (.4, 0., .8175)}
    monitor.cancelled = lambda: False
    def pump():
        monitor.clock += .1
        monitor.bodies_at = time.monotonic()
    monitor.pump = pump
    expected = copy.deepcopy(monitor.bodies)
    path = tmp_path/'observations.json'
    monitor.verify_tower(expected, lambda: True, path)
    result = batch.read_json(path)
    assert result['simulation_separation_s'] >= 2. and len(result['samples']) == 2
    path.unlink()
    monitor.bodies['ur5e_stack_block_1'] = (.8, 0., .8)
    with pytest.raises(batch.TrialError, match='23층 위치 오차'):
        monitor.verify_tower(expected, lambda: True, path)
    assert not path.exists()


@pytest.mark.parametrize('hand', [.0, .4, .555181411, .93])
def test_uncommanded_effort_gripper_drift_does_not_deadlock_startup(hand):
    home = {'arm': 1.5, 'left_outer_knuckle_joint': .4}
    feedback = {'arm': (1.5255223893050376, 100.), 'left_outer_knuckle_joint': (hand, 100.)}
    assert batch.initial_feedback_problem(home, feedback, 100.1) is None


@pytest.mark.parametrize('fault', ['stale', 'missing', 'bad_arm', 'out_of_bounds_hand', 'nan'])
def test_initial_pose_gate_still_requires_fresh_bounded_feedback_and_arm_home(fault):
    home = {'arm': 1.5, 'left_outer_knuckle_joint': .4}
    feedback = {'arm': (1.51, 100.), 'left_outer_knuckle_joint': (.55, 100.)}
    if fault == 'stale':
        feedback['arm'] = (1.51, 90.)
    elif fault == 'missing':
        del feedback['arm']
    elif fault == 'bad_arm':
        feedback['arm'] = (1.56, 100.)
    elif fault == 'out_of_bounds_hand':
        feedback['left_outer_knuckle_joint'] = (.94, 100.)
    else:
        feedback['left_outer_knuckle_joint'] = (float('nan'), 100.)
    assert batch.initial_feedback_problem(home, feedback, 100.1)


class FakeProcess:
    def __init__(self, harness, command, logfile, env):
        self.harness, self.command, self.logfile = harness, command, Path(logfile)
        self.stage = 'server' if 'stack_gazebo.launch.py' in command else 'scene' if 'stack_mode:=scene' in command else 'run'
        self.stopped = self.sent = self.approved = self.done = False
        self.logfile.write_text('')
        if self.stage == 'server':
            harness.servers.append(self)
        if self.stage == 'run':
            selection = next(s.split(':=', 1)[1] for s in command if s.startswith('selection_file:='))
            data = yaml.safe_load(Path(selection).read_text())['/**']['ros__parameters']
            self.pair = (data['obstacle_cells'][0], data['target_cell'])
            self.mode = harness.modes.pop(0)
            self.archive = data_archive(harness.args.archive_root, self.pair)
            harness.current = self

    def poll(self):
        return 0 if self.done or self.stopped else None

    def finished(self):
        return self.done

    def stop(self):
        self.stopped = True

    def drain(self):
        lines = []
        if self.stage == 'scene' and not self.sent:
            self.sent = self.done = True
            lines = ['STACK_STATUS phase=SCENE moved=0 total=0 target=14 layers=1']
        elif self.stage == 'run' and not self.sent:
            self.sent = True
            if self.mode == 'timeout':
                lines = ['STACK_PLANNING source=10 target=14 layer=2/23']
            else:
                events = ready_event(self.archive)
                lines = ['STACK_EXECUTION_ARCHIVE '+json.dumps(dict(directory=str(self.archive), plan_id=events.plan_id)),
                         'STACK_PREFLIGHT_PASSED moves=22 layers=23 segments=300',
                         'STACK_STEP_READY '+json.dumps(events.ready)]
        elif self.stage == 'run' and self.approved and not self.done:
            self.done = True
            if self.mode == 'failed':
                lines = ['STACK_ERROR synthetic fixture contact failure']
            else:
                result = batch.read_json(self.archive/'execution_result.json')
                result.update(status='succeeded', completed_moves=22, layers=23, completed_commands=1)
                batch.write_json(self.archive/'execution_result.json', result)
                if self.mode == 'archive_error':
                    (self.archive/'sent.cdr').unlink()
                lines = [f'STACK_STATUS phase=DONE moved=22 total=22 target={self.pair[1]} layers=23',
                         f'STACK_SUCCESS moved=22 layers=23 target={self.pair[1]}']
        if lines:
            with self.logfile.open('a') as stream:
                stream.write('\n'.join(lines)+'\n')
        return lines


class Harness:
    def __init__(self, args, modes):
        self.args, self.modes, self.servers, self.current = args, list(modes), [], None
        self.approvals, self.stops = [], 0

    def process(self, command, logfile, env):
        return FakeProcess(self, command, logfile, env)

    def pump(self):
        time.sleep(.002)

    def wait_idle(self):
        assert all(server.stopped for server in self.servers)

    def ready(self):
        return True

    def approve(self, service):
        self.approvals.append(service)
        self.current.approved = True

    def request_stop(self):
        self.stops += 1

    def verify_tower(self, expected, keep_alive, path):
        assert len(expected) == 23 and keep_alive()
        if self.current.mode == 'tower_failed':
            raise batch.TrialError('synthetic fixture: older cube fell')
        batch.write_json(path, dict(status='passed', sample_fixture_only=True))


def test_sequence_saves_success_keeps_failures_continues_and_resumes_without_overwrite(tmp_path):
    args = batch.parse_args(['--output-dir', str(tmp_path/'reports'), '--archive-root', str(tmp_path/'archives'), '--max-cases', '5', '--planning-timeout', '.01'])
    campaign = batch.Campaign(args.output_dir, 'fixture_signature', {'model_sha256': MODEL_HASH})
    harness = Harness(args, ['succeeded', 'failed', 'timeout', 'archive_error', 'tower_failed'])
    runner = batch.Runner(args, campaign, parameters(), harness, harness.process)
    try:
        assert runner.run() == 1
        selected = batch.adjacent_diagonals()[:5]
        assert [campaign.state['results'][batch.key(p)]['status'] for p in selected] == [
            'succeeded', 'failed', 'timeout', 'archive_error', 'failed']
        success = campaign.state['results'][batch.key(selected[0])]
        bundle = Path(success['successful_path'])
        original = bundle.read_bytes()
        with zipfile.ZipFile(bundle) as zipped:
            assert zipped.testzip() is None
            assert 'tower_verification.json' in zipped.namelist()
            assert json.loads(zipped.read('batch_verification.json'))['status'] == 'passed'
        assert batch.read_json(args.output_dir/'failed_cases.json')['count'] == 4
        assert all(server.stopped for server in harness.servers) and len(harness.servers) == 5
        assert len(harness.approvals) == 4  # A planning timeout never gets an approval.
        failed_tower = campaign.state['results'][batch.key(selected[4])]
        folder = Path(failed_tower['archive_directory'])
        with pytest.raises(ValueError, match='whole-tower'):
            seal_successful_archive(folder)
    finally:
        campaign.close()
    resumed = batch.Campaign(args.output_dir, 'fixture_signature', {'model_sha256': MODEL_HASH})
    try:
        assert batch.pending_pairs(resumed.state)[0] == batch.adjacent_diagonals()[5]
        assert batch.pending_pairs(resumed.state, True) == selected[1:]
        args.retry_failed, args.max_cases = True, 1
        retry = Harness(args, ['succeeded'])
        assert batch.Runner(args, resumed, parameters(), retry, retry.process).run() == 1
        assert resumed.state['results'][batch.key(selected[1])]['attempts'] == 2
        assert len(resumed.state['results'][batch.key(selected[1])]['history']) == 2
        assert bundle.read_bytes() == original
        assert batch.read_json(args.output_dir/'failed_cases.json')['count'] == 3
    finally:
        resumed.close()


def test_stop_during_approval_does_not_count_success_or_continue(tmp_path):
    args = batch.parse_args(['--output-dir', str(tmp_path/'reports'), '--archive-root', str(tmp_path/'archives'), '--max-cases', '2'])
    campaign = batch.Campaign(args.output_dir, 'fixture', {'model_sha256': MODEL_HASH})
    harness = Harness(args, ['succeeded'])
    runner = batch.Runner(args, campaign, parameters(), harness, harness.process)
    def stop_during_approval(_):
        runner.stopping = True
        raise batch.Interrupted()
    harness.approve = stop_during_approval
    try:
        assert runner.run() == 130
        assert len(harness.servers) == 1 and harness.servers[0].stopped
        row = campaign.state['results'][batch.key((11, 14))]
        assert row['status'] == 'interrupted'
        assert not list(args.archive_root.rglob('successful_path.zip'))
        assert batch.read_json(args.output_dir/'failed_cases.json')['count'] == 0
        assert batch.pending_pairs(campaign.state)[0] == (11, 14)
    finally:
        campaign.close()


def test_owned_process_captures_full_log_and_reaps_its_sleeping_child(tmp_path):
    child_code = 'import time; time.sleep(120)'
    code = ('import subprocess,sys,time\n'
            'child=subprocess.Popen([sys.executable,"-c",'+repr(child_code)+'])\n'
            'print(child.pid,flush=True)\n'
            'print("마지막 기록",flush=True)\n'
            'try:\n time.sleep(120)\n'
            'except KeyboardInterrupt:\n child.wait(timeout=2)\n')
    process = batch.ManagedProcess([sys.executable, '-u', '-c', code], tmp_path/'process.log', os.environ.copy())
    try:
        deadline = time.monotonic()+3.
        while '마지막 기록' not in (tmp_path/'process.log').read_text():
            assert time.monotonic() < deadline
            time.sleep(.01)
        child_pid = int((tmp_path/'process.log').read_text().splitlines()[0])
    finally:
        process.stop()
    assert process.poll() is not None and not process.reader.is_alive()
    with pytest.raises(ProcessLookupError):
        os.kill(child_pid, 0)
    assert '마지막 기록' in ''.join(process.drain())


def test_three_environment_errors_stop_before_testing_every_remaining_layout(tmp_path):
    args = batch.parse_args(['--output-dir', str(tmp_path/'reports'), '--archive-root', str(tmp_path/'archives')])
    campaign = batch.Campaign(args.output_dir, 'fixture', {})
    harness = Harness(args, [])
    def not_ready():
        raise batch.TrialError('fixture simulator unavailable', 'environment_error')
    harness.ready = not_ready
    try:
        runner = batch.Runner(args, campaign, parameters(), harness, harness.process)
        assert runner.run() == 130
        assert len(harness.servers) == 3 and all(s.stopped for s in harness.servers)
        summary = batch.read_json(args.output_dir/'summary.json')
        assert summary['counts']['pending'] == 57
        assert batch.read_json(args.output_dir/'failed_cases.json')['count'] == 3
        assert all((args.output_dir/row['log']).is_file() for row in campaign.state['results'].values())
    finally:
        campaign.close()


def test_low_disk_space_leaves_all_cases_untested(tmp_path, monkeypatch):
    args = batch.parse_args(['--output-dir', str(tmp_path/'reports'), '--archive-root', str(tmp_path/'archives')])
    campaign = batch.Campaign(args.output_dir, 'fixture', {})
    harness = Harness(args, [])
    monkeypatch.setattr(batch.shutil, 'disk_usage', lambda _: NS(free=10))
    try:
        assert batch.Runner(args, campaign, parameters(), harness, harness.process).run() == 130
        assert not harness.servers
        assert batch.read_json(args.output_dir/'failed_cases.json')['count'] == 0
        assert batch.read_json(args.output_dir/'summary.json')['counts']['pending'] == 60
    finally:
        campaign.close()


def test_abandoned_attempt_folder_is_not_overwritten_after_an_abrupt_exit(tmp_path):
    args = batch.parse_args(['--output-dir', str(tmp_path/'reports'), '--archive-root', str(tmp_path/'archives'), '--max-cases', '1'])
    campaign = batch.Campaign(args.output_dir, 'fixture', {'model_sha256': MODEL_HASH})
    abandoned = args.output_dir/'cases/obstacle_11_target_14/attempt_001'
    abandoned.mkdir(parents=True)
    (abandoned/'untouched.log').write_text('before the checkpoint')
    harness = Harness(args, ['succeeded'])
    try:
        assert batch.Runner(args, campaign, parameters(), harness, harness.process).run() == 0
        assert (abandoned/'untouched.log').read_text() == 'before the checkpoint'
        assert campaign.state['results'][batch.key((11, 14))]['attempts'] == 2
        success = campaign.state['results'][batch.key((11, 14))]
        Path(success['successful_path']).write_bytes(b'modified successful zip')
    finally:
        campaign.close()
    with pytest.raises(ValueError, match='성공 ZIP'):
        batch.Campaign(args.output_dir, 'fixture', {'model_sha256': MODEL_HASH})


def test_actual_configuration_uses_the_same_gazebo_model_and_physics_as_manual_launch():
    workspace = GRID.parents[2]
    settings, urdf, srdf, home, signature, metadata = batch.load_configuration(workspace, .016)
    assert settings['gripper_grasp_position'] == .78 and settings['grasp_height_above_table'] == .0225
    assert settings['vertical_placement_layers'] == 19 and settings['placement_approach_clearance'] == .12
    assert len(home) == 7 and home['left_outer_knuckle_joint'] == .4
    assert metadata['model_sha256'] == hashlib.sha256(urdf.encode()).hexdigest()
    assert 'gazebo_ros2_control/GazeboSystem' in urdf and len(signature) == 64
    _, _, _, _, second, _ = batch.load_configuration(workspace, .016)
    assert second == signature


def test_list_mode_has_no_ros_or_file_side_effects(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    assert batch.main(['--list', '--output-dir', str(tmp_path/'never_created')]) == 0
    assert not (tmp_path/'never_created').exists()
    assert len([line for line in capsys.readouterr().out.splitlines() if line.startswith('장애물 ')]) == 60


@pytest.mark.parametrize('option,value', [('--planning-timeout', 'nan'), ('--execution-timeout', '-1'), ('--max-cases', '-1'), ('--riser-height', '.4')])
def test_invalid_resource_arguments_are_rejected(option, value):
    with pytest.raises(SystemExit):
        batch.parse_args([option, value])
