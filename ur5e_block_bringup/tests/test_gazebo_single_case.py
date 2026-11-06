"""Single-case selection and scheduling settings; no robot endpoints."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from ur5e_block_bringup import gazebo_batch as batch

GRID = Path(__file__).resolve().parents[2]/'ur5e_a3_grid'


def test_only_requested_case_is_registered_and_resumed(tmp_path):
    folder = tmp_path/'case'
    case = [(1, 6)]
    campaign = batch.Campaign(folder, 'v1', {}, cases=case)
    try:
        assert campaign.state['cases'] == [[1, 6]]
        assert batch.pending_pairs(campaign.state) == case
        campaign.state['results']['obstacle_1_target_6'] = dict(status='succeeded')
        assert batch.pending_pairs(campaign.state) == []
        batch.write_reports(folder, campaign.state)
        assert batch.read_json(folder/'summary.json')['total'] == 1
        assert '이웃 대각선 1가지' in (folder/'report.html').read_text()
        # Leave the synthetic success unsaved; no fabricated execution archive.
        campaign.state['results'].clear()
        campaign.checkpoint()
    finally:
        campaign.close()
    with pytest.raises(ValueError, match='모델·설정·코드'):
        batch.Campaign(folder, 'v1', {}, cases=[(11, 14)])
    resumed = batch.Campaign(folder, 'v1', {}, cases=case)
    resumed.close()


def test_cpu_budget_and_single_case_output(monkeypatch, tmp_path):
    monkeypatch.setenv('UR5E_WS', str(tmp_path))
    monkeypatch.setattr(batch, 'available_cpus', lambda: 20)
    args = batch.parse_args(['--obstacle', '1', '--target', '6'])
    assert args.cases == [(1, 6)]
    assert args.workers == 18 and args.physics_threads == 1
    assert args.output_dir == tmp_path/'log/gazebo_1_6_parallel'
    assert not args.output_dir.exists()
    serial = batch.parse_args(['--workers', '1', '--physics-threads', '1'])
    assert serial.workers == serial.physics_threads == 1
    assert len(serial.cases) == 60


@pytest.mark.parametrize('arguments', [
    ['--obstacle', '1'], ['--target', '6'], ['--obstacle', '1', '--target', '2'],
    ['--obstacle', '4', '--target', '9'], ['--workers', '-1'], ['--workers', '65'],
    ['--physics-threads', '65']])
def test_bad_scope_or_budget_rejected(arguments):
    with pytest.raises(SystemExit) as error:
        batch.parse_args(arguments)
    assert error.value.code == 2


def test_listing_one_case_never_starts_ros_or_creates_files(monkeypatch, tmp_path, capsys):
    monkeypatch.setenv('UR5E_WS', str(tmp_path))
    assert batch.main(['--obstacle', '1', '--target', '6', '--list']) == 0
    output = capsys.readouterr().out
    assert '총 1가지' in output and '총 60가지' not in output
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('threads', [1, 4, 18])
def test_parallel_world_preserves_physical_model(threads):
    original = (GRID/'worlds/stack.world').read_text()
    before = ET.fromstring(original)
    after = ET.fromstring(batch.parallel_world(original, threads))
    solver = after.find('world/physics/ode/solver')
    assert solver.findtext('island_threads') == str(threads if threads > 1 else 0)
    assert solver.findtext('thread_position_correction') == ('true' if threads > 1 else 'false')
    assert after.findtext('world/physics/max_step_size') == '0.001'
    assert after.findtext('world/physics/real_time_update_rate') == '1000'
    assert after.findtext('world/physics/ode/solver/iters') == '100'
    for name in ('island_threads', 'thread_position_correction'):
        solver.remove(solver.find(name))
    assert ET.tostring(after) == ET.tostring(before)


def test_non_ode_world_is_not_silently_rewritten():
    with pytest.raises(ValueError, match='ODE'):
        batch.parallel_world('<sdf><world><physics type="bullet"/></world></sdf>', 18)


def test_parallel_settings_and_actual_world_are_in_success_zip(tmp_path):
    from test_gazebo_batch import Harness, parameters
    import zipfile
    import json
    args = batch.parse_args(['--obstacle', '1', '--target', '6',
        '--output-dir', str(tmp_path/'reports'), '--archive-root', str(tmp_path/'archives')])
    args.world_file = tmp_path/'parallel.world'
    world = batch.parallel_world((GRID/'worlds/stack.world').read_text(), args.physics_threads)
    args.world_file.write_text(world)
    campaign = batch.Campaign(args.output_dir, 'fixture', {}, cases=args.cases)
    try:
        harness = Harness(args, ['succeeded'])
        # The real runner's model binding is unchanged by parallel scheduling.
        from test_gazebo_batch import MODEL_HASH
        campaign.state['metadata']['model_sha256'] = MODEL_HASH
        assert batch.Runner(args, campaign, parameters(), harness, harness.process).run() == 0
        result = campaign.state['results']['obstacle_1_target_6']
        with zipfile.ZipFile(result['successful_path']) as bundle:
            assert bundle.read('parallel.world').decode() == world
            settings = json.loads(bundle.read('parallelism.json'))
            assert settings['planner_workers'] == args.workers
            assert settings['world_sha256'] == batch.sha256(args.world_file)
    finally:
        campaign.close()
