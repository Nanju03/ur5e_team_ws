"""Shareable workspace installation contracts; no ROS, network, or robot startup."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('team_workspace_installer', REPO/'scripts/install_workspace.py')
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


def fixture_repo(tmp_path):
    repo = tmp_path/'repo'
    source = repo/'workspace'
    (source/'scripts').mkdir(parents=True)
    (source/'env.bash').write_text('export EXAMPLE=1\n')
    (source/'scripts/start.sh').write_text('#!/bin/bash\ntrue\n')
    (source/'scripts/start.sh').chmod(0o755)
    (source/'COLCON_IGNORE').touch()
    return repo


def test_fresh_install_is_complete_and_does_not_disable_workspace_colcon(tmp_path):
    repo = fixture_repo(tmp_path)
    target = tmp_path/'new_ws'
    result = installer.install(repo, target)
    assert result['updates'] == 2
    assert (target/'env.bash').read_text() == 'export EXAMPLE=1\n'
    assert (target/'scripts/start.sh').stat().st_mode & 0o111
    assert not (target/'COLCON_IGNORE').exists()
    assert len(json.loads((target/'.workspace-install.json').read_text())['files']) == 2
    assert installer.install(repo, target)['updates'] == 0


def test_conflict_refuses_every_change_before_writing_any_file(tmp_path):
    repo = fixture_repo(tmp_path)
    target = tmp_path/'ws'
    target.mkdir()
    (target/'env.bash').write_text('personal setting\n')
    with pytest.raises(ValueError, match='기존 파일'):
        installer.install(repo, target)
    assert (target/'env.bash').read_text() == 'personal setting\n'
    assert not (target/'scripts').exists()
    assert not (target/'.workspace-install.json').exists()


def test_explicit_update_backs_up_local_files_and_preserves_unrelated_data(tmp_path):
    repo = fixture_repo(tmp_path)
    target = tmp_path/'ws'
    target.mkdir()
    (target/'env.bash').write_text('keep in backup\n')
    (target/'personal.txt').write_text('untouched')
    result = installer.install(repo, target, overwrite=True)
    backup = Path(result['backup'])
    assert (backup/'env.bash').read_text() == 'keep in backup\n'
    assert (target/'personal.txt').read_text() == 'untouched'
    assert installer.install(repo, target, overwrite=True)['backup'] is None


def test_dry_run_never_creates_a_workspace(tmp_path):
    repo = fixture_repo(tmp_path)
    target = tmp_path/'never_created'
    result = installer.install(repo, target, dry_run=True)
    assert result['updates'] == 2 and not target.exists()


@pytest.mark.parametrize('kind', ['file_link', 'directory_link', 'directory_instead_of_file'])
def test_installer_does_not_follow_unsafe_targets(tmp_path, kind):
    repo = fixture_repo(tmp_path)
    target, outside = tmp_path/'ws', tmp_path/'outside'
    target.mkdir()
    outside.mkdir()
    if kind == 'file_link':
        (outside/'value').write_text('untouched')
        (target/'env.bash').symlink_to(outside/'value')
    elif kind == 'directory_link':
        (target/'scripts').symlink_to(outside, target_is_directory=True)
    else:
        (target/'env.bash').mkdir()
    with pytest.raises(ValueError):
        installer.install(repo, target, overwrite=True)
    assert not (outside/'start.sh').exists()


def test_real_template_includes_runtime_dependencies_and_latest_guide(tmp_path):
    installer.install(REPO, tmp_path/'actual_ws')
    target = tmp_path/'actual_ws'
    for name in ('env.bash', 'build.sh', 'start_block_gazebo.sh', 'start_block_real_gui.sh',
                 'start_block_robot.sh', 'start_block_sim.sh', 'start_block_view.sh', 'test_block_diagonals.sh',
                 'scripts/gazebo_env.bash', 'scripts/prepare_gazebo_compat.sh', 'scripts/check_setup.py',
                 'scripts/probe_standalone_sim.py', 'docs/real_robot.md', 'docs/gazebo_diagonal_tests.md',
                 'robot_calibration/ur5e_20261031.yaml', 'physical_setup.yaml'):
        assert (target/name).is_file(), name
    assert not (target/'COLCON_IGNORE').exists()
    assert (target/'docs/real_robot.md').read_bytes() == (REPO/'docs/real_robot.md').read_bytes()


def test_command_line_installs_without_ros_environment(tmp_path):
    target = tmp_path/'cli_ws'
    result = subprocess.run([sys.executable, str(REPO/'scripts/install_workspace.py'), '--workspace', str(target)],
                            text=True, capture_output=True, env={'PATH': '/usr/bin:/bin'})
    assert result.returncode == 0, result.stderr
    assert (target/'start_block_gazebo.sh').is_file()
    assert '시작하지 않았습니다' in result.stdout


@pytest.mark.parametrize('fault', ['', 'public', 'different_account', 'dirty', 'remote_sha'])
def test_publisher_checks_private_owner_and_revision_without_network(tmp_path, fault):
    repo = tmp_path/'publish_repo'
    (repo/'scripts').mkdir(parents=True)
    script = repo/'scripts/publish_private_repo.sh'
    script.write_bytes((REPO/'scripts/publish_private_repo.sh').read_bytes())
    binaries = tmp_path/'fake_bin'
    binaries.mkdir()
    log = tmp_path/'commands.jsonl'
    stub = '''#!/usr/bin/python3
import json,os,sys
from pathlib import Path
name=Path(sys.argv[0]).name;a=sys.argv[1:];fault=os.environ['FAULT']
with open(os.environ['COMMAND_LOG'],'a') as f:f.write(json.dumps([name,*a])+'\\n')
if name=='gh':
    if a[:2]==['api','user']:print('wrong_user' if fault=='different_account' else 'Nanju03')
    elif a[:2]==['repo','view']:print('false' if fault=='public' else 'true')
    elif a[:1]==['api']:print('different_sha' if fault=='remote_sha' else 'same_sha')
elif a[:1]==['status']:print(' M file' if fault=='dirty' else '',end='')
elif a[:3]==['remote','get-url','origin']:print('https://github.com/gamsaon/ur5e.git')
elif a[:3]==['remote','get-url','upstream']:sys.exit(2)
elif a[:1]==['rev-parse']:print('same_sha')
'''
    for name in ('git', 'gh'):
        p=binaries/name;p.write_text(stub);p.chmod(0o755)
    env={**os.environ,'PATH':str(binaries)+':/usr/bin:/bin','COMMAND_LOG':str(log),'FAULT':fault}
    result=subprocess.run(['bash',str(script)],env=env,capture_output=True,text=True)
    commands=[json.loads(line) for line in log.read_text().splitlines()]
    pushed=any(row[:2]==['git','push'] for row in commands)
    assert (result.returncode==0) == (fault=='')
    assert pushed == (fault in ('','remote_sha'))
    assert not any('--force' in row or '--public' in row for row in commands)
    if fault=='':
        assert ['git','remote','rename','origin','upstream'] in commands
        assert ['git','push','--set-upstream','origin','HEAD:refs/heads/main'] in commands
