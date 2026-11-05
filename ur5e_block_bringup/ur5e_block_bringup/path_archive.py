"""Persist complete Gazebo command bundles; this module never executes them."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import uuid
import zipfile
from zoneinfo import ZoneInfo

import yaml


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def prepare_execution_archive(settings, model, share, root=None):
    """Allocate one immutable run directory before a Gazebo run starts."""
    if settings.get('simulation_backend') != 'gazebo':
        raise ValueError('Execution archives currently support Gazebo only')
    target, obstacles = settings.get('target_cell'), settings.get('obstacle_cells', [])
    if (type(target) is not int or not 1 <= target <= 100
            or any(type(cell) is not int or not 1 <= cell <= 100 for cell in obstacles)
            or target in obstacles):
        raise ValueError('Select valid obstacle and target cells before recording a path')
    workspace = Path(os.environ.get('UR5E_WS', str(Path.home()/'ur5e_team_ws')))
    base = Path(root) if root else workspace/'saved_paths/gazebo'
    key = ('obstacle_' if len(obstacles) == 1 else 'obstacles_')+('_'.join(map(str, sorted(obstacles))) or 'none')+f'_target_{target}'
    stamp = datetime.now(ZoneInfo('Asia/Seoul'))
    folder = base.expanduser().resolve()/key/(stamp.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    folder.mkdir(parents=True, exist_ok=False)
    context = dict(schema=1, archive_kind='gazebo_stack_execution_v1', backend='gazebo',
                   created_at=stamp.isoformat(), obstacle_cells=sorted(obstacles), target_cell=target,
                   ros_domain=os.environ.get('ROS_DOMAIN_ID'), archive_id=folder.name,
                   model_sha256=hashlib.sha256(model['robot_description'].encode()).hexdigest(),
                   semantic_sha256=hashlib.sha256(model['robot_description_semantic'].encode()).hexdigest())
    (folder/'selection.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}))
    (folder/'execution_parameters.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': {**model, **settings}}}))
    (folder/'robot.urdf').write_text(model['robot_description'])
    (folder/'robot.srdf').write_text(model['robot_description_semantic'])
    config = folder/'config'
    config.mkdir()
    for name in ('a3_grid.yaml', 'stack_blocks.yaml', 'gazebo_physics.yaml', 'gazebo_controllers.yaml'):
        shutil.copy2(Path(share)/'config'/name, config/name)
    executable = Path(share).parents[1]/'lib/ur5e_a3_grid/stack_blocks'
    if executable.is_file():
        context['planner_executable_sha256'] = sha256(executable)
    context['configuration_sha256'] = {p.name: sha256(p) for p in config.iterdir()}
    (folder/'context.json').write_text(json.dumps(context, indent=2)+'\n')
    return folder


def checked_artifact(folder, name):
    if not isinstance(name, str) or Path(name).is_absolute() or '..' in Path(name).parts:
        raise ValueError('Invalid archived artifact path')
    path = folder/name
    if path.is_symlink() or not path.resolve().is_relative_to(folder.resolve()) or not path.is_file() or path.stat().st_size == 0:
        raise ValueError('Missing or unsafe archived artifact: '+name)
    return path


def seal_successful_archive(folder, logfile=None):
    """Package only a fully completed run with every dispatched command acknowledged."""
    folder = Path(folder)
    context = json.loads((folder/'context.json').read_text())
    result = json.loads((folder/'execution_result.json').read_text())
    recipe = json.loads((folder/'plan.recipe.json').read_text())
    if (context.get('backend') != 'gazebo' or result.get('status') != 'succeeded'
            or result.get('target_cell') != context.get('target_cell')
            or recipe.get('target_cell') != context.get('target_cell')
            or result.get('completed_moves') != recipe.get('moves')
            or result.get('layers') != recipe.get('layers')):
        raise ValueError('An incomplete or mismatched run cannot be marked as a successful path')
    batch_check = folder/'batch_verification.json'
    if batch_check.exists():
        check = json.loads(batch_check.read_text())
        if (check.get('status') != 'passed' or check.get('obstacle_cells') != context['obstacle_cells']
                or check.get('target_cell') != context['target_cell'] or check.get('layers') != recipe['layers']):
            raise ValueError('The automatic trial has not passed final whole-tower verification')
    if sha256(folder/'robot.urdf') != context['model_sha256'] or sha256(folder/'robot.srdf') != context['semantic_sha256']:
        raise ValueError('Archived model bytes changed during execution')
    for name, expected in context['configuration_sha256'].items():
        if sha256(checked_artifact(folder, 'config/'+name)) != expected:
            raise ValueError('Archived configuration changed during execution')
    bundle = folder/'successful_path.zip'
    if bundle.exists():
        manifest = json.loads((folder/'archive.json').read_text())
        if bundle.is_symlink() or any(sha256(checked_artifact(folder, name)) != expected for name, expected in manifest['sha256'].items()):
            raise ValueError('An existing successful archive changed; refusing to overwrite it')
        return bundle
    motions = [step['index'] for step in recipe['steps'] if step['kind'] != 'scene']
    journal = [json.loads(line) for line in (folder/'commands.jsonl').read_text().splitlines()]
    sent = [item for item in journal if item['event'] == 'dispatched']
    done = [item for item in journal if item['event'] == 'completed']
    if ([item['index'] for item in sent] != motions or [item['index'] for item in done] != motions
            or len(journal) != 2*len(motions) or result.get('completed_commands') != len(motions)):
        raise ValueError('The successful path is missing an executed command or completion record')
    for position, index in enumerate(motions):
        if [(x['event'], x['index']) for x in journal[position*2:position*2+2]] != [('dispatched', index), ('completed', index)]:
            raise ValueError('Executed command records are out of order')
    names = ['preview.cdr', recipe['initial_scene_file'], recipe['initial_state_file']]
    names += [step[key] for step in recipe['steps'] for key in ('file', 'scene_before_file', 'scene_after_file')]
    names += [item[key] for item in sent for key in ('file', 'start_state_file')]
    names += [item['end_state_file'] for item in done]
    for name in names:
        checked_artifact(folder, name)
    if logfile is not None:
        text = Path(logfile).read_text(errors='replace')
        if 'STACK_SUCCESS ' not in text or 'STACK_ERROR ' in text:
            raise ValueError('The execution log does not confirm a clean completed run')
        shutil.copy2(logfile, folder/'execution.log')
    (folder/'README.md').write_text(
        f"# Gazebo 성공 경로 — 장애물 {context['obstacle_cells']} / 목표 {context['target_cell']}\n\n"
        f"{recipe['layers']}층, 최종 {recipe['moves']}개 적재, 실제 전송 명령 {len(motions)}개를 기록했습니다.\n\n"
        "`plan.recipe.json`은 단계별 계획과 장면 변화, `commands.jsonl`은 실제 전송 순서입니다.\n"
        "`command_*.cdr`은 전송한 궤적이며 시작·정착 후 관절 상태도 함께 저장됩니다.\n"
        "`robot.urdf`, `robot.srdf`, `selection.yaml`, `config/`는 당시 모델과 배치입니다.\n\n"
        "이 파일은 Gazebo 실행 기록입니다. 실물 경로와 구분해 사용합니다.\n"
        "현재 GUI의 계산 없는 파일 재생 기능은 아직 구현되지 않았습니다.\n"
        "재생기에서는 시작 자세·모델·배치·제어기를 확인하고 기존 승인·충돌·중지 검사를 유지해야 합니다.\n")
    files = sorted(p for p in folder.rglob('*') if p.is_file() and p.name not in ('successful_path.zip', 'archive.json') and not p.name.endswith('.tmp'))
    if any(p.is_symlink() for p in files):
        raise ValueError('Archive symlinks are not supported')
    manifest = dict(schema=1, archive_kind=context['archive_kind'], backend='gazebo', status='succeeded',
                    plan_id=result['plan_id'], obstacle_cells=context['obstacle_cells'], target_cell=context['target_cell'],
                    layers=recipe['layers'], completed_moves=recipe['moves'], command_count=len(motions),
                    replay_implemented=False, sha256={str(p.relative_to(folder)): sha256(p) for p in files})
    (folder/'archive.json').write_text(json.dumps(manifest, indent=2)+'\n')
    temporary = folder/'successful_path.zip.tmp'
    with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zipped:
        for path in files+[folder/'archive.json']:
            zipped.write(path, str(path.relative_to(folder)))
    temporary.replace(bundle)
    return bundle


def archive_from_log(logfile):
    """Called by the GUI output worker after the backend closes its log stream."""
    text = Path(logfile).read_text(errors='replace')
    if 'STACK_SUCCESS ' not in text or 'STACK_ERROR ' in text:
        return None
    records = [json.loads(line.split('STACK_EXECUTION_ARCHIVE ', 1)[1]) for line in text.splitlines()
               if 'STACK_EXECUTION_ARCHIVE ' in line]
    if not records:
        return None
    return seal_successful_archive(records[-1]['directory'], logfile)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Package a completed Gazebo execution without moving a robot')
    parser.add_argument('directory')
    parser.add_argument('--log')
    args = parser.parse_args()
    print(seal_successful_archive(args.directory, args.log))
