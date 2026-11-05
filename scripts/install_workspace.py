#!/usr/bin/env python3
"""Install tracked workspace scripts without starting ROS or replacing local files silently."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import sys
import uuid
from zoneinfo import ZoneInfo


REPOSITORY = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(repository, workspace, overwrite=False, dry_run=False):
    repository, workspace = Path(repository).resolve(), Path(workspace).expanduser().resolve()
    source = repository/'workspace'
    entries = [(p, workspace/p.relative_to(source)) for p in sorted(source.rglob('*'))
               if p.is_file() and p.name != 'COLCON_IGNORE' and '__pycache__' not in p.parts]
    if not entries:
        raise ValueError('workspace/ 설치 파일이 없습니다.')
    # Check every target before creating or replacing any file.
    conflicts, updates = [], []
    for src, dst in entries:
        if src.is_symlink() or dst.is_symlink() or not dst.resolve().is_relative_to(workspace):
            raise ValueError('설치 위치 밖으로 향하는 링크를 허용하지 않습니다: '+str(dst))
        if dst.exists() and not dst.is_file():
            raise ValueError('파일 대신 디렉터리가 있습니다: '+str(dst))
        if dst.exists() and digest(src) == digest(dst):
            continue
        updates.append((src, dst))
        if dst.exists():
            conflicts.append(dst)
    if conflicts and not overwrite and not dry_run:
        raise ValueError('기존 파일과 다릅니다. --dry-run으로 확인하고 --overwrite로 백업 후 갱신하세요:\n'
                         +'\n'.join(str(p) for p in conflicts))
    for _, dst in updates:
        print(('백업 후 갱신: ' if dst in conflicts else '설치: ')+str(dst))
    if dry_run:
        return dict(dry_run=True, updates=len(updates), conflicts=len(conflicts))
    backup = workspace/'.workspace_backups'/(datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    for src, dst in updates:
        dst.parent.mkdir(parents=True, exist_ok=True)
        if dst in conflicts:
            old = backup/dst.relative_to(workspace)
            old.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(dst, old)
        shutil.copy2(src, dst)
    metadata = dict(source=str(repository), files={str(dst.relative_to(workspace)): digest(src) for src, dst in entries})
    marker = workspace/'.workspace-install.json'
    marker.write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    print(f'완료: {len(updates)}개 설치/갱신. ROS·Gazebo·로봇 드라이버는 시작하지 않았습니다.')
    return dict(updates=len(updates), backup=str(backup) if conflicts else None)


def main(argv=None):
    default = REPOSITORY.parent.parent if REPOSITORY.parent.name == 'src' else Path.home()/'ur5e_team_ws'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=default)
    parser.add_argument('--overwrite', action='store_true', help='기존 파일은 백업한 뒤 갱신')
    parser.add_argument('--dry-run', action='store_true', help='변경 내용을 출력만 함')
    args = parser.parse_args(argv)
    try:
        install(REPOSITORY, args.workspace, args.overwrite, args.dry_run)
    except (OSError, ValueError) as error:
        print('설치 오류: '+str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
