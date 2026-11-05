# Git으로 팀원에게 공유하기

[저장소 안내](../README.md)

공유 대상은 **https://github.com/Nanju03/ur5e_team_ws** 비공개 저장소입니다.
Git 저장소 위치는 `~/ur5e_team_ws/src/ur5e`이고, 작업공간 루트는 실행 파일과 빌드 결과를 보관합니다.

## 최초 업로드

검증한 코드를 커밋한 뒤 GitHub CLI로 Nanju03 계정에 로그인합니다.
GitHub 연결 앱의 로그인은 이 PC의 Git/CLI 로그인과 별도입니다.
연결 도구에는 새 저장소 생성 기능이 없어 최초 생성·업로드는 CLI를 사용합니다.

```bash
sudo apt install gh
gh auth login --hostname github.com --web
cd ~/ur5e_team_ws/src/ur5e
./scripts/publish_private_repo.sh
```

도우미는 로그인 계정과 비공개 여부를 확인하고 저장소가 없으면 비공개로 만듭니다.
기존 원본 `gamsaon/ur5e` remote는 `upstream`으로 보관하고 새 공유 저장소를 `origin`으로 연결합니다.
검토·커밋된 HEAD를 main으로 올리고 원격 SHA가 같은지 확인합니다. 강제 push는 하지 않습니다.
공개 저장소이거나 다른 계정이면 업로드를 거절합니다.

## 팀원 접근 권한

GitHub 저장소 **Settings → Collaborators**에서 팀원의 GitHub 계정을 초대합니다.
팀원은 초대를 수락한 뒤 [설치 안내](other_laptop_setup.md)를 따릅니다.
이 작업에서 팀원 초대나 이메일·메시지 전송은 자동 실행하지 않습니다.

## 평소 코드 공유

```bash
cd ~/ur5e_team_ws/src/ur5e
git switch -c feature/작업이름
# 코드와 문서 수정, 검사
git add 수정한_파일
git commit -m "변경 내용"
git push -u origin HEAD
```

GitHub에서 main으로 Pull Request를 열어 팀원 검토 후 병합합니다.
다른 팀원은 main에서 `git pull --ff-only`하고 설치 스크립트와 빌드를 갱신합니다.
`workspace/`의 실행 스크립트·문서도 바뀌었다면 `python3 scripts/install_workspace.py --overwrite`를 실행합니다.

## 공유 범위

새 로그·저장 경로·Gazebo 상태 녹화·보정 시험 결과·로그인 정보·빌드 산출물은 올리지 않습니다.
코드에 필요한 팀 로봇 기구학 보정은 이미 원본에서 추적 중인 YAML을 사용합니다.
성공 궤적 ZIP은 대용량 실행 자료이므로 Git 소스와 별도로 전달합니다.
Git bundle은 비공개 원격에 접근하지 않아도 소스·커밋을 전달할 수 있는 로컬 대안입니다.

```bash
git clone /전달받은/ur5e_team_ws.bundle ~/ur5e_team_ws/src/ur5e
cd ~/ur5e_team_ws/src/ur5e
python3 scripts/install_workspace.py
```
