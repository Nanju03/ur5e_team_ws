# UR5e 팀 작업공간

Ubuntu 22.04 / ROS 2 Humble의 **UR5e + DH AG95 블록 적재 프로젝트**입니다.
실물 제어, Gazebo 접촉 시뮬레이션, RViz·GenericSystem, 배치 GUI, 성공 궤적 기록과 이웃 대각선 60가지 자동 시험을 포함합니다.

팀 공유 저장소: **[Nanju03/ur5e_team_ws](https://github.com/Nanju03/ur5e_team_ws) (공개)**.
원본은 [gamsaon/ur5e](https://github.com/gamsaon/ur5e), 기준 커밋은 `473c8328641719649939720dfcc15e8356d7fc82`입니다.
원본 UR·DH·serial 소스와 라이선스는 각 디렉터리에 보존합니다. Git 서브모듈이나 다른 개인 작업공간은 필요하지 않습니다.

## 팀원 설치

공개 저장소이므로 초대나 로그인 없이 소스를 받을 수 있습니다. 변경 사항을 올리려면 GitHub 인증과 쓰기 권한이 필요합니다.

```bash
mkdir -p ~/ur5e_team_ws/src
git clone https://github.com/Nanju03/ur5e_team_ws.git ~/ur5e_team_ws/src/ur5e
cd ~/ur5e_team_ws/src/ur5e
python3 scripts/install_workspace.py
```

이 명령은 실행 스크립트·공통 환경·문서·실물 설정을 `~/ur5e_team_ws`에 설치합니다.
기존 파일을 조용히 덮어쓰지 않으며 ROS·Gazebo·로봇을 시작하지 않습니다.
의존성 설치·빌드는 **[팀원 PC 설치 절차](docs/other_laptop_setup.md)**를 따릅니다.
ROS 2 Humble·apt MoveIt를 사용하며, 개발 PC의 MoveIt 2.5.10으로 검증했습니다.

```bash
cd ~/ur5e_team_ws
./check_dependencies.sh
./build.sh
```

## 실행 선택

| 환경 | 명령 | ROS domain | 안내 |
| --- | --- | --- | --- |
| Gazebo 물리 | `./start_block_gazebo.sh` | 44 | [집기·접촉·적재](docs/gazebo.md) |
| RViz·GenericSystem | `./start_block_sim.sh` | 42 | [설치 확인](docs/other_laptop_setup.md) |
| 실물 드라이버 | `./start_block_robot.sh allow_execution:=true use_rviz:=false` | 0 | [실물 사용법](docs/real_robot.md) |
| 실물 RViz | 별도 터미널에서 `./start_block_view.sh` | 0 | 같은 실물 드라이버에 연결 |
| 실물 GUI | 별도 터미널에서 `./start_block_real_gui.sh` | 0 | 경로 계획·승인·적재 |
| 대각선 60가지 시험 | `./test_block_diagonals.sh` | 44 | [초기화·연속 시험·실패 목록](docs/gazebo_diagonal_tests.md) |

명령은 설치 후 작업공간 루트에서 실행합니다. 화면 없는 대각선 시험도 실제 Gazebo 물리를 사용합니다.
`--show-gazebo`로 화면을 표시하고, 중단 후 같은 명령으로 미완료 배치를 이어서 시험합니다.
실패 배치만 재시험하려면 `--retry-failed`를 사용합니다.

현재 기본 적층은 **19층까지 위에서 내려놓기, 20층부터 측면 삽입**입니다.
탑 위 이동은 그리퍼·운반 블록의 최저점을 기준으로 **120mm** 여유를 확보합니다.
그리퍼는 열린 상태 0.40rad로 진입하며, 필요하면 다른 블록 위 임시 적재·회수로 접근 공간을 만듭니다.
잡기·놓기는 실물과 Gazebo의 설정을 구분합니다.

## 경로와 결과

Gazebo의 원본 계획과 실제 전송한 팔·그리퍼 궤적을 배치별로 기록합니다.
전체 완료와 기록 검사를 통과한 실행만 `successful_path.zip`으로 보관합니다.
[저장 형식과 범위](docs/saved_paths.md), [실물 속도·가속도 설정](docs/speed_and_timing.md)을 참고합니다.
**저장 파일을 계산 없이 재생하는 기능은 아직 구현되지 않았습니다.**

```text
~/ur5e_team_ws/
  src/ur5e/                이 Git 저장소: 코드·설정·문서·설치 템플릿
  env.bash, build.sh       공통 환경과 빌드
  start_*.sh               환경별 실행
  scripts/, docs/          설치된 실행 보조 파일·문서
  robot_calibration/       팀 실물 UR5e 보정
  build/, install/, log/   PC별 빌드·실행 결과 (공유 제외)
  compat/                  선택 Gazebo bridge (공유 제외)
  saved_paths/gazebo/      배치별 성공 궤적 (소스와 별도 관리)
```

실물 보정은 기존 팀 로봇의 값이며 받침 16mm·어댑터 0mm입니다.
다른 로봇이나 설치 환경에는 해당 보정·치수를 사용하고 새로 계획합니다.
Gazebo 기록을 실물 경로로 사용하지 않습니다.

## 개발·업데이트

Git 명령과 원본 코드·문서 수정은 `~/ur5e_team_ws/src/ur5e`에서 합니다.

```bash
cd ~/ur5e_team_ws/src/ur5e
git pull --ff-only
python3 scripts/install_workspace.py --overwrite
cd ~/ur5e_team_ws
./build.sh
```

`--overwrite`는 바뀐 설치 파일을 `.workspace_backups/`에 보관한 뒤 갱신합니다.
현재 작업공간에서 직접 수정한 루트 스크립트·문서는 대응하는 `workspace/` 원본에도 반영해 커밋합니다.
공유 방법과 GitHub 최초 업로드는 **[Git 공유 안내](docs/team_git.md)**에 정리했습니다.

## 검증 범위

2026-11-07의 [P0 32개 가상 시험 결과](development/validation/p0_rviz_20261107/README.md)와
[실측 손가락 범위 수정 검증](development/validation/finger_envelope_20261107.json)을 보관합니다.
P0 시험은 계획 시간을 포함한 배치당 5분 시험이며, 21개가 5분까지 진행하고 11개가 피드백 검사에서 중단했습니다.
[시험 계획·결과 기록 양식](docs/reports/UR5e_테스트계획_결과보고서_20261105.pdf)도 함께 제공합니다.

개발 PC에서 전체 Python 검사 1,314개, Release 빌드, 짧은 Gazebo 자동 계획·승인·명령 기록·중단을 확인했습니다.
사용자의 11번 장애물·14번 목표 전체 23층 Gazebo 적재 성공 로그를 확인했습니다.
60가지 전체 시험과 실물 전체 적재 성공을 검증한 결과는 아닙니다.
개인 원시 실행 로그·진행 상태·궤적·로그인 정보·빌드 결과는 이 저장소에 새로 포함하지 않습니다.
`development/validation/`에는 과거 개발 기록과 위의 검토한 시험 요약을 보관합니다.

패키지별 상세 안내: [적재/GUI](ur5e_a3_grid/README.md), [실물 검토 실행기](ur5e_block_bringup/README.md).
과거 기본 실습은 [UR·DH 실습 가이드](docs/UR5e_DH_노트북_실습가이드.md)에 보존했습니다.
