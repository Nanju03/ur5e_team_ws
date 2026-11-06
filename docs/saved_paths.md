# 성공 경로 저장

[Gazebo 사용법](gazebo.md) · [전체 안내](../README.md)

**현재 작업:** [케이스별 병렬 계획·Gazebo 적층 시험](gazebo_diagonal_tests.md#권장-케이스별-병렬-실행)은
`./test_block_diagonals.sh --jobs 4`로 실행합니다. 서로 다른 네 배치를 맡고 완료한 작업자는 다음 배치를 가져갑니다.
[1→6 한 배치만 시험](gazebo_single_case.md)하는 기능도 유지합니다.
2026-11-06 사용자 요청으로 이전 이웃 대각선 경로 ZIP과 미완료 경로는 삭제했습니다.
아래의 첫 성공 증빙 로그는 보존했으며, 새 성공 ZIP은 다음 전체 시험 완료 후 생성됩니다.

실행 로그·궤적과 첫 성공 증빙은 각 개발 PC의 로컬 자료이며 Git 소스에는 포함하지 않습니다. 아래 경로는 실행한 PC에 생성됩니다.

현재 Gazebo GUI에서 ⑤ 실행을 시작하면 해당 배치의 경로 기록 폴더를 자동 생성합니다.
경로 생성 후 원본 계획을 저장하고, 실행 중 실제 전송한 팔·그리퍼 명령과 시작·정착 후 관절 상태를 저장합니다.
마지막 복귀 구간까지 성공한 실행만 파일 목록·SHA-256을 확인해 **`successful_path.zip`**으로 묶습니다.
실행 중 중단되거나 기록이 빠지면 성공 ZIP을 만들지 않습니다.

여러 배치를 연속 시험하려면 **[이웃 대각선 60가지 자동 시험](gazebo_diagonal_tests.md)**을 사용합니다.
자동 시험도 같은 성공 경로 폴더에 저장하며, 실패한 배치는 별도 보고서에 정리합니다.

## 장애물 11번·목표 14번

저장 위치는 다음과 같습니다.

```text
~/ur5e_team_ws/saved_paths/gazebo/obstacle_11_target_14/
  20261104_205109_original_success/   첫 성공의 증빙: 로그·배치, 관절 궤적 없음
  <재실행 날짜와 고유 번호>/          다음 실행부터 생성
    successful_path.zip              실행 성공 후 생성
    archive.json                     성공 여부·파일 SHA-256
    plan.recipe.json                 계획 궤적·장면 변화·단계 순서
    preview.cdr                      전체 미리보기
    commands.jsonl                   실제 전송 순서·완료 기록·액션 설정
    command_*.cdr                    전송한 RobotTrajectory
    command_*_start_state.cdr         전송 직전 상태
    command_*_end_state.cdr           완료·정착 후 상태
    robot.urdf / robot.srdf           당시 로봇 모델
    selection.yaml                   적용 배치·동작 설정
    execution_parameters.yaml        모델을 포함한 적용 매개변수
    execution.log / config/          실행 로그·물성·제어기 설정
```

각 재실행은 별도 폴더를 사용하므로 이전 성공 파일을 덮어쓰지 않습니다.
다른 배치는 예를 들어 `obstacle_15_target_10/`처럼 구분합니다.
여러 장애물을 선택하면 `obstacles_1_11_target_14/`처럼 정렬한 번호를 사용합니다.

**첫 성공의 원본 관절 궤적은 당시 코드가 파일로 저장하지 않았습니다.**
`20261104_205109_run.log`에서 23층·최종 22개·총 운반 23회와 DONE을 확인했지만,
로그의 구간 이름·완료 위치만으로 모든 관절의 시각별 궤적을 정확하게 복원할 수는 없습니다.
첫 성공 폴더의 `success_evidence.json`에는 `trajectory_present: false`, `replay_ready: false`로 표시했습니다.
사용자가 같은 배치를 재실행해 새 성공 궤적을 자동 기록하기로 했으며, 별도 재실행은 시작하지 않았습니다.

## 지금 다시 기록하기

1. 기존 Gazebo 시작 터미널에서 Ctrl+C로 종료합니다.
2. 새 코드·GUI로 다시 시작합니다.

   ```bash
   cd ~/ur5e_team_ws
   ./start_block_gazebo.sh
   ```

3. 장애물 11번·목표 14번을 선택하고 **③ 배치 초기화 → ④ 경로 확인 → ⑤ 가상 실행 → 상단 OK**로 진행합니다.
4. 최종 완료 후 로그에 **`STACK_PATH_ARCHIVED`**, 상태창에 **성공 경로 저장: …/successful_path.zip**이 표시됩니다.
   ZIP 생성이 끝날 때까지 GUI를 켜 둡니다.
5. 오류가 나면 **로그 전체 복사**를 사용합니다. 정리되지 않은 기록 폴더는 성공 경로로 취급하지 않습니다.

수동 CLI 실행도 같은 기록 폴더를 생성하며, 기본 launch의 ZIP 포장은 GUI 출력 작업이 수행합니다.
대각선 자동 시험 실행기는 GUI 없이 ZIP까지 포장합니다.
일반 CLI로 끝낸 성공 폴더는 다음과 같이 파일 검사와 ZIP 포장만 할 수 있습니다. 로봇은 움직이지 않습니다.

```bash
source ~/ur5e_team_ws/env.bash
python3 -m ur5e_block_bringup.path_archive /절대경로/실행기록폴더 --log /절대경로/실행로그.log
```

## 계산 없는 재생과의 관계

**이번 구현은 경로 기록·성공 확인·보관 기능입니다. 현재 GUI에 저장 ZIP을 바로 재생하는 기능은 없습니다.**
지금의 ⑤ 실행은 기존처럼 계획합니다. 이후 재생기는 이 파일의 전송 명령·장면 순서를 읽어
IK·OMPL 경로 생성을 생략하도록 구현할 수 있습니다.
시작 자세, 모델·보정, 블록 좌표·크기, 장애물, 속도·가속도와 제어기 조건은 저장 경로와 맞아야 하며,
현재 상태·충돌·명령 완료·중지 검사는 재생기에도 필요합니다.
Gazebo 기록과 실물 기록은 다른 모델·그리퍼·물성을 사용하므로 별도로 관리합니다.
