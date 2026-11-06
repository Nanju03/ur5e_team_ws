# Gazebo 장애물 1번·목표 6번만 다시 시험하기

[Gazebo 사용법](gazebo.md) · [성공 경로 형식](saved_paths.md) · [60가지 시험](gazebo_diagonal_tests.md)

서로 다른 여러 배치를 동시에 처리하려면 [케이스별 병렬 실행](gazebo_diagonal_tests.md#권장-케이스별-병렬-실행)의
`--jobs 4`를 사용합니다. 이 문서의 `--workers 18`은 한 케이스 내부의 OMPL 후보 수입니다.

이 명령은 Gazebo에서 배치를 새로 만들고, 현재 알고리즘으로 23층 전체 경로를 계획한 뒤
가상 로봇으로 끝까지 쌓습니다. 마지막 탑의 23개 블록 위치와 저장 파일까지 검사한 경우만
성공 경로 ZIP을 만듭니다. 실물 드라이버·LAN·USB는 사용하지 않습니다.

## 실행

기존 Gazebo/자동 시험 터미널은 Ctrl+C로 종료합니다. 다음 명령 하나로 진행하며,
별도 GUI의 ③·④·⑤·OK를 누를 필요는 없습니다. 이 PC에는 수정 코드 빌드까지 적용했습니다.

```bash
cd ~/ur5e_team_ws
./test_block_diagonals.sh --obstacle 1 --target 6 --workers 18 --physics-threads 1
```

화면을 열지 않는 실행이 가장 가볍습니다. 진행 모습을 보려면 같은 명령 끝에
`--show-gazebo`를 추가합니다. RViz는 `--show-rviz`로 선택합니다.

목록만 확인하는 명령은 시뮬레이터를 시작하거나 파일을 만들지 않습니다.

```bash
./test_block_diagonals.sh --obstacle 1 --target 6 --list
```

장애물 1번·목표 6번 **한 배치만** 등록됩니다. `--max-cases 1`만 지정하면 기존 60가지
목록의 첫 배치인 11→14가 선택되므로 위의 `--obstacle/--target`을 사용해야 합니다.

## CPU와 속도

- `--workers 18`: OMPL이 필요한 연결에서 최대 18개 후보를 병렬 탐색합니다.
  첫 정확한 해를 얻으면 다른 후보를 중단하고, 경로 단순화·충돌·관절 한계 검사를 유지합니다.
  Cartesian/IK 연결과 블록 이동 순서는 앞 상태에 의존하므로 순서대로 처리합니다.
- `--workers 0` 또는 생략: 사용 가능한 논리 CPU 수에서 2를 뺀 값입니다. 이 PC는 20개이므로 18입니다.
  CPU를 특정 코어에 고정하거나 점유율을 보장하는 설정은 아닙니다.
- `--physics-threads 1` (기본값): Gazebo 물리를 직렬로 계산합니다. ODE의 island thread pool은 0으로 설정합니다. 로봇·탑처럼 연결된 접촉계에서는
  스레드가 많아질 때 오히려 동기화 비용이 커질 수 있어 경로 탐색 스레드와 별도로 설정합니다.
- 배치 전용 `parallel.world`는 ODE의 island/position-correction 스레드 수만
  조정합니다. 1ms 물리 간격, solver 100회, 질량·마찰·접촉·제어기·동작 속도 설정을 유지합니다.
  실시간 실행 속도 상한과 정착 기준 0.005rad, 최종 위치 검사도 유지합니다.

병렬 설정을 늘려도 Gazebo의 실제 동작 시간과 정착 대기는 남습니다.
20개 CPU가 계속 모두 사용되거나 전체 시험이 18배 빨라지는 것은 아닙니다.
실제 물리 병렬화는 `--physics-threads 2`, `4`, `18`처럼 따로 비교할 수 있습니다.
`--physics-threads 0`은 경로 탐색의 `--workers`와 같은 수를 사용합니다.
스레드 설정을 바꾸면 같은 시험 폴더에 섞지 않도록 아래처럼 다른 결과 폴더를 사용합니다.

```bash
./test_block_diagonals.sh --obstacle 1 --target 6 --workers 18 --physics-threads 4 \
  --output-dir ~/ur5e_team_ws/log/gazebo_1_6_threads4
```

구현 근거: [MoveIt Humble의 병렬 OMPL](https://github.com/moveit/moveit2/blob/humble/moveit_planners/ompl/ompl_interface/src/model_based_planning_context.cpp),
[Gazebo ODE 스레드 설정](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEPhysics.cc).

## 결과·중단·재시도

```text
~/ur5e_team_ws/log/gazebo_1_6_parallel/
  report.html                  한 배치의 진행표
  failed_cases.md               실패 원인 (중단은 실패로 세지 않음)
  state.json                   재개 상태와 시도 이력
  parallel.world               실제 사용한 물리 실행 설정
  cases/obstacle_1_target_6/attempt_001/
    simulator.log / scene.log / run.log / result.json

~/ur5e_team_ws/saved_paths/gazebo/obstacle_1_target_6/<고유 실행 번호>/
  successful_path.zip           전체 적층·탑 검증·파일 검증 성공 후에만 생성
```

ZIP에는 사용한 `parallel.world`와 `parallelism.json`도 함께 들어갑니다.
시험 도중 Ctrl+C로 중단하면 자기 Gazebo 프로세스만 정리합니다. 같은 명령으로 다시 실행하면
중단 배치를 초기 배치부터 다시 시험합니다. 이미 성공했거나 실패로 판정한 시도는 건너뜁니다.
실패 판정 후 다시 시험하려면 기존 명령에 `--retry-failed`를 추가합니다.

새 알고리즘이나 스레드 수로 변경하면 새 `--output-dir`를 지정해야 합니다.
Gazebo 성공 ZIP은 가상 모델의 기록이며 실물 재생 라이브러리에 자동 등록되지 않습니다.

## 기존 경로 정리 기록 (2026-11-06)

사용자 요청에 따라 기존 이웃 대각선 60개 배치 중 실제 생성돼 있던 **29개 배치의 실행 폴더 30개**를
삭제했습니다. 성공 ZIP 16개와 미완료 경로를 포함해 1,691,656,835바이트입니다.
기존 `log/gazebo_diagonal_tests`의 성공·실패 보고서와 로그는 역사 기록으로 보존했습니다.
그 보고서가 참조하는 경로 ZIP은 삭제됐으므로 재개에 사용하지 않습니다.
관절 궤적이 없던 `20261104_205109_original_success`의 첫 성공 증빙은 보존했습니다.
실물 저장 라이브러리와 이전 Gazebo 상태 녹화 JSONL도 이번 정리 대상에 포함하지 않았습니다.

삭제 목록은 `~/ur5e_team_ws/log/gazebo_path_reset_20261106/deletion.json`입니다.

## 적용 후 확인

- 두 패키지를 Release로 빌드했습니다. 최종 회귀 테스트는 1,430개가 통과했습니다.
- 오프라인 OMPL 검사에서 후보 수 18을 확인했고 프로세스 스레드는 12개에서 최대 31개로 증가했습니다
  (기존 12개 + 관측 1개 + 탐색 18개). 반환 경로도 충돌 검사를 통과했습니다.
- 1→6의 23층 전체 경로 445개 구간을 생성하고 Gazebo 동작 시작·중단 정리를 확인했습니다.
  최종 직렬 물리 설정의 pipeline 초기화 이후 경로 생성은 해당 시험에서 약 11.1초였습니다.
- 짧은 정착 구간의 시뮬레이션/실제 시간 비율은 물리 18스레드에서 약 0.10,
  직렬 설정에서 약 0.17이었습니다. 이 값은 전체 적층 시간의 개선 보장이 아닙니다.
- 속도 상한을 없애는 개발 중 설정은 빈 world에서 모델 생성 시간 초과가 발생해 제외했습니다.
  최종 설정은 원래의 1kHz 실행 속도 상한을 유지합니다.

확인용 시험은 의도적으로 중단했습니다. **이번 코드의 전체 23층 물리 적층 성공이나 새 성공 ZIP은
아직 없습니다.** 위 실행 명령으로 전체 시험을 시작하면 성공한 경우 새 경로가 자동 저장됩니다.
진단 기록은 `~/ur5e_team_ws/log/gazebo_single_parallel_validation_20261106/`에 있습니다.
