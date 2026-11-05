# 실물 경로 계획·이동 최적화 (2026-11-05)

[실물 사용법](real_robot.md) · [기존 시간 분석](speed_and_timing.md) · [저장 경로 빠른 실행](saved_physical_replay.md) · [v15 높이·하강 속도](side_hover_and_descent.md)

적용 대상은 실물 집기·쌓기의 `physical_plan/physical_run`이다.
Gazebo 물성·속도와 블록 없는 팔 시운전·초기 자세 복구의 저속 설정은 별도다.

## ④가 오래 걸렸던 이유

기존 실물 계획 로그에는 **450개 단계·107,517개 충돌 표본**이 있었다.
경로 생성 후 Python에서 각 표본마다 `/check_state_validity` 서비스를 호출했다.
보간 표본 생성, Python/ROS 직렬화·서비스 응답, 장면 적용과 형상 검사에 비용이 든다.
CPU가 빨라지면 일부는 단축되지만, 코어를 늘리는 것만으로 반복 통신 비용이 사라지지는 않는다.
Gazebo·RViz를 함께 실행하면 자원도 공유한다.

새 계획 검증은 **지속 실행하는 로컬 C++ MoveIt 검사기**에 표본을 묶어 전달한다.
기존의 0.05초·0.01rad 기준 quintic 보간 표본과 운반 손목 5개 형상 검사는 유지한다.
전체 장면의 world 물체, 부착 블록, 충돌 허용 행렬, 링크 padding/scale을 반영한다.
검사 실패·시간 초과·중지·비정상 응답은 승인 가능한 계획으로 저장하지 않는다.
손가락의 허용 범위도 저장할 때 검사한다. 실행에서는 충돌 표본을 다시 검사하지 않고 저장 결과를 재사용한다. 현재 장면·센서·시작 자세 확인과 승인 절차는 유지한다.

실행 중 장면 확인도 반복되는 메쉬의 정규화 계산을 재사용한다. 매번 현재 메쉬의 전체 바이트로
캐시를 조회하므로 꼭짓점·삼각형이 바뀌면 다시 계산한다. 위치·충돌 허용 행렬도 계속 비교한다.
52KB 장면 표본에서 기존 해시 0.31~0.33초가 캐시 적용 후 약 0.06초로 줄었다.
이는 장면 확인 한 번의 실측이며 전체 계획 시간의 개선 비율을 뜻하지 않는다.
장면 해시 정책은 `physical_mesh_digest_pose_round12_v2`로 바뀌어 이전 정책의 계획은 다시 만든다.

```text
STACK_PLAN_TIMING stage=search seconds=...
STACK_PLAN_TIMING stage=collision_validation backend=local_cpp seconds=... samples=...
```

첫 항목에는 초기 준비와 경로 생성이 포함된다. 두 번째는 표본 생성·장면 처리·로컬 충돌 검증이다.
OMPL/IK 재시도에는 `STACK_APPROACH_RETRY`, `STACK_REARRANGEMENT_RETRY`를 함께 확인한다.

## 불필요한 우회 줄이기

- 매 블록을 놓은 뒤 중앙 대기 위치로 돌아오던 실물 동작을 생략한다.
  다음 집기 접근이 실패할 때만 대기 위치를 경유한다. 시작·마지막 복귀는 유지한다.
- 현재 손목 방향과 가까운 정육면체 집기 방향부터 검사한다.
- Cartesian 직접 연결이 실패하면 관절 공간의 직접 연결을 먼저 검사한다.
  모든 중간 표본의 충돌·관절 한계·운반 자세와 탑 접근 여유 조건을 만족해야 사용한다.
  실패하면 기존 staging·OMPL 탐색을 사용한다.
- 탑 위 120mm 여유, 19/20층 전환, 열린 그리퍼 진입과 임시 적재·회수 순서는 유지한다.

눈에 보이는 블록 외에 팔꿈치·손목·그리퍼 뒤쪽과 운반 블록도 검사한다.
모든 우회를 제거하거나 전역 최단 경로를 보장하는 방식은 아니다.

## 실물 속도·가속도

| 구간 | phase | 속도 배율 | 가속도 배율 |
| --- | --- | --- | --- |
| 최종 접촉 구간 밖 수직 하강 | `descent_fast` | 0.20 | 0.20 |
| 집기·놓기 마지막 40mm 이상, 수평 삽입·짧은 내려놓기 | `approach` | 0.05 | 0.05 |
| 집은 직후·놓은 직후 처음 40mm 이탈 | `contact` | 0.05 | 0.05 |
| 이후 상승·후퇴, 운반·재정렬 | `lift` / `retreat` / `transfer` | 1.00 | 1.00 |

하강·이탈 거리가 40mm 이하면 해당 구간 전체를 저속으로 처리한다. 20층부터는 받침보다 10mm 높게 수평 정렬한 뒤 수직으로 내려놓는다.
그리퍼 0.40 → 잡기 0.73, 놓기 0.70 → 0.60은 기존 값이다.
손가락 동작 중 팔 정지와 각 명령 시작·끝 정지 조건도 유지한다.

**1.00은 설정된 관절 한도의 100%이며 무제한 속도나 물리적 최고 성능을 뜻하지 않는다.**
MoveIt 시간화와 실제 제어기 quintic의 최대 속도·가속도를 확인하고 필요하면 시간을 늘린다.
현재 실물 모델의 가속도 기준은 관절별 1rad/s²이므로 빠른 이동의 한도는 1rad/s²이다.
URDF·MoveIt 한도와 펜던트 속도 슬라이더·로봇 측 제한도 적용된다.
[MoveIt 시간화 설명](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html),
[UR 제어기 설명](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html).

새 고속 프로필의 실물 파지 안정성과 23층 완료는 별도 현장 검증 대상이다.
첫 실물 검증은 펜던트 실행 배율을 낮춰 소수 블록부터 확인한 뒤 확대한다.
GUI의 **팔 시운전 속도 (최대 0.05)**는 블록 없는 시운전용이며 이 프로필을 바꾸지 않는다.

구현 위치 (`~/ur5e_team_ws/src/ur5e/` 기준):

| 파일 | 역할 |
| --- | --- |
| `ur5e_block_bringup/launch/block_task.launch.py` | 접근 0.05·운반 1.00, 이탈 40mm |
| `ur5e_block_bringup/ur5e_block_bringup/physical_stack.py` | `ARM_SPEEDS`, `CONTACT_CLEARANCE`, 시간화·시작 상태 검증 |
| `ur5e_a3_grid/src/stack_blocks.cpp` | 저속 이탈·고속 이동 분리, 우회 경로 생성 |
| `ur5e_block_bringup/ur5e_block_bringup/collision_worker.py` | 로컬 검사 IPC·중지·타임아웃 |
| `ur5e_a3_grid/src/physical_collision_checker.cpp` | 오프라인 MoveIt 형상 검사 |

배율을 바꾸면 계획기·검증기·GUI를 같은 프로필로 맞추고 `RECIPE_POLICY`를 변경한다.
공통 YAML만 수정하면 실물 런치의 값이 우선한다.

## 적용

빌드 후 기존 GUI를 닫고 다시 연다. 이미 실행 중인 동작을 새 코드로 바꾸거나
이전 검토 파일을 고속 재생하지 않는다. 배치와 빈 그리퍼를 확인하고 ④에서 새로 계획한다.

```bash
cd ~/ur5e_team_ws
./start_block_real_gui.sh
```

현재 정책은 `physical_nearest_target_buffer_v16`이다. 목표 가까운 임시 받침을 우선 선택하며 v15의 높이·속도는 유지한다. 기존 v13/v14/v15 자료는 보존하고 새 기준은 ④에서 다시 계획한다. [임시 적재 기준](near_target_buffer.md)을 확인한다.
드라이버를 고속 코드 적용만을 위해 재연결할 필요는 없다.

`GRIPPER_SENSOR_STALE`로 중단된 경우에는 [센서 지연 중단 안내](gripper_feedback_stop.md)를 먼저 따른다.
기존 Gazebo 기록은 보존되지만 실행 파일 변경 시 자동 캠페인 지문이 달라진다.
변경된 빌드에서 새 자동 시험을 시작할 때는 새 `--output-dir`를 사용한다.

## 미리 저장한 경로로 실행하기

구현되어 있다. ④는 배치·모델·보정·속도 정책과 초기 장면·시작 자세가 일치하는
`physical_path_library` 자료를 먼저 찾는다. 찾으면 탐색·시간화·충돌 재검사를 생략하고
새 검토 세션을 만든다. 없으면 새 경로를 계획·사전 검사해 자동 보관한다.
⑤는 저장된 궤적·그리퍼 명령·장면 이벤트를 사용하며 경로 표본을 재검사하지 않는다.

장기 보관 자료와 1시간 유효한 실행 검토 세션을 구분한다. 실행 때에는 파일 무결성,
현재 배치·장면·제어기·센서·시작 자세와 사전 검사한 손가락 범위를 확인한다.
구간 사이 클라이언트 연결, 최근 감시 결과와 정지 이력을 재사용해 중복 대기를 줄인다.
시작 자세가 다르면 자동 연결하지 않는다. Gazebo 성공 ZIP의 실물 재생은 지원하지 않는다.

사용 절차, 이전 형식의 처리과 저장 위치는 [저장 경로 빠른 실행](saved_physical_replay.md)에 있다.
