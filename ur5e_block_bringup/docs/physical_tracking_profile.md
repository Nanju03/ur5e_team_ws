# 실제 UR5e 궤적 추종 설정

이 문서는 `physical_plan/physical_run`의 관절 추종 검사와 검증 범위를 설명합니다.
관절 오차 단위는 rad이며, TCP 위치 오차의 mm 값과 같지 않습니다.

| 단계 | 이동 중 위치 오차 | 속도·가속도 비율 |
| --- | --- | --- |
| `transfer` | 모든 팔 관절 0.1rad | 1.00 / 1.00 |
| `approach` | 모든 팔 관절 0.1rad | 0.05 / 0.05 |
| `contact` (처음 40mm 이탈) | 모든 팔 관절 0.1rad | 0.05 / 0.05 |
| `lift`, `retreat` (이탈 이후) | 모든 팔 관절 0.1rad | 1.00 / 1.00 |

사용자 요청에 따라 실물 집기·쌓기의 이동 중 중단 기준을 **0.1rad**로 설정합니다.
`transfer`는 공중 이동·재정렬을 뜻하며, 진입·상승·후퇴에도 같은 관절별 기준을 적용합니다.
이 값은 액션 목표의 `path_tolerance`이며, 목표점의 허용 위치 오차와 구분합니다.
초기 자세 복구와 일반 공중·X/Y 맞춤 경로의 이동 중 기준은 **0.005rad**로 유지합니다.

모든 팔 동작의 정지 목표는 관절별 위치 0.001rad, 속도 0.01rad/s이며 목표 도달
추가 대기는 5초입니다. 시작 상태와 다음 단계 연결도 0.001rad로 확인하고, 궤적의
시작·끝 속도와 가속도는 0으로 유지합니다. 이동 중 오차 검사와 정지 목표 검사는
서로 다른 조건입니다. ROS2 제어기의 기본 허용값은 액션 목표에 명시한 값으로
덮어쓰며, 값 0은 명시적인 정밀도 요구가 아니라 기본값 사용을 뜻합니다.
[ROS2 Humble JTC 사용 설명](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html),
[허용 오차 파라미터](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html).

충돌 검사는 중단 기준과 별도입니다. 계획 궤적의 표본과 실제 파지 범위의 그리퍼 형상을
검사하며, 공중 운반 경로에는 기존 손목 각도 편차 −0.010, −0.005, 0, +0.005, +0.010rad의
다섯 형상을 검사합니다. 블록을 잡은 동안에는 검증된 손가락 형상 격자와 교차하여
각 팔 경로 표본의 충돌을 확인합니다. 손가락 범위는 저장할 때 사전 검사하며 실행에서는 재검사 없이 같은 단계·장면·모델·궤적의
결과를 재사용합니다. 실측 파지 유지 구간이 사전 검사 범위 안인지 확인합니다.
v13 경로는 1회 오프라인 준비로 변환하며 더 오래된 정책의 경로는 다시 계획합니다.
추가 손목 검사 범위는 프로필의 `transfer_sweep_limit=0.010`으로 기록하여,
제어기의 `path_tolerance=0.1`과 혼동하지 않습니다.

이 검사는 손목과 손가락의 지정된 형상을 표본 검사하는 방식입니다. 모든 팔 관절의
가능한 오차 조합, 격자 사이의 모든 연속 상태 또는 실제 접촉·파지 성공을 보증하지
않습니다. **팔 6관절의 ±0.1rad 오차 조합 전체를 충돌 검증한 것은 아닙니다.**
관절별 중단 기준을 팔 전체의 Cartesian 충돌 여유로 해석하지 않습니다.
현재 AG95 모델의 빈 그리퍼 0.4rad 형상에서 손목 3번만 0.010rad 회전하면
그리퍼 충돌 경계점의 변위 상한은 약 0.72mm입니다. 이는 URDF 회전축과 형상으로
계산한 단일 관절의 기하 상한이며, 다른 팔 관절의 오차나 보정 오차를 포함하지 않습니다.

하드웨어의 `servoj_gain=2000`, `servoj_lookahead_time=0.03s`, 500Hz 주기는
이 추종 검사 변경으로 조정하지 않습니다. URScript 문서는 짧은 lookahead와 높은
gain이 진동을 유발할 수 있다고 설명합니다. lookahead는 속도로 위치를 예측하는
설정이며 측정한 일정한 지연 시간이나 1차계 시정수로 취급하지 않습니다.
[UR servoj 설명](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-scriptmanual/all_scripts/servoj_qavt0-008lookahead_time.htm),
[UR 드라이버 하드웨어 파라미터](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/hardware_interface_parameters.html).

UR의 scaled trajectory controller는 로봇이 보고하는 speed scaling에 맞춰 궤적 시간을
진행시킵니다. 500Hz 설정은 제어 주기 2ms를 뜻하지만, 그 자체로 추종 오차가 없음을
뜻하지 않습니다. 실제 반복 시험에서는 액션 결과와 관절별 desired/actual/error를
확인해야 합니다.
[UR scaled trajectory controller 설명](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html).
