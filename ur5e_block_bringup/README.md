# 기존 실물 환경과 블록 쌓기 프로젝트 연결

기준 환경은 `~/ur5e_ws/src/ur5e_ros2`입니다. 기존 UR 드라이버·DH 드라이버·serial
라이브러리를 재사용합니다. 별도 모니터나 가상 하드웨어를 시작하지 않습니다.

## AG95 단독 파지 보정 — launch 실행 권장

팔 없이 AG95만 연결한 경우 아래 launch를 사용합니다. GUI, UR5e 드라이버, RViz,
MoveIt 노드를 실행할 필요가 없으며 별도 `.sh` 파일도 사용하지 않습니다.
그리퍼 본체를 단단히 고정하고 블록 받침과 비상 전원 차단 수단을 준비합니다.
**드라이버 초기화는 보정 명령과 별개로 손가락을 움직일 수 있습니다.** 손은 동작 범위에서 치웁니다.

각 터미널에서 같은 환경을 불러옵니다.

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/local_setup.bash
source ~/ur5e_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
```

터미널 1에서 실제 USB 포트를 지정합니다.

```bash
ros2 launch ur5e_block_bringup gripper_calibration.launch.py \
  com_port:=/dev/ttyUSB0 execution_mode:=continuous
```

기본값은 `gripper_only:=true`, `start_driver:=true`, `force_percent:=20`,
`startup_operation:=monitor`, `workspace_clear:=false`입니다. 기본 launch는 상태를 감시하며
파지 시험을 자동으로 시작하지 않습니다. 힘 20%는 설정 비율이며 실측 힘(N)이 아닙니다.
열림각은 0.50rad, 닫힘 상한은 0.78rad, 간격은 0.01rad, 단계 대기는 0.4초, 블록 폭은 25mm입니다.
위 예시는 `execution_mode:=continuous`를 명시하여 한 작업 안의 여러 각도를 순서대로 실행합니다.
현재 코드의 기본 `execution_mode:=step`은 각 각도·모델 간격을 표시한 뒤 한 단계씩 확인하는 방식입니다.
step 모드에서는 `/status`의 `pending_approval`을 검토하고 터미널에 출력되는
`/gripper_calibration/approve_next/r<토큰>` 경로에 `std_srvs/srv/Trigger "{}"`를 호출합니다.
이 승인은 해당 단계 하나만 실행하며, 이전 단계의 토큰은 재사용할 수 없습니다.

보정 노드는 2초 동안 기존 그리퍼 제어기를 확인한 뒤 없을 때만 기존 DH launch를 자식 프로세스로 시작합니다.
이미 드라이버가 있다면 중복 실행하지 말고 다음과 같이 재사용합니다. 기존 드라이버의
`dh_ag95_gripper_controller`와 `force_forward_controller`가 활성 상태여야 합니다.

```bash
ros2 launch ur5e_block_bringup gripper_calibration.launch.py \
  start_driver:=false execution_mode:=continuous
```

자신이 시작한 드라이버는 보정 노드가 소유합니다. 정상 종료 시 진행 중 목표의 종료와 새 센서 표본으로
그리퍼 정지를 확인한 뒤 자식 드라이버를 종료합니다. 재사용한 외부 드라이버는 종료하지 않습니다.

터미널 2에서 상태를 조회합니다. `/status` 응답의 `message`는 JSON이며 `/state`에서도 같은 상태를 볼 수 있습니다.
초기 연결 확인 중에는 물리 동작 서비스를 받지 않으므로 준비 상태가 될 때까지 기다립니다.

```bash
ros2 service call /gripper_calibration/status std_srvs/srv/Trigger "{}"
# 상태를 계속 보고 싶을 때
ros2 topic echo /gripper_calibration/state std_msgs/msg/String
```

다음 순서로 시험합니다. **동작 서비스의 `success: true`는 접수 응답입니다. 완료는 `/status`의
`busy: false`와 `phase`로 확인한 뒤 다음 명령을 보냅니다.** 잡힘 뒤에는 `HOLD`, 빈 기준과 열기 완료 뒤에는
`IDLE` 상태를 확인합니다. 상태 토픽을 보고 있다면 명령은 같은 환경을 불러온 다른 터미널에서 보냅니다.

1. 그리퍼 사이에서 블록을 빼고 고정·받침·손 제외·전원 차단 준비를 확인합니다.
   `/ready`는 상태 조회가 아니라 **물리 명령 허용 확인**입니다. `true`는 세션 동안 유지됩니다.

   ```bash
   ros2 service call /gripper_calibration/ready std_srvs/srv/SetBool "{data: true}"
   ros2 service call /gripper_calibration/baseline std_srvs/srv/Trigger "{}"
   ```

   빈 공간에서 닫힘 상한까지 단계적으로 닫은 뒤 기준 열림각으로 돌아갑니다. 상태에서 완료를 확인합니다.

2. 25mm 블록을 받침 위에 놓고 손가락 양옆에 맞춘 뒤 손을 치우고 단계 잡기를 시작합니다.

   ```bash
   ros2 service call /gripper_calibration/grasp std_srvs/srv/Trigger "{}"
   ```

   실제 잡힘 센서가 감지되면 더 닫지 않고 유지합니다. 자동으로 블록을 들어 올리거나 열지 않습니다.
   `HOLD`와 센서 상태를 확인한 뒤 실제 잡힘을 확인합니다. 성공이면 다음 명령을 보냅니다.

   ```bash
   ros2 service call /gripper_calibration/success std_srvs/srv/Trigger "{}"
   ```

   실패라면 `/success` 대신 아래 명령으로 기록합니다. 두 판정 중 하나만 사용합니다.

   ```bash
   ros2 service call /gripper_calibration/failure std_srvs/srv/Trigger "{}"
   ```

3. 블록이 받침에 안전하게 놓여 있는지 확인한 뒤 엽니다. 실제 열림 확인까지 기다립니다.

   ```bash
   ros2 service call /gripper_calibration/open std_srvs/srv/Trigger "{}"
   ```

4. 잡기 → 실제 성공 판정 → 열기를 같은 조건에서 3회 이상 완료합니다.
   상태의 `recommendation`이 생기면 시험한 추천값을 확인한 뒤 저장합니다.

   ```bash
   ros2 service call /gripper_calibration/apply std_srvs/srv/Trigger "{}"
   ```

추천값은 `~/ur5e_ws/gripper_calibration.yaml`에, CSV·JSON 시험 기록은
`~/.ros/ur5e_a3_stack/gripper_calibration/`에 저장합니다. 추천에는 시험하지 않은 추가 조임을 넣지 않으며,
성공·놓기가 검증된 힘 그룹 중 가장 낮은 설정을 사용합니다. 힘을 자동으로 올리지 않습니다.
필요하면 블록을 놓고 `phase: IDLE`인 상태에서 직접 설정을 바꿀 수 있으며 다음 동작에서 반영됩니다.

```bash
ros2 param set /gripper_calibration force_percent 30
```

중지하거나 허용을 철회하는 명령은 다음과 같습니다. `/stop`은 현재 시험과 예약된 초기 동작을 취소합니다.
`/ready false`는 여기에 물리 명령 허용 철회까지 수행합니다. 중지 접수 뒤에도
`stop_confirmed: true`, `busy: false`를 확인합니다.

```bash
ros2 service call /gripper_calibration/stop std_srvs/srv/Trigger "{}"
ros2 service call /gripper_calibration/ready std_srvs/srv/SetBool "{data: false}"
```

빈 공간 기준 측정만 launch 직후 **한 번** 수행하려면, 블록을 빼고 준비 조건을 확인한 뒤 다음을 사용합니다.

```bash
ros2 launch ur5e_block_bringup gripper_calibration.launch.py \
  com_port:=/dev/ttyUSB0 execution_mode:=continuous \
  startup_operation:=baseline workspace_clear:=true
```

완료하면 감시 상태로 남으며 자동 반복이나 자동 파지를 하지 않습니다.
연결·준비가 30초 안에 완료되지 않으면 예약 동작을 취소하고 감시 상태로 남습니다.
이 보정은 실제 팔 자동 쌓기의 검증이 아닙니다. 실물 자동 쌓기는 아래의 별도 실행 경로를 사용합니다.

## 팔과 함께 기존 실물 환경 연결

### 실물 집기·쌓기

실물 GUI에서 **④ 저장 경로 불러오기 / 새 경로 계획**을 누르면 일치하는 경로를 먼저 불러옵니다.
저장 자료가 없으면 현재 팔·그리퍼 자세에서 선택한 모든 블록의 경로와 허용 손가락 형상을 검사하고 자동 보관합니다. 계획이 끝난 뒤 **⑤ 실물 집기·쌓기 실행**을 누르고,
표시된 경로를 보고 **OK**를 눌러야 움직입니다. 단계별 모드는 팔 이동과 손가락 개폐를
하나씩 승인하고, 전체 연속 모드는 전체 계획을 한 번 승인합니다.

계획에는 경로 생성과 전체 충돌 검사가 포함됩니다. 화면의 **실물 경로 충돌 검사:
현재/전체 단계 · 검사한 상태 수**로 검증 진행을 확인합니다. 이 단계에서는 실제 팔과
그리퍼가 움직이지 않습니다. 검증용 통신 연결은 한 작업 동안 재사용하며 검사 지점은 유지합니다.

- 잡기 명령은 **0.73rad**입니다. 명령 완료를 잡기로 간주하고 다음 상승을 진행합니다.
  목표 도달 성공 또는 장치의 접촉 정지 응답을 확인하며, 물체 잡힘·이탈 상태값은
  집기 성공이나 운반 중단의 조건으로 사용하지 않습니다. 실제 블록에 닿으면 명령각보다
  먼저 멈출 수 있습니다. 잡기 명령이 끝난 측정 각도를 기준으로,
  사전에 검사한 0.690~0.733rad 범위에 실측 유지 구간이 포함되는지 확인합니다.
  포함되면 저장 결과를 재사용하며, 벗어나면 팔을 정지한 채 부족한 손가락 형상만 로컬 충돌 검사합니다. 통신 지연·검사한 그리퍼 형상 범위 밖의 위치 변화는 중단합니다.
  파지 유지의 기준각 여유는 **0.005rad**입니다. 닫힘 명령은 **0.73rad**, 확인 상한은
  **0.733rad**로 유지하며, 넓어진 범위도 충돌 검사를 통과해야 합니다. 빈 그리퍼·모서리 맞춤과
  놓기 완료의 정밀도는 기존 설정을 유지합니다. 허용 범위가 바뀐 이전 경로는 다시 계획합니다.
- 놓는 자리에서는 팔을 멈춘 채 **실측 파지각 기반 1차 열림(최대 0.70rad) → 0.60rad** 순서로 엽니다.
  두 번째 열기 명령의 도착 위치를 확인한 뒤 후퇴합니다. 물체 감지 상태는 놓기 판정에 사용하지 않습니다.
- 다음 집기 전 높은 대기 위치에서 빈 그리퍼를 **0.40rad**로 열고 접근합니다.
  좁은 0.70rad 준비 동작은 제거했습니다. 목표 적재로 다른 집기가 막히면 노출된 다른
  단일 블록 위에 임시 적재한 뒤 회수하는 순서를 자동 탐색합니다. 임시 탑은 2층까지이며
  받침 블록은 위의 블록을 회수한 뒤 집습니다. 11번 장애물·14번 목표에서는
  10번을 다른 블록 위에 보관 → 15번을 14번에 적재 → 10번을 임시 위치에서 회수합니다.
  임시 운반은 목표 진행 수와 따로 검증합니다. 전체 경로와 회수 가능성을 확인한 뒤 실행하며,
  현재 v15는 진입 높이·하강 속도가 달라 v13/v14 경로를 다시 계획합니다. 기존 자료는 보존합니다.
- 체크를 해제한 전체 경로 모드에서 집기·놓기의 마지막 20mm 하강 또는 수평 진입과 처음 20mm 이탈은 **속도·가속도 비율 0.10**입니다.
  그 위 수직 하강은 **0.50**이며, 접촉·접근 여유는 **20mm**입니다.
  이후 상승·후퇴·운반은 설정된 관절 한도의 **속도·가속도 비율 1.00**입니다.
  UR의 속도 슬라이더와 로봇 측 제한도 적용되며 실물 파지·적층 안정성은 별도 검증 대상입니다.
  각 동작의 시작과 끝에서는 팔이 멈춥니다. 가속도 설정이 바뀌었으므로 기존 경로를 다시 계획해야 합니다.
  속도 비율은 m/s 값이 아닙니다.
- 실물 계획은 같은 보간·손목 표본을 로컬 C++ MoveIt에서 검사합니다. 반복되는 정적 메쉬의
  정규화 계산도 재사용하되 현재 메쉬 전체 내용·위치·충돌 허용 행렬의 변경을 계속 감지합니다.
  센서 신선도 0.5초와 동작 직전 검증·작업자 승인은 유지합니다.
  [계획 최적화](../docs/planning_optimization.md), [센서 지연 중단](../docs/gripper_feedback_stop.md)을 참고하세요.
- 실물 집기·쌓기의 이동 중 관절 추종 허용오차는 요청한 **0.1rad**입니다.
  진입·상승·운반·후퇴의 팔 6개 관절에 모두 적용합니다.
  초기 자세 복구와 공중·X/Y 맞춤 시운전의 기존 **0.005rad**는 유지합니다.
  정지 목표는 실물 집기·쌓기의 모든 팔 관절에서 위치 **0.001rad**,
  속도 **0.01rad/s**, 추가 대기 **5초**로 맞춥니다. 넓어진 손목 범위와 파지 형상은
  경로 표시·승인과 실행 전에 충돌 검사합니다. 이는 팔 전체의 연속 오차 범위를
  수학적으로 보증한다는 뜻은 아닙니다. [추종 설정의 근거와 검증 범위](docs/physical_tracking_profile.md)를 참고하세요.
- 쌓을 칸의 기존 25mm 블록은 바닥 1층으로 유지합니다.
  기본 열림 **0.40rad**, 모서리 맞춤 **0.65rad**, 각 X/Y 맞춤 정지 후 OK로 재개하는 동작을 사용합니다.

이 경로는 `stack_mode:=physical_plan/physical_run`을 사용합니다. 이전 C++
`stack_mode:=run`의 GenericSystem 제한은 유지합니다. 경로·배치·보정·모델이 바뀌면
다시 계획해야 합니다. 잡은 상태에서 중단하거나 놓기를 완료하지 못하면 블록 형상과
복구 잠금을 남기며, 일반 초기화로 물체를 없애거나 바로 재실행하지 않습니다.
가상 검증과 실물 경로 검사 통과는 실제 파지·적재 성공을 뜻하지 않습니다.

조작창은 왼쪽 조작·플레이트와 오른쪽 실행 로그로 반씩 나뉩니다. 가운데 경계를 드래그해
크기를 바꿀 수 있고, 로그는 가로·세로로 스크롤하거나 선택해 복사할 수 있습니다.
OK와 중지 버튼은 왼쪽 위에 고정합니다.

**플레이트 맞춤 · 모서리 X/Y**는 A·B·C·D 각 모서리 홈에서 X와 Y를 각각 확인합니다.
X 맞춤 위치에 정지한 뒤 판을 맞추고 손을 빼서 OK를 누르면, 위로 상승해 90° 회전하고
같은 홈의 Y 맞춤 위치로 내려갑니다. 그리퍼 **0.65rad**, 전체 끝부분의 상판 위
**10mm 여유**는 유지합니다. 낮은 위치에서 회전하지 않습니다.
연속 모드도 총 **8개 맞춤 구간을 각각 OK**로 승인하며, 각 도착 위치에서 정지합니다.
Y 맞춤 후에도 조정이 끝날 때까지 다음 구간을 보내지 않습니다. 마지막 D-Y는 정지한 채
완료합니다. 이전 한 방향 맞춤 경로는 사용할 수 없으므로 새로 계획합니다.
맞춤은 빈 모서리 홈을 기준으로 하며, 정렬이 끝난 뒤 블록을 배치합니다.
수동 조정 시간을 확보하도록 X/Y 맞춤 경로는 계획 후 30분 안에 사용하며, 대기 중에도
현재 자세·그리퍼·모델·충돌 장면을 계속 확인합니다. 일반 공중 경로의 유효시간은 유지합니다.

집기 중단 후 GUI의 **안전 확인 후 초기 자세 복구**는 현재 그리퍼 각도를 유지하며 팔만 복귀합니다.
`grip_state`의 초기 값 0~3을 모두 허용하며, 그리퍼를 자동으로 열거나 초기화하지 않습니다.
실제 잡고 있는 블록과 RViz의 부착 물체가 일치하는지 확인하고 경로를 OK로 승인합니다.
새 관절·그리퍼 센서와 정지는 계속 확인하며, 복귀 중 각도 변경·통신 중단·새로운 블록 이탈·충돌은 중지합니다.
복구 팔 경로의 속도·가속도 비율은 **0.05**입니다. 이미 초기 자세라면 이동 목표를 보내지 않습니다.

완료 후 탑·남은 블록·부착 물체 장면을 유지하고 검토한 실행 계획만 무효화합니다.
중단 기록은 새 빈 센서 상태와 부착 블록 없음이 확인될 때만 해제합니다.
중간 배치에서 다시 계획할 때는 ③으로 장면을 초기화하지 말고 ④로 진행합니다.
명령행에서 같은 방식을 선택하려면 `recovery_preserve_gripper:=true`를 사용합니다.
이 인수를 생략한 기존 명령행 방식은 빈 그리퍼 확인 → 초기 자세·0.4rad → 장면 초기화를 유지합니다.

실행기는 이동 전에 상태 감시용 서비스 연결과 현재 장면의 응답을 확인합니다.
새 경로 계획의 시작 단계에서 모델·하드웨어·컨트롤러 조회 응답이 시간 초과되면
같은 읽기 요청을 한 번만 다시 보냅니다. 서비스가 없거나 응답 내용이 잘못된 경우는
재시도하지 않으며, 이동 명령이나 계획 장면 쓰기는 자동 재전송하지 않습니다.
`STACK_READ_RPC_TIMEOUT`에는 실패한 서비스 이름과 조회 단계·시도 횟수를 기록합니다.
이동 중 상태 응답이 끊기면 목표를 취소하고 `STACK_STOP_REASON`에 서비스명과 원인을 남깁니다.
`physical_stack`의 `process has died ... exit code 1`은 해당 작업이 오류로 종료됐다는 뜻입니다.
UR 드라이버 종료 여부는 별도 상태로 확인하며, 작업 중지 후에는 현재 자세에서 다시 계획합니다.

### 드라이버 연결

```bash
~/ur5e_ws/start_block_robot.sh
```

이 명령은 기존 UR 제어 프로그램을 Headless로 연결하고 AG95를 초기화합니다.
그리퍼 손가락이 움직일 수 있습니다. 자동 블록 쌓기는 시작하지 않습니다.
MoveIt 궤적 실행은 기본적으로 꺼져 있으며 `allow_execution:=true`로 명시할 수 있습니다.

실물 연결은 한 번만 실행합니다. 같은 ROS 도메인에 팔 또는 그리퍼 드라이버가 남아 있으면
새 하드웨어 launch는 자식 노드를 만들기 전에 거절합니다. 일부 드라이버만 종료된 경우에도
기존 컨트롤러를 재설정하지 않습니다. 이미 연결돼 있으면 각각의 터미널에서
`~/ur5e_ws/start_block_view.sh`와 `~/ur5e_ws/start_block_gui.sh`로 화면만 엽니다.

동작 없이 팔 드라이버·모델만 확인하는 구성은 다음과 같습니다.

```bash
~/ur5e_ws/start_block_robot.sh start_gripper:=false headless_mode:=false activate_arm_controller:=false
```

연결 지점:

- 팔: `/scaled_joint_trajectory_controller/follow_joint_trajectory`.
- 그리퍼: `/gripper/dh_ag95_gripper_controller/gripper_cmd`.
- 두 실제 드라이버가 `/joint_states`에 각자 측정한 관절을 발행합니다.
- 사용자 UR5e+AG95 모델용 `robot_state_publisher`는 하나만 시작합니다.
- 실제 추출한 보정값을 UR 드라이버와 사용자 모델에 동일하게 적용합니다.

ZIP의 `ur_description`은 기존 하드웨어 패키지 위에 올리지 않습니다.
자체 모델은 `ur5e_block_description`에 두고 기존 UR·AG95 메시를 참조합니다.
이 실행에는 `~/ur5e_block_ws/install/setup.bash`를 source할 필요가 없습니다.

## 이전 GUI 보정 방식 — 참고

현재 권장 방식은 위의 launch와 서비스입니다. 아래 GUI 진입점과 기존 사용 설명은 참고용으로 보존합니다.

### 팔 없이 AG95만 연결한 경우

두 터미널에서 먼저 같은 환경을 불러옵니다. 별도 `.sh` 파일은 사용하지 않습니다.

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/local_setup.bash
source ~/ur5e_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
```

터미널 1에서 그리퍼 드라이버만 시작합니다. 기존 UR/가상 팔 launch는 먼저 종료합니다.
`com_port`에는 실제 USB-RS485 포트를 사용합니다. 아래 예시는 `/dev/ttyUSB0`입니다.

```bash
ros2 launch dh_gripper_driver dh_ag95_control.launch.py \
  use_mock_hardware:=false controller_mode:=action force_percent:=20 \
  gui:=false launch_robot_state_publisher:=false \
  joint_states_topic:=/joint_states com_port:=/dev/ttyUSB0
```

터미널 2에서 전용 보정창을 엽니다. UR5e 드라이버, RViz, MoveIt 노드를 실행할 필요가 없습니다.

```bash
ros2 run ur5e_block_bringup gripper_calibration_gui --gripper-only
```

그리퍼 본체를 단단히 고정하고, 블록을 받침에 놓고, 손을 움직임 범위에서 치운 뒤 시험합니다.
드라이버 시작 시 초기화로 손가락이 움직일 수 있습니다. 전원 차단 수단을 준비합니다.
빈 상태 기준 측정 → 25mm 블록 단계 잡기 → 성공/실패 기록 → 받침 위에서 열기를 반복합니다.
`--gripper-only`에서만 팔 피드백과 RSP 모델 서비스를 요구하지 않습니다.
로컬 모델 파일은 계속 사용하므로 모델 간격은 센서로 측정한 폭과 구분해 표시합니다.
모델이 같은 프로젝트의 기존 파지 프로파일과 동일하여, 검증한 각도를 이후 팔 연동에도 사용할 수 있습니다.

단독 모드는 2초의 ROS 연결 확인 뒤 시험을 허용합니다. 팔 관절 피드백이나 다른 로봇 제어기,
혼합된 팔 하드웨어가 발견되면 세션을 차단하고 진행 중 명령을 취소합니다.
센서·힘 readback·새 표본 기반 정지 확인·가상 기록 적용 제한은 기존과 동일합니다.
ROS에 표시되지 않는 물리 장치의 부재까지 확인하는 기능은 아니므로 실제로 분리된 그리퍼에서만 사용합니다.

### 팔에 장착한 상태에서 사용하는 경우

기존 로봇 launch가 있으면 정지 상태에서 종료하고, 작업 공간을 정리한 뒤 두 터미널에서 실행합니다.

```bash
# 터미널 1: 수동으로 위치를 맞출 팔의 상태를 읽고 AG95를 연결합니다.
ros2 launch ur5e_block_bringup real_robot.launch.py \
  headless_mode:=false activate_arm_controller:=false \
  allow_execution:=false use_gripper_calibration:=false
```

```bash
# 터미널 2: 기존 GUI에서 「25mm 파지 보정」을 누릅니다.
ros2 launch ur5e_a3_grid stack_gui.launch.py backend:=real
```

보정 연결은 팔 제어기를 활성화하거나 팔 이동 목표를 보내지 않습니다.
AG95 연결 시 기존 초기화로 손가락이 움직일 수 있습니다. 기본 힘 20%는 초기화 완료 후의 설정입니다.
보정 창을 여는 동작은 장치를 초기화하거나 손가락을 움직이지 않습니다.

1. 그리퍼가 빈 상태에서 **빈 공간 기준 측정**을 합니다. 기본 열림각 0.50rad에서
   0.78rad까지 0.01rad씩 닫아 간섭이 없는지 확인한 뒤 기준 열림각으로 돌아갑니다.
2. 팔을 수동으로 움직여 받침 위 25mm 블록의 양옆에 손가락을 맞추고 팔을 멈춥니다.
3. **25mm 블록 1회 단계 잡기**를 누릅니다. 실제 잡힘 센서가 감지되면 더 닫지 않고 유지합니다.
   팔을 자동으로 들어 올리거나 손가락을 자동으로 열지 않습니다.
4. 실제로 잡혔는지 확인하고 **성공 확인** 또는 **실패 확인**을 기록합니다.
   잡힌 상태에서 사용자가 팔을 수동으로 움직여 확인할 수 있으며, 결과 기록은 다시 팔이 정지한 뒤 가능합니다.
5. 블록을 받침에 놓고 **기준 열림각으로 열기**를 누릅니다. 실제 열림이 확인되어야 놓기 완료로 기록됩니다.
6. 같은 조건에서 성공·놓기를 3회 이상 반복하면 추천값이 표시됩니다. 필요하면 힘을 직접 바꾸어 반복할 수 있습니다.
   힘을 자동으로 올리지 않으며, 같은 힘에서 검증된 그룹 중 가장 낮은 설정을 고릅니다.
7. **추천값 적용**을 확인하면 `~/ur5e_ws/gripper_calibration.yaml`에 기록합니다.
   이 버튼은 실제 손가락을 움직이지 않습니다. 기존 팔 경로는 다시 계획해야 합니다.

CSV와 JSON 기록은 `~/.ros/ur5e_a3_stack/gripper_calibration/`에 보관하며 창에 정확한 경로를 표시합니다.
명령각과 센서각, 모델에서 계산한 추정 간격, raw 위치, 잡힘/놓침 상태, 힘 설정%, 통신 표본 번호와 경과 시간을 기록합니다.
AG95 공개 Modbus 상태에는 실측 파지력(N)·전류 레지스터가 없어 그 값을 측정했다고 표시하지 않습니다.
센서 상태는 [제조사 AG Modbus 매뉴얼](https://www.dh-robotics.com/wp-content/uploads/2022/12/AG-%E7%B3%BB%E5%88%97%E6%93%8D%E4%BD%9C%E6%89%8B%E5%86%8C_V2.5MODBUS-%E7%89%88-.pdf)의
0x0201(이동/빈 도달/잡힘/놓침), 0x0202(실제 위치), 0x0101(설정한 힘 비율)을 읽습니다.

파지 보정 중에는 다른 팔 계획·실행을 잠급니다. 손가락 명령 중 팔이 움직이거나 센서가 오래되면 취소하고,
원격 목표의 종료와 그 이후 새 센서 표본으로 정지를 확인할 때까지 창을 유지합니다.
파지 유지 중 통신이 끊기거나 놓침이 감지된 회차는 추천에서 제외합니다.
추천은 실제로 시험한 명령각을 사용하며 추가 조임이나 시험하지 않은 힘을 추정해서 넣지 않습니다.
가상 센서 시험 기록은 실물 프로파일로 적용할 수 없습니다.

일반 연결은 검증된 프로파일의 힘 설정을 사용하고, native 블록 계획은 열림·닫힘 각도를 읽습니다.
수동 보정 연결은 이전 프로파일 적용을 끄고 20%에서 새 시험을 준비합니다.
기존 C++ 직접 실행의 GenericSystem 제한은 유지됩니다. 이 보정만으로 전체 자동 쌓기 검증이 완료되지는 않습니다.

검증 기록: Python 테스트 404개와 메모리 기반 드라이버 테스트 70개를 통과했습니다.
도메인93의 가상 센서·action으로 기준 측정, 성공/놓기 3회, 추천, stale 통신 취소를 확인했고,
도메인94의 GenericSystem으로 실제 제어기 로딩·7개 상태 인터페이스·action 연결을 확인했습니다.
실물 시험은 실행하지 않았습니다. 전체 드라이버 lint에는 수정 전과 동일한 12개 legacy 파일의
형식 오류와 외부 XML 스키마 다운로드 실패가 남아 있습니다. 수정한 파일의 형식 검사는 통과했으며
비교 근거는 `~/ur5e_ws/validation/gripper_driver_lint_audit.txt`에 있습니다.

같은 환경에서 원래 쌓기 코드의 장면·계획을 사용할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/local_setup.bash
source ~/ur5e_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
ros2 launch ur5e_block_bringup block_task.launch.py stack_mode:=scene
```

기존 `scene/plan/reach/reset` 진입점은 장면 확인에 사용할 수 있습니다.
실물 GUI의 집기·쌓기는 위의 `physical_plan/physical_run` 경로를 사용합니다.
기존 `stack_blocks` 직접 실행의 가상 하드웨어 제한은 유지합니다.

검증 기록(2026-10-31): 관련 6개 패키지 빌드 및 보정 처리 테스트 22개 통과.
기존 UR 드라이버를 Headless 자동 시작·운동 컨트롤러·그리퍼 없이 연결해 실제
`/joint_states` 수신과 `Calibration checked successfully`를 확인했습니다.
기존 드라이버와 사용자 모델의 연결 검증이며 그리퍼 초기화나 자동 적재 시험은 아닙니다.
결과: `~/ur5e_ws/validation/native_connection_result.json`.

실물 저장 경로의 장기 보관·기존 파일 변환·중복 검사 생략은 [빠른 실행 안내](../docs/saved_physical_replay.md)를 참고하세요.

전체 경로 v18은 마지막 20mm·처음 20mm 이탈을 0.10, 접촉 구간 밖의 하강을 0.50으로 처리합니다.
기본 한 블록 v20은 집기 접근·하강 1.00/1.00, 잡은 뒤 연속 운반은 10층까지 1.00/1.00, 11층부터 0.35/0.20, 최종 놓기 0.10/0.10입니다. 자세는 19층까지 수직·20층부터 측면을 유지합니다.
Python 공통 설정은 `ur5e_block_bringup/physical_motion.py`이며 [최신 높이·속도 안내](../docs/planning_optimization.md)를 확인하세요.
