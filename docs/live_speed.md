# ROS 토픽으로 실시간 속도 보기

실물 드라이버가 실행 중일 때 별도 터미널에서 연다. 기존 제어 GUI와 RViz는 재시작하지 않는다.

```bash
cd ~/ur5e_team_ws
./show_robot_speed.sh
```

이 창은 domain 0의 토픽만 구독하며 로봇·그리퍼 명령이나 컨트롤러 설정을 변경하지 않는다.
창을 닫으면 표시기만 종료된다. 약 10Hz로 표시하며, 피드백이 0.5초 이상 끊기면 `—`로 표시한다.

| 표시 | 출처와 의미 |
| --- | --- |
| 관절 실측 속도 | `/joint_states`의 `velocity`, rad/s 및 °/s. `name`으로 6개 팔 관절을 구분한다. |
| 블록 집기점 속도 | 현재 `/robot_description`의 `grasp_link` 모델과 관절 위치·속도로 계산한 병진 속도, mm/s. |
| UR 설정 TCP 속도 | `/tcp_pose_broadcaster/pose`의 위치 차분으로 계산한 약 0.1초 구간 병진 속도, mm/s. |
| 적용 속도 배율 | `/speed_scaling_state_broadcaster/speed_scaling`. 현재 드라이버는 100.0이 100%이다. 실제 이동 속도와 구분한다. |

집기점과 펜던트에 설정한 TCP는 서로 다른 지점일 수 있다. 위치 차분은 센서 잡음·시간 구간의 영향을 받아
정지 중에도 작은 값이 나올 수 있다. TCP 두 표시는 계산값이며 제어용 속도 피드백으로 사용하지 않는다.
관절 표의 부호는 회전 방향이다. 속도 토픽을 받기 위해 로봇을 움직일 필요는 없다.

터미널에서만 읽으려면 다음 명령을 사용한다.

```bash
./show_robot_speed.sh --console
```

다른 ROS domain은 `./show_robot_speed.sh --domain 42`처럼 지정한다.
해당 환경에 TCP pose 토픽이나 `grasp_link` 모델이 없으면 그 표시는 `—`로 남는다.

원본 토픽을 직접 확인하려면 다음과 같이 실행한다. `/joint_states`에는 그리퍼만 담긴 메시지도
섞여 있으므로 `name`과 `velocity`를 함께 읽는다.

```bash
cd ~/ur5e_team_ws
source env.bash
export ROS_DOMAIN_ID=0
ros2 topic echo /joint_states
# 위 구독을 Ctrl+C로 끝낸 후 배율 확인
ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling
```
