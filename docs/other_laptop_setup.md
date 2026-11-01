# 다른 노트북에서 구성하기

Ubuntu 22.04와 ROS 2 Humble을 기준으로 합니다. 이 저장소는 UR5e·AG95 드라이버의 수정본,
사용자 로봇 모델, 플레이트 STL, MoveIt 설정, GUI·쌓기·복구 코드를 포함합니다.
빌드 결과물과 저장된 실행 궤적 대신 소스에서 다시 빌드하고 실제 시작 자세에서 경로를 새로 계획합니다.

## 1. 저장소와 설정 복사

ROS 2 Humble을 설치한 뒤 필요한 도구와 기본 ROS 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install \
  ros-humble-ur ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robot-state-publisher \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  git build-essential cmake

mkdir -p ~/ur5e_ws/src
git clone https://github.com/gamsaon/ur5e.git ~/ur5e_ws/src/ur5e_ros2
mkdir -p ~/ur5e_ws/robot_calibration
cp ~/ur5e_ws/src/ur5e_ros2/workspace/physical_setup.yaml ~/ur5e_ws/
cp ~/ur5e_ws/src/ur5e_ros2/workspace/robot_calibration/ur5e_20261031.yaml ~/ur5e_ws/robot_calibration/
cp ~/ur5e_ws/src/ur5e_ros2/workspace/start_*.sh ~/ur5e_ws/
chmod +x ~/ur5e_ws/start_*.sh
```

기존 작업공간의 설정을 유지할 노트북에서는 복사 전에 해당 파일을 보관합니다.
보정 YAML은 개발에 사용한 **동일한 실물 UR5e**의 값입니다. 다른 로봇을 연결한다면
그 로봇에서 추출한 보정값을 사용해야 합니다. 실물 받침 설정은 16mm·그리퍼 어댑터 없음입니다.

rosdep을 처음 사용하는 노트북에서만 한 번 실행합니다.

```bash
sudo rosdep init
```

## 2. MoveIt 오버레이 재구성

개발 노트북에서는 `~/ws_moveit2`의 MoveIt 소스를 먼저 불러왔습니다. 같은 소스 구성을
재현하려면 포함한 manifest로 8개 저장소의 커밋을 고정합니다. 해당 저장소들은 업로드 시점에
로컬 수정이 없었습니다. 설치된 Humble MoveIt만 사용하는 환경에서는 이 단계를 생략할 수
있지만, 본 프로젝트의 실제 검증은 이 소스 오버레이와 함께 수행했습니다.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2
vcs import src < ~/ur5e_ws/src/ur5e_ros2/workspace/moveit2_humble.repos
for moveit_dependency in "$HOME"/ws_moveit2/src/*; do
  if [ -d "$moveit_dependency/.git" ]; then
    git -C "$moveit_dependency" submodule update --init --recursive
  fi
done
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Task Constructor의 두 하위 모듈도 위 순서에 포함됩니다. 핵심 apt 패키지의 개발 버전은
[`environment_versions.txt`](../workspace/environment_versions.txt)에 기록했습니다.
이 목록은 참고용이며 apt 패키지를 고정 설치하는 lockfile은 아닙니다.

## 3. UR5e 작업공간 빌드

새 터미널에서 기존 overlay 변수를 지우고 의도한 구성만 불러옵니다.

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ws_moveit2/install/local_setup.bash" ]; then
  source "$HOME/ws_moveit2/install/local_setup.bash"
fi
cd ~/ur5e_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
```

기존 ZIP의 `ur_description` overlay를 추가로 불러오지 않습니다. UR 모델의 메시는
`ur_description`, AG95 메시는 저장소의 `dh_ag95_description`, 플레이트는
`ur5e_block_description/meshes/cube_plate.STL`을 사용합니다.

기본 경로는 `~/ur5e_ws`, `~/ws_moveit2`입니다. 복사한 스크립트에는 사용자 이름이 들어간
절대 경로가 없으며, 필요하면 `UR5E_WS`, `MOVEIT_WS` 환경변수로 불러올 작업공간을 지정할
수 있습니다. 일부 개인 설정·그리퍼 보정 결과의 기본 저장 위치는 여전히
`~/ur5e_ws`이므로 같은 기본 디렉터리 구성을 권장합니다.

## 4. 로봇 없이 화면과 모션 확인

```bash
~/ur5e_ws/start_block_sim.sh
```

이 실행은 도메인 42의 GenericSystem과 GUI를 사용합니다. 실물 UR·그리퍼 드라이버를
시작하지 않습니다. GUI에서 배치·계획·가상 실행을 선택하고 경로를 본 뒤 OK로 진행합니다.

## 5. 같은 실물 로봇 연결

네트워크·USB 장치 설정은 [실습 가이드](UR5e_DH_노트북_실습가이드.md)를 따릅니다.
기본 로봇 IP는 `192.168.0.2`, 노트북 유선 IP는 `192.168.0.1`입니다.
티치펜던트를 원격 제어 모드로 둡니다. 드라이버 초기화 시 그리퍼 손가락이 움직일 수 있습니다.

터미널 1에서 드라이버와 계획 서버를 **한 번만** 실행합니다.

```bash
~/ur5e_ws/start_block_robot.sh allow_execution:=true use_rviz:=false
```

별도 터미널 2·3에서 뷰어와 GUI를 엽니다.

```bash
~/ur5e_ws/start_block_view.sh
```

```bash
~/ur5e_ws/start_block_gui.sh
```

이미 드라이버가 동작 중이면 연결 스크립트를 또 실행하지 않고 뷰어·GUI만 엽니다.
실물 실행은 도메인 0을 사용합니다. 복사한 스크립트는 기존처럼 추가 launch 인수를 그대로
전달합니다. 실제 GUI 작업·모서리 X/Y 맞춤·복구 절차는
[`ur5e_block_bringup/README.md`](../ur5e_block_bringup/README.md)를 따릅니다.

## 포함 범위

빌드·설치 디렉터리, ROS 로그, 예전 실행 궤적·승인 토큰·현재 잡힘 복구 잠금은 옮기지 않습니다.
작업 배치는 GUI에서 다시 저장하고 현재 모델·보정·시작 관절값으로 새로 계획합니다.
검증 도구와 과거 결과가 별도로 포함돼 있더라도, 기록 당시 설정과 현재 명령 조건은 다를 수 있습니다.
