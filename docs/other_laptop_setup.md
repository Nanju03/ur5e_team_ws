# 팀원 PC에 설치하기

[저장소 안내](../README.md) · [Gazebo](gazebo.md) · [실물](real_robot.md)

Ubuntu 22.04와 ROS 2 Humble을 사용합니다. 이 저장소의 UR·DH·serial 소스와 작업공간 설치 파일을 함께 사용합니다.
현재 개발 PC는 **apt의 MoveIt 2.5.10**으로 검증했으며, 기존 `~/ws_moveit2` overlay를 자동으로 불러오지 않습니다.
과거 MoveIt 소스 manifest는 참고용으로 보존합니다.

## 1. 소스 받기

공개 저장소이므로 초대나 GitHub 로그인 없이 아래 명령으로 소스를 받습니다.
변경 사항을 push하려는 팀원에게만 GitHub 인증과 저장소 쓰기 권한이 필요합니다.

```bash
mkdir -p ~/ur5e_team_ws/src
git clone https://github.com/Nanju03/ur5e_team_ws.git ~/ur5e_team_ws/src/ur5e
cd ~/ur5e_team_ws/src/ur5e
python3 scripts/install_workspace.py
```

설치기는 `workspace/` 파일을 작업공간 루트에 복사합니다. ROS·Gazebo·로봇은 시작하지 않습니다.
기존 파일이 다르면 교체를 거절합니다. `--dry-run`으로 확인하고, 백업 후 교체하려면 `--overwrite`를 사용합니다.
다른 경로에는 `--workspace /원하는/작업공간`으로 설치합니다.

## 2. 의존성 설치

ROS 2 Humble을 먼저 설치한 뒤 실행합니다.

```bash
sudo apt update
sudo apt install git build-essential cmake python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pytest python3-tk python3-numpy python3-yaml ros-humble-ur ros-humble-moveit ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-robot-state-publisher
```

rosdep을 처음 사용하는 PC에서만 `sudo rosdep init`을 한 번 실행합니다.

```bash
source /opt/ros/humble/setup.bash
rosdep update
cd ~/ur5e_team_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys 'ament_python warehouse_ros_mongo'
./check_dependencies.sh
./build.sh
```

`warehouse_ros_mongo`는 사용하지 않는 선택 기능이며, `ament_python` 빌드 지원은 colcon/setuptools를 사용합니다.
Gazebo의 최근 controller manager에는 bridge **0.4.10 이상**을 사용합니다.

```bash
dpkg-query -W ros-humble-gazebo-ros2-control ros-humble-controller-manager
```

시스템 bridge가 0.4.7 등으로 남아 있으면 공식 패키지를 이 작업공간에만 풀어 사용할 수 있습니다.
이 명령은 시스템 패키지를 교체하지 않습니다. apt가 0.4.10 이상을 제공해야 합니다.

```bash
./scripts/prepare_gazebo_compat.sh
```

다운로드한 `.deb`·공유 라이브러리·`compat/`는 Git에 포함하지 않습니다.
로컬 bridge는 Gazebo 시작 스크립트가 자동으로 적용합니다.

## 3. 가상 환경 검증

빠른 설치 검사는 로봇 없는 domain 42를 사용합니다.

```bash
cd ~/ur5e_team_ws
./check_setup.sh
```

Gazebo 접촉·물리 시험은 별도로 실행합니다.

```bash
./start_block_gazebo.sh
```

[Gazebo 절차](gazebo.md)를 따릅니다. 대각선 전체 자동 시험은 기존 Gazebo를 종료한 뒤 시작합니다.

```bash
./test_block_diagonals.sh
```

[60가지 시험](gazebo_diagonal_tests.md)과 [저장 파일 형식](saved_paths.md)을 확인합니다.

## 4. 실물 연결

별도 터미널 세 개에서 각각 실행합니다.

```bash
cd ~/ur5e_team_ws
./start_block_robot.sh allow_execution:=true use_rviz:=false
```

```bash
cd ~/ur5e_team_ws
./start_block_view.sh
```

```bash
cd ~/ur5e_team_ws
./start_block_real_gui.sh
```

로봇 IP·USB 권한·배치·승인·중단·복구는 [실물 사용법](real_robot.md)을 따릅니다.
포함한 `robot_calibration/ur5e_20261031.yaml`은 기존 팀 UR5e의 값입니다.
다른 로봇은 그 로봇에서 추출한 보정 파일을 사용합니다. 받침 16mm·어댑터 0mm도 현장과 일치해야 합니다.
설치·빌드 과정은 실물 드라이버나 동작을 자동 시작하지 않습니다.

## 5. 코드 업데이트

```bash
cd ~/ur5e_team_ws/src/ur5e
git pull --ff-only
python3 scripts/install_workspace.py --overwrite
cd ~/ur5e_team_ws
./build.sh
```

기존 설치 사본은 변경될 때만 `.workspace_backups/`로 보관합니다.
GUI를 다시 열고 배치와 경로를 새로 계획합니다. 로그·저장 궤적·로그인 정보는 공유 대상에 포함하지 않습니다.
