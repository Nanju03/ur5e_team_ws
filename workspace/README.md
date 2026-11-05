# UR5e 팀 작업공간

Ubuntu 22.04 / ROS 2 Humble의 UR5e·AG95 블록 적재 작업공간입니다.
Git 소스는 `src/ur5e/`에 있습니다. 설치·업데이트·Git 사용법은
[Git 저장소 README](https://github.com/Nanju03/ur5e_team_ws)를 따릅니다.

## 빌드와 실행

```bash
cd ~/ur5e_team_ws
./build.sh
./start_block_gazebo.sh
```

- [Gazebo 사용법](docs/gazebo.md)
- [실제 로봇 사용법](docs/real_robot.md)
- [대각선 60가지 자동 시험](docs/gazebo_diagonal_tests.md)
- [성공 경로 저장](docs/saved_paths.md)
- [실물 속도·가속도](docs/speed_and_timing.md)

Gazebo는 ROS 44, RViz·GenericSystem은 ROS 42, 실물은 ROS 0을 사용합니다.
실물은 드라이버·뷰어·GUI를 별도 터미널에서 시작합니다.
기본 잡기는 실물 0.73rad / Gazebo 0.78rad이며, 19층까지 수직 적재하고 20층부터 측면으로 삽입합니다.

`build/`, `install/`, `log/`, `saved_paths/`, `compat/`는 각 PC에서 생성합니다.
이 디렉터리를 Git 소스 안으로 복사하지 않습니다. 실물은 표시된 경로를 사용자가 승인해야 움직입니다.
자동 대각선 시험의 자동 승인은 localhost Gazebo에만 적용합니다.

## Git 업데이트

작업을 멈추고 소스를 갱신한 뒤 설치 사본과 빌드를 업데이트합니다.

```bash
cd ~/ur5e_team_ws/src/ur5e
git pull --ff-only
python3 scripts/install_workspace.py --overwrite
cd ~/ur5e_team_ws
./build.sh
```

기존 설치 파일이 바뀌면 `.workspace_backups/`에 보관한 뒤 갱신합니다.
코드·문서의 원본은 `src/ur5e/`에서 수정·커밋합니다. 작업공간 루트는 Git 저장소가 아닙니다.
