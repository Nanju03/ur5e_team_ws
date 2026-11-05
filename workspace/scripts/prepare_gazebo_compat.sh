#!/usr/bin/env bash
# Extract an official ROS bridge locally. No sudo, system install, or robot startup.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/../env.bash"
gazebo_download_dir="$(mktemp -d "$UR5E_WS/log/gazebo_bridge_download.XXXXXX")"
trap 'rm -rf -- "$gazebo_download_dir"' EXIT
cd "$gazebo_download_dir"
if [[ $# -eq 1 ]]; then
  cp -- "$1" ./bridge.deb
elif [[ $# -eq 0 ]]; then
  apt-get download ros-humble-gazebo-ros2-control
else
  printf 'Usage: %s [absolute_path_to_official_ros_bridge.deb]\n' "$0" >&2
  exit 2
fi
gazebo_packages=( ./*.deb )
if [[ ${#gazebo_packages[@]} -ne 1 || ! -f "${gazebo_packages[0]}" ]]; then
  printf 'Expected one official ROS bridge package.\n' >&2
  exit 1
fi
[[ "$(dpkg-deb -f "${gazebo_packages[0]}" Package)" == ros-humble-gazebo-ros2-control ]]
gazebo_bridge_version="$(dpkg-deb -f "${gazebo_packages[0]}" Version)"
dpkg --compare-versions "$gazebo_bridge_version" ge 0.4.10
gazebo_bridge_arch="$(dpkg-deb -f "${gazebo_packages[0]}" Architecture)"
[[ "$gazebo_bridge_arch" == "$(dpkg --print-architecture)" ]]
gazebo_compat_dir="$UR5E_WS/compat"
mkdir -p "$gazebo_compat_dir"
touch "$gazebo_compat_dir/COLCON_IGNORE"
gazebo_compat_stage="$(mktemp -d "$gazebo_compat_dir/bridge.XXXXXX")"
dpkg-deb -x "${gazebo_packages[0]}" "$gazebo_compat_stage"
if [[ -d "$gazebo_compat_dir/gazebo_ros2_control" ]]; then
  mv "$gazebo_compat_dir/gazebo_ros2_control" "$gazebo_compat_dir/previous_$(date +%Y%m%d_%H%M%S)_$$"
fi
mv "$gazebo_compat_stage" "$gazebo_compat_dir/gazebo_ros2_control"
printf 'Local Gazebo bridge ready: %s (%s). System ROS packages were unchanged.\n' "$gazebo_bridge_version" "$gazebo_bridge_arch"
