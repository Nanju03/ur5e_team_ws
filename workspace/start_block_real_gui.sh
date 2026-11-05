#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
export ROS_DOMAIN_ID=0
exec ros2 launch ur5e_a3_grid stack_gui.launch.py backend:=real domain_id:=0 \
  calibration_file:="$UR5E_WS/robot_calibration/ur5e_20261031.yaml" \
  physical_profile:="$UR5E_WS/physical_setup.yaml" "$@"
