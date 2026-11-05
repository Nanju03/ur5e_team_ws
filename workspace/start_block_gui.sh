#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
exec ros2 launch ur5e_a3_grid stack_gui.launch.py backend:=simulation \
  domain_id:=42 simulation_riser_height:=0.016 \
  calibration_file:="$UR5E_WS/robot_calibration/ur5e_20261031.yaml" \
  physical_profile:="$UR5E_WS/physical_setup.yaml" "$@"
