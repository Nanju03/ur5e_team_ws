#!/usr/bin/env bash
set -e
# Connection for a manually positioned arm; no grasp trial starts automatically.
ur5e_task_ws="${UR5E_WS:-$HOME/ur5e_ws}"
ur5e_task_moveit_ws="${MOVEIT_WS:-$HOME/ws_moveit2}"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
if [ -f "$ur5e_task_moveit_ws/install/local_setup.bash" ]; then
  source "$ur5e_task_moveit_ws/install/local_setup.bash"
fi
source "$ur5e_task_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
exec ros2 launch ur5e_block_bringup real_robot.launch.py \
  calibration_file:="$ur5e_task_ws/robot_calibration/ur5e_20261031.yaml" \
  physical_profile:="$ur5e_task_ws/physical_setup.yaml" \
  headless_mode:=false activate_arm_controller:=false allow_execution:=false \
  use_gripper_calibration:=false "$@"
