#!/usr/bin/env bash
set -e
# Robot-free GenericSystem only; separate from the real-robot domain (0).
ur5e_task_ws="${UR5E_WS:-$HOME/ur5e_ws}"
ur5e_task_moveit_ws="${MOVEIT_WS:-$HOME/ws_moveit2}"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
if [ -f "$ur5e_task_moveit_ws/install/local_setup.bash" ]; then
  source "$ur5e_task_moveit_ws/install/local_setup.bash"
fi
source "$ur5e_task_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
exec ros2 launch ur5e_a3_grid stack_sim_gui.launch.py "$@"
