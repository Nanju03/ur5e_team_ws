# Source this file in every terminal used for this workspace.
ur5e_task_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_NAMESPACE
source /opt/ros/humble/setup.bash
# Use the installed Humble MoveIt by default. Optional overlays must be explicit.
if [[ -n "${MOVEIT_WS:-}" ]]; then
  if [[ ! -f "$MOVEIT_WS/install/local_setup.bash" ]]; then
    printf 'MoveIt workspace not built: %s\n' "$MOVEIT_WS" >&2
    return 1
  fi
  source "$MOVEIT_WS/install/local_setup.bash"
fi
if [[ -f "$ur5e_task_ws/install/local_setup.bash" ]]; then
  source "$ur5e_task_ws/install/local_setup.bash"
fi
export UR5E_WS="$ur5e_task_ws"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$UR5E_WS/log/ros"
mkdir -p "$ROS_LOG_DIR"
unset ur5e_task_ws
