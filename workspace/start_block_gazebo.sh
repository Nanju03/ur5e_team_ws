#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
source "$UR5E_WS/scripts/gazebo_env.bash"
python3 - <<'PY'
import socket
import time
import rclpy
rclpy.init(args=[])
node = rclpy.create_node('ur5e_gazebo_start_check')
deadline = time.monotonic()+1.5
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
others = set(node.get_node_names())-{node.get_name()}
node.destroy_node()
rclpy.shutdown()
if others:
    raise SystemExit('ROS domain 44 is already in use; close its Gazebo/GUI first: '+', '.join(sorted(others)))
with socket.socket() as probe:
    if probe.connect_ex(('127.0.0.1', 11355)) == 0:
        raise SystemExit('Gazebo port 11355 is already in use; close that simulator first')
PY
exec ros2 launch ur5e_a3_grid stack_gazebo.launch.py "$@"
