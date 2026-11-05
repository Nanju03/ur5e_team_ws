#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
cd "$UR5E_WS"
python3 -c 'import tkinter, yaml, numpy, rclpy, ament_package, ament_index_python, setuptools; print("Python dependencies: OK")'
# ament_python is provided by the existing colcon/setuptools build support.
# MongoDB is optional and is used only by warehouse_db.launch.py, not this simulator.
exec rosdep check --from-paths src --ignore-src --rosdistro humble \
  --skip-keys 'ament_python warehouse_ros_mongo'
