#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
exec ros2 launch ur5e_a3_grid stack_sim_gui.launch.py "$@"
