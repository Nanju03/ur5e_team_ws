#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
source "$UR5E_WS/scripts/gazebo_env.bash"
export PYTHONUNBUFFERED=1
exec python3 -m ur5e_block_bringup.gazebo_batch "$@"
