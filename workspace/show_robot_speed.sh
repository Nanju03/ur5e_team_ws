#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
export ROS_DOMAIN_ID="${UR5E_SPEED_DOMAIN:-0}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 -m ur5e_block_bringup.speed_monitor "$@"
