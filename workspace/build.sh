#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
cd "$UR5E_WS"
export CMAKE_BUILD_PARALLEL_LEVEL=2
exec colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
