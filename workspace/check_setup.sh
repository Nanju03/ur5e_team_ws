#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
if [[ "${1:-}" == "--gui" ]]; then
  export QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1
  exec xvfb-run -a -s '-screen 0 1920x1080x24' \
    python3 "$UR5E_WS/scripts/check_setup.py" "$@"
fi
exec python3 "$UR5E_WS/scripts/check_setup.py" "$@"
