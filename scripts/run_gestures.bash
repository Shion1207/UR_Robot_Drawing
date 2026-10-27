#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash
if [[ ! -x .venv-gesture/bin/python ]]; then
  echo 'Chạy ./scripts/install_gesture_deps.bash trước.' >&2
  exit 1
fi
source .venv-gesture/bin/activate
exec ros2 launch assign4_bringup gesture.launch.py "$@"
