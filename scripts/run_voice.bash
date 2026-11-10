#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash
if [[ ! -x .venv/bin/python ]]; then
  echo 'Chạy ./scripts/install_deps.bash và ./scripts/build.bash trước.' >&2; exit 1
fi
source .venv/bin/activate
exec ros2 launch assign7_bringup voice.launch.py "$@"
