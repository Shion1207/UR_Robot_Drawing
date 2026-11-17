#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash
if [[ ! -x .venv/bin/python ]]; then
  echo 'Chạy scripts/install_deps.bash trước.' >&2; exit 1
fi
if ! .venv/bin/python -c 'import importlib.util,sys; sys.exit(0 if all(importlib.util.find_spec(n) for n in ("deepface","tensorflow")) else 1)'; then
  echo 'Dependency chưa cài đủ. Chạy ./scripts/install_deps.bash khi mạng ổn định.' >&2
  exit 1
fi
if [[ ! -s "${DEEPFACE_HOME}/.deepface/weights/facial_expression_model_weights.h5" ]]; then
  echo 'Thiếu weights Emotion. Chạy ./scripts/install_deps.bash để tải model trước.' >&2
  exit 1
fi
source .venv/bin/activate
exec ros2 launch assign8_bringup emotion.launch.py "$@"
