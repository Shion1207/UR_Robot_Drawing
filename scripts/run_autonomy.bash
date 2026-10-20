#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash

if ! ros2 pkg prefix assign4_bringup >/dev/null 2>&1; then
  echo "Workspace chưa được build. Hãy chạy ./scripts/build.bash trước." >&2
  exit 1
fi

# Tận dụng weight đã có trên máy; nếu không có, Ultralytics sẽ dùng/tải tên
# model mặc định trong launch file. Đối số model_path:=... của người dùng luôn
# được ưu tiên.
model_argument_provided=false
for argument in "$@"; do
  if [[ "${argument}" == model_path:=* ]]; then
    model_argument_provided=true
    break
  fi
done

local_weight="${HOME}/yolo_project/yolov8n.pt"
if [[ "${model_argument_provided}" == false && -f "${local_weight}" ]]; then
  set -- "model_path:=${local_weight}" "$@"
fi

exec ros2 launch assign4_bringup autonomy.launch.py "$@"

