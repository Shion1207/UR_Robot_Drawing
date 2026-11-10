#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv --system-site-packages .venv || {
    echo 'Thiếu venv. Cài sudo apt install python3-venv rồi thử lại.' >&2; exit 1;
  }
fi
.venv/bin/python -m pip install -r requirements.txt
mkdir -p src/assign7_bringup/models
export HF_HUB_DISABLE_XET=1
.venv/bin/python -c 'from faster_whisper.utils import download_model; download_model("base", output_dir="src/assign7_bringup/models/whisper-base")'
echo 'Đã cài dependency và model Whisper tiếng Việt. Chạy ./scripts/build.bash.'
