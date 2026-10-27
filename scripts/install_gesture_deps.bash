#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
if [[ ! -x .venv-gesture/bin/python ]]; then
  python3 -m venv --system-site-packages .venv-gesture || {
    echo 'Cần python3-venv. Cài: sudo apt install python3-venv' >&2
    exit 1
  }
fi
.venv-gesture/bin/python -m pip install -r requirements-gesture.txt
# Ensure local cv2 shadows any system headless cv2 used by unrelated apps.
.venv-gesture/bin/python -m pip install --no-deps --force-reinstall opencv-contrib-python==4.11.0.86
mkdir -p src/assign4_bringup/models
model_path="${project_root}/src/assign4_bringup/models/gesture_recognizer.task"
if [[ ! -s "${model_path}" ]]; then
  curl --fail --location --retry 3 \
    'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task' \
    --output "${model_path}.download"
  mv "${model_path}.download" "${model_path}"
fi
.venv-gesture/bin/python -c 'import cv2, mediapipe, numpy; print("MediaPipe", mediapipe.__version__, "OpenCV", cv2.__version__, "NumPy", numpy.__version__)'
echo 'Đã cài MediaPipe + model trong workspace. Tiếp theo: ./scripts/build.bash'
