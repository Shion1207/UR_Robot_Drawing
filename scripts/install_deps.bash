#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv --system-site-packages .venv
fi
.venv/bin/python -m pip install --timeout 180 --retries 5 -r requirements.txt
.venv/bin/python -m pip install --timeout 180 --retries 5 --no-deps deepface==0.0.93 tf-keras==2.15.1 retina-face==0.0.17
# system-site-packages is for ROS, not an invitation to reuse a headless cv2 build.
# Install these wheels inside this venv even if a global package satisfies pip.
.venv/bin/python -m pip install --ignore-installed --no-deps numpy==1.26.4 opencv-python==4.11.0.86
source setup_local.bash
.venv/bin/python -c 'from deepface import DeepFace; DeepFace.build_model("Emotion", task="facial_attribute"); print("Emotion model ready; no webcam opened.")'
echo 'Đã cài model. Chạy ./scripts/build.bash'
