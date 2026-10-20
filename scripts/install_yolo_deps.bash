#!/usr/bin/env bash
set -euo pipefail

# NumPy 1.x tương thích với cv_bridge được phát hành cùng ROS 2 Humble.
python3 -m pip install --user \
  'numpy>=1.23.0,<2.0' \
  'opencv-python>=4.8.0,<4.12' \
  'ultralytics>=8.3.0,<9.0'

echo "Đã cài dependency YOLO tương thích với ROS 2 Humble/cv_bridge."

