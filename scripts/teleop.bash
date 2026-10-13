#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash
exec ros2 run assign4_bringup wasd_teleop.py

