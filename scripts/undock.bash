#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash
timeout 90s ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
