#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash
timeout 5s ros2 service call /emotion/stop std_srvs/srv/Trigger '{}'
