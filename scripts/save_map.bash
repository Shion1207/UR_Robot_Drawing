#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash

map_prefix="${project_root}/src/assign4_bringup/maps/map"
ros2 run nav2_map_server map_saver_cli -f "${map_prefix}" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
echo "Saved ${map_prefix}.yaml and ${map_prefix}.pgm"

