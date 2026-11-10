#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
python_cmd=python3
if [[ -x .venv/bin/python ]]; then python_cmd=.venv/bin/python; fi
exec "${python_cmd}" src/assign7_bringup/scripts/segment_map.py \
  --map src/assign7_bringup/maps/map.yaml \
  --seeds src/assign7_bringup/config/region_seeds.yaml \
  --output src/assign7_bringup/semantic "$@"
