#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 {simulation|mapping|localization|navigation} [launch arguments...]" >&2
  exit 2
fi

mode="$1"
shift
case "${mode}" in
  simulation|mapping|localization|navigation) ;;
  *) echo "Unknown mode: ${mode}" >&2; exit 2 ;;
esac

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash

if ! ros2 pkg prefix assign4_bringup >/dev/null 2>&1; then
  echo "Workspace is not built. Run ./scripts/build.bash first." >&2
  exit 1
fi

exec ros2 launch assign4_bringup assign4.launch.py mode:="${mode}" "$@"

