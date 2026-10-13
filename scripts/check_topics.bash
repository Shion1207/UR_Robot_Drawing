#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash

failed=0
for topic in /scan /odom /cmd_vel; do
  type="$(timeout 10s ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ -n "${type}" ]]; then
    echo "[OK] ${topic} (${type})"
  else
    echo "[MISSING] ${topic}"
    failed=1
  fi
done

if timeout 8s ros2 topic echo /scan --once >/dev/null 2>&1; then
  echo "[OK] /scan is publishing data"
else
  echo "[FAIL] /scan has no data"
  failed=1
fi

if timeout 8s ros2 topic echo /odom --once >/dev/null 2>&1; then
  echo "[OK] /odom is publishing data"
else
  echo "[FAIL] /odom has no data"
  failed=1
fi

exit "${failed}"

