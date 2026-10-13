#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash

action_name="$(
  ros2 action list -t |
    awk '$2 == "[irobot_create_msgs/action/Undock]" {print $1; exit}'
)"

if [[ -z "${action_name}" ]]; then
  echo "Undock action is not ready. Wait for Gazebo to finish starting and try again." >&2
  exit 1
fi

echo "Sending Undock goal on ${action_name}..."
ros2 action send_goal "${action_name}" irobot_create_msgs/action/Undock '{}'
