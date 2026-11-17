#!/usr/bin/env bash
# Only removes the known simulated dock in our dedicated partition/world.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash
services="$(timeout 10s ign service -l)"
if ! grep -Fx '/world/gesture_arena/remove' <<< "${services}" >/dev/null; then
  echo 'Chưa thấy gesture_arena trong partition assign8. Chạy mô phỏng trước.' >&2; exit 1
fi
undock_result="$(timeout 150s ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}')"
printf '%s\n' "${undock_result}"
if [[ "${undock_result}" != *'status: SUCCEEDED'* ]]; then
  echo 'Undock chưa thành công, không xóa dock.' >&2; exit 1
fi
result="$(ign service -s /world/gesture_arena/remove --reqtype ignition.msgs.Entity \
  --reptype ignition.msgs.Boolean --timeout 5000 --req 'name: "standard_dock", type: MODEL')"
if [[ "${result}" != *'data: true'* ]]; then
  echo "Không dọn được dock: ${result}" >&2; exit 1
fi
# Limited to this flat simulation. Bumper/cliff stay enabled; rear scan still gates motion.
timeout 20s ros2 param set --no-daemon --spin-time 4 /motion_control safety_override backup_only
echo 'Đã nhả và dọn dock trong mô phỏng. Chạy scripts/run_emotions.bash để nhận diện thật.'
