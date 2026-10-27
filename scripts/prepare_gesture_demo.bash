#!/usr/bin/env bash
# Preparation only, before recording: release robot and remove dock collision.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
source setup_local.bash
# grep is included with Ubuntu; ripgrep (rg) is not a runtime dependency.
# Capture the list before matching so pipefail cannot treat an early matcher
# exit as a failed Gazebo service query.
service_list="$(ign service -l)"
if ! grep -Fx '/world/gesture_arena/remove' <<< "${service_list}" >/dev/null; then
  echo 'Chỉ dùng với world gesture_arena đã khởi động xong.' >&2
  exit 1
fi
echo 'Đang nhả robot khỏi dock; chờ action hoàn tất...'
undock_result="$(./scripts/undock.bash)"
printf '%s\n' "${undock_result}"
if [[ "${undock_result}" != *'status: SUCCEEDED'* ]]; then
  echo 'Undock chưa thành công, giữ nguyên dock. Kiểm tra mô phỏng rồi thử lại.' >&2
  exit 1
fi
# The stock simulator always spawns a dock facing the robot. Remove only that
# model in our dedicated arena, otherwise the first forward segment meets it.
result="$(ign service -s /world/gesture_arena/remove \
  --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean \
  --timeout 5000 --req 'name: "standard_dock", type: MODEL')"
if [[ "${result}" != *'data: true'* ]]; then
  echo "Không xóa được dock trong gesture_arena: ${result}" >&2
  exit 1
fi
# Create 3 keeps a reverse-travel safety budget, including travel during Undock.
# Only the backup budget is overridden, exclusively in this verified flat sim
# world; cliff/bumper safety stays enabled and our rear LiDAR still gates /cmd_vel.
ros2 param set /motion_control safety_override backup_only
echo 'Đã nhả và dọn dock. Chờ LiDAR cập nhật vài giây, rồi chạy run_gestures.bash.'
