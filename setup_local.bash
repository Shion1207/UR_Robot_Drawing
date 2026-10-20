#!/usr/bin/env bash
# Source this file from the assign4 root: source setup_local.bash

_assign4_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_LOG_DIR="${_assign4_root}/log/ros"
mkdir -p "${ROS_LOG_DIR}"
_assign4_restore_nounset=false
if [[ $- == *u* ]]; then
  _assign4_restore_nounset=true
  set +u
fi
source /opt/ros/humble/setup.bash

# Gazebo / RViz use vendored Qt plugins that do not provide the Wayland
# backend. Force XCB through XWayland to avoid a black Gazebo window.
export QT_QPA_PLATFORM=xcb

_assign4_vendor="${_assign4_root}/vendor/opt/ros/humble"
if [[ -d "${_assign4_vendor}" ]]; then
  export AMENT_PREFIX_PATH="${_assign4_vendor}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${_assign4_vendor}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${_assign4_vendor}/lib:${_assign4_vendor}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="${_assign4_vendor}/local/lib/python3.10/dist-packages:${_assign4_vendor}/lib/python3.10/site-packages:${PYTHONPATH:-}"
  export PATH="${_assign4_vendor}/bin:${PATH}"
  export QT_PLUGIN_PATH="${_assign4_root}/vendor/usr/lib/x86_64-linux-gnu/qt5/plugins:${QT_PLUGIN_PATH:-}"
  export QML2_IMPORT_PATH="${_assign4_root}/vendor/usr/lib/x86_64-linux-gnu/qt5/qml:${QML2_IMPORT_PATH:-}"
fi

if [[ -f "${_assign4_root}/install/setup.bash" ]]; then
  source "${_assign4_root}/install/setup.bash"
fi

unset _assign4_vendor
unset _assign4_root
if [[ "${_assign4_restore_nounset}" == true ]]; then
  unset _assign4_restore_nounset
  set -u
else
  unset _assign4_restore_nounset
fi

