#!/usr/bin/env bash
# Source from assign7; never source assign4's overlay or its Python venv.
_assign7_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_assign7_had_nounset=false
if [[ $- == *u* ]]; then _assign7_had_nounset=true; set +u; fi
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${_assign7_root}/log/ros"
mkdir -p "${ROS_LOG_DIR}"
export QT_QPA_PLATFORM=xcb
_assign7_vendor="${_assign7_root}/vendor/opt/ros/humble"
if [[ -d "${_assign7_vendor}" ]]; then
  export AMENT_PREFIX_PATH="${_assign7_vendor}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${_assign7_vendor}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${_assign7_vendor}/lib:${_assign7_vendor}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="${_assign7_vendor}/local/lib/python3.10/dist-packages:${_assign7_vendor}/lib/python3.10/site-packages:${PYTHONPATH:-}"
  export PATH="${_assign7_vendor}/bin:${PATH}"
  export QT_PLUGIN_PATH="${_assign7_root}/vendor/usr/lib/x86_64-linux-gnu/qt5/plugins:${QT_PLUGIN_PATH:-}"
  export QML2_IMPORT_PATH="${_assign7_root}/vendor/usr/lib/x86_64-linux-gnu/qt5/qml:${QML2_IMPORT_PATH:-}"
fi
if [[ -f "${_assign7_root}/install/setup.bash" ]]; then source "${_assign7_root}/install/setup.bash"; fi
unset _assign7_root _assign7_vendor
if [[ "${_assign7_had_nounset}" == true ]]; then unset _assign7_had_nounset; set -u; else unset _assign7_had_nounset; fi
