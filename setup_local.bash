#!/usr/bin/env bash
# Source from assign8; never source assign4's overlay or its Python venv.
_assign8_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_assign8_had_nounset=false
if [[ $- == *u* ]]; then _assign8_had_nounset=true; set +u; fi
source /opt/ros/humble/setup.bash
# Do not silently inherit another course/workspace's DDS domain from .bashrc.
export ROS_DOMAIN_ID="${ASSIGN8_ROS_DOMAIN_ID:-88}"
export IGN_PARTITION=assign8
export DEEPFACE_HOME="${_assign8_root}/models"
export TF_NUM_INTRAOP_THREADS=2
export TF_NUM_INTEROP_THREADS=1
export OMP_NUM_THREADS=2
export CUDA_VISIBLE_DEVICES=-1
export TF_CPP_MIN_LOG_LEVEL=2
export ROS_LOG_DIR="${_assign8_root}/log/ros"
mkdir -p "${ROS_LOG_DIR}"
export QT_QPA_PLATFORM=xcb
_assign8_vendor="${_assign8_root}/vendor/opt/ros/humble"
if [[ -d "${_assign8_vendor}" ]]; then
  export AMENT_PREFIX_PATH="${_assign8_vendor}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${_assign8_vendor}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${_assign8_vendor}/lib:${_assign8_vendor}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="${_assign8_vendor}/local/lib/python3.10/dist-packages:${_assign8_vendor}/lib/python3.10/site-packages:${PYTHONPATH:-}"
  export PATH="${_assign8_vendor}/bin:${PATH}"
  export QT_PLUGIN_PATH="${_assign8_root}/vendor/usr/lib/x86_64-linux-gnu/qt5/plugins:${QT_PLUGIN_PATH:-}"
  export QML2_IMPORT_PATH="${_assign8_root}/vendor/usr/lib/x86_64-linux-gnu/qt5/qml:${QML2_IMPORT_PATH:-}"
fi
if [[ -f "${_assign8_root}/install/setup.bash" ]]; then source "${_assign8_root}/install/setup.bash"; fi
unset _assign8_root _assign8_vendor
if [[ "${_assign8_had_nounset}" == true ]]; then unset _assign8_had_nounset; set -u; else unset _assign8_had_nounset; fi
