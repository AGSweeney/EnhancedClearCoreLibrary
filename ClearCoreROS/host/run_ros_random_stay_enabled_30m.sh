#!/usr/bin/env bash
# 30-minute stay-enabled ROS random-move endurance (JTC). No mid-run disable/disconnect.
set -eo pipefail
export MAMBA_ROOT_PREFIX="${HOME}/micromamba"
eval "$("${HOME}/bin/micromamba" shell hook -s bash)"
set +u
micromamba activate ros_env
cd /mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/ros2_ws
source install/setup.bash
set +u

export CCROS_HOST=172.16.82.114
export CCROS_BATTERY_SECONDS="${CCROS_BATTERY_SECONDS:-1800}"
export CCROS_WS=/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/ros2_ws
export CCROS_LOG_ROOT=/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs
export CCROS_REPO=/mnt/d/CCDev/EnhancedClearCoreLibrary

exec python3 /mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/host/run_ros_random_stay_enabled_30m.py
