#!/usr/bin/env bash
# Wait for the bring-up battery to finish, then start stay-enabled random ROS moves.
set -eo pipefail
LOG=/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs/battery_20261004_202712
echo "waiting for $LOG/summary.json"
while [ ! -f "$LOG/summary.json" ]; do
  if ! pgrep -f "run_ros_battery_30m.py" >/dev/null 2>&1; then
    echo "battery process gone; waiting for summary"
  fi
  sleep 10
done
echo "bring-up battery finished; starting stay-enabled random 30m"
sleep 2
exec /mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/host/run_ros_random_stay_enabled_30m.sh
