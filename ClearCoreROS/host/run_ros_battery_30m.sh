#!/usr/bin/env bash
# Launch the 30-minute ClearCoreROS ROS hardware battery with detailed logging.
set -eo pipefail
export MAMBA_ROOT_PREFIX="${HOME}/micromamba"
eval "$("${HOME}/bin/micromamba" shell hook -s bash)"
set +u
micromamba activate ros_env
cd /mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/ros2_ws
colcon build --packages-select clearcore_bridge clearcore_hardware --event-handlers console_cohesion+
source install/setup.bash
set +u

# Do not match this launcher script itself.
pkill -f 'ros2 launch clearcore_bridge|ros2 launch clearcore_hardware|ros2_control_node|robot_state_publisher|/spawner|send_trajectory_goal.py' 2>/dev/null || true
sleep 1

export CCROS_HOST=172.16.82.114
export CCROS_BATTERY_SECONDS="${CCROS_BATTERY_SECONDS:-1800}"
export CCROS_WS=/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/ros2_ws
export CCROS_LOG_ROOT=/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs
export CCROS_REPO=/mnt/d/CCDev/EnhancedClearCoreLibrary

python3 - <<'PY'
import json, socket
s=socket.create_connection(("172.16.82.114",9200),2)
s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"disable"}\n')
print(s.recv(4096).decode())
s.close()
PY

exec python3 /mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/host/run_ros_battery_30m.py
