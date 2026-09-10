#!/usr/bin/env bash
# Load ROS before enabling nounset.

source /opt/ros/humble/setup.bash
if [ -f "${HOME}/chassis_project/ros2_ws/install/setup.bash" ]; then
    source "${HOME}/chassis_project/ros2_ws/install/setup.bash"
fi
set -u

# Request software disable before stopping this project's node.
timeout 3 ros2 service call /chassis/enable std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1 || true
pkill -f "chassis_can_control.*chassis_can_node" 2>/dev/null || true
echo "Software enable was cleared when available; serial chassis node stopped."
