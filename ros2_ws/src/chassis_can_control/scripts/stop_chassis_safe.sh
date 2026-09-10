#!/usr/bin/env bash

# Request software disable if the ROS node exists, then take can0 down.
source /opt/ros/humble/setup.bash
if [ -f "${HOME}/chassis_project/ros2_ws/install/setup.bash" ]; then
    source "${HOME}/chassis_project/ros2_ws/install/setup.bash"
fi
set -u

timeout 3 ros2 service call /chassis/enable std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1 || true

sudo ip link set can0 down 2>/dev/null || true
echo "Software enable was cleared when available; can0 is down."
