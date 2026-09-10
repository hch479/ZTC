#!/usr/bin/env bash
# Load ROS before enabling nounset because the Humble setup script reads optional variables.
set -eo pipefail

WORKSPACE="${HOME}/chassis_project/ros2_ws"
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

echo "Available stable serial device names:"
ls -l /dev/serial/by-id/ 2>/dev/null || true
echo "Starting serial chassis node with software motor enable OFF."
echo "Press Ctrl-C to stop. Do not enable until all wheels are safely lifted."
exec ros2 launch chassis_can_control chassis_serial.launch.py
