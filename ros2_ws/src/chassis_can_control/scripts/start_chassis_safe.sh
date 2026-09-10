#!/usr/bin/env bash
set -eo pipefail

# Safe commissioning launcher for the RDK X5.
# This script configures classic CAN at 1 Mbit/s and starts the ROS 2 node.
# It does NOT call /chassis/enable, so motor commands remain disabled.

WORKSPACE="${HOME}/chassis_project/ros2_ws"

sudo bash "${WORKSPACE}/src/chassis_can_control/scripts/setup_can.sh" can0 1000000
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

echo "Starting chassis_can_node with software motor enable OFF."
echo "Press Ctrl-C to stop. Do not enable until the wheels are safely lifted."
exec ros2 launch chassis_can_control chassis_can.launch.py
