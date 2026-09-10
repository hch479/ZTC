#!/usr/bin/env bash
set -euo pipefail

# Read-only CAN commissioning test. No CAN frames are transmitted.
WORKSPACE="${HOME}/chassis_project/ros2_ws"

sudo bash "${WORKSPACE}/src/chassis_can_control/scripts/setup_can.sh" can0 1000000

echo
echo "Listening for five seconds. Expected STM32 IDs include 185 and 700."
set +e
timeout 5 candump -L can0
RESULT=$?
set -e

echo
ip -details -statistics link show can0

if [ "${RESULT}" -eq 124 ]; then
    echo
    echo "No frame arrived. Check GND-GND, CAN_L-CAN_L, CAN_H-CAN_H,"
    echo "C30D power, and the two 120-ohm terminations."
fi

sudo ip link set can0 down
