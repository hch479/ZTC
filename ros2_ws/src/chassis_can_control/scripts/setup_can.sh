#!/usr/bin/env bash
set -euo pipefail

# Usage: sudo ./setup_can.sh [interface] [bitrate]
CAN_INTERFACE="${1:-can0}"
CAN_BITRATE="${2:-1000000}"

# Bringing an already-down interface down may return an error; it is harmless.
ip link set "${CAN_INTERFACE}" down 2>/dev/null || true
ip link set "${CAN_INTERFACE}" type can bitrate "${CAN_BITRATE}" restart-ms 100
ip link set "${CAN_INTERFACE}" txqueuelen 1000
ip link set "${CAN_INTERFACE}" up
ip -details -statistics link show "${CAN_INTERFACE}"

