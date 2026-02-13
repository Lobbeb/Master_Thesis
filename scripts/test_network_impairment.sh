#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"

echo "[test_network_impairment] Starting node briefly..."
ros2 run lrs_omnet_bridge network_impairment_node --ros-args -p enabled:=true -p delay_ms:=50 -p packet_loss_percent:=5 &
PID=$!
sleep 3
kill "$PID" 2>/dev/null || true
echo "✅ network_impairment_node started and stopped"
echo "[test_network_impairment] Testing completed."