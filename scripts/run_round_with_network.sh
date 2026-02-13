#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/env.sh"

# Usage:
#   bash scripts/run_round_with_network.sh <run_id> <delay_ms> <loss_percent> [jitter_ms] [uav] [world] [--with-cameras]
if [ $# -lt 3 ]; then
  echo "Usage: $0 <run_id> <delay_ms> <loss_percent> [jitter_ms] [uav] [world] [--with-cameras]"
  exit 2
fi

RUN_ID="$1"
DELAY_MS="$2"
LOSS_PCT="$3"
JITTER_MS="${4:-10}"
UAV="${5:-dji0}"
WORLD="${6:-orchard}"
WITH_CAM="${7:-}"

# Start bridge: /coord/events_raw -> impaired -> /coord/events
ros2 run lrs_omnet_bridge ros_omnet_bridge \
  --ros-args \
  -p enabled:=true \
  -p input_topic:=/coord/events_raw \
  -p output_topic:=/coord/events \
  -p delay_ms:="$DELAY_MS" \
  -p delay_stddev_ms:="$JITTER_MS" \
  -p packet_loss_percent:="$LOSS_PCT" \
  >/tmp/ros_omnet_bridge_${RUN_ID}.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
  set +e
  kill "$BRIDGE_PID" 2>/dev/null || true
  wait "$BRIDGE_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Publish event markers to raw
export EVENT_TOPIC="/coord/events_raw"

# Run the normal harness
bash scripts/run_round.sh "$RUN_ID" "impaired" "$UAV" "$WORLD" "$WITH_CAM"

# Append network params to meta.yaml
META="${HOME}/halmstad_ws/runs/${RUN_ID}/meta.yaml"
if [ -f "$META" ]; then
  cat >> "$META" << EOF
network:
  enabled: true
  delay_ms: ${DELAY_MS}
  jitter_ms: ${JITTER_MS}
  loss_percent: ${LOSS_PCT}
event_topic_raw: /coord/events_raw
event_topic_impaired: /coord/events
EOF
fi
