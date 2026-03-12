#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 <topic> <log_file> [poll_interval_s]" >&2
  exit 2
fi

TOPIC="$1"
LOG_FILE="$2"
POLL_INTERVAL_S="${3:-0.5}"

# ROS setup scripts do not tolerate nounset reliably in this environment.
set +u
source /opt/ros/jazzy/setup.bash
source /home/ruben/halmstad_ws/install/setup.bash
set -u

mkdir -p "$(dirname "$LOG_FILE")"

echo "[wait_and_echo_topic] waiting for topic: $TOPIC" >&2
until ros2 topic list --no-daemon 2>/dev/null | grep -Fxq "$TOPIC"; do
  sleep "$POLL_INTERVAL_S"
done

echo "[wait_and_echo_topic] recording topic: $TOPIC -> $LOG_FILE" >&2
exec ros2 topic echo --no-daemon "$TOPIC" > "$LOG_FILE"
