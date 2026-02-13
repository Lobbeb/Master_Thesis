#!/usr/bin/env bash
source "$(dirname "$0")/env.sh"
set -euo pipefail

QOS_FILE="$WS_ROOT/config/rosbag_qos.yaml"
if [ ! -f "$QOS_FILE" ]; then
  echo "ERROR: QoS override file missing: $QOS_FILE"
  exit 2
fi

if [ $# -lt 4 ]; then
  echo "Usage: $0 <run_id> <condition> <uav_name> <world> [--with-cameras]"
  exit 2
fi

RUN_ID="$1"
CONDITION="$2"
UAV="$3"
WORLD="$4"
WITH_CAMERAS="false"
if [ "${5:-}" = "--with-cameras" ]; then
  WITH_CAMERAS="true"
fi

RUN_ROOT="${RUN_ROOT:-$HOME/halmstad_ws/runs}"
RUN_DIR="$RUN_ROOT/$RUN_ID"
mkdir -p "$RUN_DIR"
rm -rf "$RUN_DIR/bag"

START_TS="$(date -Iseconds)"

pub_evt () {
  local s="$1"
  ros2 topic pub -1 /coord/events std_msgs/msg/String "{data: '$s'}" >/dev/null
  sleep 0.2
  ros2 topic pub -1 /coord/events std_msgs/msg/String "{data: '$s'}" >/dev/null
}

ros2 run lrs_halmstad contract_check "$WORLD" "$UAV" 10

TOPICS=(
  "/clock"
  "/a201_0000/platform/odom"
  "/a201_0000/platform/odom/filtered"
  "/a201_0000/platform/cmd_vel"
  "/a201_0000/tf"
  "/a201_0000/tf_static"
  "/coord/events"
)
if [ "$WITH_CAMERAS" = "true" ]; then
  TOPICS+=("/a201_0000/sensors/camera_0/color/image")
fi

cat > "$RUN_DIR/meta.yaml" << EOF
run_id: ${RUN_ID}
timestamp: ${START_TS}
condition: ${CONDITION}
world: ${WORLD}
uav_name: ${UAV}
with_cameras: ${WITH_CAMERAS}
mode: simul_follow
EOF

cleanup() {
  set +e
  if [ -n "${UGV_PID:-}" ]; then
    kill "$UGV_PID" 2>/dev/null || true
    wait "$UGV_PID" 2>/dev/null || true
  fi
  if [ -n "${BAG_PID:-}" ]; then
    kill "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[run_round_simul] Recording rosbag to $RUN_DIR/bag ..."
ros2 bag record -o "$RUN_DIR/bag" \
  --qos-profile-overrides-path "$QOS_FILE" \
  --topics "${TOPICS[@]}" >/dev/null 2>&1 &
BAG_PID=$!
sleep 1.0

pub_evt "ROUND_START"
pub_evt "SIMUL_PHASE_START"

# ---------- UGV background motion (20 segments/cycles) ----------
# Uses your existing experiment_defaults.yaml params but overrides cycles to 20.
python3 - << 'PY' &
import os, time, yaml, subprocess, signal
from pathlib import Path

defaults_path = Path(os.path.expanduser("~/halmstad_ws/src/lrs_halmstad/resource/experiment_defaults.yaml"))
defaults = yaml.safe_load(defaults_path.read_text()) if defaults_path.exists() else {}

speed_fwd = float(defaults.get("ugv_forward_speed", 0.5))
t_fwd = float(defaults.get("ugv_forward_time_s", 4.0))
speed_turn = float(defaults.get("ugv_turn_speed", 0.5))
t_turn = float(defaults.get("ugv_turn_time_s", 3.14))
cycles = 20

topic = "/a201_0000/platform/cmd_vel"
msg_fwd = "{twist: {linear: {x: " + str(speed_fwd) + ", y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
msg_turn = "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: " + str(speed_turn) + "}}}"

def run_for_seconds(msg: str, secs: float):
    p = subprocess.Popen(
        ["ros2", "topic", "pub", "-r", "10", topic, "geometry_msgs/msg/TwistStamped", msg],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(secs)
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=2.0)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

for _ in range(cycles):
    run_for_seconds(msg_fwd, t_fwd)
    run_for_seconds(msg_turn, t_turn)
PY
UGV_PID=$!

# ---------- UAV foreground follow (20 teleports) ----------
python3 ~/halmstad_ws/scripts/uav_follow_ugv_20.py \
  --world "$WORLD" \
  --uav "$UAV" \
  --steps 75 \
  --z 10.0 \
  --offset_x 0.0 \
  --offset_y 0.0 \
  --yaw_mode fixed \
  --yaw_deg 0.0 \
  --dt 2.0 \
  --log_csv "$RUN_DIR/uav_setpoints.csv"

# Wait for UGV background to finish
wait "$UGV_PID" 2>/dev/null || true

pub_evt "SIMUL_PHASE_END"
pub_evt "ROUND_END"

kill "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true

echo "[run_round_simul] Done. Bag info:"
ros2 bag info "$RUN_DIR/bag" | head -n 80 || true
