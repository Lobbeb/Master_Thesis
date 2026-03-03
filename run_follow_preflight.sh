#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

is_truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

gazebo_world_name() {
  local world="$1"
  if [[ "$world" == "construction" ]]; then
    printf 'office_construction'
  else
    printf '%s' "$world"
  fi
}

has_service() {
  local service="$1"
  ros2 service list 2>/dev/null | grep -Fxq "$service"
}

normalize_timeout_s() {
  local raw="${1:-15}"
  if [[ "$raw" =~ ^[0-9]+$ ]]; then
    printf '%s' "$raw"
    return
  fi
  if [[ "$raw" =~ ^[0-9]+\.[0-9]+$ ]]; then
    printf '%s' "${raw%%.*}"
    return
  fi
  printf '15'
}

wait_for_service() {
  local service="$1"
  local timeout_s="$2"
  local timeout_int
  timeout_int="$(normalize_timeout_s "$timeout_s")"
  local t0
  t0="$(date +%s)"
  while true; do
    if has_service "$service"; then
      return 0
    fi
    if (( $(date +%s) - t0 >= timeout_int )); then
      return 1
    fi
    sleep 1
  done
}

has_topic_prefix() {
  local prefix="$1"
  ros2 topic list 2>/dev/null | awk -v p="$prefix" 'index($0, p) == 1 { found=1; exit } END { exit(found ? 0 : 1) }'
}

wait_for_topic_prefix() {
  local prefix="$1"
  local timeout_s="$2"
  local timeout_int
  timeout_int="$(normalize_timeout_s "$timeout_s")"
  local t0
  t0="$(date +%s)"
  while true; do
    if has_topic_prefix "$prefix"; then
      return 0
    fi
    if (( $(date +%s) - t0 >= timeout_int )); then
      return 1
    fi
    sleep 1
  done
}

has_publishers() {
  local topic="$1"
  local out
  out="$(ros2 topic info "$topic" 2>/dev/null || true)"
  if [[ -z "$out" ]]; then
    return 1
  fi
  local count
  count="$(printf '%s\n' "$out" | awk -F: '/Publisher count/ {gsub(/ /,"",$2); print $2; exit}')"
  [[ -n "$count" ]] && (( count > 0 ))
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local timeout_int
  timeout_int="$(normalize_timeout_s "$timeout_s")"
  local t0
  t0="$(date +%s)"
  while true; do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    if (( $(date +%s) - t0 >= timeout_int )); then
      return 1
    fi
    sleep 1
  done
}

check_mode_guard() {
  if ! is_truthy "$PREFLIGHT_MODE_GUARD_ENABLE"; then
    return 0
  fi
  if [[ ! -f "$MODE_FILE" ]]; then
    return 0
  fi

  local active_mode=""
  local active_pid=""
  read -r active_mode active_pid < "$MODE_FILE" || true
  if [[ -n "$active_pid" ]] && kill -0 "$active_pid" 2>/dev/null; then
    if [[ "$active_mode" == "camera_stack" ]]; then
      echo "[run_follow_preflight] mode conflict: camera_stack is active (pid=$active_pid). Stop it before follow run."
      exit 6
    fi
  else
    rm -f "$MODE_FILE"
  fi
}

ODOM_TOPIC="${1:-/a201_0000/platform/odom}"
CONTROLLER_MANAGER="${2:-/a201_0000/controller_manager}"
CONTROLLER_NAME="${3:-platform_velocity_controller}"
WORLD="${4:-orchard}"
UAV_NAME="${5:-dji0}"
UAV_BACKEND="${6:-setpose}"

PREFLIGHT_TIMEOUT_S="${PREFLIGHT_TIMEOUT_S:-15}"
PREFLIGHT_ACTIVATE_CONTROLLER="${PREFLIGHT_ACTIVATE_CONTROLLER:-true}"
PREFLIGHT_REQUIRE_CLOCK="${PREFLIGHT_REQUIRE_CLOCK:-true}"
PREFLIGHT_REQUIRE_SET_POSE_SERVICE="${PREFLIGHT_REQUIRE_SET_POSE_SERVICE:-true}"
PREFLIGHT_REQUIRE_UAV_NAMESPACE_TOPICS="${PREFLIGHT_REQUIRE_UAV_NAMESPACE_TOPICS:-true}"
PREFLIGHT_MODE_GUARD_ENABLE="${PREFLIGHT_MODE_GUARD_ENABLE:-true}"
PREFLIGHT_SET_POSE_TIMEOUT_S="${PREFLIGHT_SET_POSE_TIMEOUT_S:-$PREFLIGHT_TIMEOUT_S}"
PREFLIGHT_UAV_TOPICS_TIMEOUT_S="${PREFLIGHT_UAV_TOPICS_TIMEOUT_S:-$PREFLIGHT_TIMEOUT_S}"
PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS="${PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS:-true}"
PREFLIGHT_FORBID_CONTROLLER_TOPIC_PUBLISHERS="${PREFLIGHT_FORBID_CONTROLLER_TOPIC_PUBLISHERS:-true}"
PREFLIGHT_FORBID_INTENT_TOPIC_PUBLISHERS="${PREFLIGHT_FORBID_INTENT_TOPIC_PUBLISHERS:-true}"
STATE_DIR="${STATE_DIR:-/tmp/halmstad_ws}"
MODE_FILE="$STATE_DIR/run_mode.lock"

case "$UAV_BACKEND" in
  setpose|controller) ;;
  *)
    echo "[run_follow_preflight] invalid backend '$UAV_BACKEND' (expected setpose|controller)"
    exit 9
    ;;
esac

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  source "$WS_ROOT/install/setup.bash"
fi
if [[ -f "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash" ]]; then
  source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
fi
set -u

check_mode_guard

echo "[run_follow_preflight] backend=$UAV_BACKEND"
echo "[run_follow_preflight] controller_manager=$CONTROLLER_MANAGER"
if ! timeout 8s ros2 control list_controllers -c "$CONTROLLER_MANAGER" >/dev/null 2>&1; then
  echo "[run_follow_preflight] controller manager unavailable: $CONTROLLER_MANAGER"
  exit 2
fi

if is_truthy "$PREFLIGHT_ACTIVATE_CONTROLLER"; then
  echo "[run_follow_preflight] activating controller: $CONTROLLER_NAME"
  ros2 control set_controller_state "$CONTROLLER_NAME" active -c "$CONTROLLER_MANAGER" >/dev/null 2>&1 || true
fi

if ! ros2 control list_controllers -c "$CONTROLLER_MANAGER" 2>/dev/null | grep -E -q "${CONTROLLER_NAME}.*active"; then
  echo "[run_follow_preflight] controller not active: $CONTROLLER_NAME"
  ros2 control list_controllers -c "$CONTROLLER_MANAGER" || true
  exit 3
fi

if is_truthy "$PREFLIGHT_REQUIRE_CLOCK"; then
  echo "[run_follow_preflight] waiting for /clock"
  if ! timeout "${PREFLIGHT_TIMEOUT_S}s" ros2 topic echo --once /clock >/dev/null 2>&1; then
    echo "[run_follow_preflight] /clock did not publish within ${PREFLIGHT_TIMEOUT_S}s"
    exit 4
  fi
fi

echo "[run_follow_preflight] waiting for odom topic: $ODOM_TOPIC"
if ! timeout "${PREFLIGHT_TIMEOUT_S}s" ros2 topic echo --once "$ODOM_TOPIC" >/dev/null 2>&1; then
  echo "[run_follow_preflight] odom did not publish within ${PREFLIGHT_TIMEOUT_S}s on $ODOM_TOPIC"
  exit 5
fi

if is_truthy "$PREFLIGHT_REQUIRE_SET_POSE_SERVICE"; then
  GZ_WORLD="$(gazebo_world_name "$WORLD")"
  SET_POSE_SERVICE="/world/${GZ_WORLD}/set_pose"
  echo "[run_follow_preflight] waiting for service: $SET_POSE_SERVICE"
  if ! wait_for_service "$SET_POSE_SERVICE" "$PREFLIGHT_SET_POSE_TIMEOUT_S"; then
    echo "[run_follow_preflight] set_pose service not available within ${PREFLIGHT_SET_POSE_TIMEOUT_S}s: $SET_POSE_SERVICE"
    exit 7
  fi
fi

if is_truthy "$PREFLIGHT_REQUIRE_UAV_NAMESPACE_TOPICS"; then
  UAV_PREFIX="/${UAV_NAME}/"
  echo "[run_follow_preflight] waiting for UAV namespace topics: $UAV_PREFIX"
  if ! wait_for_topic_prefix "$UAV_PREFIX" "$PREFLIGHT_UAV_TOPICS_TIMEOUT_S"; then
    echo "[run_follow_preflight] no topics found for ${UAV_PREFIX} within ${PREFLIGHT_UAV_TOPICS_TIMEOUT_S}s"
    echo "[run_follow_preflight] this usually means UAV name mismatch or UAV spawn missing"
    exit 8
  fi
fi

if is_truthy "$PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS"; then
  CAMERA_TOPIC="/${UAV_NAME}/camera0/image_raw"
  CAMERA_INFO_TOPIC="/${UAV_NAME}/camera0/camera_info"
  echo "[run_follow_preflight] waiting for UAV camera topics: $CAMERA_TOPIC and $CAMERA_INFO_TOPIC"
  if ! wait_for_topic "$CAMERA_TOPIC" "$PREFLIGHT_UAV_TOPICS_TIMEOUT_S"; then
    echo "[run_follow_preflight] missing topic within ${PREFLIGHT_UAV_TOPICS_TIMEOUT_S}s: $CAMERA_TOPIC"
    exit 10
  fi
  if ! wait_for_topic "$CAMERA_INFO_TOPIC" "$PREFLIGHT_UAV_TOPICS_TIMEOUT_S"; then
    echo "[run_follow_preflight] missing topic within ${PREFLIGHT_UAV_TOPICS_TIMEOUT_S}s: $CAMERA_INFO_TOPIC"
    exit 11
  fi
fi

if [[ "$UAV_BACKEND" == "controller" ]] && is_truthy "$PREFLIGHT_FORBID_CONTROLLER_TOPIC_PUBLISHERS"; then
  CTRL_TOPICS=(
    "/${UAV_NAME}/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
    "/${UAV_NAME}/update_pan"
    "/${UAV_NAME}/update_tilt"
  )
  CTRL_CONFLICTS=()
  for _topic in "${CTRL_TOPICS[@]}"; do
    if has_publishers "$_topic"; then
      CTRL_CONFLICTS+=("$_topic")
    fi
  done
  if [[ "${#CTRL_CONFLICTS[@]}" -gt 0 ]]; then
    echo "[run_follow_preflight] controller backend conflict: command topics already have publishers:"
    for _topic in "${CTRL_CONFLICTS[@]}"; do
      echo "  - $_topic"
    done
    echo "[run_follow_preflight] stop any external UAV controller before running follow controller backend"
    exit 12
  fi
fi

if is_truthy "$PREFLIGHT_FORBID_INTENT_TOPIC_PUBLISHERS"; then
  INTENT_TOPIC="/${UAV_NAME}/pose_cmd"
  if has_publishers "$INTENT_TOPIC"; then
    echo "[run_follow_preflight] intent conflict: topic already has publishers: $INTENT_TOPIC"
    echo "[run_follow_preflight] stop any previous follow/manual intent publisher before starting this run"
    exit 13
  fi
fi

echo "[run_follow_preflight] ok"
