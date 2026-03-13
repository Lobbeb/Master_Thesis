#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEFAULT_BAG_ROOT="$WS_ROOT/bags/experiments"

BAG_INPUT=""
DRY_RUN=false
LOOP=false
RATE=""
TOPICS=()

usage() {
  cat <<EOF
Usage: $0 <bag> <topic1> [topic2 ...] [loop:=true|false] [rate:=1.0] [dry_run:=true|false]

Bag resolution:
- absolute bag path
- workspace-relative path
- path relative to bags/experiments/
- experiment run directory containing a bag/ subdirectory

Examples:
  ./run.sh monitor_bag warehouse/const_test_yolo_0313-015514 /coord/leader_estimate_status
  ./run.sh monitor_bag warehouse/depth_test_yolo_0313-015917 /coord/leader_estimate_status /coord/leader_distance_debug
  ./run.sh monitor_bag warehouse/depth_test_yolo_0313-015917/bag /dji0/follow/actual/distance_3d_m loop:=true
EOF
}

coerce_bool() {
  case "$1" in
    true|false)
      printf '%s\n' "$1"
      ;;
    *)
      echo "Invalid boolean value: $1" >&2
      exit 2
      ;;
  esac
}

resolve_existing_dir() {
  local path="$1"
  if [ -d "$path" ]; then
    readlink -f "$path"
    return 0
  fi
  return 1
}

bag_dir_from_candidate() {
  local candidate="$1"
  local resolved=""
  local mcaps=()

  if [ -f "$candidate" ] && [ "$(basename "$candidate")" = "metadata.yaml" ]; then
    candidate="$(dirname "$candidate")"
  fi

  if ! resolved="$(resolve_existing_dir "$candidate")"; then
    return 1
  fi

  if [ -d "$resolved/bag" ]; then
    resolved="$(readlink -f "$resolved/bag")"
  fi

  shopt -s nullglob
  mcaps=("$resolved"/bag_*.mcap)
  shopt -u nullglob
  if [ -f "$resolved/metadata.yaml" ] || [ "${#mcaps[@]}" -gt 0 ]; then
    printf '%s\n' "$resolved"
    return 0
  fi

  return 1
}

resolve_bag_dir() {
  local input="$1"
  local candidate=""
  local resolved=""
  local -a candidates=()

  if [[ "$input" = /* ]]; then
    candidates+=("$input")
  else
    candidates+=("$input" "$WS_ROOT/$input" "$DEFAULT_BAG_ROOT/$input")
  fi

  for candidate in "${candidates[@]}"; do
    if resolved="$(bag_dir_from_candidate "$candidate")"; then
      printf '%s\n' "$resolved"
      return 0
    fi
  done

  return 1
}

start_topic_echo() {
  local topic="$1"
  local prefix="[$topic] "

  (
    ros2 topic echo --no-daemon "$topic" 2>&1 | stdbuf -oL awk -v prefix="$prefix" '{ print prefix $0; fflush(); }'
  ) &
  ECHO_PIDS+=("$!")
}

cleanup() {
  local pid=""
  for pid in "${ECHO_PIDS[@]:-}"; do
    pkill -TERM -P "$pid" 2>/dev/null || true
    kill "$pid" 2>/dev/null || true
  done
}

if [ "$#" -eq 0 ]; then
  usage
  exit 1
fi

for arg in "$@"; do
  case "$arg" in
    -h|--help|help)
      usage
      exit 0
      ;;
    bag:=*)
      BAG_INPUT="${arg#bag:=}"
      ;;
    loop:=*)
      LOOP="$(coerce_bool "${arg#loop:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    rate:=*)
      RATE="${arg#rate:=}"
      ;;
    *)
      if [ -z "$BAG_INPUT" ]; then
        BAG_INPUT="$arg"
      else
        TOPICS+=("$arg")
      fi
      ;;
  esac
done

if [ -z "$BAG_INPUT" ]; then
  echo "Bag path is required." >&2
  usage >&2
  exit 2
fi

if [ "${#TOPICS[@]}" -eq 0 ]; then
  echo "At least one topic is required." >&2
  usage >&2
  exit 2
fi

if [ -n "$RATE" ]; then
  case "$RATE" in
    ''|*[!0-9.]*)
      echo "Invalid rate value: $RATE" >&2
      exit 2
      ;;
  esac
fi

if ! BAG_DIR="$(resolve_bag_dir "$BAG_INPUT")"; then
  echo "Could not resolve bag path: $BAG_INPUT" >&2
  echo "Tried relative to: $WS_ROOT and $DEFAULT_BAG_ROOT" >&2
  exit 1
fi

if [ "$DRY_RUN" = true ]; then
  echo "Bag dir: $BAG_DIR"
  echo "Loop: $LOOP"
  echo "Rate: ${RATE:-default}"
  echo "Topics: ${#TOPICS[@]}"
  printf '%s\n' "${TOPICS[@]}"
  exit 0
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ECHO_PIDS=()
trap cleanup EXIT INT TERM

echo "[monitor_bag] Bag: $BAG_DIR"
echo "[monitor_bag] Topics: ${#TOPICS[@]}"

for topic in "${TOPICS[@]}"; do
  start_topic_echo "$topic"
done

sleep 1

PLAY_CMD=(ros2 bag play "$BAG_DIR")
if [ "$LOOP" = true ]; then
  PLAY_CMD+=(--loop)
fi
if [ -n "$RATE" ]; then
  PLAY_CMD+=(--rate "$RATE")
fi
PLAY_CMD+=(--topics "${TOPICS[@]}")

"${PLAY_CMD[@]}"
