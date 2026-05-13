#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
CONFIG_ROOT="$WS_ROOT/src/lrs_halmstad/config"
RVIZ_CONFIG_DIR="$CONFIG_ROOT/rviz_configs"
BASE_RVIZ_CONFIG="$RVIZ_CONFIG_DIR/waypoints_testing.rviz"
RVIZ_CONFIG_OVERRIDE=""
RVIZ_SOFTWARE_RENDERING="${RVIZ_SOFTWARE_RENDERING:-auto}"

source "$SCRIPT_DIR/lidar_mode_common.sh"

lidar_mode_parse_args 3d "$@"

if [ "$LIDAR_MODE" = "3d" ] && [ "$LIDAR_SCAN_TOPIC" = "$(lidar_mode_scan_topic 3d)" ]; then
  echo "[run_nav2_rviz] 3D lidar mode: using direct pc2ls topic $LIDAR_SCAN_TOPIC" >&2
elif [ -f "$SIM_WORLD_FILE" ] && [[ "$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)" == baylands* ]] && [ "$LIDAR_MODE" = "2d" ] && [ "$LIDAR_SCAN_TOPIC" = "$(lidar_mode_scan_topic 2d)" ]; then
  LIDAR_SCAN_TOPIC="${LIDAR_SCAN_TOPIC}_relay"
  echo "[run_nav2_rviz] Baylands 2D lidar mode: using localization-owned relay topic $LIDAR_SCAN_TOPIC" >&2
fi

RVIZ_PASSTHROUGH_ARGS=()
for arg in "${LIDAR_REMAINING_ARGS[@]}"; do
  case "$arg" in
    rviz_config:=*)
      RVIZ_CONFIG_OVERRIDE="${arg#rviz_config:=}"
      ;;
    config:=*)
      RVIZ_CONFIG_OVERRIDE="${arg#config:=}"
      ;;
    rviz_software_rendering:=*|software_rendering:=*)
      RVIZ_SOFTWARE_RENDERING="${arg#*:=}"
      ;;
    *)
      RVIZ_PASSTHROUGH_ARGS+=("$arg")
      ;;
  esac
done
LIDAR_REMAINING_ARGS=("${RVIZ_PASSTHROUGH_ARGS[@]}")

if [ -n "$RVIZ_CONFIG_OVERRIDE" ]; then
  RVIZ_CONFIG_CANDIDATES=()

  add_rviz_config_candidate() {
    local candidate="$1"
    local existing

    [ -n "$candidate" ] || return 0
    for existing in "${RVIZ_CONFIG_CANDIDATES[@]}"; do
      [ "$existing" = "$candidate" ] && return 0
    done
    RVIZ_CONFIG_CANDIDATES+=("$candidate")
  }

  add_rviz_config_variants() {
    local base="$1"

    add_rviz_config_candidate "$base"
    if [[ "$base" != *.rviz ]]; then
      add_rviz_config_candidate "$base.rviz"
    fi
  }

  case "$RVIZ_CONFIG_OVERRIDE" in
    "~")
      RVIZ_CONFIG_OVERRIDE="$HOME"
      ;;
    "~/"*)
      RVIZ_CONFIG_OVERRIDE="$HOME/${RVIZ_CONFIG_OVERRIDE#~/}"
      ;;
  esac

  if [[ "$RVIZ_CONFIG_OVERRIDE" = /* ]]; then
    add_rviz_config_variants "$RVIZ_CONFIG_OVERRIDE"
  else
    add_rviz_config_variants "$RVIZ_CONFIG_DIR/$RVIZ_CONFIG_OVERRIDE"
    add_rviz_config_variants "$CONFIG_ROOT/$RVIZ_CONFIG_OVERRIDE"
    add_rviz_config_variants "$WS_ROOT/$RVIZ_CONFIG_OVERRIDE"
    add_rviz_config_variants "$PWD/$RVIZ_CONFIG_OVERRIDE"
    add_rviz_config_variants "$RVIZ_CONFIG_OVERRIDE"
  fi

  for candidate in "${RVIZ_CONFIG_CANDIDATES[@]}"; do
    if [ -f "$candidate" ]; then
      if [[ "$candidate" != *.rviz ]]; then
        echo "[run_nav2_rviz] Found file, but RViz display configs must be .rviz files: $candidate" >&2
        exit 2
      fi
      BASE_RVIZ_CONFIG="$candidate"
      break
    fi
  done

  if [ ! -f "$BASE_RVIZ_CONFIG" ]; then
    echo "[run_nav2_rviz] RViz config not found: $RVIZ_CONFIG_OVERRIDE" >&2
    echo "[run_nav2_rviz] Checked:" >&2
    printf '  - %s\n' "${RVIZ_CONFIG_CANDIDATES[@]}" >&2
    exit 2
  fi
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

RVIZ_VIEW_X=""
RVIZ_VIEW_Y=""
AMCL_TMP="$(mktemp)"
AMCL_ERR_TMP="$(mktemp)"
if timeout 2s ros2 topic echo --no-daemon --once /a201_0000/amcl_pose >"$AMCL_TMP" 2>"$AMCL_ERR_TMP"; then
  if AMCL_VIEW_ENV="$(python3 - "$AMCL_TMP" <<'PY'
import sys
from pathlib import Path
import yaml

text = Path(sys.argv[1]).read_text(encoding="utf-8")
docs = [doc for doc in yaml.safe_load_all(text) if isinstance(doc, dict)]
if not docs:
    raise SystemExit(1)

msg = None
for doc in reversed(docs):
    pose = (((doc.get("pose") or {}).get("pose")) or {})
    position = pose.get("position") or {}
    if "x" in position and "y" in position:
        msg = doc
        break

if msg is None:
    raise SystemExit(1)

pose = (((msg.get("pose") or {}).get("pose")) or {})
position = pose.get("position") or {}
x = float(position["x"])
y = float(position["y"])
print(f"RVIZ_VIEW_X={x!r}")
print(f"RVIZ_VIEW_Y={y!r}")
PY
)"; then
    eval "$AMCL_VIEW_ENV"
    echo "[run_nav2_rviz] Centering RViz view on AMCL pose: x=$RVIZ_VIEW_X y=$RVIZ_VIEW_Y"
  fi
fi
rm -f "$AMCL_TMP" "$AMCL_ERR_TMP"

RVIZ_CONFIG="$BASE_RVIZ_CONFIG"
if [ "$LIDAR_MODE" = "2d" ] || [ "$LIDAR_SCAN_TOPIC" != "$(lidar_mode_raw_scan_topic 3d)" ] || [ -n "$RVIZ_VIEW_X" ] || [ -n "$RVIZ_VIEW_Y" ]; then
  RVIZ_SCAN_TOPIC="$LIDAR_SCAN_TOPIC"
  if [[ "$RVIZ_SCAN_TOPIC" == /a201_0000/* ]]; then
    RVIZ_SCAN_TOPIC="<robot_namespace>/${RVIZ_SCAN_TOPIC#/a201_0000/}"
  fi
  mkdir -p /tmp/halmstad_ws
  RVIZ_CONFIG_STEM="$(basename "$BASE_RVIZ_CONFIG")"
  RVIZ_CONFIG_STEM="${RVIZ_CONFIG_STEM%.rviz}"
  RVIZ_CONFIG="/tmp/halmstad_ws/${RVIZ_CONFIG_STEM}.$(echo "$LIDAR_MODE" | tr -cd '[:alnum:]').rviz"
  RVIZ_SCAN_TOPIC="$RVIZ_SCAN_TOPIC" RVIZ_VIEW_X="$RVIZ_VIEW_X" RVIZ_VIEW_Y="$RVIZ_VIEW_Y" \
    python3 - "$BASE_RVIZ_CONFIG" "$RVIZ_CONFIG" <<'PY'
import os
import re
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
text = src.read_text(encoding="utf-8")

scan_topic = os.environ.get("RVIZ_SCAN_TOPIC", "")
if scan_topic:
    text = text.replace("<robot_namespace>/sensors/lidar2d_0/scan", scan_topic)
    text = text.replace("<robot_namespace>/sensors/lidar3d_0/scan_from_points", scan_topic)
    text = text.replace("<robot_namespace>/sensors/lidar3d_0/scan", scan_topic)

view_x = os.environ.get("RVIZ_VIEW_X", "").strip()
view_y = os.environ.get("RVIZ_VIEW_Y", "").strip()
if view_x and view_y:
    marker = "      Name: Current View\n"
    start = text.find(marker)
    if start != -1:
        start += len(marker)
        end = text.find("    Saved: ~", start)
        if end != -1:
            block = text[start:end]
            block, x_count = re.subn(r"(?m)^      X: .*$", f"      X: {view_x}", block, count=1)
            block, y_count = re.subn(r"(?m)^      Y: .*$", f"      Y: {view_y}", block, count=1)
            if x_count and y_count:
                text = text[:start] + block + text[end:]

dst.write_text(text, encoding="utf-8")
PY
fi

if [ -n "${WSL_INTEROP:-}" ] || grep -qi microsoft /proc/version 2>/dev/null; then
  if command -v xdpyinfo >/dev/null 2>&1 && xdpyinfo -display "${DISPLAY:-:0}" >/dev/null 2>&1; then
    export QT_QPA_PLATFORM=xcb
    unset WAYLAND_DISPLAY
    echo "[run_nav2_rviz] WSL detected: using QT_QPA_PLATFORM=xcb"
  else
    echo "[run_nav2_rviz] Warning: X11 display '${DISPLAY:-:0}' is not reachable. WSLg may need a reset with 'wsl --shutdown'." >&2
  fi

  if [ "$RVIZ_SOFTWARE_RENDERING" = "auto" ] || [ "$RVIZ_SOFTWARE_RENDERING" = "true" ]; then
    if [ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
      export LIBGL_ALWAYS_SOFTWARE=1
      echo "[run_nav2_rviz] WSL detected: using LIBGL_ALWAYS_SOFTWARE=1 for RViz map rendering"
    fi
  fi
elif [ "$RVIZ_SOFTWARE_RENDERING" = "true" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  echo "[run_nav2_rviz] Forced software rendering: LIBGL_ALWAYS_SOFTWARE=1"
fi

cd "$CONFIG_ROOT"
echo "[run_nav2_rviz] Using RViz config: $RVIZ_CONFIG"

ros2 launch nav2_bringup rviz_launch.py \
  namespace:=a201_0000 \
  use_namespace:=true \
  use_sim_time:=true \
  rviz_config:="$RVIZ_CONFIG" \
  "${LIDAR_REMAINING_ARGS[@]}"
