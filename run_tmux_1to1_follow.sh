#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
TMUX_STATE_DIR="$STATE_DIR/tmux_sessions"
WORLD="warehouse"
SESSION="halmstad-1to1"
MAP_PATH=""
GUI=""
ATTACH=true
DRY_RUN=false
LAYOUT="panes"
BASE_DELAY_S=9
BASE_DELAY_SET=false
COMMON_ARGS=()

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  SESSION="halmstad-${WORLD}-1to1"
  shift
fi

for arg in "$@"; do
  case "$arg" in
    session:=*)
      SESSION="${arg#session:=}"
      ;;
    map:=*)
      MAP_PATH="${arg#map:=}"
      ;;
    gui:=*)
      GUI="${arg#gui:=}"
      ;;
    attach:=*)
      ATTACH="${arg#attach:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    layout:=*)
      LAYOUT="${arg#layout:=}"
      ;;
    panes:=*)
      case "${arg#panes:=}" in
        true|yes|1)
          LAYOUT="panes"
          ;;
        false|no|0)
          LAYOUT="windows"
          ;;
        *)
          echo "Invalid panes option: ${arg#panes:=}" >&2
          echo "Use panes:=true or panes:=false" >&2
          exit 2
          ;;
      esac
      ;;
    delay_s:=*|spawn_delay_s:=*)
      BASE_DELAY_S="${arg#*:=}"
      BASE_DELAY_SET=true
      ;;
    localization_delay_s:=*|nav2_delay_s:=*|follow_delay_s:=*)
      echo "Use delay_s:=... instead of per-step delay overrides." >&2
      ;;
    camera:=*|height:=*|mount_pitch_deg:=*|uav_name:=*)
      COMMON_ARGS+=("$arg")
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: $0 [world] [camera:=detached] [height:=7] [mount_pitch_deg:=45] [uav_name:=dji0] [map:=/path/map.yaml] [gui:=true|false] [delay_s:=9] [session:=name] [attach:=true|false] [dry_run:=true|false] [layout:=windows|panes]" >&2
      exit 2
      ;;
  esac
done

if [ -z "$GUI" ]; then
  EFFECTIVE_GUI=true
else
  case "$GUI" in
    true|false)
      EFFECTIVE_GUI="$GUI"
      ;;
    *)
      echo "Invalid gui option: $GUI" >&2
      echo "Use gui:=true or gui:=false" >&2
      exit 2
      ;;
  esac
fi

case "$LAYOUT" in
  windows|panes)
    ;;
  *)
    echo "Invalid layout: $LAYOUT" >&2
    echo "Use layout:=windows or layout:=panes" >&2
    exit 2
    ;;
esac

SESSION_SAFE="$(printf '%s' "$SESSION" | tr -c 'A-Za-z0-9_.-' '_')"
SESSION_STATE_FILE="$TMUX_STATE_DIR/${SESSION_SAFE}.env"

apply_default_delays() {
  if [ "$EFFECTIVE_GUI" = false ]; then
    [ "$BASE_DELAY_SET" = true ] || BASE_DELAY_S=9
  else
    [ "$BASE_DELAY_SET" = true ] || BASE_DELAY_S=7
  fi

  SPAWN_DELAY_S="$BASE_DELAY_S"
  LOCALIZATION_DELAY_S=$((BASE_DELAY_S + 1))
  NAV2_DELAY_S="$LOCALIZATION_DELAY_S"
  FOLLOW_DELAY_S="$LOCALIZATION_DELAY_S"
}

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

build_line() {
  local delay_s="$1"
  local wait_for_sim="$2"
  shift 2
  local line=""
  printf -v line 'cd %q && ' "$WS_ROOT"
  if [ "$wait_for_sim" = true ]; then
    printf -v line '%swhile [ ! -f %q ]; do sleep 1; done && ' "$line" "$SIM_PID_FILE"
  fi
  if [ "$delay_s" != "0" ] && [ "$delay_s" != "0.0" ]; then
    printf -v line '%ssleep %q && ' "$line" "$delay_s"
  fi
  printf -v line '%s%s' "$line" "$(shell_join "$@")"
  printf '%s' "$line"
}

write_session_state() {
  mkdir -p "$TMUX_STATE_DIR"
  {
    printf 'SESSION=%q\n' "$SESSION"
    printf 'LAYOUT=%q\n' "$LAYOUT"
    printf 'EFFECTIVE_GUI=%q\n' "$EFFECTIVE_GUI"
    printf 'GAZEBO_PANE_ID=%q\n' "$gazebo_pane"
    printf 'SPAWN_PANE_ID=%q\n' "$spawn_pane"
    printf 'LOCALIZATION_PANE_ID=%q\n' "$localization_pane"
    printf 'NAV2_PANE_ID=%q\n' "$nav2_pane"
    printf 'FOLLOW_PANE_ID=%q\n' "$follow_pane"
  } > "$SESSION_STATE_FILE"
}

apply_default_delays

GAZEBO_CMD=(./run_gazebo_sim.sh "$WORLD")
if [ -n "$GUI" ]; then
  GAZEBO_CMD+=("$GUI")
fi

SPAWN_CMD=(./run_spawn_uav.sh "$WORLD" "${COMMON_ARGS[@]}")
LOCALIZATION_CMD=(./run_localization.sh "$WORLD")
if [ -n "$MAP_PATH" ]; then
  LOCALIZATION_CMD+=("$MAP_PATH")
fi
NAV2_CMD=(./run_nav2.sh)
FOLLOW_CMD=(./run_1to1_follow.sh "$WORLD" "${COMMON_ARGS[@]}")

GAZEBO_LINE="$(build_line 0 false "${GAZEBO_CMD[@]}")"
SPAWN_LINE="$(build_line "$SPAWN_DELAY_S" true "${SPAWN_CMD[@]}")"
LOCALIZATION_LINE="$(build_line "$LOCALIZATION_DELAY_S" true "${LOCALIZATION_CMD[@]}")"
NAV2_LINE="$(build_line "$NAV2_DELAY_S" true "${NAV2_CMD[@]}")"
FOLLOW_LINE="$(build_line "$FOLLOW_DELAY_S" true "${FOLLOW_CMD[@]}")"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  echo "Attach with: tmux attach -t $SESSION" >&2
  exit 1
fi

if [ "$DRY_RUN" = true ]; then
  echo "Session: $SESSION"
  echo "Layout: $LAYOUT"
  echo "GUI: $EFFECTIVE_GUI"
  echo "Base delay: $BASE_DELAY_S"
  echo "Delays: spawn=$SPAWN_DELAY_S localization=$LOCALIZATION_DELAY_S nav2=$NAV2_DELAY_S follow=$FOLLOW_DELAY_S"
  echo "[gazebo]       $GAZEBO_LINE"
  echo "[spawn]        $SPAWN_LINE"
  echo "[localization] $LOCALIZATION_LINE"
  echo "[nav2]         $NAV2_LINE"
  echo "[follow]       $FOLLOW_LINE"
  exit 0
fi

if [ "$LAYOUT" = "windows" ]; then
  tmux new-session -d -s "$SESSION" -n gazebo
  tmux new-window -t "$SESSION" -n spawn
  tmux new-window -t "$SESSION" -n localization
  tmux new-window -t "$SESSION" -n nav2
  tmux new-window -t "$SESSION" -n follow

  tmux setw -t "$SESSION" automatic-rename off
  tmux setw -t "$SESSION" allow-rename off

  gazebo_pane="$(tmux display-message -p -t "$SESSION:gazebo" '#{pane_id}')"
  spawn_pane="$(tmux display-message -p -t "$SESSION:spawn" '#{pane_id}')"
  localization_pane="$(tmux display-message -p -t "$SESSION:localization" '#{pane_id}')"
  nav2_pane="$(tmux display-message -p -t "$SESSION:nav2" '#{pane_id}')"
  follow_pane="$(tmux display-message -p -t "$SESSION:follow" '#{pane_id}')"

  write_session_state

  tmux send-keys -t "$SESSION:gazebo" "$GAZEBO_LINE" C-m
  tmux send-keys -t "$SESSION:spawn" "$SPAWN_LINE" C-m
  tmux send-keys -t "$SESSION:localization" "$LOCALIZATION_LINE" C-m
  tmux send-keys -t "$SESSION:nav2" "$NAV2_LINE" C-m
  tmux send-keys -t "$SESSION:follow" "$FOLLOW_LINE" C-m
  tmux select-window -t "$SESSION:gazebo"
else
  tmux new-session -d -s "$SESSION" -n sim
  gazebo_pane="$(tmux display-message -p -t "$SESSION:sim.0" '#{pane_id}')"
  follow_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$gazebo_pane" -v -l 34%)"
  localization_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$gazebo_pane" -v -l 50%)"
  spawn_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$gazebo_pane" -h -l 50%)"
  nav2_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$localization_pane" -h -l 50%)"

  tmux select-pane -t "$gazebo_pane" -T gazebo
  tmux select-pane -t "$spawn_pane" -T spawn
  tmux select-pane -t "$localization_pane" -T localization
  tmux select-pane -t "$nav2_pane" -T nav2
  tmux select-pane -t "$follow_pane" -T follow

  write_session_state

  tmux send-keys -t "$gazebo_pane" "$GAZEBO_LINE" C-m
  tmux send-keys -t "$spawn_pane" "$SPAWN_LINE" C-m
  tmux send-keys -t "$localization_pane" "$LOCALIZATION_LINE" C-m
  tmux send-keys -t "$nav2_pane" "$NAV2_LINE" C-m
  tmux send-keys -t "$follow_pane" "$FOLLOW_LINE" C-m
  tmux select-pane -t "$gazebo_pane"
fi

if [ "$ATTACH" = true ]; then
  exec tmux attach -t "$SESSION"
fi

echo "Started tmux session: $SESSION"
echo "Attach with: tmux attach -t $SESSION"
