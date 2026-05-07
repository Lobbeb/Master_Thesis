#!/usr/bin/env bash

route_lidar_config_path() {
  printf '%s/src/lrs_halmstad/config/baylands_route_lidar.yaml\n' "$WS_ROOT"
}

route_lidar_arg_present() {
  local prefix="$1"
  shift
  local arg=""
  for arg in "$@"; do
    [[ "$arg" == "$prefix"* ]] && return 0
  done
  return 1
}

route_lidar_preset_args() {
  local route="$1"
  local lidar="${2:-3d}"
  shift 2 || true

  [ "$lidar" = "3d" ] || return 0

  python3 - "$(route_lidar_config_path)" "$route" "$@" <<'PY'
import sys
from pathlib import Path

import yaml

config_path = Path(sys.argv[1])
route = Path(sys.argv[2]).name
if route.endswith(".yaml"):
    route = route[:-5]
if route.startswith("baylands_waypoints_"):
    route = route[len("baylands_waypoints_") :]
if route.endswith("_rviz"):
    route = route[:-5]
existing = sys.argv[3:]

if not config_path.is_file():
    raise SystemExit(0)

data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
settings = (data.get("routes") or {}).get(route) or {}

for key, value in settings.items():
    prefix = f"{key}:="
    if any(arg.startswith(prefix) for arg in existing):
        continue
    print(f"{prefix}{value}")
PY
}
