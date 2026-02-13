#!/usr/bin/env bash
set -euo pipefail

# RUNBOOK expects: scripts/run_batch.sh <N> <condition> <uav> <world>
# 

if [ $# -lt 4 ]; then
  echo "Usage: $0 <num_runs> <condition> <uav_name> <world>"
  exit 2
fi

N="$1"
COND="$2"
UAV="$3"
WORLD="$4"

source "$(dirname "$0")/env.sh"

TS="$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="${HOME}/runs/batch_${TS}"
mkdir -p "$BATCH_DIR"

for i in $(seq 1 "$N"); do
  RUN_ID="$(printf "%s/%s_%03d" "$BATCH_DIR" "$COND" "$i")"
  echo "[run_batch] $RUN_ID"
  bash scripts/run_round.sh "$RUN_ID" "$COND" "$UAV" "$WORLD"
done

echo "[run_batch] Done: $BATCH_DIR"
