#!/usr/bin/env bash
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 <run_dir>"
  exit 2
fi

RUN_DIR="$1"
echo "== meta.yaml =="
sed -n '1,200p' "$RUN_DIR/meta.yaml" || true

echo
echo "== rosbag info =="
ros2 bag info "$RUN_DIR/bag" | head -n 80 || true
