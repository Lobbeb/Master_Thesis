#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source ~/halmstad_ws/install/setup.bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard
