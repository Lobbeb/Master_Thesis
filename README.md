
# Master_Thesis
dudu
=======
cat > README.md <<'EOF'

# Master Thesis Workspace (ROS 2 + Gazebo harness)

This repository contains the ROS 2 workspace needed to run Stage-1 baseline experiments and the Task-3 bridge scaffolding.

## Quick start (fresh machine)

1. Install ROS 2 Jazzy + Gazebo (system-level setup not included here).
2. Clone this repo:
   git clone <https://github.com/Lobbeb/Master_Thesis>
3. Bootstrap deps:
   cd halmstad_ws
   ./bootstrap.sh
4. Build:
   source scripts/env.sh
   colcon build --symlink-install

## Run (Stage-1 baseline)

Terminal A: start Gazebo world + Husky (your existing launch flow)  
Terminal B: spawn UAVs / bridges (your existing launch flow)  
Terminal C:
cd ~/halmstad_ws
source scripts/env.sh
bash scripts/run_round.sh run_base baseline dji0 orchard --with-cameras

Outputs are stored under:
runs/<runnano  2af3396 (Initial commit: Stage-1 harness + scripts + configs)
