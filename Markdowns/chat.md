# Halmstad ROS2 + Gazebo Testbed — Stage-1 Harness Status (handoff note)

This file summarizes the current state of the project after rebuilding the missing “Stage-1 automation” (scripts + checks + run folders), and documents the remaining issues we must fix before moving on to OMNeT++ and Stage-2 (leashing/follow).

## 0) Environment context

We are running ROS 2 Jazzy + Gazebo (orchard world) inside WSL.
GPU acceleration situation:

- Initially OpenGL/Vulkan were on llvmpipe (CPU).
- /dev/dxg exists.
- With:
  - `export MESA_D3D12_DEFAULT_ADAPTER_NAME=AMD`
  - `export GALLIUM_DRIVER=d3d12`
    glxinfo shows D3D12 (AMD RX 6700 XT), while Vulkan still listed llvmpipe.
    This was “good enough” to continue; simulation runs.

## 1) Workspace / codebase layout (what exists)

Workspace root: `~/halmstad_ws`

Packages:

- `src/lrs_halmstad`
- `src/lrs_omnet_bridge`

Tree highlights:

- `src/lrs_halmstad/launch/`
  - `spawn_uavs.launch.py` (spawns dji0–dji2 and bridges set_pose)
  - `spawn_robot.launch.py`
  - other spawn launch files
- `src/lrs_halmstad/lrs_halmstad/`
  - `command.py` (CLI: setpose/scan; talks to `/world/<world>/set_pose`)
  - `generate_sdf.py`
- `src/lrs_halmstad/resource/`
  - `experiment_defaults.yaml` (Stage-1 parameters)
- `src/lrs_omnet_bridge/scripts/`
  - `network_impairment_node.py`
  - `ros_omnet_bridge.py`
  - `omnet_server.py`

Stage-1 harness folder (new/reintroduced):

- `~/halmstad_ws/scripts/`
  - `env.sh`
  - `run_round.sh`
  - (optional analysis scripts if present)
- `~/halmstad_ws/config/`
  - `rosbag_qos.yaml` (QoS overrides for rosbag)

Run artifacts:

- `~/halmstad_ws/runs/<run_id>/`
  - `meta.yaml`
  - `bag/` (rosbag2 output, MCAP)

## 2) Stage-1 baseline sim is working (confirmed)

We confirmed:

- `ros2 launch clearpath_gz simulation.launch.py world:=orchard use_sim_time:=true gui:=true` works once `~/clearpath/robot.yaml` exists.
- `spawn_uavs.launch.py` spawns 3 UAVs + UGV in orchard.
- Service exists:
  - `/world/orchard/set_pose [ros_gz_interfaces/srv/SetEntityPose]`
- Moving UAV works (manually):
  - `ros2 run lrs_halmstad command --ros-args -p command:=setpose -p world:=orchard -p name:=dji0 -p x:=1.0 -p y:=2.0 -p z:=10.0`
    exits with code 0 and the UAV moves.

## 3) What “Step 1 / Task 1” actually is (goal)

Goal: rebuild a deterministic Stage-1 experiment harness:

- one command that creates a run folder
- logs meta.yaml
- records a rosbag (mcap)
- produces event markers
- moves UAV in a repeatable sweep
- moves UGV in a repeatable pattern
- includes a topic/service “contract check” so runs fail early if the testbed isn’t ready

This is NOT Stage-2 yet. This is “Stage-1 verification harness” so thesis experiments can be repeated.

## 4) Files we changed/created for Step 1

### 4.1 `scripts/env.sh` (sourced)

Purpose: set up ROS env + overlay in a safe way (should not crash terminal).
Key points:

- Source `/opt/ros/jazzy/setup.bash`
- cd to workspace root
- source `install/setup.bash` if present (warn if not)

### 4.2 `scripts/run_round.sh` (executed)

Purpose: do one deterministic run.
Expected usage:

- `RUN_ROOT=~/halmstad_ws/runs bash scripts/run_round.sh <run_id> <condition> <uav> <world> [--with-cameras]`

What it does:

- sources `scripts/env.sh` BEFORE `set -u` (nounset) because ROS setup can break under `-u`
- runs `ros2 run lrs_halmstad contract_check <world> <uav> 10`
- creates `runs/<id>/meta.yaml`
- starts rosbag recording
- publishes `/coord/events` markers (ROUND_START etc.)
- does UAV sweep by repeatedly calling `ros2 run lrs_halmstad command setpose ...`
- does UGV motion by publishing cmd_vel
- stops recording and prints bag info

### 4.3 `lrs_halmstad` additions (contract checker + command behavior)

We introduced/used:

- `ros2 run lrs_halmstad contract_check orchard dji0 10`
  to detect missing topics/services BEFORE recording.

We also modified `command.py` behavior so `command setpose ...` exits cleanly after a single set_pose call (instead of spinning forever / requiring Ctrl+C).
That solved:

- “setpose hangs”
- “rclpy shutdown already called” issues when Ctrl+C happened at wrong time

### 4.4 QoS overrides for rosbag

We hit a rosbag issue:

- `/tf` and some topics recorded as 0 messages due to QoS mismatch warnings.
  Fix approach:
- create `~/halmstad_ws/config/rosbag_qos.yaml`
- record with:
  - `ros2 bag record -o <bagdir> --qos-profile-overrides-path ~/halmstad_ws/config/rosbag_qos.yaml --topics ...`

We initially got:

- `Invalid QoSProfile: History and/or depth settings are required.`
  Then fixed YAML to include required `history` and `depth`, after which rosbag reported “Overriding subscription profile …” and `/tf` started recording.

## 5) What currently works (Step-1 deliverables)

- run folder + meta.yaml gets created
- rosbag writes MCAP + metadata.yaml
- `/tf` now records nonzero messages after QoS override
- `/coord/events` exists and records event markers (we saw Count: 2)

Example bag info (verify100):

- /clock, odom, odom/filtered, tf, tf_static recorded
- /coord/events recorded (Count 2)
  This confirms recording pipeline works.

## 6) Current issues (the ones still confusing/unfinished)

### 6.1 UAV “pose topic” mismatch

Our Stage-1 plan originally expected a topic like:

- `/${UAV}/pose` (e.g. `/dji0/pose`)

But in this testbed, `ros2 topic list | grep -i dji0` shows camera topics only:

- `/dji0/camera0/image_raw`
- `/dji0/camera0/camera_info`

And pose-related output showed:

- `/a201_0000/set_pose` (a service/topic name in the robot stack; not UAV pose)

Conclusion:

- We cannot record `/dji0/pose` because it doesn’t exist (yet).
- We need to decide what the “UAV pose ground truth” is:
  Option A: record Gazebo entity pose topic (if available via `/world/.../pose/info` or similar)
  Option B: create our own `/dji0/pose` publisher inside `command.py` or a small node that publishes the pose each time we set it
  Option C: log commanded set_pose positions only (events/meta), and treat that as UAV trajectory for Stage-1

Right now Stage-1 runs do not include UAV pose telemetry unless we implement one of these.

### 6.2 `/cmd_vel` logging confusion (and why it showed 0 messages)

We saw `ros2 bag info` show:

- `/a201_0000/cmd_vel` Count: 0 sometimes
  and later discovered topic chain:
- `/a201_0000/cmd_vel` publisher exists (cmd_vel_bridge), but internal subscription had BEST_EFFORT and the actual controller path uses:
  - `/a201_0000/platform/cmd_vel`
  - `/a201_0000/platform_velocity_controller/cmd_vel_out`

So recording `/a201_0000/cmd_vel` isn’t necessarily the best “ground truth” for motion commands.
Better is likely:

- record `/a201_0000/platform/cmd_vel` (input to platform)
  and/or
- record `/a201_0000/platform_velocity_controller/cmd_vel_out` (what controller outputs)

Also: in our run script, we were publishing Geometry Twist (not TwistStamped) at one point, which mismatched the topic type and could produce “no movement / no messages” behavior.
We ended up switching to publishing TwistStamped to match the topic type.

### 6.3 `run_round.sh` gets “service not available” intermittently

During sweeps:

- `WAIT for service: /world/orchard/set_pose`
- sometimes prints “service not available, waiting... (1/10s) …”
  but then continues.

This suggests:

- per-call node startup in `ros2 run lrs_halmstad command setpose ...` sometimes races with service availability / bridge readiness
- repeated process spawning is heavy and causes intermittent delays

It still works, but this is not ideal for future proofing.

### 6.4 Event markers recorded only 2 messages

We expected a full sequence:

- ROUND_START
- UAV_PHASE_START/END
- UGV_PHASE_START/END
- ROUND_END

But bag shows `/coord/events` Count: 2.
That means either:

- not all publishes executed (script interrupted)
- or `/coord/events` publishing is not reliable / is late vs bag start
- or topic was missing earlier and only some publishes succeeded

We need to fix this so event markers always appear.

## 7) The original “Stage-1 verification” (what it should be)

Baseline verification should be:

Terminal A (sim):

- launch Gazebo + Clearpath sim for orchard

Terminal B (UAV spawn):

- spawn UAVs and ensure `/world/orchard/set_pose` exists

Terminal C (Stage-1 harness):

- run one deterministic run, wait until it completes:
  - `RUN_ROOT=~/halmstad_ws/runs bash scripts/run_round.sh verifyXYZ baseline dji0 orchard`

Then verify:

- `runs/verifyXYZ/meta.yaml` exists
- `runs/verifyXYZ/bag/metadata.yaml` exists
- `ros2 bag info runs/verifyXYZ/bag` shows non-zero counts for core topics
- UAV motion happened (visually)
- UGV motion happened (visually)
- `/coord/events` contains the full event sequence

If any of those fail, fix the code until this passes reliably.

## 8) What we still need to implement (finish Step-1 properly)

These are the minimum “future-proof” fixes still missing:

A) Stop spawning `ros2 run ... command setpose` 60+ times

- Instead: create one persistent sweeper node/script that calls `/world/<world>/set_pose` repeatedly from ONE process.
- This eliminates service race + makes runs stable.

B) Decide and implement “UAV pose telemetry”

- Either publish `/dji0/pose` ourselves, or record Gazebo’s pose topic, or log commanded positions to a CSV during the run.

C) Fix cmd_vel topic selection

- Choose one authoritative topic for “commands” and one for “actual motion”:
  - commands: `/a201_0000/platform/cmd_vel` (TwistStamped)
  - motion: `/a201_0000/platform/odom` and/or `/a201_0000/platform/odom/filtered`
- Update run_round.sh TOPICS accordingly.
- Update contract_check accordingly.

D) Make `/coord/events` reliable

- Ensure the topic exists (create it by publishing after bag start, or add a tiny “events node” that latches).
- Ensure we log all expected markers.

When A–D are done, Step-1 is truly complete.

## 9) Next tasks after Step-1 (project plan)

After Stage-1 harness is stable, move to:

### Task 3 / OMNeT++ side (next)

Two “levels” exist in the repo:

- Quick path: ROS-based impairment node in `lrs_omnet_bridge` (delay/loss injection)
- Heavy path: full OMNeT++ integration in `src/lrs_omnet/` (future realism layer)

Plan:

- Use ROS-based impairment first (fast, thesis experiments earlier)
- Keep full OMNeT++ integration as later/future work unless we need protocol-level claims

### Stage-2 (later) — leashing + follow adapter behavior

Stage-2 is not “recording & sweep”.
Stage-2 is behavior:

- UGV + UAV coordination where UAV follows/leashes/relays relative to UGV movement
- requires a control policy or a “follow adapter” node that listens to UGV state and commands UAV set_pose (or velocity) accordingly
- will build on Stage-1 harness: same run folder structure, same logging, but different “behavior script”

### Baylands + PX4 + MARBLE_HUSKY_SENSOR_CONFIG_5 (later)

Future alignment with another group:

- Switch world to Baylands
- Use the MARBLE Husky sensor model from Gazebo Fuel:
  - https://app.gazebosim.org/OpenRobotics/fuel/models/MARBLE_HUSKY_SENSOR_CONFIG_5
- Move from earlier “bebop” references to PX4-based stack (as the correct plan)

This should be a clean swap if Stage-1 is parameterized (world/model) and if we avoid hardcoding orchard/dji0 assumptions.

## 10) Notes about partner message (WSL “image”)

Partner likely means:

- they use a separate WSL distribution (or WSL export/import “image”) dedicated to OMNeT++ tooling
- e.g. not starting default Ubuntu user home, but a separate distro/home like `/home/opp_env`
  Reason:
- OMNeT++ install/path dependencies can be messy
- A prebuilt WSL image with OMNeT++ already installed avoids manual setup errors
  This can be added as a future step after Stage-1 is stable:
- create a dedicated WSL distro for OMNeT++ work, keep ROS/Gazebo separate, reduce conflicts.

## 11) Current blocking symptom summary (quick)

- Stage-1 runs produce bags, but:
  - UAV pose topic isn’t real (needs decision)
  - cmd_vel topic choice/type causes 0-message bags sometimes
  - event markers not fully recorded
  - repeated “ros2 run ... setpose” process spawning causes service waiting spam and instability

We need to implement A–D in section 8.

And a overall plan and a summary of it is
our proposed sequencing is sensible:

Sync/restore the pipeline from your markdown (finish Task 2 verification first) <- WE ARE HERE

TASK1_CHECKLIST

Stage 2 behaviors (leashing/follow adapter, scaling behaviors) — do after Task 2 is reproducible

PX4 + Baylands + MARBLE_HUSKY_SENSOR_CONFIG_5 integration (environment parity with the other group) — do after you have a stable baseline pipeline

I’d treat the Fuel model integration as a dedicated branch/task because it can change assumptions (sensors/topics/URDF/config) and will break comparisons if you mix it mid-verification.

EXTRA INFORMATION HERE AND TEXT:
You have the core simulation stack working (Clearpath + Gazebo + UAV spawn + set_pose).
You have Task 3 assets preserved (lrs_omnet_bridge as a ROS package + lrs_omnet/ OMNeT project folder).
But the automation layer (the scripts/ harness and analysis) that your markdown talks about was part of what got deleted.

So yes: you do not need to “do Task 3 again”. What you do need, if you want to follow your runbook 100% or improve it, is to recreate (or re-add) the missing Stage-1 harness pieces. That’s Task 1/2-ish work, not Task 3.

Confirming your key point

is not “redo OMNeT++”.

is “choose the impairment path later”.
Since you already have Task 3 preserved, we can treat the decision as: when you start impairments, you’ll start with the ROS-based impairment node (lrs_omnet_bridge) and keep full OMNeT++ as optional. Nothing to rebuild right now.

What the “next work” really is

It’s exactly those 6 improvements you liked — they replace the deleted scripts/ folder with something better and more integrated into your current repo:

Deterministic verify runner (creates run folder, metadata, rosbag, optional actions)

setpose helper library + CLI (so command.py + future Stage 2 reuse it)

Topic/Service contract checker (fail fast if world/service/topics missing)

Baseline launch unification (one launch file to bring up what you need)

Parameterization (orchard/baylands configs without code edits)

Lightweight metrics logger (CSV outputs; thesis-friendly)

Where this should live in your current tree

Because your workspace only has two ROS packages, the clean approach is:

Put (2), (3), (6), and the deterministic runner (1) as Python modules under
src/lrs_halmstad/lrs_halmstad/

Put (4) as a new launch file under
src/lrs_halmstad/launch/

Put (5) as YAML under
src/lrs_halmstad/resource/

This way it’s versioned, built, and discoverable as part of the package, instead of a fragile top-level scripts/ folder.

So what are we doing right now?

Rebuild the deleted scripts/ functionality, but in a cleaner way inside lrs_halmstad, while keeping Task 3 intact and untouched.

If you say “yes”, the very next concrete step is: I’ll give you the exact files to create (paths + full code) for:

gz_pose.py (set_pose helper)

check_env.py (contract checker)

run_verify.py (deterministic Stage 1 run)

baseline_stack.launch.py (optional launch unification)

experiment_defaults.yaml (parameters)

No need to touch OMNeT++ at all until you decide to run impairments.

END.
