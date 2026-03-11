# Codex Change Protocol

Purpose:
- Keep a simple running protocol of significant changes.
- Add an entry after each major fix, implementation, or behavior change.

When to log:
- New feature implementation
- Runtime or launch fix
- Config/default change affecting behavior
- Dependency/environment fix needed to run
- Documentation updates that change run instructions

Entry template:
```md
## YYYY-MM-DD HH:MM (local)
- Type: [feature|fix|config|docs|env]
- Files:
  - path/to/file
- Summary:
  - short explanation of what changed
- Why:
  - reason/problem addressed
- Behavior impact:
  - what users will observe
- Core logic impact:
  - none | minor | significant
- Verification:
  - command(s) run and result
```

---

## 2026-03-10 14:55 (local)
- Type: env
- Files:
  - system package manager (apt)
- Summary:
  - Installed `ros-jazzy-clearpath-nav2-demos`.
- Why:
  - `run_localization.sh` and `run_nav2.sh` require `clearpath_nav2_demos`.
- Behavior impact:
  - Nav2/localization launches can resolve package dependencies.
- Core logic impact:
  - none
- Verification:
  - `ros2 pkg list | grep -Fx clearpath_nav2_demos` returned installed.

## 2026-03-10 15:05 (local)
- Type: docs
- Files:
  - `README.md`
  - `RUNNING_SIM.md`
  - `CURRENT_STATE.md`
  - `src/lrs_halmstad/README.md`
- Summary:
  - Replaced hardcoded `/home/ruben/...` references with portable paths and added explicit dependency notes.
- Why:
  - Docs were partially machine-specific and confusing on this system.
- Behavior impact:
  - Cleaner onboarding and reproducible local run instructions.
- Core logic impact:
  - none
- Verification:
  - Manual review of updated markdowns.

## 2026-03-10 15:10 (local)
- Type: fix
- Files:
  - `run_gazebo_sim.sh`
  - `run_localization.sh`
  - `run_nav2.sh`
  - `run_slam.sh`
  - `src/lrs_halmstad/clearpath/setup.bash`
  - `src/lrs_halmstad/launch/run_round_follow_yolo.launch.py`
- Summary:
  - Removed machine-specific hardcoded paths and fixed script environment/path handling.
- Why:
  - Launch wrappers failed or resolved wrong locations on this machine.
- Behavior impact:
  - Startup scripts now work from this workspace path.
- Core logic impact:
  - none
- Verification:
  - Script syntax checks and runtime launch validation.

## 2026-03-10 15:12 (local)
- Type: fix
- Files:
  - `src/lrs_halmstad/lrs_halmstad/simulator.py`
  - `src/lrs_halmstad/lrs_halmstad/camera_tracker.py`
- Summary:
  - Replaced external `tf_transformations.quaternion_from_euler` import with local helper function.
- Why:
  - Runtime compatibility issue with NumPy 2 + `tf_transformations`.
- Behavior impact:
  - Prevents crash in current environment.
- Core logic impact:
  - minor (implementation-level compatibility change, intended same math behavior)
- Verification:
  - Follow stack executed without that crash.

## 2026-03-10 16:05 (local)
- Type: feature
- Files:
  - upstream `main` from `https://github.com/Lobbeb/halmstad_ws`
  - `run_tmux_1to1.sh`
  - `stop_tmux_1to1.sh`
  - `run_1to1_yolo.sh`
  - `src/lrs_halmstad/launch/run_follow_motion.launch.py`
  - `src/lrs_halmstad/config/run_follow_defaults.yaml`
  - `src/lrs_halmstad/lrs_halmstad/camera_tracker.py`
  - `src/lrs_halmstad/lrs_halmstad/simulator.py`
- Summary:
  - Integrated the latest upstream `main` refactor into this workspace and kept the local NumPy 2 quaternion compatibility patch on top.
- Why:
  - Your friend updated the project on upstream `main` with the unified 1-to-1 / YOLO stack, new tmux entrypoints, tracker configs, and launch/config renames.
- Behavior impact:
  - Main entrypoints are now `./run_tmux_1to1.sh` and `./stop_tmux_1to1.sh`.
  - YOLO now runs through the unified `run_1to1_follow.launch.py` path.
  - Old `run_round_*` launch/config paths were replaced by `run_follow_*` paths upstream.
- Core logic impact:
  - significant upstream project update, plus minor retained local compatibility patch in `camera_tracker.py` and `simulator.py`
- Verification:
  - `bash -n` passed for the updated shell scripts.
  - `python3 -m py_compile` passed for the touched Python runtime files.

## 2026-03-10 17:10 (local)
- Type: feature
- Files:
  - `run_record_experiment.sh`
  - `run_tmux_1to1.sh`
  - `stop_tmux_1to1.sh`
  - `README.md`
  - `CURRENT_STATE.md`
- Summary:
  - Added an opt-in experiment rosbag recorder that fits the existing tmux/wrapper flow and keeps the default topic set small.
- Why:
  - The current codebase needed a clean way to capture experiment runs without embedding bagging logic inside the ROS nodes or recording everything by default.
- Behavior impact:
  - `run_tmux_1to1.sh ... record:=true` now starts a recorder window alongside the normal run.
  - Recorded runs are stored under `runs/experiments/<world>/...` with `metadata.json`, `topics.txt`, and `bag/`.
  - Default bagging excludes image topics; `profile:=vision` opts into image recording.
- Core logic impact:
  - none
- Verification:
  - `bash -n run_record_experiment.sh run_tmux_1to1.sh stop_tmux_1to1.sh` passed.
  - `./run_record_experiment.sh warehouse mode:=yolo profile:=vision dry_run:=true` printed the expected topic set.
  - `./run_tmux_1to1.sh warehouse mode:=yolo record:=true record_profile:=vision dry_run:=true tmux_attach:=false` printed the expected start commands.

## 2026-03-10 18:34 (local)
- Type: fix
- Files:
  - `stop_tmux_1to1.sh`
- Summary:
  - Fixed the tmux stop script after a broken rewrite and restored executable permissions.
- Why:
  - `./stop_tmux_1to1.sh warehouse` ended with `syntax error: unexpected end of file`, which blocked clean shutdown of recorded runs.
- Behavior impact:
  - The stop script now shuts down the tmux-managed stack cleanly again.
  - `dry_run:=true` messaging is clearer and no longer looks like a real session kill.
- Core logic impact:
  - none
- Verification:
  - `bash -n stop_tmux_1to1.sh` passed.
  - `./stop_tmux_1to1.sh warehouse dry_run:=true` now executes cleanly from WSL.

## 2026-03-10 23:36 (local)
- Type: feature
- Files:
  - `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
  - `src/lrs_halmstad/launch/run_follow_motion.launch.py`
  - `src/lrs_halmstad/launch/run_1to1_follow.launch.py`
  - `src/lrs_halmstad/config/run_follow_defaults.yaml`
  - `src/lrs_halmstad/setup.py`
  - `run_tmux_1to1.sh`
- Summary:
  - Rewrote `leader_estimator.py` around Ultralytics detection/tracking, removed the old pseudo-tracking state machine, and exposed tracker selection cleanly through launch/config.
- Why:
  - The old estimator mixed detector loading, bbox continuity heuristics, debounce/holdover logic, and a world-frame alpha-beta tracker, which conflicted with clean integration of Ultralytics tracking.
- Behavior impact:
  - `leader_estimator` now uses Ultralytics `track()` when `tracker_enable:=true` and falls back to raw per-frame detections when disabled.
  - BoT-SORT is the default tracker, with `tracker_name` / `tracker_yaml` selectable from launch.
  - `run_tmux_1to1.sh ... mode:=yolo` now forwards `tracker_enable`, `tracker_name`, and `tracker_yaml` to the YOLO path.
  - Status/debug output now includes tracked target metadata such as `track_id`, track hits, and track age.
- Core logic impact:
  - significant
- Verification:
  - `python3 -m py_compile src/lrs_halmstad/lrs_halmstad/leader_estimator.py src/lrs_halmstad/launch/run_follow_motion.launch.py src/lrs_halmstad/launch/run_1to1_follow.launch.py` passed.
  - `colcon build --packages-select lrs_halmstad --symlink-install` passed.
  - `ros2 launch lrs_halmstad run_1to1_follow.launch.py --show-args | rg 'tracker_(enable|name|yaml)'` showed the new tracker launch arguments.
  - `./run_tmux_1to1.sh warehouse mode:=yolo tracker_name:=bytetrack tracker_enable:=true tracker_yaml:=/tmp/custom.yaml dry_run:=true tmux_attach:=false` forwarded the tracker arguments into `run_1to1_yolo.sh`.
  - A subprocess smoke test started `ros2 run lrs_halmstad leader_estimator ... -p tracker_enable:=true -p tracker_name:=botsort` and confirmed the node stayed alive for at least 3 seconds without startup/tracker initialization errors.

## 2026-03-11 11:24 (local)
- Type: feature
- Files:
  - `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
  - `src/lrs_halmstad/lrs_halmstad/leader_tracking.py`
  - `src/lrs_halmstad/lrs_halmstad/leader_projection.py`
  - `src/lrs_halmstad/lrs_halmstad/leader_types.py`
  - `src/lrs_halmstad/config/run_follow_defaults.yaml`
  - `run_tmux_1to1.sh`
- Summary:
  - Replaced the heavy single-file estimator rewrite with a cleaner modular split: a thinner ROS node, a dedicated Ultralytics tracking helper, a dedicated projection/sanity helper, and shared estimator datatypes.
- Why:
  - The first rewrite still felt too close to the original estimator and kept too much structure inside one file. The goal was to keep the external contract while making the tracked estimator path smaller, clearer, and less coupled to old pseudo-tracking ideas.
- Behavior impact:
  - The estimator still publishes the same core estimate/status/fault topics used by follow/camera/debug tools.
  - The active tracked target now flows through a dedicated tracker helper instead of living inside one large node file.
  - The defaults file only keeps estimator parameters that the new implementation actually uses.
- Core logic impact:
  - significant
- Verification:
  - `python3 -m py_compile src/lrs_halmstad/lrs_halmstad/leader_types.py src/lrs_halmstad/lrs_halmstad/leader_tracking.py src/lrs_halmstad/lrs_halmstad/leader_projection.py src/lrs_halmstad/lrs_halmstad/leader_estimator.py` passed.
  - `python3 -m py_compile src/lrs_halmstad/launch/run_follow_motion.launch.py src/lrs_halmstad/launch/run_1to1_follow.launch.py` passed.
  - `colcon build --packages-select lrs_halmstad --symlink-install` passed.
  - A subprocess smoke test confirmed `ros2 run lrs_halmstad leader_estimator ... -p tracker_enable:=true -p tracker_name:=botsort` stayed alive for at least 3 seconds.
  - A subprocess `ros2 launch ... --show-args` check still showed `tracker_enable`, `tracker_name`, and `tracker_yaml`.
  - `./run_tmux_1to1.sh warehouse mode:=yolo tracker_name:=bytetrack tracker_enable:=true tracker_yaml:=/tmp/custom.yaml dry_run:=true tmux_attach:=false` still forwarded tracker arguments into `run_1to1_yolo.sh`.

## 2026-03-11 11:40 (local)
- Type: fix
- Files:
  - `run_1to1_yolo.sh`
- Summary:
  - Updated the YOLO wrapper default weights path to the actual top-level `models/warehouse_v1-v1-yolo26n.pt` file present in this workspace.
- Why:
  - The previous default still pointed at `models/detection/mymodels/...`, which no longer exists here and caused `YOLO_DISABLED` / `weights_missing` during tracker testing.
- Behavior impact:
  - Running the YOLO path without an explicit `weights:=...` argument now starts with a valid model by default on this workspace.
- Core logic impact:
  - none
- Verification:
  - `bash -n run_1to1_yolo.sh` passed.
  - `test -f models/warehouse_v1-v1-yolo26n.pt` returned present.

## 2026-03-11 12:05 (local)
- Type: perf
- Files:
  - `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
  - `src/lrs_halmstad/lrs_halmstad/leader_tracking.py`
  - `src/lrs_halmstad/config/run_follow_defaults.yaml`
- Summary:
  - Reduced estimator-side image overhead by switching camera/depth subscriptions to sensor-data QoS, avoiding unnecessary decode copies, fusing the Ultralytics model at load time, and downscaling only the published debug image topic.
- Why:
  - Tracker testing was functionally working, but the estimator was CPU-heavy and the debug GUI looked stale/jittery. The goal was to improve runtime smoothness without changing the tracking/follow logic.
- Behavior impact:
  - The tracker logic and output topics are unchanged.
  - The estimator now drops stale sensor frames more aggressively instead of building image backlog.
  - `/coord/leader_debug_image` is published at a smaller width by default, which reduces GUI and ROS image transport cost.
  - The default YOLO `imgsz` in the estimator config is now `512` instead of `640` for a lighter CPU baseline.
- Core logic impact:
  - none
- Verification:
  - `python3 -m py_compile src/lrs_halmstad/lrs_halmstad/leader_estimator.py src/lrs_halmstad/lrs_halmstad/leader_tracking.py src/lrs_halmstad/lrs_halmstad/leader_projection.py src/lrs_halmstad/lrs_halmstad/leader_types.py` passed.
  - `colcon build --packages-select lrs_halmstad --symlink-install` passed.

## 2026-03-11 12:12 (local)
- Type: fix
- Files:
  - `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
- Summary:
  - Restored the debug image publisher QoS to reliable `depth=1` so `rqt` image viewers can subscribe to `/coord/leader_debug_image` again.
- Why:
  - The previous best-effort debug-image QoS reduced overhead, but it likely made the `rqt` debug pane appear as a blank gray placeholder instead of showing the estimator overlay.
- Behavior impact:
  - Debug image display in `rqt` should work again.
  - Tracking logic and estimator outputs are unchanged.
- Core logic impact:
  - none
- Verification:
  - `python3 -m py_compile src/lrs_halmstad/lrs_halmstad/leader_estimator.py` passed.
  - `colcon build --packages-select lrs_halmstad --symlink-install` passed.

## 2026-03-11 14:08 (local)
- Type: fix
- Files:
  - `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
- Summary:
  - Replaced the incompatible `SensorDataQoS` import with Jazzy-compatible `qos_profile_sensor_data`.
- Why:
  - `leader_estimator` was crashing on startup with `ImportError: cannot import name 'SensorDataQoS' from 'rclpy.qos'`, which left `/coord/leader_debug_image` blank because the estimator node was dead.
- Behavior impact:
  - The estimator should now start again in YOLO sessions.
  - Debug image/status topics should exist again after a full restart of the tmux stack.
- Core logic impact:
  - none
- Verification:
  - `python3 -m py_compile src/lrs_halmstad/lrs_halmstad/leader_estimator.py` passed.
  - `colcon build --packages-select lrs_halmstad --symlink-install` passed.
  - Direct startup repro no longer fails with the previous `ImportError`.


