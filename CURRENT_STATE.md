## Current State

Assumption:
- start in `~/halmstad_ws`

### Active Baseline

Tested world:
- `warehouse`

Standard run order:
1. `./run_gazebo_sim.sh warehouse`
2. `./run_spawn_uav.sh warehouse uav_name:=dji0`
3. `./run_localization.sh warehouse`
4. `./run_nav2.sh`
5. `./run_1to1_follow.sh warehouse`

YOLO variant:
- `./run_1to1_yolo.sh warehouse`

### What Works Now

The main odom-follow failure mode is resolved:
- UAV tracking is now stable through normal turns
- the fix was changing the leader truth source from UGV raw/filtered odom drift to AMCL-aligned pose
- user confirmed that this change made the UAV track the UGV "flawlessly"

Current odom-follow path:
- `leader_mode:=odom`
- launch default leader topic is now `/<ugv>/amcl_pose_odom`
- `/<ugv>/amcl_pose_odom` is synthesized from `/<ugv>/amcl_pose`

Relevant files:
- [run_round_follow_motion.launch.py](/home/ruben/halmstad_ws/src/lrs_halmstad/launch/run_round_follow_motion.launch.py)
- [run_round_follow_yolo.launch.py](/home/ruben/halmstad_ws/src/lrs_halmstad/launch/run_round_follow_yolo.launch.py)
- [pose_cov_to_odom.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/pose_cov_to_odom.py)
- [follow_uav_odom.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow_uav_odom.py)
- [camera_tracker.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/camera_tracker.py)
- [sim_dataset_capture.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/sim_dataset_capture.py)

Important hindsight:
- the long yaw/turn debugging path was misleading because the debug topics were internally self-consistent while the leader truth source itself was drifting
- if a future regression again looks like "numbers are fine but the image drifts", validate the pose source against AMCL before retuning yaw math
- the same lesson applies to dataset capture: target geometry must use `/<ugv>/amcl_pose_odom`, not raw `/platform/odom`

### Camera Baseline

Current 1-to-1 default:
- detached camera model
- `uav_camera_mode:=detached_model`
- `pan_enable: true`
- `tilt_enable: true`
- `default_pan_deg: 0.0`
- `default_tilt_deg: -45.0`

Interpretation:
- detached camera motion is now the validated visible-camera path in Gazebo
- pan and tilt both track the shared offset leader look target
- attached/integrated mode remains available only as an override path

Important consequence:
- `camera:=attached` is now an opt-in override, not the baseline
- if camera framing regresses again, compare the detached model against `/dji0/camera/target/*` before retuning follow math

### Clock / Sim Time

Current Gazebo sim launch now uses a guarded clock path:
- Gazebo `/clock` -> ROS `/clock_raw`
- [clock_guard.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/clock_guard.py) republishes monotonic ROS `/clock`

Why:
- Gazebo clock was observed to jump backward occasionally
- that produced repeated `Detected jump back in time. Clearing TF buffer.` warnings

Current expected runtime check:
- `ros2 topic info /clock -v`
- publisher count should be `1`
- publisher should be `clock_guard`

### Current Tuning Defaults

From [run_round_follow_defaults.yaml](/home/ruben/halmstad_ws/src/lrs_halmstad/config/run_round_follow_defaults.yaml):
- `d_target: 7.0`
- `z_alt: 7.0`
- `follow_speed_mps: 5.0`
- `follow_speed_gain: 2.0`
- `follow_yaw_rate_rad_s: 5.0`
- `follow_yaw_rate_gain: 4.0`
- `tick_hz: 10.0`
- `yaw_deadband_rad: 0.01`

Camera defaults:
- `default_pan_deg: 0.0`
- `default_tilt_deg: 0.0`
- `pan_enable: false`
- `tilt_enable: true`

### Debug / Verification

Useful checks:
- `/dji0/follow/target/*`
- `/dji0/follow/actual/*`
- `/dji0/follow/error/*`
- `/dji0/follow/debug/*`
- `/dji0/camera/target/*`
- `/dji0/camera/actual/*`
- `/dji0/camera/error/*`

Pose-source validation:
- compare `/platform/odom`, `/platform/odom/filtered`, and `/amcl_pose` in `map`
- use this if follow starts looking correct numerically but drifts visually again

Manual checks:
- `ros2 topic echo /a201_0000/platform/odom`
- `ros2 topic echo /a201_0000/platform/odom/filtered`
- `ros2 topic echo /a201_0000/amcl_pose`
- `ros2 topic echo /dji0/follow/debug/yaw_mode`

### Remaining Limitation

Current remaining camera limitation:
- attached/integrated camera mode is still available, but it is no longer the validated default path
- if you switch back to attached mode, treat it as an override path that may differ visually from detached

Most likely next task:
- keep tuning follow geometry and camera targeting around the detached baseline
- only revisit attached mode if you explicitly want to debug that path again

Things that should not be "fixed back":
- do not switch follow back to raw `/platform/odom` or `/platform/odom/filtered`
- do not remove `clock_guard` unless the upstream Gazebo clock issue is proven fixed
- do not pitch the whole UAV body to change camera look angle; use mount pitch and gimbal tilt instead

Local-only helpers:
- `run_follow_yaw_debug.py` and `run_pose_source_debug.py` were useful during debugging
- they are now intentionally ignored in git and should not be relied on as committed tooling

### Build State

Recent builds passed:
- `colcon build --packages-select lrs_halmstad`

If a new chat continues from here, start by reading:
- this file
- [RUNNING_SIM.md](/home/ruben/halmstad_ws/RUNNING_SIM.md)
