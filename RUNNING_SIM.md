# Running Sim

Default tested path: `warehouse`.

Assumption:
- you already start in `~/halmstad_ws`

## Start 1-to-1 Sim With Nav2 And AMCL Follow

Run these in order in separate terminals:

1. Start Gazebo:

```bash
./run_gazebo_sim.sh warehouse
```

Without GUI:

```bash
./run_gazebo_sim.sh warehouse false
```

2. Spawn one UAV:

```bash
./run_spawn_uav.sh warehouse uav_name:=dji0
```

3. Start localization:

```bash
./run_localization.sh warehouse
```

If you want a different saved map:

```bash
./run_localization.sh warehouse maps/warehouse_manual.yaml
```

4. Start Nav2:

```bash
./run_nav2.sh
```

5. Start the AMCL-based odom follow stack:

```bash
./run_1to1_follow.sh warehouse
```

Current baseline:
- this odom-follow path now uses `/<ugv>/amcl_pose_odom`, not raw `/platform/odom`
- `/<ugv>/amcl_pose_odom` is synthesized from `/<ugv>/amcl_pose` by [pose_cov_to_odom.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/pose_cov_to_odom.py)
- current UAV camera mode is attached/integrated: `uav_camera_mode:=integrated_joint`
- attached mount pitch defaults to `45 deg`
- current camera defaults are `pan_enable: false` and `tilt_enable: true`
- if you override the mount pitch at spawn time, pass the same value to follow:

```bash
./run_spawn_uav.sh warehouse uav_name:=dji0 mount_pitch_deg:=35
./run_1to1_follow.sh warehouse mount_pitch_deg:=35
```

Important runtime note:
- Gazebo sim time is guarded through [clock_guard.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/clock_guard.py)
- expected `/clock` publisher is `clock_guard`
- if Gazebo is restarted or the world is reset, restart localization, Nav2, and follow

## Start The Camera Views

Saved multi-camera / debug layout:

```bash
./run_rqt_perspective.sh
```

Then under `Perspectives`, choose one from the `perspectives/` folder if needed.

UAV camera: ``/dji0/camera0/image_raw``

UGV camera: ``/a201_0000/sensors/camera_0/color/image``

YOLO debug image: ``/coord/leader_debug_image``


Only use the YOLO debug image when you started the YOLO flow from the section below.

## Control Height And Distance During Runtime

These work while `run_1to1_follow.sh` or `run_1to1_yolo.sh` is running.

Recommended live control:
- change `d_euclidean` during runtime
- this updates the derived `d_target` and `z_alt` together
- no restart is needed for this control
- the direct `ros2` commands below assume the terminal is already sourced

Change the 3D UAV-to-UGV follow distance:

```bash
ros2 param set /follow_uav d_euclidean x.xx
```

Examples:

```bash
ros2 param set /follow_uav d_euclidean 5.0
ros2 param set /follow_uav d_euclidean 10.0
ros2 param set /follow_uav d_euclidean 15.0
```

Advanced direct controls:

```bash
ros2 param set /follow_uav d_target 9.0
ros2 param set /follow_uav z_alt 9.0
```

Use `d_target` and `z_alt` only if you explicitly want to control horizontal distance and altitude separately.

## Capture Images For Datasets

Do this while the follow stack is already running.

Default output:
- `datasets/warehouse_auto`

Recommended workflow:
1. start the sim
2. start the UAV and follow stack
3. start dataset capture
4. change `d_euclidean` during runtime
5. let Nav2 drive the UGV through the route
6. once happy, switch world / map campaign and restart all terminals if needed

This gives:
- multiple camera heights / distances
- multiple view angles
- multiple relative poses between UGV and UAV
- saved images and labels in `datasets/`

Wrapper:

```bash
./run_capture_dataset.sh warehouse
```

Examples:

```bash
./run_capture_dataset.sh warehouse class:=car
./run_capture_dataset.sh warehouse hz:=1.0
./run_capture_dataset.sh warehouse out:=datasets/warehouse_auto_v2
```

If you want to keep dataset campaigns separate:

```bash
./run_capture_dataset.sh warehouse out:=datasets/warehouse_run2
```

## Run The Sim With A YOLO Model

Start the normal sim first:

```bash
./run_gazebo_sim.sh warehouse
./run_spawn_uav.sh warehouse
./run_localization.sh warehouse
./run_nav2.sh
```

Then start the YOLO follow flow:

```bash
./run_1to1_yolo.sh warehouse
```

Warehouse `car` filter:

```bash
./run_1to1_yolo.sh warehouse target:=car
```

Different YOLO weights:

```bash
./run_1to1_yolo.sh warehouse weights:=detection/yolo26/yolo26s.pt
```

Estimate error topic:

```bash
ros2 topic echo /coord/leader_estimate_error
```

Available YOLO models:

These can be passed to `weights:=...` as relative paths under `/home/ruben/halmstad_ws/models`.

Detection models:
- `detection/yolo26/yolo26n.pt`
- `detection/yolo26/yolo26s.pt`
- `detection/yolo26/yolo26m.pt`
- `detection/yolo26/yolo26l.pt`
- `detection/yolo26/yolo26x.pt`

Custom models:
- `mymodels/yolo26v1.pt`
- `mymodels/yolo26v2.pt`

OBB models:
- `obb/yolo26/yolo26n-obb.pt`
- `obb/yolo26/yolo26s-obb.pt`
- `obb/yolo26/yolo26m-obb.pt`
- `obb/yolo26/yolo26l-obb.pt`
- `obb/yolo26/yolo26x-obb.pt`

YOLOv5 models:
- `detection/yolo5/yolov5n.pt`
- `detection/yolo5/yolov5nu.pt`
- `detection/yolo5/yolov5s.pt`
- `detection/yolo5/yolov5su.pt`

## Scan A New Map With RViz

Use this when the current world does not already have a good Nav2 map.

1. Start Gazebo:

```bash
./run_gazebo_sim.sh warehouse
```

2. Start SLAM:

```bash
./run_slam.sh
```

3. Drive the UGV:

If the Gazebo GUI drive controls work, use them.

Keyboard option:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -r cmd_vel:=/a201_0000/cmd_vel
```

4. Start RViz:

```bash
./run_nav2_rviz.sh
```

5. Save the map:

Example save path for `warehouse_manual`:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli \
  -t /a201_0000/map \
  -f maps/warehouse_manual
```

This creates:
- `maps/warehouse_manual.yaml`
- `maps/warehouse_manual.pgm`

## Available Arguments And Values

### `./run_gazebo_sim.sh`

- `WORLD`
  - any world name resolvable through Gazebo resource paths
  - `warehouse`, `orchard`, `solar_farm`, `pipeline`, `office`, `construction`, `walls`, `baylands`
- `GUI`
  - `true`
  - `false`
- extra forwarded launch arguments
  - `x:=...`
  - `y:=...`
  - `z:=...`
  - `yaw:=...`
  - `rviz:=true|false`
  - `auto_start:=true|false`
  - `use_sim_time:=true|false`

### `./run_spawn_uav.sh`

- positional `WORLD`
  - common example: `warehouse`
  - if omitted, it follows the active Gazebo world when available
- `name:=...`
  - UAV namespace / model name
  - common examples: `dji0`, `dji1`, `dji2`
- `height:=...`
  - UAV spawn height in meters
- `mount_pitch_deg:=...`
  - attached camera mount pitch in degrees
  - common examples: `35`, `45`
- `sensor_roll_deg:=...`
  - camera sensor roll offset in degrees
- `sensor_pitch_deg:=...`
  - camera sensor pitch offset in degrees
- `sensor_yaw_deg:=...`
  - camera sensor yaw offset in degrees
- extra forwarded launch arguments
  - anything else is passed through to `spawn_uav_1to1.launch.py`

### `./run_localization.sh`

- positional `WORLD`
  - current tested example: `warehouse`
- optional second positional argument: map path
  - path to the `.yaml` occupancy map
  - example: `maps/warehouse_manual.yaml`
- extra forwarded launch arguments
  - anything else is passed through to `localization.launch.py`

### `./run_nav2.sh`

- extra forwarded launch arguments
  - anything else is passed through to `nav2.launch.py`

### `./run_1to1_follow.sh`

- positional `WORLD`
  - current tested example: `warehouse`
- `height:=...`
  - maps to `uav_start_z:=...`
- `mount_pitch_deg:=...`
  - maps to `camera_mount_pitch_deg:=...`
- extra forwarded launch arguments
  - anything else is passed through to `run_1to1_follow.launch.py`
- common forwarded launch arguments:
  - `uav_name:=dji0`
  - `uav_camera_mode:=integrated_joint|detached_model`
  - `ugv_mode:=nav2|external|none`
  - `ugv_set_initial_pose:=true|false`
  - `ugv_goal_sequence_csv:='x1,y1,yaw1;x2,y2,yaw2'`
  - `start_uav_simulator:=true|false`
  - `ugv_start_delay_s:=...`
- common forwarded values in the underlying launch:
  - `leader_mode:=odom|estimate|pose`
  - `start_leader_estimator:=auto|true|false`
  - `leader_perception_enable:=true|false`
  - `leader_range_mode:=ground|const|auto|depth`
  - `leader_constant_range_m:=...`
  - `target_class_name:=...`
  - `target_class_id:=-1|<non-negative>`
  - `yolo_weights:=...`
  - `models_root:=...`
  - `yolo_device:=cpu|cuda|cuda:0`
  - `ugv_initial_pose_x:=...`
  - `ugv_initial_pose_y:=...`
  - `ugv_initial_pose_yaw_deg:=...`
  - `uav_start_x:=...`
  - `uav_start_y:=...`
  - `uav_start_yaw_deg:=...`

### `./run_1to1_yolo.sh`

- positional `WORLD`
  - current tested example: `warehouse`
- `weights:=...`
  - maps to `yolo_weights:=...`
  - example: `weights:=detection/yolo26/yolo26s.pt`
- `height:=...`
  - maps to `uav_start_z:=...`
- `mount_pitch_deg:=...`
  - maps to `camera_mount_pitch_deg:=...`
- `target:=...`
  - maps to `target_class_name:=...`
  - example: `target:=car`
- extra forwarded launch arguments
  - anything else is passed through to `run_1to1_follow.launch.py`

### `./run_yolo.sh`

- `weights:=...`
  - maps to `yolo_version:=...`
  - example: `weights:=detection/yolo26/yolo26s.pt`
- `target:=...`
  - maps to `target_class_name:=...`
  - example: `target:=car`

### `./run_capture_dataset.sh`

- `WORLD`
  - current tested example: `warehouse`
- default output folder
  - `datasets/<WORLD>_auto`
- `class:=...`
  - object class label to save in the dataset
  - current useful example: `car`
- `id:=...`
  - numeric class id to save instead of a class name
- `hz:=...`
  - capture frequency in Hz
  - example: `1.0`
- `negatives:=true|false`
  - whether to save negative examples
- `out:=...`
  - output directory for images and labels
  - examples: `datasets/warehouse_auto`, `datasets/warehouse_auto_v2`
- extra forwarded dataset parameters
  - any other `name:=value` is passed as a ROS parameter to `sim_dataset_capture`

Extra useful commands:

- inspect the map or set your own initial pose / Nav2 goal:

```bash
./run_nav2_rviz.sh
```

- open the saved multi-camera / debug `rqt` layout:

```bash
./run_rqt_perspective.sh
```
