# lrs_halmstad

Simple command reference for running the simulator, spawning UAVs, and monitoring sensors.

## 1. Start Husky simulation (Gazebo)

```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/halmstad_ws/src/lrs_halmstad/clearpath rviz:=false world:=orchard use_sim_time:=true
```

Arguments:
- `setup_path`: path to the Clearpath setup folder in this workspace
- `rviz`: `true` or `false` (start RViz automatically or not)
- `world`: map to load. Options: `construction`, `office`, `orchard`, `pipeline`, `solar_farm`, `warehouse`
- `use_sim_time`: `true` for Gazebo simulation time

Notes:
- Husky cmd topic is `/a201_0000/cmd_vel`

## 2. Spawn UAVs (with cameras)

```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard
```

Arguments:
- `world`: world name (should match the running simulator world, for example `orchard`)

Useful topics:
- `/dji0/camera0/image_raw`
- `/dji1/camera0/image_raw`
- `/dji2/camera0/image_raw`

## 3. Monitor camera images

```bash
ros2 run  rqt_image_view rqt_image_view dummy
```

How to use:
- Opens `rqt_image_view`
- Select an image topic (for example `/dji0/camera0/image_raw`) in the GUI

## 4. Monitor lidar / robot state in RViz

```bash
rviz2 -d ~/halmstad_ws/src/lrs_halmstad/clearpath/husky.rviz --ros-args -r /tf:=/a201_0000/tf -r /tf_static:=/a201_0000/tf_static -p use_sim_time:=true
```

Arguments:
- `-d`: RViz config file to load
- `-r /tf:=...`: remap TF topic to the simulated Husky namespace
- `-r /tf_static:=...`: remap static TF topic to the simulated Husky namespace
- `-p use_sim_time:=true`: use Gazebo simulation time

## 5. Move a UAV camera (teleport/set pose)

```bash
ros2 run lrs_halmstad command --ros-args -p command:=setpose -p x:=3.0 -p z:=15.0 -p pitch:=-65.0 -p yaw:=0.0 -p name:=dji1
```

Arguments:
- `command:=setpose`: run the set-pose action
- `x`: target x position
- `z`: target z (height)
- `pitch`: camera pitch angle (degrees)
- `yaw`: yaw angle (degrees)
- `name`: UAV name (for example `dji0`, `dji1`, `dji2`)

## 6. Run UAV sweep/scan motion

```bash
ros2 run lrs_halmstad command --ros-args -p command:=scan -p x:=10.0 -p y:=10.0 -p z:=15.0 -p pitch:=-89.0 -p yaw:=0.0 -p name:=dji0
```

Arguments:
- `command:=scan`: run scan/sweep motion
- `x`: scan center/target x
- `y`: scan center/target y
- `z`: flight height
- `pitch`: camera pitch angle (degrees)
- `yaw`: initial yaw angle (degrees)
- `name`: UAV name (for example `dji0`, `dji1`, `dji2`)

