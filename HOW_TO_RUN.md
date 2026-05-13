# Running the shit with everything to capture the data

## Follow mode

För att starta navigation och localization på UGVn behövs:

* Genererade waypoints (routes) för enstaka rutter: **halmstad_ws/src/lrs_halmstad/config/baylands_waypoints**
* Eller den stora routen för hela mappen: **halmstad_ws/src/lrs_halmstad/config/baylands_waypoints.yaml**
* En startpunkt/waypoint (kan egentligen vara vart som helst med lättare för UGVn att klara målet om den är förbestämd.)

Detta sätts automatiskt av olika scripts om du har med i dina args:

```bash
waypoint:=...  (initialposition)        nav2_goals:=...  (route)
```

Exempel:
```bash
waypoint:=parkinglot_west_0          nav2_goals:=parkinglot_west
```

Så för att köra med en UGV och vanlig follow (utan yolo) på UAVn så är bästa sätt:

```bash
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
source ~/halmstad_ws/install/setup.bash
```

> *Tips - lägg till detta i din *.bashrc* fil:*

```bash
build() {
    colcon build --symlink-install --packages-select lrs_halmstad lrs_halmstad_gui_plugins pointcloud_to_laserscan
    source /opt/ros/jazzy/setup.bash
    source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
    source ~/halmstad_ws/install/setup.bash
}
```

```bash
./run.sh nav2_tuning start nav2_goals:=... waypoint:=... spawn_uav:=true
```

Om du explicit ska skriva varenda argument:

```bash
./run.sh nav2_tuning start|restart|stack_stop|follow|route_stop|stop|attach|status
  world:=baylands
  profile:=standard|minimal
  lidar:=2d|3d
  pause_after_goal_s:=0.0
  gui:=true|false 
  perspective:=NAV2 
  start_rqt:=true|false 
  start_collision_monitor:=true|false 
  map:=maps/baylands.yaml
  clock_mode:=guarded|direct
  mute_ugv_camera:=true|false
  start_optional_teleop:=true|false
  spawn_uav:=true|false
  start_camera_tracker:=true|false
  with_route_driver:=true|false 
  follow_start_delay_s:=15.0
  uav_camera_update_rate:=10
  spawn_post_delay_s:=10
  rebuild:=true|false
  session:=name 
  tmux_attach:=true|false
  nav2_goals:=rotundan 
  waypoint:=rotundan_0
  dry_run:=true|false
```

Examples:

* ./run.sh nav2_tuning start
* ./run.sh nav2_tuning start profile:=minimal
* ./run.sh nav2_tuning start waypoint:=rotundan_0
* ./run.sh nav2_tuning start waypoint:=strip_2 nav2_goals:=strip
* ./run.sh nav2_tuning start spawn_uav:=true
* ./run.sh nav2_tuning restart
* ./run.sh nav2_tuning stack_stop
* ./run.sh nav2_tuning follow
* ./run.sh nav2_tuning route_stop
* ./run.sh nav2_tuning stop
* ./run.sh nav2_tuning attach

 Sen attacha med tmux för att kika på vad som händer:

```bash
 tmux attach
```

Första sidan visar Gazebo starten och UAV spawning, nästa sida innehåller Localization, Nav2 och Follow koden. (Ctrl+B, sen klicka på N för att bläddra)

*Nav2 Goals/routes du kan välja på: (I ordern dem är i den stora rutten):*

* rotundan          (finished)
* road_to_west      (finished)
* parkinglot_west   (finished)
* road_to_spawn     (needs tuning)
* spawn             (finished)
* road_to_east      (needs tuning)
* parkinglot_east   (needs tuning)
* road_to_strip     (needs tuning)
* strip             (needs tuning)

*Enstaka rutter inte kopplade till baylands_waypoints rutten:*

* art
* playground

*Waypoints du kan välja på:*

* rotundan_0 \- 8
* road_to_west_0 \- 8
* parkinglot_west_0 \- 21
* road_to_spawn_0 \- 10
* spawn_0 \- 8
* road_to_east_0 \- 11
* parkinglot_east_0 \- 9
* road_to_strip_0 \- 9
* strip_0 \- 16

Medans du kör kan du göra ändringar i koden (yaml filerna eller python scripts) och restarta utan att bygga om:

```bash
./run.sh nav2_tuning restart
```

Då kör den om allt och startar på samma waypoint som förut och med samma rutt.
Vill du börja ifrån en ny waypoint men med samma rutt skriver du:

```bash
./run.sh nav2_tuning restart waypoint:=NEW_WAYPOINT_HERE
```

Vill du byta rutt i samma session, ge både waypoint och route:

```bash
./run.sh nav2_tuning restart waypoint:=strip_0 nav2_goals:=strip
```

Vill du påbörja en helt ny session:

```bash
./run.sh nav2_tuning stop
./run.sh nav2_tuning start nav2_goals:=NEW_ROUTE_HERE
```

Väljs ingen waypoint tas den första i routen automatiskt.

Lidar intställningar för varje waypoint ligger i ~*src/lrs_halmstad/config/baylands_route_lidar.yaml*
De sätts automatiskt av localization vid start och av nav2-drivern vid waypoint-övergångar.
Manuellt kan det tunas med:

```bash
ros2 param set /a201_0000/pointcloud_to_laserscan min_height -0.4
ros2 param set /a201_0000/pointcloud_to_laserscan max_height 0.5
./run.sh pc2ls_sweep sweep:=min min_start:=-0.7 max:=0.5 step:=0.05 dwell_s:=1
```

Skulle det hänga sig och det ligger massa bakgrunds processer så kör:

```bash
./run.sh kill_all_ros2
```

Det dödar ALLT.

## Visualisation

Kör bara RViz, det är bäst.

```bash
./run.sh nav2_rviz lidar:=3d config:=follow
```

Configs att välja på (ligger i halmstad_ws/src/lrs_halmstad/config/rviz_configs)

* waypoints_testing
* localization_testing
* follow <-- KÖR MED DENNA

Jag la till images för kamerorna i där, men vill du köra med rqt så kan du göra det:

```bash
./run.sh rqt_perspective
```

## Dataset Collection

Alla dataset hamnar under `~/halmstad_ws/datasets/`

### Leader UAV (`dji0`) - automatisk varied capture

```bash
cd ~/halmstad_ws
./run.sh collect_leader_dataset baylands
```

### Leader UAV (`dji0`) - manuell capture

Starta först follow-sim:

```bash
cd ~/halmstad_ws
./run.sh tmux_1to1 baylands mode:=follow waypoint:=parkinglot_west_0 nav2_goals:=parkinglot_west
```

Starta sedan capture:

```bash
cd ~/halmstad_ws
./run.sh capture_dataset baylands out:=datasets/baylands_leader_manual uav_name:=dji0 hz:=0.5 save_overlay:=false save_metadata:=true
```

### Support UAVs (`dji1` och `dji2`) - capture båda samtidigt

Starta först vanlig follow:

```bash
cd ~/halmstad_ws
./run.sh tmux_1to1 baylands mode:=follow waypoint:=parkinglot_west_0 nav2_goals:=parkinglot_west
```

Starta support-UAV:erna:

```bash
cd ~/halmstad_ws
./run.sh support_follow_odom baylands support_with_camera:=true support_bridge_gimbal:=true
```

Starta kamerasvep:

```bash
cd ~/halmstad_ws
./run.sh support_camera_scan baylands yaw_amplitude_deg:=120 period_s:=12 pan_phase_offsets_deg:=0,180 pitch_deg:=-22
```

Starta capture för båda:

```bash
cd ~/halmstad_ws
./run.sh support_capture_pair baylands out:=datasets/baylands_support_pair hz:=1.0 save_overlay:=false
```
