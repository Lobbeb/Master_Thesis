# Running the shit with everything to capture the data

## Follow mode:
För att starta navigation och localization på UGVn behövs:

- Genererade waypoints (routes) för enstaka rutter: **halmstad_ws/src/lrs_halmstad/config/baylands_waypoints**
    - Eller den stora routen för hela mappen: **halmstad_ws/src/lrs_halmstad/config/baylands_waypoints.yaml**
- En startpunkt/waypoint (kan egentligen vara vart som helst med lättare för UGVn att klara målet om den är förbestämd.)

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
Tips: lägg till detta i din *.bashrc* fil
```bash 
build() {
    colcon build --symlink-install --packages-select lrs_halmstad lrs_halmstad_gui_plugins pointcloud_to_laserscan
    source /opt/ros/jazzy/setup.bash
    source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
    source ~/halmstad_ws/install/setup.bash
```
```bash
./run.sh.sh nav2_tuning start nav2_goals:= ... waypoint:= ... spawn_uav:=true
```
Om du explicit ska skriva varenda argument:
```bash
./run.sh.sh nav2_tuning start|restart|stack_stop|follow|route_stop|stop|attach|status
  world:=baylands
  profile:=standard|minimal
  lidar:=2d|3d
  pause_after_goal_s:=10.0
  gui:=true|false 
  perspective:=NAV2 
  start_rqt:=true|false 
  start_collision_monitor:=true|false 
  map:=maps/baylands.yaml
  sensor_profile:=full|nav|minimal 
  clock_mode:=guarded|direct
  mute_ugv_camera:=true|false
  start_optional_teleop:=true|false
  spawn_uav:=true|false
  with_route_driver:=true|false 
  follow_start_delay_s:=3.0
  uav_camera_update_rate:=2
  gazebo_ready_timeout_s:=30
  gazebo_post_ready_delay_s:=20
  spawn_post_delay_s:=5
  rebuild:=true|false
  session:=name 
  tmux_attach:=true|false
  nav2_goals:=rotundan 
  waypoint:=rotundan_0
  dry_run:=true|false
```
Examples:
 - ./run.sh.sh nav2_tuning start
 - ./run.sh.sh nav2_tuning start profile:=minimal
 - ./run.sh.sh nav2_tuning start waypoint:=rotundan_0
 - ./run.sh.sh nav2_tuning restart
 - ./run.sh.sh nav2_tuning stack_stop
 - ./run.sh.sh nav2_tuning follow
 - ./run.sh.sh nav2_tuning route_stop
 - ./run.sh.sh nav2_tuning stop
 - ./run.sh.sh nav2_tuning attach

*Nav2 Goals/routes du kan välja på: (I ordern dem är i den stora rutten):*
- rotundan
- road_to_west
- parkinglot_west
- road_to_spawn
- spawn
- road_to_east
- parkinglot_east
- road_to_strip
- strip

*Enstaka rutter inte kopplade till baylands_waypoints rutten:*
- art
- playground

Medans du kör kan du göra ändringar i koden (yaml filerna eller python scripts) och restarta utan att bygga om:

```bash
./run.sh nav2_tuning restart
```
Då kör den om allt och startar på samma waypoint som förut och med samma rutt.
Vill du börja ifrån en ny waypoint men med samma rutt skriver du:
```bash
./run.sh nav2_tuning restart waypoint:= NEW WAYPOINT HERE
```
Men om du vill påbörja en ny rutt så måste den byggas m:
```bash
./run.sh nav2_tuning stop
./run.sh nav2_tuning start nav2_goals:= NEW ROUTE HERE
```
Väljs ingen waypoint tas den första i routen automatiskt.

Skulle det hänga sig och det ligger massa bakgrunds processer så kör:
```bash
./run.sh kill_all_ros2
```
Det dödar ALLT.

## Visualisation:
Kör bara RViz, det är bäst.
```bash
./run.sh nav2_rviz lidar:=3d config:= follow.rviz
```
Configs att välja på (ligger i halmstad_ws/perspectives/rviz)
- waypoints_testing
- localization_testing
- follow <-- KÖR MED DENNA
Jag la till images för kamerorna i där, men vill du köra med rqt så kan du göra det:
```bash
./run.sh rqt_perspectives
```

## Dataset Collection
```bash
./run.sh capture_dataset
```
