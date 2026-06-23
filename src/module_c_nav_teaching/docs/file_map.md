# Module C Teaching File Map

Use this package as a source-code version of the practice notes.

| File or folder | Teaching meaning |
| --- | --- |
| `worlds/` | Gazebo scenes. This answers "load the preset navigation scene". |
| `maps/` | Occupancy maps for known-map navigation. This answers "where does AMCL localize". |
| `config/nav2_params_baseline.yaml` | Official TurtleBot3 waffle parameters before class tuning. |
| `config/nav2_params_module_c.yaml` | Parameters students edit and screenshot. |
| `config/slam_toolbox_online_async.yaml` | Parameters for map-building practice. |
| `launch/01_gazebo_world.launch.py` | Starts Gazebo and spawns TurtleBot3. |
| `launch/02_nav2_known_map.launch.py` | Starts map server, AMCL, Nav2, and RViz. |
| `launch/03_slam_mapping.launch.py` | Starts SLAM plus Nav2 navigation servers. |
| `rviz/module_c_nav2_view.rviz` | Clean Nav2 RViz layout used by the launch files. |
| `scenarios/module_c_scenarios.yaml` | Shows which world matches which map and lesson. |

## Suggested Class Order

1. Run `01_tb3_world.world` with `01_tb3_world.yaml`.
2. Run `02_open_obstacles.world` with `02_open_obstacles.yaml`.
3. Run `03_narrow_door.world` with `03_narrow_door.yaml` and tune costmap inflation.
4. Run `04_local_room_slam.world` with `03_slam_mapping.launch.py` and build a new map.
