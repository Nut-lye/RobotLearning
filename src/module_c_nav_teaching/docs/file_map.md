# 模块 C 教学文件结构说明

可将此功能包作为实践笔记的代码版本使用。

| 文件或文件夹 | 教学含义/作用 |
| --- | --- |
| `worlds/` | Gazebo 仿真场景。对应“加载预设导航场景”环节。 |
| `maps/` | 用于已知地图导航的占据栅格地图。对应“AMCL 在何处进行定位”。 |
| `config/nav2_params_baseline.yaml` | 课堂调参练习前的 TurtleBot3 waffle 官方默认参数。 |
| `config/nav2_params_module_c.yaml` | 供学生修改并截图记录的参数文件。 |
| `config/slam_toolbox_online_async.yaml` | 用于建图实践练习的参数配置。 |
| `launch/01_gazebo_world.launch.py` | 启动 Gazebo 仿真环境并生成 TurtleBot3 机器人。 |
| `launch/02_nav2_known_map.launch.py` | 启动地图服务器（map_server）、AMCL 定位模块、Nav2 导航栈以及 RViz。 |
| `launch/03_slam_mapping.launch.py` | 同时启动 SLAM 建图算法与 Nav2 导航服务。 |
| `rviz/module_c_nav2_view.rviz` | launch 文件所调用的、专为 Nav2 定制的简洁版 RViz 布局配置文件。 |
| `scenarios/module_c_scenarios.yaml` | 记录哪个仿真场景（world）对应哪张地图（map）以及哪节课程。 |

## 推荐课程/练习顺序

1. 运行 `01_tb3_world.world` 及其配套地图 `01_tb3_world.yaml`。
2. 运行 `02_open_obstacles.world` 及其配套地图 `02_open_obstacles.yaml`。
3. 运行 `03_narrow_door.world` 及其配套地图 `03_narrow_door.yaml`，并调节代价地图膨胀（costmap inflation）参数。
4. 使用 `03_slam_mapping.launch.py` 运行 `04_local_room_slam.world` 场景，并完成一张新地图的构建。