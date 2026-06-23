# 模块 C：机器人仿真导航教学实现

这篇按 `/home/nuts_2204/ros2_ws/doc/nav_doc/人工智能训练师-技术文件.pdf` 样题里的模块 C 来写。

模块 C 的任务要求可以拆成四句话：

1. 加载预设导航场景，启动导航功能。
2. 调整导航参数，确保路径规划流畅。
3. 启动导航，机器人按要求抵达终点。
4. 对修改的参数截图，粘贴到答题报告。

现在推荐使用新的课程包：

```text
src/module_c_nav_teaching/
├── config/       # baseline 参数、模块 C 调参文件、SLAM 参数
├── docs/         # 每类文件的教学含义
├── launch/       # 按步骤拆开的启动文件
├── maps/         # 已知地图导航用 map
├── models/       # 比赛给的 Gazebo 模型放这里
├── rviz/         # 干净的 Nav2 RViz 配置
├── scenarios/    # world/map/config 对应表
└── worlds/       # Gazebo 场景
```

旧的 `src/tb3_nav_practice` 保留为最小跑通包；上课建议用 `module_c_nav_teaching`，因为它把“哪个文件对应哪个任务步骤”拆清楚了。

## 1. 先构建课程包

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select module_c_nav_teaching
source install/setup.bash
```

使用 `--symlink-install` 是为了教学方便：修改 `src/module_c_nav_teaching/config/nav2_params_module_c.yaml`
后，重启 Nav2 launch 就能使用新参数。

## 2. 文件和任务要求怎么对应

| 任务要求 | 对应文件 |
| --- | --- |
| 加载预设导航场景 | `worlds/*.world` 和 `launch/01_gazebo_world.launch.py` |
| 启动导航功能 | `launch/02_nav2_known_map.launch.py` 或 `launch/03_slam_mapping.launch.py` |
| 调整导航参数 | `config/nav2_params_module_c.yaml` |
| 截图修改参数 | 截 `config/nav2_params_module_c.yaml` 中带 `Module C tuning` 的区域 |
| 管理 world/map 对应关系 | `scenarios/module_c_scenarios.yaml` |

## 3. 已经准备好的 world 和 map

已知地图导航：

| 场景 | world | map | 教学重点 |
| --- | --- | --- | --- |
| 官方基准 | `01_tb3_world.world` | `01_tb3_world.yaml` | 跑通标准 AMCL + Nav2 |
| 开放障碍 | `02_open_obstacles.world` | `02_open_obstacles.yaml` | 看 costmap 和 planner 如何绕障 |
| 窄门走廊 | `03_narrow_door.world` | `03_narrow_door.yaml` | 讲 `inflation_radius`、`robot_radius`、目标容差 |

未知地图/SLAM 练习：

| 场景 | world | map |
| --- | --- | --- |
| 本地房间场景 | `04_local_room_slam.world` | 由 `slam_toolbox` 生成 |
| 小型 DQN 场景 | `05_tb3_dqn_stage1_slam.world` | 由 `slam_toolbox` 生成 |

完整对应表在：

```text
src/module_c_nav_teaching/scenarios/module_c_scenarios.yaml
```

## 4. 为什么要提前准备 world 和 models

Gazebo 的 world 文件里常出现：

```xml
<uri>model://turtlebot3_world</uri>
<uri>model://ground_plane</uri>
<uri>model://sun</uri>
```

如果 Gazebo 在本地找不到这些模型，它可能会访问在线模型库，终端里常见：

```text
Getting models from http://models.gazebosim.org
```

比赛或教学机网络不稳定时，这一步会卡住，后面的 `/spawn_entity` 服务也起不来。

`launch/01_gazebo_world.launch.py` 已经设置：

```python
SetEnvironmentVariable('GAZEBO_MODEL_PATH', ...)
SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')
SetEnvironmentVariable('SVGA_VGPU10', '0')
```

含义：

- 优先从 `src/module_c_nav_teaching/models` 找赛题给的本地模型。
- 再从 TurtleBot3 官方安装目录找 `turtlebot3_world`、`turtlebot3_dqn_world` 等模型。
- 再从 `/usr/share/gazebo-11/models` 找 `ground_plane`、`sun`。
- 清空 `GAZEBO_MODEL_DATABASE_URI`，避免运行时去网上下载模型。

如果赛题额外给了模型文件夹，例如：

```text
my_wall/
├── model.config
└── model.sdf
```

把整个 `my_wall` 文件夹放到：

```text
src/module_c_nav_teaching/models/my_wall
```

然后重新构建：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select module_c_nav_teaching
source install/setup.bash
```

## 5. 已知地图导航：模块 C 标准流程

终端 1：启动 Gazebo 场景。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=02_open_obstacles.world x_pose:=-3.0 y_pose:=-2.0
```

终端 2：启动 Nav2、AMCL、RViz。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=02_open_obstacles.yaml
```

这个启动文件会加载：

```text
src/module_c_nav_teaching/rviz/module_c_nav2_view.rviz
```

它不使用 TurtleBot3 官方 RViz 配置里的 Docking/Selector 面板，因此不会被 `nav2_rviz_plugins/Docking` 报错干扰。

换成窄门走廊练习：

```bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=03_narrow_door.world x_pose:=-3.3 y_pose:=0.0
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=03_narrow_door.yaml
```

RViz 操作：

1. 点击 `2D Pose Estimate`。
2. 对照 Gazebo 中机器人真实位置，在 RViz 地图上拖出初始位姿。
3. 看红色雷达线是否和地图墙体重合。
4. 点击 `Nav2 Goal` 下发终点。
5. 观察全局路径、局部代价地图和机器人运动。

## 6. 未知地图导航：SLAM 练习流程

终端 1：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=04_local_room_slam.world x_pose:=0.0 y_pose:=0.0
```

终端 2：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch module_c_nav_teaching 03_slam_mapping.launch.py
```

保存地图：

```bash
mkdir -p ~/ros2_ws/src/module_c_nav_teaching/maps/generated
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/module_c_nav_teaching/maps/generated/local_room_map
```

## 7. 模块 C 要改哪些参数

基准参数：

```text
src/module_c_nav_teaching/config/nav2_params_baseline.yaml
```

课堂调参文件：

```text
src/module_c_nav_teaching/config/nav2_params_module_c.yaml
```

已标注 `Module C tuning` 的位置包括：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 15.0
    general_goal_checker:
      xy_goal_tolerance: 0.20
```

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.22
      inflation_layer:
        inflation_radius: 0.70
        cost_scaling_factor: 3.5
```

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.22
      inflation_layer:
        inflation_radius: 0.65
        cost_scaling_factor: 3.0
```

```yaml
planner_server:
  ros__parameters:
    GridBased:
      tolerance: 0.35
      allow_unknown: true
```

修改参数后，只需要重启 Nav2：

```bash
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=03_narrow_door.yaml params_file:=nav2_params_module_c.yaml
```

如果 Gazebo 已经正常运行，不需要重启 Gazebo。

## 8. 运行时检查命令

确认传感器：

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

确认 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

确认 Nav2 节点：

```bash
ros2 lifecycle nodes
ros2 node list | grep -E 'planner|controller|bt_navigator|amcl|map_server|costmap'
```

判断顺序：

1. 没有 `/scan` 或 `/odom`，先修 Gazebo 和机器人模型。
2. 没有 `odom -> base_link`，先修机器人里程计/TF。
3. 没有 `map -> odom`，先做 `2D Pose Estimate` 或检查 AMCL/SLAM。
4. 能规划但走不好，再改 `nav2_params_module_c.yaml`。

## 9. 答题报告截图清单

至少截图四类：

1. Gazebo 中机器人和预设场景。
2. RViz 中地图、机器人、雷达、代价地图。
3. 下发 `Nav2 Goal` 后生成的路径和机器人运动过程。
4. `src/module_c_nav_teaching/config/nav2_params_module_c.yaml` 中带 `Module C tuning` 的参数区域。

报告文字可以这样写：

```text
本任务使用 module_c_nav_teaching 启动 TurtleBot3 Gazebo 预设导航场景，
使用本地 map.yaml 与 nav2_params_module_c.yaml 启动 Nav2。
为使路径规划更流畅，调整了 local/global costmap 的 inflation_radius、
controller_frequency、xy_goal_tolerance 和 planner tolerance。
调整后在 RViz 中下发 Nav2 Goal，机器人能够自动规划路径并抵达终点。
```
