# 调参与故障诊断手册

这篇文档用于机器人已经能启动，但导航效果不好时。不要一上来就乱改参数，先按“数据链路 -> 定位 -> 代价地图 -> 规划器 -> 控制器”的顺序排查。

## 1. 先判断问题发生在哪一层

完整导航链路：

```text
Gazebo/底盘
  -> /scan /odom /tf
  -> SLAM 或 AMCL
  -> /map 与 map->odom
  -> global_costmap / local_costmap
  -> planner_server
  -> controller_server
  -> /cmd_vel
```

排查时不要跳层。比如 TF 都不通时，改 `inflation_radius` 没意义。

## 2. 基础生命体征检查

### 2.1 节点是否起来

```bash
ros2 node list
ros2 lifecycle nodes
```

重点看：

- `planner_server`
- `controller_server`
- `bt_navigator`
- `behavior_server`
- `map_server` 或 `slam_toolbox`
- `amcl`
- `global_costmap`
- `local_costmap`

### 2.2 传感器是否有数据

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

如果 `/scan` 没有，先查 Gazebo 雷达插件和话题名。

如果 `/odom` 没有，先查 Gazebo 差速插件、EKF、底盘驱动。

### 2.3 TF 是否完整

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo map odom
```

结论：

- `odom -> base_link` 缺失：底盘/里程计/EKF 问题。
- `base_link -> lidar_link` 缺失：URDF 或 robot_state_publisher 问题。
- `map -> odom` 缺失：SLAM 或 AMCL 问题。

## 3. RViz 观察项

RViz 至少打开：

- `RobotModel`
- `TF`
- `/scan`
- `/map`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/plan`
- `/local_plan` 或 controller 轨迹显示
- `/local_costmap/published_footprint`
- `/global_costmap/published_footprint`

判断标准：

- 雷达红线应和地图墙体重合。
- 全局代价地图应覆盖整张地图。
- 局部代价地图应围绕机器人滚动。
- footprint 应贴近机器人真实外形。
- 下发目标后应出现全局路径。

## 4. 典型问题：机器人完全不动

检查：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom --once
ros2 lifecycle nodes
```

可能原因：

- Nav2 生命周期节点没 active。
- 没有初始位姿，AMCL 没收敛。
- TF 缺失。
- 目标点在障碍物或未知区域。
- `/cmd_vel` 有输出但 Gazebo 插件没订阅正确话题。

处理顺序：

1. 确认 `use_sim_time:=True`。
2. 重新给 `2D Pose Estimate`。
3. 检查 TF 链。
4. 检查 `/cmd_vel`。
5. 检查 Gazebo 差速插件 namespace 和 remap。

## 5. 典型问题：能规划但贴墙

优先改 costmap：

```yaml
inflation_layer:
  inflation_radius: 0.55
  cost_scaling_factor: 3.0
```

调参方向：

- 增大 `inflation_radius`：让障碍物周围安全区更大。
- 减小 `cost_scaling_factor`：让代价下降更慢，路径更偏向走中间。
- 增大 `robot_radius` 或 footprint：让机器人认为自己更大，更保守。

注意：

- 改太保守会导致过不了窄门。
- 每次改完要重启 Nav2 或重新加载参数。

## 6. 典型问题：不敢过门或窄通道

优先看：

```yaml
robot_radius
footprint
inflation_radius
cost_scaling_factor
```

处理方向：

- 减小 `inflation_radius`。
- 检查 `footprint` 是否按真实尺寸写，别把长宽写反或写大。
- 如果全局规划都不进门，看 global costmap。
- 如果全局路径进门但局部控制不走，看 local costmap 和 controller。

RViz 必看：

```text
/local_costmap/published_footprint
/global_costmap/published_footprint
```

## 7. 典型问题：规划失败

检查：

```bash
ros2 topic echo /map --once
ros2 topic echo /global_costmap/costmap --once
ros2 run tf2_ros tf2_echo map base_link
```

可能原因：

- `map -> odom` 不存在。
- 目标点落在障碍物上。
- `allow_unknown` 与地图状态不匹配。
- 全局代价地图没收到静态地图。
- 地图分辨率或 origin 不对。

参数方向：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      tolerance: 0.125
      allow_unknown: true
```

## 8. 典型问题：到终点附近转圈或失败

优先看 controller 和 goal checker：

```yaml
controller_server:
  ros__parameters:
    failure_tolerance: 0.3
    goal_checker_plugins: ["general_goal_checker"]
```

常见处理：

- 放宽 `xy_goal_tolerance`。
- 放宽 `yaw_goal_tolerance`。
- 降低 `max_vel_theta`。
- 检查目标朝向是否贴墙或不可达。
- 检查 `RotateToGoal` critic 权重是否过激。

## 9. 典型问题：机器人来回摆动

可能原因：

- 控制器速度太大。
- 局部代价地图太小。
- footprint 不准。
- controller critics 权重不合适。
- odom 噪声或 TF 抖动。

处理方向：

```yaml
FollowPath:
  max_vel_x: 0.18
  max_vel_theta: 0.8
  acc_lim_x: 1.0
  acc_lim_theta: 1.5
```

同时检查：

```bash
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 10. 参数修改优先级

比赛或训练中，优先调这些：

1. `use_sim_time`
2. `robot_radius` / `footprint`
3. `inflation_radius`
4. `cost_scaling_factor`
5. `obstacle_max_range`
6. `raytrace_max_range`
7. local costmap `width` / `height`
8. planner `tolerance` / `allow_unknown`
9. controller `max_vel_x` / `max_vel_theta`
10. goal checker tolerance

不要一开始就改行为树或写插件。普通到点导航的问题，八成在 TF、定位、costmap、footprint、controller 参数里。

