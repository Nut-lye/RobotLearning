# 模块C机器人仿真导航操作流程

## 1. 环境准备

打开终端，执行：

```bash
cd ~/ros2_ws
source ~/.bashrc
```

构建功能包：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select module_c_nav_teaching
source install/setup.bash
```

## 2. 文件结构

```text
module_c_nav_teaching/
├── worlds/    Gazebo 场景文件
├── maps/      导航地图文件
├── config/    Nav2 参数文件
├── launch/    启动文件
└── rviz/      RViz 配置文件
```

本流程使用：

```text
world: 02_open_obstacles.world
map:   02_open_obstacles.yaml
start: x=0.0, y=0.0
```

## 3. 启动 Gazebo 场景

终端 1：

```bash
cd ~/ros2_ws
source ~/.bashrc
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=02_open_obstacles.world x_pose:=0.0 y_pose:=0.0
```

检查 Gazebo 模型：

```bash
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}"
```

模型列表中应包含：

```text
waffle
```

## 4. 启动导航系统

终端 2：

```bash
cd ~/ros2_ws
source ~/.bashrc
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=02_open_obstacles.yaml
```

检查 Nav2 节点：

```bash
ros2 node list --no-daemon | grep -E 'map_server|amcl|planner_server|controller_server|bt_navigator|costmap'
```

应包含：

```text
/map_server
/amcl
/planner_server
/controller_server
/bt_navigator
/local_costmap/local_costmap
```

## 5. 查看地图信息

查看 `/map` 基本信息：

```bash
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --no-daemon \
  --qos-durability transient_local \
  --qos-reliability reliable \
  --field info
```

示例输出：

```text
resolution: 0.05
width: 200
height: 160
origin:
  position:
    x: -5.0
    y: -4.0
```

地图范围：

```text
x: -5.0 到 5.0
y: -4.0 到 4.0
```

## 6. 发布初始位姿

RViz 操作：

```text
2D Pose Estimate
位置：x=0.0, y=0.0
方向：朝 x 正方向
```

命令发布：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      w: 1.0
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
"
```

检查定位变换：

```bash
ros2 run tf2_ros tf2_echo map odom
```

持续输出坐标变换表示定位生效。

## 7. 发布导航目标

RViz 操作：

```text
Nav2 Goal
目标点：地图空闲区域
```

命令发布目标点 1：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: map
  pose:
    position:
      x: 3.0
      y: 2.0
      z: 0.0
    orientation:
      w: 1.0
behavior_tree: ''
" --feedback
```

命令发布目标点 2：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: map
  pose:
    position:
      x: -3.0
      y: 2.0
      z: 0.0
    orientation:
      w: 1.0
behavior_tree: ''
" --feedback
```

查看全局路径：

```bash
ros2 topic echo /plan --once --no-daemon
```

查看速度输出：

```bash
ros2 topic echo /cmd_vel --once
```

## 8. 手动速度发布

手动速度用于底盘运动测试。

前进：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

左转：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

导航任务运行时不执行手动 `/cmd_vel` 发布。

## 9. 导航参数

参数文件：

```text
~/ros2_ws/src/module_c_nav_teaching/config/nav2_params_module_c.yaml
```

主要参数：

```text
controller_server:
  FollowPath.max_vel_x: 0.35
  FollowPath.max_speed_xy: 0.35
  FollowPath.max_vel_theta: 1.0
  FollowPath.BaseObstacle.scale: 0.08

local_costmap:
  robot_radius: 0.22
  inflation_layer.inflation_radius: 0.70
  inflation_layer.cost_scaling_factor: 3.5

global_costmap:
  robot_radius: 0.22
  inflation_layer.inflation_radius: 0.65
  inflation_layer.cost_scaling_factor: 3.0

planner_server:
  GridBased.tolerance: 0.35
```

查看运行参数：

```bash
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_speed_xy
ros2 param get /controller_server FollowPath.BaseObstacle.scale
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
```

临时设置速度：
/controller_server 负责接收全局路径，并计算出底盘电机实时需要的线速度和角速度。
```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.30
ros2 param set /controller_server FollowPath.max_speed_xy 0.30
```

* 1.max_vel_x 含义： 限制机器人在 X 轴（正前方）的最大线速度
* 2.max_speed_xy 含义： 底盘的整体平移速度绝不会超过 0.30 m/s

临时设置膨胀半径：

```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.80
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.75
```

* 1.local_costmap.inflation_radius 含义： 局部代价地图的障碍物膨胀半径
* 2.inflation_layer.inflation_radius 含义： 全局代价地图的障碍物膨胀半径

参数文件修改后，重启终端 2 的导航启动命令。

## 10. 避障观察

查看激光数据：

```bash
ros2 topic echo /scan --once
```

查看局部代价地图：

```bash
ros2 topic echo /local_costmap/costmap nav_msgs/msg/OccupancyGrid --once --no-daemon \
  --qos-durability transient_local \
  --field info
```

查看全局代价地图：

```bash
ros2 topic echo /global_costmap/costmap nav_msgs/msg/OccupancyGrid --once --no-daemon \
  --qos-durability transient_local \
  --field info
```

RViz 显示项：

```text
Map
LaserScan
Global Costmap
Local Costmap
Path
```

## 11. 场景切换

更换地图与出生点时，需要同时设置：

```text
Gazebo world
Nav2 map
Gazebo 出生点 x_pose, y_pose
AMCL 初始位姿 /initialpose
```

启动格式：

```bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=<world文件> x_pose:=<出生点x> y_pose:=<出生点y>
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=<map文件>
```

初始位姿发布格式：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
  pose:
    position:
      x: <出生点x>
      y: <出生点y>
      z: 0.0
    orientation:
      w: 1.0
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
"
```

已知场景对应关系：

```text
01_tb3_world.world       -> 01_tb3_world.yaml       -> x=-2.0, y=-0.5
02_open_obstacles.world  -> 02_open_obstacles.yaml  -> x=0.0,  y=0.0
03_narrow_door.world     -> 03_narrow_door.yaml     -> x=-3.3, y=0.0
```

开放障碍场景：

```bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=02_open_obstacles.world x_pose:=0.0 y_pose:=0.0
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=02_open_obstacles.yaml
```

窄门场景：

```bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=03_narrow_door.world x_pose:=-3.3 y_pose:=0.0
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=03_narrow_door.yaml
```

## 12. SLAM 建图

终端 1：

```bash
cd ~/ros2_ws
source ~/.bashrc
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=04_local_room_slam.world x_pose:=0.0 y_pose:=0.0
```

终端 2：

```bash
cd ~/ros2_ws
source ~/.bashrc
ros2 launch module_c_nav_teaching 03_slam_mapping.launch.py
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/module_c_nav_teaching/maps/my_slam_map
```

## 13. Behavior Tree 演示

功能包：

```text
module_c_bt_demo
```

文件结构：

```text
module_c_bt_demo/
├── behavior_trees/module_c_demo_replanning.xml
├── behavior_trees/module_c_demo_waypoints.xml
├── launch/send_bt_goal.launch.py
├── launch/send_bt_waypoints.launch.py
├── module_c_bt_demo/send_bt_goal.py
└── module_c_bt_demo/send_bt_waypoints.py
```

构建：

```bash
cd ~/ros2_ws
source ~/.bashrc
colcon build --symlink-install --packages-select module_c_bt_demo
source install/setup.bash
```

启动条件：

```text
Gazebo 已启动
Nav2 已启动
/initialpose 已发布
map -> odom 已生成
```

发送 Behavior Tree 导航目标：

```bash
ros2 launch module_c_bt_demo send_bt_goal.launch.py x:=3.0 y:=2.0 yaw:=0.0
```

切换目标点：

```bash
ros2 launch module_c_bt_demo send_bt_goal.launch.py x:=-3.0 y:=2.0 yaw:=0.0
```

发送多目标点 Behavior Tree 导航目标：

```bash
ros2 launch module_c_bt_demo send_bt_waypoints.launch.py waypoint1:=1.5,1.8,0.0 waypoint2:=3.0,2.0,0.0
```

多目标点格式：

```text
waypoint1:=x,y,yaw
waypoint2:=x,y,yaw
```

执行顺序：

```text
先到 waypoint1
再到 waypoint2
```

查看 Behavior Tree XML：

```bash
sed -n '1,120p' ~/ros2_ws/src/module_c_bt_demo/behavior_trees/module_c_demo_replanning.xml
sed -n '1,120p' ~/ros2_ws/src/module_c_bt_demo/behavior_trees/module_c_demo_waypoints.xml
```

运行效果：

```text
终端输出 action feedback
RViz 显示全局路径
机器人按 Behavior Tree 执行路径规划、路径跟踪和恢复行为
```

反馈字段：

```text
distance_remaining
number_of_recoveries
number_of_poses_remaining
```

RViz 显示项：

```text
Path
Global Costmap
Local Costmap
RobotModel
```

## 14. 进程清理

```bash
pkill -9 gzserver gzclient rviz2 component_container_isolated robot_state_publisher
ros2 daemon stop
ros2 daemon start
```

## 15. 常见检查

检查话题：

```bash
ros2 topic list --no-daemon
```

检查节点：

```bash
ros2 node list --no-daemon
```

检查 Gazebo 模型：

```bash
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}"
```

检查地图到里程计变换：

```bash
ros2 run tf2_ros tf2_echo map odom
```
