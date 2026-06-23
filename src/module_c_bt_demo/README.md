# module_c_bt_demo

Behavior Tree 导航演示包。

## 1. 功能

```text
send_bt_goal       单目标点 Behavior Tree 导航
send_bt_waypoints  多目标点连续导航
send_bt_sequence   两段式导航：到中间点后等待 3 秒，再去第二个点
```

文件结构：

```text
module_c_bt_demo/
├── behavior_trees/
│   ├── module_c_demo_replanning.xml
│   ├── module_c_demo_goal_wait.xml
│   └── module_c_demo_waypoints.xml
├── launch/
│   ├── send_bt_goal.launch.py
│   ├── send_bt_sequence.launch.py
│   └── send_bt_waypoints.launch.py
└── module_c_bt_demo/
    ├── send_bt_goal.py
    ├── send_bt_sequence.py
    └── send_bt_waypoints.py
```

## 2. 构建

```bash
cd ~/ros2_ws
source ~/.bashrc
colcon build --symlink-install --packages-select module_c_bt_demo
source install/setup.bash
```

## 3. 启动前条件

Gazebo：

```bash
ros2 launch module_c_nav_teaching 01_gazebo_world.launch.py world:=02_open_obstacles.world x_pose:=0.0 y_pose:=0.0
```

Nav2：

```bash
ros2 launch module_c_nav_teaching 02_nav2_known_map.launch.py map:=02_open_obstacles.yaml
```

初始位姿：

```text
RViz -> 2D Pose Estimate
位置：x=0.0, y=0.0
```

## 4. 单目标点导航

```bash
ros2 launch module_c_bt_demo send_bt_goal.launch.py x:=3.0 y:=2.0 yaw:=0.0
```

参数：

```text
x    目标点 x 坐标
y    目标点 y 坐标
yaw  目标朝向，单位 rad
```

## 5. 多目标点导航

默认执行两个目标点：

```bash
ros2 launch module_c_bt_demo send_bt_waypoints.launch.py
```

自定义两个目标点：

```bash
ros2 launch module_c_bt_demo send_bt_waypoints.launch.py waypoint1:=1.5,1.8,0.0 waypoint2:=3.0,2.0,0.0
```

目标点格式：

```text
waypoint1:=x,y,yaw
waypoint2:=x,y,yaw
```

执行顺序：

```text
先到 waypoint1
再到 waypoint2
```

## 6. 分段速度

`send_bt_waypoints` 支持给两个路段设置不同速度。

```bash
ros2 launch module_c_bt_demo send_bt_waypoints.launch.py \
  waypoint1:=1.5,1.8,0.0 \
  waypoint2:=3.0,2.0,0.0 \
  speed1:=0.18 \
  speed2:=0.35
```

含义：

```text
speed1  从起点到 waypoint1 的速度
speed2  从 waypoint1 到 waypoint2 的速度
```

实现方式：

```text
到达不同目标段时，脚本临时修改 /controller_server 的参数：
FollowPath.max_vel_x
FollowPath.max_speed_xy
```

## 7. 中间点等待 3 秒

两段式导航：

```bash
ros2 launch module_c_bt_demo send_bt_sequence.launch.py \
  waypoint1:=0.0,-2.0,0.0 \
  waypoint2:=3.0,-2.0,0.0 \
  speed1:=0.18 \
  speed2:=0.35
```

执行顺序：

```text
1. 设置 speed1
2. 导航到 waypoint1
3. 执行 module_c_demo_goal_wait.xml
4. 在 waypoint1 等待 3 秒
5. 设置 speed2
6. 导航到 waypoint2
```

等待行为位置：

```xml
<Wait wait_duration="3"/>
```

默认点位：

```text
waypoint1: x=0.0, y=-2.0
waypoint2: x=3.0, y=-2.0
```

该路线位于场景下方空旷区域，用于减少贴墙和卡住情况。

## 8. 查看反馈

运行多目标点导航时，终端输出：

```text
distance_remaining
poses_remaining
recoveries
Segment speed set to ...
```

字段含义：

```text
distance_remaining         剩余距离
poses_remaining            剩余目标点数量
recoveries                 恢复行为次数
Segment speed set to ...   当前路段速度设置
```

## 9. 查看 Behavior Tree

单目标点：

```bash
sed -n '1,120p' ~/ros2_ws/src/module_c_bt_demo/behavior_trees/module_c_demo_replanning.xml
```

多目标点：

```bash
sed -n '1,160p' ~/ros2_ws/src/module_c_bt_demo/behavior_trees/module_c_demo_waypoints.xml
```

中间点等待：

```bash
sed -n '1,160p' ~/ros2_ws/src/module_c_bt_demo/behavior_trees/module_c_demo_goal_wait.xml
```
