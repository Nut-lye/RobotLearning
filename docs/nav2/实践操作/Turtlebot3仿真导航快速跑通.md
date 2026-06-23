# 第一次完整导航：Turtlebot3 仿真到点

> 阅读位置：第一阶段，所有人都应该先读这一篇。
>
> 前置建议：只需要会打开终端和 source ROS 2 环境。
>
> 本章目标：不先纠结所有理论，先跑通 Gazebo + Nav2 + RViz 的最小闭环，建立“完整导航任务”长什么样的直觉。

## 本章在导航系统中的位置

这篇是整套资料的入口实践。后面的 URDF、Gazebo、TF、代价地图、规划器、控制器，都是为了解释这一篇里为什么能从起点走到终点。

本章对应完整链路：

```text
Gazebo 启动机器人和场景
  -> Nav2 启动地图、定位、规划、控制
  -> RViz 初始化位姿
  -> RViz 下发目标
  -> robot 到达终点
```

读这篇时重点观察：

- 为什么要 `use_sim_time:=True`。
- 为什么要先用 `2D Pose Estimate`。
- 下发 `Nav2 Goal` 后，RViz 中出现了哪些显示。
- 机器人运动过程中 global path 和 local costmap 如何变化。

## 接下来怎么衔接

- 想知道机器人模型怎么来的：读 `实践操作/机器人模型URDF与RViz可视化.md`。
- 想知道 Gazebo 和里程计怎么来的：读 `实践操作/Gazebo仿真与EKF里程计.md`。
- 想知道地图、定位、代价地图怎么来的：读 `实践操作/传感器建图定位与代价地图.md`。

---

# 仿真环境下的导航系统实战（以 Turtlebot3 为例）

## 1. 概述：仿真验证的核心价值

在将导航算法部署至真实的物理机器人之前，纯仿真环境（Simulation）是最高效的算法验证场。在仿真系统中，Gazebo 物理引擎将代替真实的底盘电机和物理雷达，实时生成虚拟的里程计与传感器数据。

通过这种“Sim-to-Real（仿真到现实）”的开发范式，开发者可以在极高容错率的环境下，安全地测试代价地图参数、验证规划器与控制器（Planner & Controller）的避障逻辑。并且，由于导航的核心算法层与硬件层完全解耦，仿真环境中调试完成的系统参数未来可无缝迁移至实体机器人。

## 2. 环境依赖与前置准备

在进行仿真实战前，系统需安装针对 Turtlebot3 优化的 Gazebo 仿真资源包。在 Ubuntu 终端中执行以下指令完成安装：

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo
```

## 3. 仿真系统上机操作全流程

在纯仿真测试中，所有节点均运行于同一台宿主机内。由于脱离了实体硬件的时钟，整个系统必须强制使用虚拟仿真时间（`use_sim_time:=True`），以确保 TF 坐标树与传感器数据的时间戳严格对齐。

### 步骤 1：启动 Gazebo 虚拟世界与机器人实体

开启第一个终端，声明环境变量以指定机器人型号，并启动官方预设的测试场景（包含多样化的墙壁与障碍物）：

```bash
conda deactivate
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_MODEL_DATABASE_URI=
export SVGA_VGPU10=0
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

> **预期现象：** Gazebo 仿真引擎启动，三维物理世界中生成一个包含多房间结构的测试场地，Turtlebot3 机器人模型被生成到地图坐标 `x=-2.0, y=-0.5, z=0.01` 附近。

注意：官方 `turtlebot3_world.launch.py` 的默认出生点不是“场地中央”，而是：

```text
x_pose = -2.0
y_pose = -0.5
z      = 0.01
yaw    = 0.0，也就是车头大致朝向 map 坐标系的 +X 方向
```

这个坐标非常重要。后面在 RViz 里做 `2D Pose Estimate` 时，也应该先在地图上大致点击 `x=-2.0, y=-0.5` 这个位置，再沿车头方向拖出箭头。

### 步骤 1.1：如果 Gazebo 里看不见车，先确认它到底有没有生成

看不见车不一定代表车没有生成。常见情况是 Gazebo 视角离机器人太远、机器人被墙体遮住，或者模型生成失败。

先看启动终端里有没有类似输出：

```text
[spawn_entity]: Spawn Entity started
[spawn_entity]: Successfully spawned entity [waffle]
```

再开一个临时终端检查话题：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E '^/(odom|scan|tf)$'
ros2 topic echo /odom --once
```

判断方式：

- 能看到 `/odom`、`/scan`、`/tf`，并且 `/odom` 能输出一次数据：说明机器人实体大概率已经生成，只是 Gazebo 视角没找到。
- 终端一直卡在 `Waiting for service /spawn_entity`：说明 Gazebo 的生成服务没有正常起来，去看 `错误排查/Gazebo_spawn_entity服务不可用与gzserver退出255.md`。
- 启动终端出现 `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`：说明 conda Python 污染了 ROS 2 环境，先 `conda deactivate`，再重新 `source /opt/ros/humble/setup.bash`。

在 Gazebo 窗口里，可以先用鼠标滚轮缩小视野，观察左侧模型列表里是否有 `waffle`。只要模型列表里有 `waffle`，机器人已经在世界里。

### 步骤 1.2：如何设置机器人启动位置

如果默认位置不方便观察，或者题目要求指定起点，可以在启动 Gazebo 时传入 `x_pose` 和 `y_pose`：

```bash
conda deactivate
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_MODEL_DATABASE_URI=
export SVGA_VGPU10=0
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=-2.0 y_pose:=-0.5
```

这里的 `x_pose`、`y_pose` 是 Gazebo 世界坐标，也是 Nav2 地图坐标。可以理解成：

```text
x_pose 越大，机器人越往地图右侧 / +X 方向
y_pose 越大，机器人越往地图上方 / +Y 方向
```

修改时不要把车放进墙里或障碍物里。初学时建议先使用官方默认值 `x_pose:=-2.0 y_pose:=-0.5`，等导航闭环跑通后再改起点。

还要特别注意：**Gazebo 的启动位置不会自动告诉 AMCL。** 也就是说，即使你通过 `x_pose`、`y_pose` 改了出生点，后面在 RViz 里仍然要重新用 `2D Pose Estimate` 在相同位置初始化一次。

### 步骤 2：一键激活 Nav2 系统与可视化监控

开启第二个终端，启动 Nav2 的核心导航组件以及 RViz2 监控界面。官方提供了一个高度集成的启动文件，可一键完成地图加载、生命周期管理器启动以及组件激活：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

> **预期现象：** RViz2 图形化界面弹出，并自动加载静态二维环境地图（黑白轮廓）。此时由于 AMCL（自适应蒙特卡洛定位）尚未初始化，RViz 中的机器人模型位置大概率与 Gazebo 中的真实位置存在严重错位。

### 步骤 3：核心交互一 —— 位姿初始化与空间对齐 (AMCL 粒子收敛)

为了让导航大脑建立正确的空间基准，必须手动完成初始位姿的校准。

1. **先记住 Gazebo 出生点：** 如果没有额外传参，机器人默认出生在 `x=-2.0, y=-0.5` 附近，车头大致朝向 `+X`。
2. **下发初始估算：** 在 RViz2 顶部工具栏中，点击 **`2D Pose Estimate`**（2D 位姿估计）按钮。
3. **空间校准：** 在 RViz 的二维地图上，找到大致对应 `x=-2.0, y=-0.5` 的位置，按下鼠标左键，并顺着车头朝向拖拽出一个绿色指示箭头。
4. **收敛验证：** 释放鼠标后，系统会在该区域撒布 AMCL 粒子。代表激光雷达扫描数据的红线应当和地图中的黑色墙壁轮廓逐渐重合，标志着系统的空间坐标系对齐成功。

如果你改过 Gazebo 出生点，例如：

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=1.0 y_pose:=0.5
```

那么 RViz 里的 `2D Pose Estimate` 也要点在 `x=1.0, y=0.5` 附近，而不是继续点默认位置。

判断初始化是否正确：

- 正确：红色激光点云贴着黑色墙线，机器人模型出现在地图上的合理位置。
- 错误：红色激光点云和墙线整体错开，机器人可能原地转圈、规划失败，或者路线看起来穿墙。
- 看不到机器人模型：先确认 RViz 左侧显示项里 `RobotModel`、`LaserScan`、`Map` 是否勾选，再确认 `Fixed Frame` 是 `map`。

### 步骤 4：核心交互二 —— 目标下发与自动驾驶决策

在坐标对齐完毕后，即可对系统进行全链路导航功能的测试。

1. **设定终点：** 在 RViz2 顶部工具栏中，点击 **`Nav2 Goal`** 按钮。
2. **下发任务：** 在地图内的任意无障碍区域点击并拖拽，设定目标停靠点及期望的车头朝向。
3. **系统闭环执行：** 观察 RViz 和 Gazebo 中的机器人状态。

执行过程中重点看三个现象：

- **全局规划：** Nav2 的规划器插件（Planner）会立刻基于全局代价地图，解算出一条通往目标点的绿色最优路径。
- **局部控制：** 控制器插件（Controller）将接管底盘，实时计算避障速度矢量，并输出至 Gazebo。
- **监控反馈：** 开发者可在 RViz 与 Gazebo 中同步观察到机器人平滑驶向终点的全过程。如果在路途中放置动态障碍物，局部代价地图将被激活，触发控制器的动态避障或恢复行为（Recovery Behaviors）。



路径

```
~/ros2_ws/src/sam_bot_description/
│
├── CMakeLists.txt                  # 【编译规则】必须确保 install 了以下所有子文件夹
├── package.xml                     # 【依赖声明】包含了 rclcpp, nav2_bringup 等依赖
│
├── urdf/                           # 📍 [躯壳层] 存放机器人的 3D 物理模型
│   └── sam_bot_description.urdf    # 定义了车轮、底盘、雷达、相机的相对位置与物理属性
│
├── world/                          # 📍 [环境层] 存放 Gazebo 仿真世界
│   └── my_world.sdf                # 包含了测试房间的墙壁、障碍物模型
│
├── config/                         # 📍 [大脑参数层] 整个导航系统的灵魂
│   ├── ekf.yaml                    # 卡尔曼滤波参数 (融合里程计与 IMU)
│   └── nav2_params.yaml            # 包含了代价地图(图层/STVL)、Footprint、规划器与控制器的所有限速与避障参数
│
├── behavior_trees/                 # 📍 [逻辑决策层] (高阶拓展)
│   └── follow_dynamic_object.xml   # 存放自定义的行为树逻辑 (如：动态目标追踪)
│
├── rviz/                           # 📍 [监控层] 可视化界面配置
│   └── urdf_config.rviz            # 保存了 RViz 的视角、颜色、显示的图层状态
│
└── launch/                         # 📍 [启动枢纽层] 负责把上面所有东西串联拉起
    └── display.launch.py           # 自定义的底层启动文件 (拉起 Gazebo, URDF, EKF)
```
