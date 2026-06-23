# 边建图边导航：没有静态地图时的完整流程

> 阅读位置：第一阶段之后，作为第二个完整实践。
>
> 前置建议：先跑通 `实践操作/Turtlebot3仿真导航快速跑通.md`。
>
> 本章目标：理解没有预制地图时，如何用 `slam_toolbox` 同时发布 `/map` 和 `map -> odom`，让 Nav2 在未知环境中导航。

## 本章在导航系统中的位置

常规已知地图导航依赖 `map_server + AMCL`。边建图边导航则用 `slam_toolbox` 替代它们：SLAM 一边根据 `/scan` 建图，一边发布定位修正。

本章对应链路：

```text
Gazebo /scan + /odom
  -> slam_toolbox
  -> 动态 /map + map -> odom
  -> Nav2 navigation_launch
  -> RViz 下发目标
```

什么时候用这篇：

- 题目没有给 `.yaml/.pgm` 地图。
- 题目只给 Gazebo world。
- 需要先探索并保存地图。

## 接下来怎么衔接

- 想调路径安全距离：读 `实践操作/Nav2代价地图参数配置.md`。
- 想保存地图后转为已知地图导航：看本章 Map Saver 部分。

---

# 同步建图与导航 (SLAM While Navigating)

## 1. 核心理论：并发系统的协同机制

在常规的导航流中，机器人依赖一张预先构建好的静态地图（地图已知）。但在实际的未知环境探索、灾后搜救或动态仓库布设中，机器人必须具备边构建地图、边自主导航（SLAM While Navigating）的并发处理能力。

该任务的底层核心在于两大异步独立状态机（SLAM Toolbox 与 Nav2）的高频内聚与数据协同：

- **数据流解耦与对齐：** 传统的导航依靠 `map_server` 发布固定的 `/map` 话题。而在并发任务中，`map_server` 被关闭，改由 `slam_toolbox` 扮演地图发布者的角色。随着机器人移动，雷达不断击中未知墙壁，SLAM 节点以高频（如 0.5Hz）动态更新并发布不断扩张的 `/map` 话题。
- **代价地图的动态响应：** Nav2 的全局代价地图（Global Costmap）通过其静态层（Static Layer）实时订阅由 SLAM 节点发布的动态 `/map`。每当 SLAM 拓宽了地图边界或修正了局部畸变，全局代价地图会瞬间重构其底层网格成本。同时，局部代价地图（Local Costmap）利用雷达即时点云（`/scan`）死守安全底线，防止机器人撞击尚未被 SLAM 固化到全局地图中的临时障碍物。
- **动态重规划机制：** 当导航算法规划了一条通往未知区域的路径时，若机器人在行驶过程中雷达突然扫出一条隐藏在“未知迷雾”中的死胡同墙壁，SLAM 节点会立刻将该墙壁标记为致命障碍物（Lethal Cost）。Nav2 探测到路径被阻断后，将瞬间终止当前的执行树，触发路径重规划（Re-planning），驱使机器人绕开死路。







## 2. 方案一：全栈自动化仿真流水线（一键集成法）

针对快速验证算法性能的需求，Nav2 官方提供了一条高度封装的综合启动流水线。该方案将 Gazebo 物理世界、机器人模型状态发布器、SLAM 拓扑服务器、Nav2 导航大脑以及 RViz2 可视化界面压缩至单条指令中。

在主控终端中声明底盘型号，并启动全栈仿真环境：

Bash

```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

> **架构要点：**
>
> 注意末尾的参数 `slam:=True`。该参数指示系统卸载静态地图服务器（`map_server`）和定位模块（`amcl`），转而级联启动 `slam_toolbox`。此时系统强制使用仿真时钟（`use_sim_time:=True`）。



## 3. 方案二：分布式架构拆解验证（多节点级联法）

### 步骤 1：构建虚拟物理环境 (底盘与传感器仿真)

开启第一个终端，单独将机器人底盘和物理环境在 Gazebo 引擎中点火。此步骤负责向网络提供基础的 `/scan` 话题及 `odom` $\rightarrow$ `base_link` 的物理坐标转换：

Bash

```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 步骤 2：点亮 SLAM 核心 (构建环境认知层)

开启第二个终端，独立启动 SLAM 节点。该节点负责吞噬上一步产生的雷达数据，并在动态解算位姿后，向系统广播 `/map` 话题以及打通 `map` $\rightarrow$ `odom` 的关键坐标链路：

Bash

```
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

### 步骤 3：激活导航行为树 (加载控制与决策层)

开启第三个终端，启动纯粹的 Nav2 导航服务器。由于上一步的 SLAM 已经源源不断地发布动态地图，导航的大脑将直接挂载到该动态数据流上：

Bash

```
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

## 4. 运行验证与保存环境战利品

### 4.1 盲区探索与动态刷图验证

当上述多节点级联启动成功后，RViz2 监控界面将被点亮：

1. **初识世界：** 读者将观察到地图绝大部分区域被灰色覆盖（代表未知盲区），仅机器人身旁有雷达即时扫出的一圈白色自由区。由于 SLAM 节点在动态修正里程计误差，**此时无需执行 `2D Pose Estimate` 手动校准**，机器人的空间定位已天然对齐。
2. **驱散迷雾：** 在 RViz2 顶部工具栏点击 `Nav2 Goal` 按钮，在灰色的未知迷雾区域点击并拖拽下一个远端终点。
3. **闭环行为：** 规划器会穿过未知区域划出绿色路径，控制器驱动底盘向盲区挺进。随着机器人的前行，灰色的盲区如同“游戏开黑雾”一般被动态擦除，白色的通道与黑色的新墙壁实时在 RViz 中展出，代价地图的缓冲边缘同步紧随墙壁向外膨胀。

### 4.2 保存环境战利品 (Map Saver)

当引导机器人遍历整个 Gazebo 场景、完成全图的绘制后，内存中的栅格数据必须被持久化固化。开启第四个终端，执行地图拦截保存指令：

Bash

```
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/my_sim_map
```

> **技术产物指标：**
>
> 该命令会在用户的主目录下生成两个核心文件：
>
> - `my_sim_map.pgm`：一张高精度的环境黑白像素图片。
> - `my_sim_map.yaml`：记录了该地图的绝对分辨率、物理原点坐标以及占用率阈值的配置文件。
>
> 至此，同步建图任务圆满结束。在后续的纯导航开发中，开发者可直接通过命令 `map:=~/my_sim_map.yaml` 加载该战利品，关闭 SLAM 节点，转入基于静态地图的高效 AMCL 定位导航模式。