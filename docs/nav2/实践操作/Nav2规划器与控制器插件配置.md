# 规划器与控制器插件：决定路径怎么走、机器人怎么跟

> 阅读位置：第三阶段后半，costmap 和 footprint 已经基本正确之后。
>
> 前置建议：先读 `实践操作/Nav2代价地图参数配置.md`、`实践操作/机器人Footprint轮廓配置.md`。
>
> 本章目标：理解并配置 `planner_server` 和 `controller_server`，让全局路径和局部运动更适合机器人底盘。

## 本章在导航系统中的位置

当地图、定位、代价地图都正常后，Nav2 的核心动作就分成两步：Planner 先算路线，Controller 再让机器人沿路线走。

本章对应导航链路：

```text
global_costmap
  -> planner_server 生成全局路径
  -> controller_server 跟踪路径并避障
  -> /cmd_vel
  -> Gazebo 或真实底盘
```

调参判断：

- 没有路径：优先看 planner 和 global costmap。
- 有路径但走不好：优先看 controller 和 local costmap。
- 到点附近失败：优先看 controller、goal checker、角速度和目标容差。

## 接下来怎么衔接

- 读 `实践操作/Nav2调参与故障诊断手册.md`：按问题现象定位 planner/controller 参数。
- 特殊任务再读 `进阶专题/Groot行为树可视化编辑.md`、`进阶专题/Nav2自定义行为树插件编写.md`。

---

# 配置导航核心算法 —— 规划器与控制器插件

## 1. 核心理论：导航动作服务器的解耦架构

在 Nav2 系统中，导航算法并非一个庞大的黑盒，而是被精妙地拆分并运行在不同的 ROS 2 动作服务器（Action Servers）上。其中最核心的两个服务器为：

1. **规划服务器 (Planner Server)：** 负责宏观的全局路径规划（即计算从 A 点到 B 点的完整路线）。
2. **控制服务器 (Controller Server)：** 负责微观的局部运动控制（即沿着规划好的路线行驶、避开动态障碍物，并最终输出给电机的速度指令 `cmd_vel`）。

Nav2 采用了高度模块化的**插件机制 (Plugin-based Architecture)**。这意味着开发者不需要修改 Nav2 的底层源码，只需在配置文件中更改插件名称，就能让机器人从“A* 算法”无缝切换到“混合 A* (Hybrid-A*)”，或从“DWA 局部控制”切换到“纯跟踪 (Pure Pursuit)”。

针对不同物理结构的机器人（如差速轮、阿克曼转向、全向轮、足式），必须为其搭配符合其运动学约束的特定插件组合。

## 2. 规划服务器 (Planner Server)：全局路径规划

规划服务器读取全局代价地图，并搜索出一条通往目标点的无碰撞路径。

### 2.1 基于栅格的经典规划器 (适用于：可以原地旋转的机器人)

这类算法直接在代价地图的 2D 网格上搜索，通常将机器人简化为一个圆形轮廓。

- **NavFn Planner:** 基于经典的 Dijkstra 或 A* 算法。计算速度快，但生成的路径贴近障碍物时不够平滑。
- **Smac 2D Planner:** Nav2 官方推荐的 2D A* 实现。具备平滑器和多分辨率特性，路径质量优于 NavFn。
- **Theta Star Planner:** 使用视线 (Line of sight) 算法，能够生成不局限于网格离散方向的对角线平滑路径。

> **局限性：** 无法保证为“阿克曼（汽车结构）”或“长条形”机器人生成可行路径，因为它们没有考虑车辆的最小转弯半径，且可能会在狭窄弯道发生车尾扫扫碰撞。

### 2.2 考虑运动学约束的高阶规划器 (适用于：汽车结构、异形或高速机器人)

这类算法在搜索路径时，会同时计算机器人的朝向、转弯半径以及真实的物理轮廓。

- **Smac Hybrid-A\* Planner:** 混合 A* 算法。完美支持阿克曼 (Ackermann) 转向和足式机器人。它能够规划出符合车辆最小转弯半径的平滑曲线（甚至包含倒车入库的轨迹），并进行全轮廓的碰撞检查。
- **Smac Lattice Planner:** 基于状态空间格 (State Lattice) 的规划器。只需修改配置即可适配差速、全向、阿克曼等任意底盘，极其灵活。

### 2.3 规划器选型总结表

| **插件名称 (Plugin)**            | **适用的底盘运动学类型**               | **适用外形**    |
| -------------------------------- | -------------------------------------- | --------------- |
| **NavFn / Smac 2D / Theta Star** | 差速驱动 (Differential)、全向轮 (Omni) | 仅推荐圆形      |
| **Smac Hybrid-A***               | 阿克曼 (Ackermann)、足式 (Legged)      | 任意异形/长条形 |
| **Smac Lattice**                 | 差速、全向、阿克曼                     | 任意异形/长条形 |

## 3. 控制服务器 (Controller Server)：局部轨迹与执行

当规划器给出全局路径后，控制服务器会以高频（如 20Hz）运行。它只关注机器人紧前方的局部代价地图，负责生成即时的速度指令，并灵活躲避突然出现的行人。

- **DWB Controller (动态窗口法):** Nav2 的默认控制器。它会模拟未来几秒内可能产生的多条短轨迹，通过一组“评价器 (Critics)”（如：距离目标的远近、距离障碍物的远近、是否偏离路线）进行打分，选择得分最高的轨迹输出速度。适用于**差速**和**全向**机器人。
- **TEB Controller (时间弹性带):** 一种基于模型预测控制 (MPC) 的最优控制器。它不仅考虑避障，还强烈优化执行时间和运动学约束。适用于所有底盘，尤其是**阿克曼**机器人。
- **RPP Controller (规则化纯跟踪):** 经典的纯跟踪 (Pure Pursuit) 算法升级版。它不擅长复杂的动态避障，但极其擅长**高精度的循迹行驶**。适用于工业巡检或果园喷洒等对轨迹精度要求极高的场景。

### 3.1 控制器选型总结表

| **插件名称 (Plugin)** | **核心任务特征**       | **适用的底盘运动学类型** |
| --------------------- | ---------------------- | ------------------------ |
| **DWB Controller**    | 灵活的动态避障         | 差速、全向               |
| **TEB Controller**    | 时间最优与复杂约束处理 | 差速、全向、阿克曼、足式 |
| **RPP Controller**    | 高精度路径精准跟踪     | 差速、阿克曼、足式       |

## 4. 工程实现：配置算法插件 (`nav2_params.yaml`)

为了让理论落地，我们需要将选定的插件注册到 Nav2 的参数文件中。

假设 `sam_bot` 是一个**差速驱动**的机器人，我们将为其配置 **Smac 2D** 作为全局规划器，以及 **DWB** 作为局部控制器。

请在您的 `nav2_params.yaml` 文件中追加以下服务器配置：

YAML

```
# ==========================================
# 规划服务器 (Planner Server) 配置
# ==========================================
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    # 1. 声明要加载的插件命名空间（这里命名为 GridBased）
    planner_plugins: ['GridBased']
    
    # 2. 将该命名空间绑定到具体的算法插件类 (Smac 2D)
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.125
      downsample_costmap: false
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000

# ==========================================
# 控制服务器 (Controller Server) 配置
# ==========================================
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    
    # 1. 声明要加载的局部控制插件命名空间（这里命名为 FollowPath）
    controller_plugins: ["FollowPath"]
    
    # 2. 将该命名空间绑定到具体的算法插件类 (DWB)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      
      # 声明 DWB 的评价器组合
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      # (此处省略具体的评价器权重参数以保持简洁)
```

**运行与验证：**

在加载此配置文件启动 Nav2 后（`ros2 launch nav2_bringup navigation_launch.py params_file:=...`），在 RViz 的顶部工具栏中选择 `2D Goal Pose`，在地图上点击并拖拽以设定终点。您将看到规划器瞬间生成一条全局路径，随后控制器会驱动机器人的底盘平滑地驶向终点。  



