# Nav2 理论总入口
 

## 为什么要拆分

Nav2 的知识点天然分成几层：

```text
通信与节点管理
  -> 行为树调度
  -> 状态估计与 TF
  -> 环境表达与代价地图
  -> 规划器与控制器
```

## 推荐阅读顺序

### 1. 通信与生命周期

阅读：

- `理论知识/Nav2通信与生命周期.md`

你会理解：

- 为什么导航任务用 Action，不用普通 Service。
- Goal、Feedback、Result 分别是什么。
- 为什么 Nav2 节点需要生命周期状态。
- 生命周期管理器为什么能让整套导航系统统一启动和停止。

### 2. 行为树与导航服务器

阅读：

- `理论知识/Nav2行为树与导航服务器.md`
- `理论知识/行为树黑板机制.md`

你会理解：

- 行为树为什么是 Nav2 的任务调度器。
- Planner、Controller、Recovery 分别负责什么。
- 黑板如何在行为树节点之间传递目标点、路径和状态。
- 为什么卡住以后 Nav2 会尝试恢复、清代价地图、重新规划。

### 3. 状态估计与 TF

阅读：

- `理论知识/Nav2状态估计与TF.md`
- `实践操作/机器人模型URDF与RViz可视化.md`
- `实践操作/Gazebo仿真与EKF里程计.md`
- `实践操作/传感器建图定位与代价地图.md`

你会理解：

- `map`、`odom`、`base_link`、`base_laser/lidar_link` 的职责。
- `map -> odom` 和 `odom -> base_link` 分别由谁发布。
- 为什么雷达必须通过 TF 转换到机器人本体坐标系。
- URDF 如何通过 `robot_state_publisher` 变成 TF 树。

### 4. 环境表达与代价地图

阅读：

- `理论知识/Nav2环境表达与代价地图.md`
- `实践操作/Nav2代价地图参数配置.md`
- `实践操作/机器人Footprint轮廓配置.md`

你会理解：

- 为什么普通地图不能直接用于导航。
- Global Costmap 和 Local Costmap 的分工。
- Static Layer、Obstacle Layer、Voxel Layer、Inflation Layer 的作用。
- 为什么 `inflation_radius`、`cost_scaling_factor`、`robot_radius/footprint` 会直接影响导航效果。

### 5. 规划器与控制器

阅读：

- `实践操作/Nav2规划器与控制器插件配置.md`
- `实践操作/Nav2调参与故障诊断手册.md`

你会理解：

- Planner 负责生成全局路线。
- Controller 负责跟踪路径并输出 `/cmd_vel`。
- Recovery 负责失败后的脱困动作。
- 不同问题应该优先看 costmap、planner 还是 controller。

## 和实践文档的关系

理论读完不是目的，能跑通和能调参才是目的。

实践入口：

- `实践操作/Turtlebot3仿真导航快速跑通.md`：先跑通官方仿真导航。
- `实践操作/SLAM边建图边导航.md`：没有静态地图时，用 SLAM while navigating。
- `模块C知识源整理.md`：把比赛/训练任务翻译成 Nav2 操作。
- `实践操作/Nav2调参与故障诊断手册.md`：按故障现象排查和调参。

