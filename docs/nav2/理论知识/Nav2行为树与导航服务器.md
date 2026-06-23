# Nav2 行为树与导航服务器

> 阅读位置：Nav2 理论第二层。
>
> 前置建议：先读 `理论知识/Nav2通信与生命周期.md`，知道 Action 和生命周期是什么。
>
> 本章目标：理解 Nav2 为什么用行为树调度 Planner、Controller、Recovery，以及导航失败后为什么能恢复和重试。

## 1. 行为树是什么

行为树是 Nav2 的任务总指挥。

它本身不负责计算路径，也不直接控制底盘。它负责决定：

- 什么时候规划路径。
- 什么时候沿路径行走。
- 什么时候判断失败。
- 什么时候执行恢复动作。
- 什么时候重新规划。
- 什么时候返回成功或失败。

行为树由许多节点组成，每个节点执行后会返回状态：

- `SUCCESS`：成功。
- `FAILURE`：失败。
- `RUNNING`：仍在运行。

父节点根据这些状态决定下一步。

## 2. 为什么 Nav2 要用行为树

导航不是一条直线逻辑：

```text
规划 -> 执行 -> 到达
```

真实情况更像：

```text
规划
  -> 执行
  -> 遇到障碍物
  -> 控制失败
  -> 清除局部代价地图
  -> 后退或旋转
  -> 重新规划
  -> 继续执行
```

行为树适合表达这种“主流程 + 失败恢复 + 重试”的逻辑。

## 3. Nav2 的核心导航服务器

| 服务器 | 主要职责 | 常见插件或动作 | 典型问题 |
| --- | --- | --- | --- |
| `planner_server` | 根据全局代价地图生成路径 | NavFn、Smac 2D、Theta Star | 没路径、绕远、穿不过未知区 |
| `controller_server` | 根据局部代价地图跟踪路径并输出 `/cmd_vel` | DWB、RPP、TEB | 有路径但走不好、摆动、到点失败 |
| `behavior_server` / recoveries | 执行失败恢复动作 | Spin、BackUp、Wait、ClearCostmap | 卡住后能否脱困 |
| `bt_navigator` | 加载行为树并调度导航任务 | NavigateToPose | 整体导航成功/失败 |

## 4. 机器人去目标点的典型流程

以“去厨房”为例：

1. 用户在 RViz 中下发 `Nav2 Goal`。
2. `bt_navigator` 接收目标。
3. 行为树调用 `ComputePathToPose`。
4. `planner_server` 根据 global costmap 生成全局路径。
5. 行为树调用 `FollowPath`。
6. `controller_server` 根据 local costmap 输出速度 `/cmd_vel`。
7. Gazebo 或真实底盘执行速度。
8. 如果控制失败，行为树调用恢复动作。
9. 恢复后重新规划。
10. 到达目标，返回成功。

## 5. 行为树中的恢复逻辑

常见恢复动作：

- `Spin`：原地旋转，重新扫描周围。
- `BackUp`：后退，离开障碍物或死角。
- `Wait`：等待动态障碍物离开。
- `ClearCostmap`：清除局部或全局代价地图中的异常障碍物。

这些恢复动作不是随机执行的，而是在行为树中按逻辑排列。

## 6. 黑板 Blackboard

行为树节点之间通过黑板传递数据。

例如：

```text
ComputePathToPose 输出 path
  -> 写入黑板 {path}
  -> FollowPath 从黑板读取 {path}
```

动态目标追踪中：

```text
GoalUpdater 接收新目标
  -> 写入 {updated_goal}
  -> ComputePathToPose 读取新目标并重算路径
```

黑板的详细解释见：

- `理论知识/行为树黑板机制.md`

## 7. 什么时候需要改行为树

普通到点导航一般不需要改行为树，优先调：

- costmap
- footprint
- planner
- controller
- goal checker

需要改行为树的情况：

- 题目要求动态目标追踪。
- 题目要求插入自定义动作。
- 题目要求特殊恢复流程。
- 默认导航逻辑不满足任务。

对应文档：

- `进阶专题/行为树动态目标追踪.md`
- `进阶专题/Groot行为树可视化编辑.md`
- `进阶专题/Nav2自定义行为树插件编写.md`

## 8. 本章和调参的关系

判断问题发生在哪：

- 没有全局路径：优先看 `planner_server` 和 global costmap。
- 有路径但不走：优先看 `controller_server`、local costmap、TF、`/cmd_vel`。
- 失败后不会自救：看 behavior server、恢复动作、行为树配置。
- 任务逻辑要变：看行为树 XML 和黑板。

