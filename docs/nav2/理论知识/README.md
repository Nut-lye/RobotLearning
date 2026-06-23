# 理论知识目录

这个目录只放 Nav2 的底层概念，不直接承担完整上机流程。建议在跑通过 `实践操作/Turtlebot3仿真导航快速跑通.md` 后再读。

推荐顺序：

1. `Nav2理论总入口.md`
2. `Nav2通信与生命周期.md`
3. `Nav2行为树与导航服务器.md`
4. `Nav2状态估计与TF.md`
5. `Nav2环境表达与代价地图.md`
6. `行为树黑板机制.md`

读完后应该能解释：

- Nav2 为什么用 Action 做长任务。
- 生命周期节点为什么要统一激活。
- 行为树如何调度 planner、controller、recovery。
- `map -> odom -> base_link -> lidar_link` 这条 TF 链为什么必须完整。
- Costmap 为什么是规划和避障的核心。

