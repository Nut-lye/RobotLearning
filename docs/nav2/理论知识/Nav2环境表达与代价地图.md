# Nav2 环境表达与代价地图

> 阅读位置：Nav2 理论第四层。
>
> 前置建议：先读 `理论知识/Nav2状态估计与TF.md`，确认机器人位置和传感器坐标关系已经理解。
>
> 本章目标：理解 Nav2 如何把地图和传感器数据转换成可用于规划和避障的 costmap。

## 1. 为什么普通地图不能直接导航

黑白地图只能告诉机器人：

- 哪些地方像墙。
- 哪些地方像空地。

但导航还需要知道：

- 机器人有体积，不能贴着墙走。
- 动态障碍物可能不在静态地图上。
- 障碍物移走后需要清除。
- 距离障碍物越近风险越高。

所以 Nav2 引入代价地图 Costmap。

## 2. Costmap 是什么

Costmap 可以理解为风险地形图。

它把空间划分成网格，每个网格有代价值：

- `0`：自由空间。
- `1 ~ 252`：不同程度风险区域。
- `253`：机器人外形已经接触障碍物边界。
- `254`：致命障碍物。

规划器和控制器不会直接“看墙”，它们看的是 costmap 中的代价值。

## 3. Global Costmap 和 Local Costmap

### Global Costmap

用途：

- 服务全局路径规划。
- 帮机器人从当前位置规划到远处目标。

特点：

- 坐标系通常是 `map`。
- 覆盖整张已知地图。
- 更新频率较低。
- 重点关注全局路线是否可达。

### Local Costmap

用途：

- 服务局部控制和动态避障。
- 帮机器人在眼前几米范围内安全运动。

特点：

- 坐标系通常是 `odom`。
- 以机器人为中心滚动。
- 更新频率较高。
- 重点关注眼前障碍物和实时避障。

## 4. Costmap 图层

Costmap 是多层叠加的。

### Static Layer

来源：

- `/map`

作用：

- 表达墙、房间边界、固定障碍物。

常用于：

- global costmap。

### Obstacle Layer

来源：

- `/scan`
- 其他 2D 障碍物传感器。

作用：

- 实时标记动态障碍物。
- 通过 raytrace 清除已经移走的障碍物。

关键参数：

```yaml
observation_sources: scan
scan:
  topic: /scan
  marking: True
  clearing: True
  obstacle_max_range: 2.5
  raytrace_max_range: 3.0
```

### Voxel Layer

来源：

- `/scan`
- 深度相机或 3D 点云。

作用：

- 用体素网格表达三维障碍物。
- 适合需要考虑高度的场景。

### Inflation Layer

来源：

- 已经标记出的障碍物。

作用：

- 把障碍物向外膨胀成安全缓冲区。
- 让规划器倾向于走通道中间，而不是贴墙。

关键参数：

```yaml
inflation_layer:
  inflation_radius: 0.55
  cost_scaling_factor: 3.0
```

## 5. Inflation 参数怎么理解

### `inflation_radius`

表示障碍物向外扩张多远。

增大它：

- 路径更远离障碍物。
- 更安全。
- 可能过不了窄门。

减小它：

- 更容易通过狭窄空间。
- 更容易贴墙。

### `cost_scaling_factor`

表示代价值随距离下降的快慢。

值越大：

- 代价下降越快。
- 机器人可能更愿意靠近障碍物。

值越小：

- 代价下降越慢。
- 路径更倾向于走中间。

## 6. Footprint 和 robot_radius

机器人不是一个点。Costmap 必须知道机器人占地范围。

两种写法：

### `robot_radius`

适合圆形或近似圆形机器人。

```yaml
robot_radius: 0.22
```

优点：

- 简单。
- 计算快。

### `footprint`

适合长方形或不规则机器人。

```yaml
footprint: "[ [0.21, 0.195], [0.21, -0.195], [-0.21, -0.195], [-0.21, 0.195] ]"
```

优点：

- 更准确。
- 对窄门和贴边控制更真实。

详细配置见：

- `实践操作/机器人Footprint轮廓配置.md`

## 7. 典型问题对应参数

### 机器人贴墙

优先看：

- `inflation_radius`
- `cost_scaling_factor`
- `robot_radius`
- `footprint`

处理：

- 增大 `inflation_radius`。
- 减小 `cost_scaling_factor`。
- 检查 footprint 是否小于真实机器人。

### 机器人不敢过门

优先看：

- `inflation_radius`
- `robot_radius`
- `footprint`

处理：

- 减小 `inflation_radius`。
- 检查 footprint 是否写太大。

### 动态障碍物清不掉

优先看：

- `clearing`
- `raytrace_max_range`
- 传感器话题和 TF。

### 局部避障反应慢

优先看：

- local costmap `update_frequency`
- `/scan` 频率
- local costmap `width` / `height`

## 8. 实践文档

理论理解后，继续读：

- `实践操作/Nav2代价地图参数配置.md`
- `实践操作/机器人Footprint轮廓配置.md`
- `实践操作/Nav2调参与故障诊断手册.md`

