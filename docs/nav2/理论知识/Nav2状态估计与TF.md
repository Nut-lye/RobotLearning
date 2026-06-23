# Nav2 状态估计与 TF

> 阅读位置：Nav2 理论第三层。
>
> 前置建议：先读 `实践操作/机器人模型URDF与RViz可视化.md`、`实践操作/Gazebo仿真与EKF里程计.md`，知道机器人模型和 Gazebo 里程计从哪里来。
>
> 本章目标：理解 `map -> odom -> base_link -> sensor` 这条 TF 链，以及 SLAM、AMCL、EKF、URDF 在其中分别负责什么。

## 1. 为什么 Nav2 离不开 TF

机器人导航时，所有数据都来自不同坐标系：

- 地图在 `map` 坐标系。
- 里程计在 `odom` 坐标系。
- 机器人中心在 `base_link` 坐标系。
- 雷达在 `base_laser` 或 `lidar_link` 坐标系。

雷达说“前方 1 米有墙”，但这个“前方”是雷达自己的前方。Nav2 必须知道雷达装在机器人哪里，才能把雷达数据转换到机器人和地图坐标系里。

这就是 TF 的作用：在不同坐标系之间转换位置和方向。

## 2. 核心坐标系

### `map`

全局地图坐标系。

特点：

- 原点通常是建图起点或地图文件定义的 origin。
- 墙、房间、障碍物在 `map` 中的位置相对固定。
- 全局路径规划依赖它。

### `odom`

里程计坐标系。

特点：

- 由底盘里程计或 EKF 连续推算。
- 短时间内平滑。
- 长时间会漂移。
- 控制器依赖它的连续性。

### `base_link`

机器人本体坐标系。

特点：

- 通常位于机器人中心或驱动轮中心。
- 机器人前进方向一般是 `base_link` 的 x 正方向。
- 导航控制的核心参考点。

### `base_laser` / `lidar_link`

雷达坐标系。

特点：

- 表示雷达安装位置。
- 雷达数据 `/scan` 的 `frame_id` 通常指向这里。
- 需要通过 TF 转到 `base_link` 或 `map`。

## 3. 三段关键 TF

完整链路：

```text
map -> odom -> base_link -> lidar_link
```

### `map -> odom`

谁发布：

- 已知地图导航：AMCL。
- 未知地图/边建图边导航：slam_toolbox。

含义：

- 修正里程计漂移。
- 把机器人短期里程计坐标放回全局地图。

特点：

- 低频。
- 可能有轻微跳变。
- 如果没有它，Nav2 无法知道机器人在地图中的位置。

### `odom -> base_link`

谁发布：

- 底盘驱动。
- Gazebo 差速驱动插件。
- `robot_localization` 的 EKF。

含义：

- 根据轮子、IMU 等数据估计机器人短期运动。

特点：

- 高频。
- 连续平滑。
- 控制器非常依赖它。

### `base_link -> lidar_link`

谁发布：

- `robot_state_publisher` 读取 URDF 后发布。
- 或临时用 `static_transform_publisher` 发布。

含义：

- 雷达相对机器人中心的固定安装位置。

特点：

- 静态不变。
- 如果缺失，雷达数据无法正确进入 costmap。

## 4. 状态估计：里程计、IMU、AMCL、SLAM

### 局部短期估计：Odometry + IMU

机器人通过轮子编码器和 IMU 推算当前位置。

优点：

- 高频。
- 平滑。
- 适合控制器。

缺点：

- 会随时间漂移。

Gazebo 中通常由差速插件和 IMU 插件模拟这些数据。`robot_localization` 可以用 EKF 融合它们，输出更稳定的 `/odometry/filtered` 和 `odom -> base_link`。

### 全局长期修正：AMCL

AMCL 用于已知地图定位。

它会：

1. 在地图中撒粒子。
2. 根据雷达扫描和地图墙体匹配程度筛选粒子。
3. 估计机器人在地图中的位置。
4. 发布 `map -> odom`。

RViz 中的 `2D Pose Estimate` 就是给 AMCL 一个初始猜测。

### 未知环境：SLAM Toolbox

没有地图时使用 SLAM。

它会：

1. 订阅 `/scan` 和 TF。
2. 一边构建 `/map`。
3. 一边估计机器人位置。
4. 发布 `map -> odom`。

这就是 `实践操作/SLAM边建图边导航.md` 的核心。

## 5. URDF 如何变成 TF

URDF 由两类核心标签组成：

- `<link>`：机器人部件，例如车体、轮子、雷达。
- `<joint>`：部件之间的连接关系，例如雷达固定在车体上方。

示例：

```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="base_laser"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
</joint>
```

`robot_state_publisher` 读取后会发布：

```text
base_link -> base_laser
```

所以：

- URDF 不只是给 RViz 显示外观。
- 它也是 TF 静态结构的来源。
- Nav2 的传感器转换和碰撞理解都依赖它。

更完整的 URDF 实践见：

- `实践操作/机器人模型URDF与RViz可视化.md`

## 6. 常用检查命令

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

检查话题：

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /map --once
```

## 7. 典型问题判断

### RViz 雷达和地图墙体对不上

可能原因：

- AMCL 初始位姿错误。
- `map -> odom` 不准。
- 雷达 TF 安装位置错误。
- `use_sim_time` 不一致。

### 机器人在 RViz 中跳动

如果是低频瞬移：

- 多半是 `map -> odom` 定位修正明显。

如果是高频抖动：

- 多半是 `odom -> base_link` 里程计或 EKF 不稳定。

### Costmap 不显示障碍物

可能原因：

- `/scan` 没数据。
- scan 的 `frame_id` 没有对应 TF。
- `base_link -> lidar_link` 缺失。
- costmap 的 observation source 配错话题名。

