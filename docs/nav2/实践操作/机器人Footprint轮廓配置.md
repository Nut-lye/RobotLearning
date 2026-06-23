# Footprint：让 Nav2 知道机器人真实占地

> 阅读位置：第三阶段，代价地图之后、规划器控制器之前。
>
> 前置建议：先读 `实践操作/Nav2代价地图参数配置.md`，知道代价地图如何表达风险。
>
> 本章目标：配置 `robot_radius` 或 `footprint`，让 Nav2 在规划和控制时按机器人真实尺寸做碰撞判断。

## 本章在导航系统中的位置

机器人不是地图上的一个点。Nav2 必须知道机器人的半径或多边形外轮廓，才能判断一条路径是否真的能通过。

本章对应导航链路：

```text
机器人真实尺寸
  -> robot_radius / footprint
  -> costmap 碰撞检测
  -> 是否贴墙、是否能过门、是否误判碰撞
```

典型现象：

- 机器人贴墙甚至蹭墙：footprint 太小或膨胀不足。
- 窄门明明能过但 Nav2 不走：footprint 或 inflation 太保守。
- 到障碍物旁边局部控制失败：local costmap 的 footprint 更要准确。

## 接下来怎么衔接

- 读 `实践操作/Nav2规划器与控制器插件配置.md`：在 costmap 和 footprint 正确后，再考虑规划器和控制器算法。
- 读 `实践操作/Nav2调参与故障诊断手册.md`：根据贴墙、过门、转圈等现象调参。

---

# 配置机器人的物理轮廓 (Footprint)

## 1. 核心理论：机器人的“自我认知”

在 Nav2 导航系统中，代价地图 (Costmap) 不仅需要记录障碍物的位置，还需要结合机器人自身的物理轮廓来进行碰撞检测。当机器人投影到二维地平面时，其外形轮廓即被称为 **Footprint (足迹)**。

为了在“计算性能”与“避障精度”之间取得最佳平衡，Nav2 支持两种轮廓定义方式，并允许在全局和局部代价地图中采用不同的策略：

### 1.1 圆形轮廓 (`robot_radius`)

- **原理：** 系统将机器人视为一个标准的圆形，仅需一个半径参数即可定义。碰撞检测时，算法只需计算网格中心到障碍物的直线距离，计算极快。
- **应用场景 (全局代价地图)：** 在进行长距离宏观路径规划（如 NavFn 算法）时，通常不需要考虑极其极限的狭窄空间穿插。因此，全局代价地图通常使用 `robot_radius`，以最大横截面半径近似替代机器人形状，从而大幅节省路径搜索的计算资源。

### 1.2 多边形轮廓 (`footprint`)

- **原理：** 开发者输入一组按顺序排列的二维坐标点 `[x, y]`，在 `base_link` 坐标系下勾勒出机器人的精确几何形状。
- **应用场景 (局部代价地图)：** 在进行微观的实时避障和轨迹控制（如 DWB 或 TEB 算法）时，机器人经常需要穿越狭窄的门框或贴近障碍物行驶。此时，圆形近似会导致系统过于保守（认为过不去）。因此，局部代价地图通常采用多边形 `footprint`，进行精确但计算量稍大的碰撞校验。

## 2. 工程实现：配置 Nav2 参数文件

在本节的工程实践中，针对 `sam_bot` 的长方形车体结构，系统将采取混合配置策略：

- **全局代价地图：** 采用 `robot_radius: 0.3`（半径 0.3 米的圆）进行快速全局规划。
- **局部代价地图：** 采用长 0.42m、宽 0.39m 的精确矩形 `footprint` 进行极限避障。

### 2.1 编写完整的参数文件

在功能包配置目录中创建文件 `~/ros2_ws/src/sam_bot_description/config/nav2_params.yaml`，并将以下**完整且可直接运行**的配置代码写入（此文件已包含上一节的图层配置，并直接集成了 Footprint 设定）：

YAML

```
# ==========================================
# 全局代价地图 (Global Costmap) - 使用圆形半径
# ==========================================
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      
      # 【核心设定】：全局地图采用半径 0.3 米的圆形近似，加快规划速度
      robot_radius: 0.3
      # 注意：不要在此处配置 footprint，系统优先读取 footprint
      
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

# ==========================================
# 局部代价地图 (Local Costmap) - 使用精确多边形
# ==========================================
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      
      # 【核心设定】：局部地图采用精确的矩形多边形阵列，支持极限避障
      footprint: "[ [0.21, 0.195], [0.21, -0.195], [-0.21, -0.195], [-0.21, 0.195] ]"
      # 注意：此处已移除 robot_radius
      
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
```

*(确保已执行过 `colcon build` 重新编译项目以使 config 文件生效)*  

## 3. 系统联调与可视化验证

配置完成后，需要启动仿真环境并通过 RViz 验证两种 Footprint 是否被系统正确解析并发布。在主机终端中开启三个独立的终端，按顺序执行以下指令：  

### 步骤 1：启动基础仿真环境

此步骤启动物理环境、机器人模型及 TF 静态树（`base_link` -> `sensors`，以及 Gazebo 提供的 `odom` -> `base_link`）。  

Bash

```
source ~/ros2_ws/install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

### 步骤 2：发布虚拟定位基准 (Mock SLAM)

由于本章节暂时不运行 SLAM 建图节点，系统缺少 `map` 到 `odom` 的坐标转换。为使全局代价地图正常初始化，需启动静态 TF 转换器进行模拟补全：  

Bash

```
source ~/ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

*(执行后，`map` 坐标系与 `odom` 坐标系将重合并固定)*

### 步骤 3：加载参数并启动 Nav2 核心模块

携带刚刚配置好的 YAML 参数文件，启动 Nav2：

Bash

```
source ~/ros2_ws/install/setup.bash
CONFIG_DIR=$(ros2 pkg prefix sam_bot_description)/share/sam_bot_description/config/nav2_params.yaml
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=$CONFIG_DIR
```

## 4. 验证指标分析

切回 RViz 界面，按照以下路径添加显示项以验证物理轮廓的生效情况：

1. **验证局部代价地图（矩形轮廓）：**
   - 点击 `Add` -> 切换至 `By topic`。
   - 选择 `/local_costmap/published_footprint` 话题下的 `Polygon`。
   - **预期现象：** 在 `odom` 参考系下，将看到一个红色的矩形线框紧密包裹着 `sam_bot` 的车体边缘。这就是局部算法判定碰撞的死区。
2. **验证全局代价地图（圆形轮廓）：**
   - 同样在 `By topic` 面板下，选择 `/global_costmap/published_footprint` 话题下的 `Polygon`。
   - 将 RViz 的全局参考系 (Fixed Frame) 更改为 `map`。
   - **预期现象：** 将看到一个半径更大的圆形线框包围着机器人。全局算法将把机器人视为这个圆柱体进行快速宏观路线搜索。