# 代价地图参数：让机器人避开墙和障碍物

> 阅读位置：第三阶段，开始真正调 Nav2 参数。
>
> 前置建议：先读 `实践操作/传感器建图定位与代价地图.md`，知道 static layer、obstacle layer、inflation layer 是什么。
>
> 本章目标：配置 `nav2_params.yaml` 中的 global/local costmap，让路径规划和局部避障有可靠的风险地图。

## 本章在导航系统中的位置

代价地图是 Nav2 调参的核心。地图和雷达只是原始信息，规划器真正读取的是代价地图。路径贴墙、不绕障碍、局部避障不稳定，通常都要先看这里。

本章对应导航链路：

```text
/map + /scan
  -> static_layer / obstacle_layer / voxel_layer
  -> inflation_layer
  -> global_costmap / local_costmap
  -> planner / controller
```

重点参数：

- `inflation_radius`：障碍物向外扩张的安全距离。
- `cost_scaling_factor`：距离障碍物越远，代价值下降的快慢。
- `obstacle_max_range`：多远内的障碍物会被标记。
- `raytrace_max_range`：多远内可以清除已经移走的障碍物。
- `width` / `height`：局部代价地图窗口大小。

## 接下来怎么衔接

- 读 `实践操作/机器人Footprint轮廓配置.md`：解决机器人真实外形和碰撞检测。
- 读 `实践操作/Nav2调参与故障诊断手册.md`：按现象反推该改哪些参数。

---

### 引言：为什么光有雷达和地图还不够？

在前面的章节中，我们已经为机器人搭建了物理身体，安装了雷达与相机，甚至通过 SLAM 技术绘制出了一张黑白的“静态房间平面图”。

但这带来了一个致命的问题：**导航算法（如 A\* 或 DWA）是看不懂“墙壁”或“行人”的。** 雷达只能告诉系统“前方 1.5 米处有一个反射点”，静态地图只能告诉系统“这里是黑色的（有障碍）”。如果直接让机器人贴着这些黑色的边缘走，它庞大的车身一定会和墙壁发生物理刮擦；如果突然窜出一个人，静态地图上也根本没有记录。

为了让机器人学会“安全避障”**，我们需要在原始传感器数据和路径规划大脑之间，架设一座桥梁——这就是**代价地图 (Costmap 2D)。

### 什么是代价地图？

你可以把代价地图理解为机器人眼中的“风险地形图”**。它将物理空间划分成一个个极小的网格（例如 5cm×5cm），并根据传感器传来的数据，给每个网格打上一个代表危险程度的“分数”（0 代表绝对安全，254 代表致命碰撞）。 不仅如此，代价地图还会以障碍物为中心，向外辐射出渐变的**“安全缓冲区（膨胀层）”，像磁场一样把机器人推向最宽敞、最安全的中心路段。

### 第 1 步：创建代价地图配置文件 (YAML)

这段代码定义了全局和局部代价地图的刷新频率、膨胀半径以及图层叠加关系。

1. 进入你之前创建的 `config` 文件夹：

   Bash

   ```
   cd ~/ros2_ws/src/sam_bot_description/config
   ```

2. 新建一个名为 `nav2_params.yaml` 的文件：

   Bash

   ```
   gedit nav2_params.yaml
   ```

写入

```
global_costmap:
  global_costmap:
    ros__parameters:
      # --- 基础运行参数 ---
      update_frequency: 1.0       # 地图内部更新频率 (1Hz)，全局地图不需要太快
      publish_frequency: 1.0      # 向外发布（如给 RViz 显示）的频率 (1Hz)
      global_frame: map           # 全局参考坐标系。全局地图死死锚定在静态 map 上
      robot_base_frame: base_link # 机器人本体的中心坐标系
      use_sim_time: True          # 仿真环境下必须为 True，使用 Gazebo 的时钟
      robot_radius: 0.22          # 机器人的物理半径 (单位：米)，用于计算绝对碰撞区
      resolution: 0.05            # 网格分辨率：每个像素代表真实世界的 5 厘米
      track_unknown_space: false  # 是否追踪未知区域（设为 false 表示未知区域当作自由空间）
      rolling_window: false       # 全局地图是固定大小的，不随机器人移动而滚动

      # --- 图层加载声明 ---
      # 声明要按顺序叠加的图层（像三明治一样从下往上叠）
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      # --- 1. 静态层配置 ---
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True # 确保能接收到建图节点以前发出的静态地图数据

      # --- 2. 障碍物层配置 (处理 2D 动态障碍) ---
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan # 定义数据来源的名字
        scan:
          topic: /scan              # 订阅激光雷达的话题
          data_type: "LaserScan"    # 数据类型为 2D 激光扫描
          max_obstacle_height: 2.0  # 只关注 2 米以下的障碍物
          marking: True             # 允许将雷达扫到的障碍物“标记”到地图上
          clearing: True            # 允许利用光线追踪把移走的障碍物从地图上“清除”
          raytrace_max_range: 3.0   # 光线追踪清除障碍物的最大有效距离 (3米内)
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5   # 标记障碍物的最大有效距离 (只相信 2.5 米内的雷达数据)
          obstacle_min_range: 0.0

      # --- 3. 膨胀层配置 (安全防撞核心) ---
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # 代价衰减因子：值越大，代价值随距离下降得越陡峭
        inflation_radius: 0.55    # 最大膨胀半径 (0.55米)。在此范围外，障碍物代价值为 0

# =========================================================================

local_costmap:
  local_costmap:
    ros__parameters:
      # --- 基础运行参数 ---
      update_frequency: 5.0       # 局部地图更新极快 (5Hz)，用于应对突然冲出的人或物
      publish_frequency: 2.0      # 显示频率 (2Hz)
      global_frame: odom          # 【关键】局部地图不看 map，只看平滑连续的 odom 坐标系
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true        # 【关键】局部地图是一个以机器人为中心“滚动”的窗口
      width: 3                    # 局部窗口的宽：3米
      height: 3                   # 局部窗口的高：3米
      resolution: 0.05
      robot_radius: 0.22

      # --- 图层加载声明 ---
      # 注意：局部地图没有 static_layer (静态层)，它只关心眼前看到的东西！
      plugins: ["voxel_layer", "inflation_layer"]

      # --- 1. 体素层配置 (处理 3D 动态障碍) ---
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True   # 允许发布 3D 体素网格供调试
        origin_z: 0.0             # 3D 空间的 Z 轴起点
        z_resolution: 0.05        # Z 轴方向的分辨率 (5厘米一层)
        z_voxels: 16              # Z 轴方向总共切分 16 层 (16 * 0.05 = 0.8米高)
        max_obstacle_height: 2.0
        mark_threshold: 0         # 一列体素中至少有几个被占用才认定为障碍物 (0表示只要扫到就算)
        observation_sources: scan
        scan:                     # 同样利用雷达数据来生成体素障碍物
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"

      # --- 2. 膨胀层配置 (安全防撞核心) ---
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # 与全局地图保持一致的缓冲策略
        inflation_radius: 0.55
      always_send_full_costmap: True
```

### 第 2 步：启动物理世界与基础节点

打开**第 1 个终端**，编译并把机器人扔进 Gazebo 世界：  

Bash

```
cd ~/ros2_ws
colcon build --packages-select sam_bot_description
source install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

*(此时 RViz 和带障碍物的 Gazebo 物理世界会启动)*  

### 第 3 步：启动 SLAM 节点 (绘制静态底层地图)

打开**第 2 个终端**，启动建图算法：  

Bash

```
sudo apt install ros-humble-slam-toolbox
cd ~/ros2_ws
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

*(此时系统开始发布 `/map` 话题，代价地图的 `static_layer` 有了数据来源)*  

### 第 4 步：挂载配置并启动 Nav2 系统

直接启动 Nav2 是不会读取你刚才写的 YAML 的。必须用 `params_file` 参数把它挂载进去。  

打开**第 3 个终端**，安装并启动 Nav2：  

Bash

```
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
cd ~/ros2_ws
source install/setup.bash

# 获取 YAML 文件的绝对路径并带入启动命令
CONFIG_DIR=$(ros2 pkg prefix sam_bot_description)/share/sam_bot_description/config/nav2_params.yaml

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=$CONFIG_DIR
```

### 第 5 步：在 RViz 中验证代价地图 (见证效果)

回到已经打开的 RViz 窗口，按以下步骤把刚刚生成的代价图层“可视化”出来：

1. **查看全局代价地图（长途规划依据）：**

   - 点击左下角 `Add` -> 切换到 `By topic` 标签。
   - 找到 `/global_costmap/costmap`，双击下面的 `Map`。
   - **效果：** 你会看到整个空间的静态墙壁外围，出现了一圈代表“膨胀代价”的渐变色边缘。

2. **查看局部代价地图（紧急避障依据）：**

   - 点击左下角 `Add` -> 切换到 `By topic` 标签。
   - 找到 `/local_costmap/costmap`，双击下面的 `Map`。
   - 为了不和全局地图颜色混淆，建议在左侧面板将 Local Costmap 的 `Color Scheme` 设为 `costmap`。
   - **效果：** 你会看到以机器人为中心的一个 $3m \times 3m$ 的小方块，这个小方块会随着机器人的移动而移动，里面的障碍物也是实时刷新的。

3. **查看 3D 体素障碍物 (Voxel Grid)：**

   如果你想看到文档里提到的体素网格（代表深度相机检测到的三维障碍），在第 4 个终端运行这个标记转换节点：

   Bash

   ```
   ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker
   ```

   然后在 RViz 中 `Add` -> `By topic` -> 选择 `/my_marker` 下的 `Marker`。将全局参考系（Fixed Frame）改为 `odom`，你就能看到悬浮在空中的红蓝立体方块了！