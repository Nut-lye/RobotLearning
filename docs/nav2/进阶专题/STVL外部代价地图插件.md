# 外部代价插件 STVL：更复杂的 3D 动态障碍感知

> 阅读位置：第四阶段进阶内容。
>
> 前置建议：先读 `实践操作/Nav2代价地图参数配置.md`，理解 costmap layer 和 voxel layer。
>
> 本章目标：通过 pluginlib 加载 STVL 时空体素层，用更高效的方式处理 3D 点云和动态障碍物。

## 本章在导航系统中的位置

普通 2D 雷达导航通常用 ObstacleLayer 或 VoxelLayer 就够了。STVL 适合深度相机、3D 雷达、人员密集、动态障碍物残影明显的场景。

本章对应链路：

```text
PointCloud2 / 深度相机 / 3D 雷达
  -> STVL costmap plugin
  -> local/global costmap
  -> controller 避开 3D 动态障碍
```

普通模块 C 到点导航不建议一开始就用 STVL，除非题目明确涉及 3D 感知或动态障碍残影。

---

# 加载外部导航插件 —— 时空体素层 (STVL) 仿真配置指南

## 1. 核心理论：基于 Pluginlib 的解耦扩展架构

在 ROS 2 与 Nav2 体系中，所有的核心算法（规划器、控制器、代价地图图层）均基于 **Pluginlib（面向 C++ 的常规插件加载库）** 架构实现。该架构的核心优势在于**完全解耦**：开发者无需重新编译 Nav2 源码，仅需在运行时通过包含特定动态链接库的 YAML 文件，即可动态加载或卸载自定义的功能图层。

在 3D 感知避障领域，系统默认内置的 `nav2_costmap_2d::VoxelLayer`（标准体素层）将深度相机或三维雷达的点云数据填充入稠密的立体网格中。然而，标准体素层存在两个显著的工业级痛点：内存开销巨大，以及动态障碍物（如行人）离去后留下的“残影”清除缓慢，极易导致机器人产生误避障决策。

为了解决上述性能瓶颈，系统引入了第三方高效插件 —— **时空体素层 (Spatio-Temporal Voxel Layer, STVL)**。

### 1.1 STVL 图层的核心优势与特征

- **稀疏体素世界模型 (OpenVDB)：** STVL 放弃了传统的稠密三维数组网格，转而采用工业级稀疏体积数据结构（基于 OpenVDB）。这使得系统仅在真正存在障碍物的空间分配内存，对于高密度 3D 传感器，**内存与 CPU 资源利用率可优化提升高达 2 倍**。
- **时空动态衰减模型 (Voxel Decay)：** STVL 将时间作为第四维度引入环境感知中。进入世界的体素会拥有一个“生命周期寿命”。即使传感器因视场限制无法执行光线追踪清除（Raytrace Clearing），体素也会根据设定的时间自发衰减并自动过期删除。这极大提升了机器人在高度动态、人员密集场景下的行进敏捷度。



## 2. 工程实现：动态插件的安装与挂载

### 2.1 环境依赖安装

在 ROS 2 Humble 版本的构建农场中，STVL 已经作为标准扩展包发布，可以通过以下指令直接进行二进制部署。若在老版本 Ubuntu 环境下遭遇 `libjemalloc` 静态内存块分配错误，可通过预加载环境变量解决。

Bash

```
# 安装 Humble 版本的 STVL 扩展包
sudo apt update
sudo apt install ros-humble-spatio-temporal-voxel-layer

# （可选备用）若遭遇 OpenVDB 与 libjemalloc 的老版本冲突，可在终端执行以下环境预加载
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libjemalloc.so.2
```

### 2.2 重构 Nav2 参数文件 (`nav2_params.yaml`)

在 ROS 2 Humble 及更高版本中，传统的 `plugin_names` 与 `plugin_types` 拆分声明已被完全废弃。系统统一采用单一的 `plugins` 字符串向量，并在各图层的独立命名空间内通过 `plugin:` 字段指定完整的类加载器路径。

请打开功能包内的 `config/nav2_params.yaml` 文件，将局部代价地图（或全局代价地图）的 `voxel_layer` 动态替换为 `stvl_layer`：

YAML

```
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
      robot_radius: 0.22
      
      # 1. 在插件列表中注册自定义的时空体素层（移除旧的 voxel_layer）
      plugins: ["stvl_layer", "inflation_layer"]

      # 2. 建立 stvl_layer 命名空间，并注入完整的参数化配置
      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer" # 显式指定 Pluginlib 类加载路径
        enabled: true
        voxel_decay: 15.0          # 体素寿命 (秒)：障碍物消失 15 秒后，若无重采样数据则自动销毁
        decay_model: 0             # 衰减数学模型：0 代表线性衰减 (Linear)，1 代表指数衰减
        voxel_size: 0.05           # 体素边长 (单位：米)，通常与代价地图分辨率对齐
        track_unknown_space: true  # 是否追踪未知空间
        max_obstacle_height: 2.0   # 标记障碍物的最大高度极限
        unknown_threshold: 15      # 清除列中体素所需的最小时间步长
        mark_threshold: 0          # 允许标记为障碍物的体素阈值
        update_footprint_enabled: true
        combination_method: 1      # 图层融合策略：1 代表取最大值覆盖 (Maximum)
        origin_z: 0.0              # 垂直坐标系原点高度
        publish_voxel_map: true    # 必须设为 true，否则节点不会向 RViz 发布 3D 调试数据
        transform_tolerance: 0.2
        mapping_mode: false        # 若设为 true，体素将永不衰减，此时等同于普通 3D 建图模式
        
        # --- 观测传感器数据源配置 ---
        observation_sources: pointcloud
        pointcloud:
          data_type: PointCloud2   # 必须使用三维点云消息作为输入
          topic: /intel_realsense_r200_depth/points # 订阅深度相机的物理/仿真点云话题
          marking: true            # 允许标记致命障碍物
          clearing: true           # 允许利用传感器视场视线进行物理清除
          obstacle_range: 3.0      # 标记障碍物的最大有效截断距离 (3米)
          min_obstacle_height: 0.1 # 过滤掉地面干扰的最小高度
          max_obstacle_height: 2.0 # 过滤掉过高悬空物的最大高度
          vertical_fov_angle: 0.8745   # 深度相机的垂直视场角 (弧度)
          horizontal_fov_angle: 1.048  # 深度相机的水平视场角 (弧度)
          clear_after_reading: true    # 读取单帧后立即清空缓冲区，完全交付给时空衰减器接管
```

## 3. 系统仿真联调与 3D 体素验证

配置完成后，系统需要重新加载参数并在 Gazebo 三维仿真环境中验证外部插件的运行状态。

### 3.1 级联点火指令

开启终端，依次执行编译构建、基础世界加载、以及携带外部插件 YAML 的 Nav2 决策服务器：

Bash

```
# 步骤 1：重构空间编译
cd ~/ros2_ws
colcon build --packages-select sam_bot_description
source install/setup.bash

# 步骤 2：启动基础底盘与物理环境仿真
ros2 launch sam_bot_description display.launch.py

# 步骤 3：加载定制的参数文件并激活导航大脑（在另一个终端运行）
source ~/ros2_ws/install/setup.bash
CONFIG_DIR=$(ros2 pkg prefix sam_bot_description)/share/sam_bot_description/config/nav2_params.yaml
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=$CONFIG_DIR
```

### 3.2 RViz2 高阶 3D 渲染可视化校验

为了在图形化监视器中正确审视 STVL 的三维网格运作情况，必须对 RViz2 的渲染视效进行定制化配置：

1. **添加显示图层：** 在 RViz2 界面左侧点击 `Add` 按钮，在 `By topic` 选项卡下，寻找到由 STVL 插件向上发布的专用话题：`/local_costmap/voxel_grid`（或 `/global_costmap/voxel_grid`）。
2. **重构样式表：**
   - 将该图层的渲染样式（Style）由默认的 `Points` 修改为 **`Boxes`（立方体块）**。
   - 将立方体的大小（Size）配置为 **`0.05`**（必须与 YAML 配置文件中的 `voxel_size` 保持 1:1 绝对相等）。
3. **预期宏观现象：** 此时，RViz 界面中机器人前方会出现一堆密密麻麻、呈矩阵排列的三维彩色/中性立体方块，它们与 Gazebo 物理世界中的立方体障碍物在三维空间中形成完美重合。当 Gazebo 中的动态障碍物移走后，RViz 中的 3D 体素方块会随着 `voxel_decay` 设定的倒计时呈现出平滑的淡出与自发湮灭效果，证明外部插件动态加载成功，感知层资源开销已成功降至理论最优水平。