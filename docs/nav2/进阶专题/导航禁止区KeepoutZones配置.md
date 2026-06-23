# 导航禁止区：让机器人知道哪里绝对不能走

> 阅读位置：第四阶段，普通 costmap 调参之后。
>
> 前置建议：先读 `实践操作/Nav2代价地图参数配置.md`，理解 costmap 图层机制。
>
> 本章目标：使用 Keepout Zones 给 Nav2 增加禁止通行区域或首选通道。

## 本章在导航系统中的位置

普通 costmap 根据墙、障碍物和膨胀层决定风险；禁止区是人为额外规定“这里不能走”，即使物理上看起来可通行。

本章对应链路：

```text
keepout_mask.pgm / keepout_mask.yaml
  -> filter_info_server / map_server
  -> KeepoutFilter
  -> global/local costmap
  -> planner 避开禁止区
```

什么时候用：

- 题目要求不能进入某些区域。
- 需要强制机器人走指定通道。
- 地图中某些区域没有墙，但规则上禁止通行。

---

# 配置导航禁止区与首选通道 (Keepout Zones)

## 1. 核心理论概述

在实际场景（如仓储或工厂）中，需要通过硬性约束限制机器人的活动范围（如禁止进入楼梯间）或强制其行驶于特定走廊。Nav2 通过 `KeepoutFilter`（禁止区域过滤器）插件实现此功能。系统由三部分构成：

1. **掩码图像 (Mask Image) 与元数据：** 定义空间权限的灰度位图。
2. **过滤器数据发布层：** 负责读取掩码并将其转化为 `OccupancyGrid` 格式发布的独立生命周期服务器。
3. **代价地图插件层：** 挂载于 Nav2 全局与局部代价地图之上，实时干预路径规划与底层控制算法的 `KeepoutFilter`。

## 2. 工程目录架构规划

请在您的工作空间（如 `sam_bot_description` 功能包）中建立以下目录结构。本教程将基于此结构创建和存放所有工程文件：

Plaintext

```
~/ros2_ws/src/sam_bot_description/
│
├── maps/                              # 📍 存放所有地图与掩码数据
│   ├── keepout_mask.pgm               # (自行绘制) 禁止区位图
│   └── keepout_mask.yaml              # (步骤 3) 掩码元数据配置
│
├── config/                            # 📍 存放所有核心参数
│   ├── keepout_info_params.yaml       # (步骤 4) 过滤器服务器运行参数
│   └── nav2_params.yaml               # (步骤 5) 包含过滤器插件的 Nav2 导航参数
│
└── launch/                            # 📍 存放启动脚本
    ├── filter_server.launch.py        # (步骤 6) 过滤器服务器集群启动脚本
    └── ...
```

## 3. 掩码文件准备 (`maps/` 目录)

### 3.1 绘制禁止区位图 (`keepout_mask.pgm`)

1. 复制您现有的环境建图文件（如 `my_map.pgm`），重命名为 `keepout_mask.pgm` 并放入 `maps/` 文件夹。
2. 使用位图编辑软件（如 GIMP），将禁止机器人进入的区域涂成**纯黑色**（灰度值 0），将允许自由行驶的区域涂成**纯白色**（灰度值 255）。保存图像。

### 3.2 编写掩码元数据 (`keepout_mask.yaml`)

在 `maps/` 目录下新建 `keepout_mask.yaml` 文件，写入以下元数据（注意：由于是基于原地图修改，`resolution` 必须与原地图一致，`origin` 的偏航角必须为 `0.0`）：

YAML

```
image: keepout_mask.pgm
mode: trinary
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

## 4. 过滤器服务器参数 (`config/keepout_info_params.yaml`)

在 `config/` 目录下新建 `keepout_info_params.yaml` 文件。该文件用于配置掩码读取服务器与信息发布服务器的底层参数：

YAML

```
costmap_filter_info_server:
  ros__parameters:
    use_sim_time: true
    type: 0                                # 0 表示当前发布的是禁止区 (Keepout) 过滤器信息
    filter_info_topic: "/costmap_filter_info"
    mask_topic: "/keepout_filter_mask"
    base: 0.0                              # 禁止区过滤器禁用线性变换系数
    multiplier: 1.0

filter_mask_server:
  ros__parameters:
    use_sim_time: true
    frame_id: "map"
    topic_name: "/keepout_filter_mask"     # 必须与上方 mask_topic 一致
    yaml_filename: "keepout_mask.yaml"     # 在启动脚本中会被动态替换为绝对路径
```

## 5. 导航大脑参数集成 (`config/nav2_params.yaml`)

修改已有的 `nav2_params.yaml`，在全局与局部代价地图中独立挂载 `KeepoutFilter` 插件。**务必将 `filters` 与 `plugins` 分开声明，防止干扰膨胀层算法。**

YAML

```
# ==========================================
# 全局代价地图 (Global Costmap)
# ==========================================
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      resolution: 0.05
      track_unknown_space: false
      rolling_window: false

      # 声明常规图层与过滤器图层
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      filters: ["keepout_filter"]

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
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

      # 禁止区过滤器配置
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "/costmap_filter_info"

# ==========================================
# 局部代价地图 (Local Costmap)
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

      plugins: ["voxel_layer", "inflation_layer"]
      filters: ["keepout_filter"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
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

      # 局部代价地图拦截配置
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "/costmap_filter_info"
```

## 6. 编写独立启动脚本 (`launch/filter_server.launch.py`)

为了使工程自给自足，我们编写专属的 Python 启动脚本，用于拉起过滤器生命周期节点集群。在 `launch/` 目录下新建 `filter_server.launch.py`：

Python

```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. 获取包路径
    pkg_dir = get_package_share_directory('sam_bot_description')
    
    # 2. 定义文件绝对路径
    params_file = os.path.join(pkg_dir, 'config', 'keepout_info_params.yaml')
    mask_yaml_file = os.path.join(pkg_dir, 'maps', 'keepout_mask.yaml')
    
    # 3. 动态重写 YAML (将相对路径替换为系统绝对路径)
    param_substitutions = {
        'use_sim_time': 'true',
        'yaml_filename': mask_yaml_file
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 4. 定义生命周期节点管理器与发布服务器
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[configured_params])

    start_costmap_filter_info_server_cmd = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[configured_params])

    # 5. 组合启动指令
    ld = LaunchDescription()
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
```

**⚠️ 编译提示：** 编写完启动脚本和参数文件后，请务必更新 `CMakeLists.txt` 以确保 `maps` 和 `config` 文件夹被正确安装，随后在工作空间根目录执行 `colcon build --packages-select sam_bot_description` 并 `source install/setup.bash`。

## 7. 完整运行操作流 (系统联调)

工程构建完毕后，请开启四个独立的终端进行级联启动与验证。

**终端 1：启动底层物理仿真世界**

Bash

```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**终端 2：点亮过滤器数据服务器 (自定义脚本)**

Bash

```
source ~/ros2_ws/install/setup.bash
ros2 launch sam_bot_description filter_server.launch.py
```

**终端 3：携带禁止区插件启动 Nav2 大脑**

Bash

```
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
CONFIG_DIR=$(ros2 pkg prefix sam_bot_description)/share/sam_bot_description/config/nav2_params.yaml

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=$CONFIG_DIR
```

**终端 4：启动 RViz2 可视化监控与下发指令**

Bash

```
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz
```

### 验证标准：

1. 在 RViz 中添加一个 `Map` 组件，话题选择 `/keepout_filter_mask`，确认黑白掩码图成功显示并与环境重合。
2. 使用 RViz 顶部的 `Nav2 Goal` 设定一个必须穿过禁止区才能到达的终点。
3. 观察全局路径（绿线）是否计算出一条**严格绕过掩码黑色区域**的远路；若强行操控机器人靠近黑区，观察底盘是否被局部控制器强行刹停或弹开。

为了直观验证该机制的逻辑闭环，您可以利用下方模拟器体验掩码绘制对全局路径的强制干预效果：