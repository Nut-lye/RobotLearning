# Gazebo 仿真与里程计：让机器人在物理世界里动起来

> 阅读位置：第二阶段，理解仿真底盘、IMU、EKF 和 `odom -> base_link`。
>
> 前置建议：先读 `实践操作/机器人模型URDF与RViz可视化.md`，知道机器人模型和 TF 是怎么来的。
>
> 本章目标：把 URDF 机器人放进 Gazebo，接入差速驱动、IMU 和 EKF，为 Nav2 提供稳定的里程计和底盘控制基础。

## 本章在导航系统中的位置

如果说 URDF 解决“机器人长什么样”，Gazebo 就解决“机器人怎么在仿真世界里动”。Nav2 控制器最终会输出 `/cmd_vel`，Gazebo 差速驱动插件接收速度指令并推动机器人，同时发布 `/odom`。

本章对应导航链路：

```text
Gazebo 差速驱动 / IMU
  -> /odom /imu
  -> robot_localization(EKF)
  -> odom -> base_link
  -> Nav2 controller 可用
```

学完本章后，你应该能判断：

- `/odom` 有没有发布。
- `odom -> base_link` 是否存在。
- `/cmd_vel` 有速度但机器人不动时，应检查 Gazebo 插件和话题命名空间。
- 为什么仿真导航必须统一 `use_sim_time:=True`。

## 接下来怎么衔接

- 读 `实践操作/传感器建图定位与代价地图.md`：理解 `/scan`、SLAM/AMCL、地图和代价地图。
- 读 `实践操作/Nav2代价地图参数配置.md`：开始配置 Nav2 用来避障的风险地图。

---

## 1. 概述

在 ROS 2 环境下进行机器人导航开发时，系统需要高精度的局部定位（里程计）来支持底层控制与避障。本文档旨在详细阐述如何通过 Gazebo 物理仿真引擎模拟机器人的底层硬件（差速电机与 IMU），并引入 `robot_localization` 包中的扩展卡尔曼滤波 (EKF) 算法，对多源传感器数据进行融合，最终为 Nav2 提供平滑且准确的里程计信息及坐标系转换 (`odom` $\rightarrow$ `base_link`)。

### 1.1 核心组件说明

- **Gazebo 仿真系统：** 在无实体机器人的开发阶段，Gazebo 负责接管底盘控制与物理反馈。其内置的**差速驱动插件 (Differential Drive Plugin)** 用于模拟轮子转动并输出基础里程计数据，**IMU 插件 (IMU Sensor Plugin)** 用于模拟陀螺仪并输出角速度与加速度。
- **Robot Localization (EKF 融合)：** 基础的轮子里程计存在打滑累积误差，而单体 IMU 存在积分漂移现象。EKF 节点同时订阅上述两种数据，通过数学模型进行优劣互补，计算出高精度的系统状态，并对外发布修正后的 `/odometry/filtered` 话题及 `odom` $\rightarrow$ `base_link` 的 TF 变换。

## 2. 环境依赖配置

在开始集成之前，系统需要安装 Gazebo 仿真及传感器融合所需的核心 ROS 2 依赖包。在 Ubuntu 终端中执行以下指令：

Bash

```
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-robot-localization
```

## 3. 滤波器 (EKF) 参数配置

系统需要一个 YAML 配置文件来指导 EKF 节点如何处理传入的传感器数据。

1. 在功能包 `sam_bot_description` 根目录下创建 `config` 文件夹。
2. 新建配置文件 `config/ekf.yaml`，并写入以下配置信息。该配置指定了工作频率、坐标系名称，并精确设定了从里程计 (`odom0`) 提取位置与偏航角，从 IMU (`imu0`) 提取姿态及角速度的矩阵策略：

YAML

```
ekf_filter_node:
    ros__parameters:
        frequency: 30.0
        two_d_mode: false
        publish_acceleration: true
        publish_tf: true
        map_frame: map
        odom_frame: odom
        base_link_frame: base_link
        world_frame: odom

        # 差速驱动里程计输入配置
        odom0: demo/odom
        odom0_config: [true,  true,  true,
                       false, false, false,
                       false, false, false,
                       false, false, true,
                       false, false, false]

        # IMU 传感器输入配置
        imu0: demo/imu
        imu0_config: [false, false, false,
                      true,  true,  true,
                      false, false, false,
                      false, false, false,
                      false, false, false]
```

## 4. URDF 模型插件注入

为使 Gazebo 能够识别机器人并生成相应的传感器数据，需在机器人的 URDF 描述文件 (`src/description/sam_bot_description.urdf`) 末尾（`</robot>` 标签上方）注入物理行为插件。

### 4.1 添加 IMU 传感器连杆与插件

XML

```
  <link name="imu_link">
    <visual>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
    </collision>
    <xacro:box_inertia m="0.1" w="0.05" d="0.05" h="0.05"/>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.01"/>
  </joint>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <namespace>/demo</namespace>
          <remapping>~/out:=imu</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
```

### 4.2 添加差速驱动插件

XML

```
  <gazebo>
    <plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
      <ros>
        <namespace>/demo</namespace>
      </ros>
      <left_joint>drivewhl_l_joint</left_joint>
      <right_joint>drivewhl_r_joint</right_joint>
      <wheel_separation>0.36</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf> <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
```

## 5. 启动脚本 (Launch) 重构

由于系统引入了物理仿真，此前用于手动模拟关节运动的滑动条界面 (`joint_state_publisher_gui`) 必须被移除。需将启动文件 `launch/display.launch.py` 更新为以下逻辑，以依次启动模型发布器、Gazebo 环境、模型生成器以及 EKF 节点：

Python

```
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    ekf_config_path = os.path.join(pkg_share, 'config/ekf.yaml')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    # 启动 Gazebo 仿真环境
    start_gazebo_cmd = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], 
        output='screen'
    )

    # 将机器人模型注入到 Gazebo 中
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )

    # 启动 EKF 卡尔曼滤波融合节点
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path),
        robot_state_publisher_node,
        start_gazebo_cmd,
        spawn_entity,
        robot_localization_node,
        rviz_node
    ])
```

## 6. 编译构建与系统验证

### 6.1 编译配置

在执行编译前，需确保 CMake 工具能够正确打包新增的配置文件。修改工程根目录下的 `CMakeLists.txt` 文件，将 `config` 目录添加至 `install` 宏中：

CMake

```
install(
  DIRECTORY src launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
```

### 6.2 编译与运行

在工作空间根目录下执行标准化构建与环境刷新流程：

Bash

```
colcon build --packages-select sam_bot_description
source install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

### 6.3 验证指标

运行成功后，系统应弹出 Gazebo 三维仿真界面与 RViz 可视化界面。在新终端中执行以下诊断命令以验证系统集成度：

1. **验证 EKF 融合数据流：**

   执行 `ros2 topic echo /odometry/filtered`，应观察到由卡尔曼滤波器输出的高频、平滑的综合里程计状态矩阵。

2. **验证 TF 树转换：**

   执行 `ros2 run tf2_ros tf2_echo odom base_link`，应观察到连续输出的平移与旋转四元数，证明 `robot_localization` 节点已成功接管并发布该坐标转换关系。