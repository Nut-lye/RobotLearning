# 机器人模型与 RViz 基础：URDF 到可视化

> 阅读位置：第二阶段，理解机器人身体和 TF 静态结构。
>
> 前置建议：如果你还没跑过完整导航，先读 `实践操作/Turtlebot3仿真导航快速跑通.md`。
>
> 本章目标：创建 `sam_bot_description`，写出 URDF、launch 和 RViz 配置，让机器人模型能在 RViz 中显示，并为后续 Gazebo、传感器、Nav2 提供基础坐标树。

## 本章在导航系统中的位置

Nav2 不是直接控制一个“抽象小点”移动，它必须知道机器人的身体结构、传感器安装位置和坐标关系。URDF 负责描述这些内容，`robot_state_publisher` 负责把 URDF 变成 TF。

本章解决的是导航链路中的第一段：

```text
URDF / xacro 机器人模型
  -> robot_state_publisher
  -> base_link、轮子、雷达等 TF
  -> RViz 可视化检查
```

学完本章后，你应该能回答：

- `base_link` 是什么。
- URDF 中 `link` 和 `joint` 分别描述什么。
- 为什么 RViz 能显示机器人模型。
- 后续 Gazebo 和 Nav2 为什么都依赖这份机器人描述。

## 接下来怎么衔接

- 继续读 `实践操作/Gazebo仿真与EKF里程计.md`：把静态模型放进 Gazebo，让它能动、有里程计。
- 继续读 `实践操作/传感器建图定位与代价地图.md`：给机器人接入雷达、地图、定位和代价地图。

---



```
# 1. 安装 Humble 版本的必备依赖包
sudo apt update
sudo apt install -y ros-humble-joint-state-publisher-gui ros-humble-xacro

# 2. 创建并进入工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 3. 创建 sam_bot_description 功能包
ros2 pkg create --build-type ament_cmake sam_bot_description

# 4. 进入功能包，并创建所需的全部子目录
cd sam_bot_description
mkdir -p src/description launch rviz
```

### 第二阶段：填充核心代码文件

现在你位于 `~/ros2_ws/src/sam_bot_description` 目录下。我们需要创建并填充 3 个核心文件。可以使用 `gedit` 或 `vscode` 创建它们。

#### 1. 机器人结构描述文件 (URDF)

创建文件：`src/description/sam_bot_description.urdf`

**完整代码**：

```
<?xml version="1.0"?>
<robot name="sam_bot" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:property name="base_width" value="0.31"/>
  <xacro:property name="base_length" value="0.42"/>
  <xacro:property name="base_height" value="0.18"/>

  <xacro:property name="wheel_radius" value="0.10"/>
  <xacro:property name="wheel_width" value="0.04"/>
  <xacro:property name="wheel_ygap" value="0.025"/>
  <xacro:property name="wheel_zoff" value="0.05"/>
  <xacro:property name="wheel_xoff" value="0.12"/>

  <xacro:property name="caster_xoff" value="0.14"/>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1.0 1.0 1.0"/>
      </material>
    </visual>
  </link>

  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0.0 0.0 ${-(wheel_radius+wheel_zoff)}" rpy="0 0 0"/>
  </joint>

  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_link">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="Gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>
    </link>

    <joint name="${prefix}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_link"/>
      <origin xyz="${x_reflect*wheel_xoff} ${y_reflect*(base_width/2+wheel_ygap)} ${-wheel_zoff}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <xacro:wheel prefix="drivewhl_l" x_reflect="-1" y_reflect="1" />
  <xacro:wheel prefix="drivewhl_r" x_reflect="-1" y_reflect="-1" />

  <link name="front_caster">
    <visual>
      <geometry>
        <sphere radius="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1.0 1.0 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_caster"/>
    <origin xyz="${caster_xoff} 0.0 ${-(base_height/2)}" rpy="0 0 0"/>
  </joint>

</robot>
```

#### 2. 一键启动脚本 (Launch)

创建文件：`launch/display.launch.py`

完整代码：

```
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
```

#### 3. RViz 预设界面配置 (RViz Config)

创建文件：`rviz/urdf_config.rviz` 并将以下**完整代码**复制进去：  

```
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1/Links1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 557
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
    - Alpha: 0.6
      Class: rviz_default_plugins/RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Marker Scale: 0.3
      Show Arrows: true
      Show Axes: true
      Show Names: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
    Saved: ~
```

### 第三阶段：修改编译配置 (CMakeLists.txt 与 package.xml)

这两个文件已经在根目录下（`~/ros2_ws/src/sam_bot_description/`）由 `ros2 pkg create` 自动生成了，我们只需要修改它们。  

#### 1. 修改 `package.xml`

用编辑器打开它，在 `<buildtool_depend>ament_cmake</buildtool_depend>` 这一行下面，添加这 5 行运行依赖：  



```
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
```

#### 2. 修改 `CMakeLists.txt`

用编辑器打开它，滚动到文件最底部，在 `if(BUILD_TESTING)` 这一行的**正上方**，添加这 4 行代码（告诉编译器把这三个文件夹打包进去）：

```
install(
  DIRECTORY src launch rviz
  DESTINATION share/${PROJECT_NAME}
)
```

### 第四阶段：编译与运行 🚀

回到你的工作空间根目录，执行最终的编译和启动命令：

```
cd ~/ros2_ws

# 1. 编译你刚刚写的包
colcon build --packages-select sam_bot_description

# 2. 刷新环境变量 (非常重要！)
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 启动节点与可视化软件
ros2 launch sam_bot_description display.launch.py
```

执行完毕后，你将看到一个蓝色的车体出现在黑色的 3D 界面中，并且会弹出一个带有滑动条的小窗口。滑动小窗口里的轮子转角（`drivewhl_l_joint` 和 `drivewhl_r_joint`），RViz 里面的轮子就会跟着实时旋转了！









添加物理属性 如电机：

请回到你的工程目录中，打开之前创建的 `src/description/sam_bot_description.urdf` 文件， 替换为以下完整代码：

```
<?xml version="1.0"?>
<robot name="sam_bot" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:property name="base_width" value="0.31"/>
  <xacro:property name="base_length" value="0.42"/>
  <xacro:property name="base_height" value="0.18"/>

  <xacro:property name="wheel_radius" value="0.10"/>
  <xacro:property name="wheel_width" value="0.04"/>
  <xacro:property name="wheel_ygap" value="0.025"/>
  <xacro:property name="wheel_zoff" value="0.05"/>
  <xacro:property name="wheel_xoff" value="0.12"/>

  <xacro:property name="caster_xoff" value="0.14"/>

  <xacro:macro name="box_inertia" params="m w h d">
    <inertial>
      <origin xyz="0 0 0" rpy="${pi/2} 0 ${pi/2}"/>
      <mass value="${m}"/>
      <inertia ixx="${(m/12) * (h*h + d*d)}" ixy="0.0" ixz="0.0" iyy="${(m/12) * (w*w + d*d)}" iyz="0.0" izz="${(m/12) * (w*w + h*h)}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertial>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0" />
      <mass value="${m}"/>
      <inertia ixx="${(m/12) * (3*r*r + h*h)}" ixy = "0" ixz = "0" iyy="${(m/12) * (3*r*r + h*h)}" iyz = "0" izz="${(m/2) * (r*r)}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="sphere_inertia" params="m r">
    <inertial>
      <mass value="${m}"/>
      <inertia ixx="${(2/5) * m * (r*r)}" ixy="0.0" ixz="0.0" iyy="${(2/5) * m * (r*r)}" iyz="0.0" izz="${(2/5) * m * (r*r)}"/>
    </inertial>
  </xacro:macro>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1.0 1.0 1.0"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <xacro:box_inertia m="15" w="${base_width}" d="${base_length}" h="${base_height}"/>
  </link>

  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0.0 0.0 ${-(wheel_radius+wheel_zoff)}" rpy="0 0 0"/>
  </joint>

  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_link">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="Gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertia m="0.5" r="${wheel_radius}" h="${wheel_width}"/>
    </link>

    <joint name="${prefix}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_link"/>
      <origin xyz="${x_reflect*wheel_xoff} ${y_reflect*(base_width/2+wheel_ygap)} ${-wheel_zoff}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <xacro:wheel prefix="drivewhl_l" x_reflect="-1" y_reflect="1" />
  <xacro:wheel prefix="drivewhl_r" x_reflect="-1" y_reflect="-1" />

  <link name="front_caster">
    <visual>
      <geometry>
        <sphere radius="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1.0 1.0 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
      </geometry>
    </collision>
    <xacro:sphere_inertia m="0.5" r="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_caster"/>
    <origin xyz="${caster_xoff} 0.0 ${-(base_height/2)}" rpy="0 0 0"/>
  </joint>

</robot>
```



保存文件后，像之前一样在终端里编译并运行：

``` 
cd ~/ros2_ws
colcon build --packages-select sam_bot_description
source install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

需要你可以这样做：

1. 在 RViz 左侧的 `Displays` 面板里找到 `RobotModel`。
2. 展开它，取消勾选 `Visual Enabled`（关闭视觉模型）。
3. 勾选 `Collision Enabled`（开启碰撞模型）。

你会看到机器人的颜色消失了，变成了粗糙的纯色网格