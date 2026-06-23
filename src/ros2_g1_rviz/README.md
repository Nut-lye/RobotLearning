# ros2_g1_rviz

这个 ROS2 小包用于把 Unitree G1 的关节状态显示到 RViz2。它不是物理仿真器，不计算接触、动力学或控制闭环；它的作用是像 MuJoCo viewer 一样直观看到机器人姿态变化。

## 功能

- 使用 `robot_state_publisher` 加载 G1 URDF 并发布 TF。
- 默认使用包内简化 G1 29DOF URDF，可离线演示。
- `demo_joint_state_publisher` 发布 `/joint_states`，让 G1 在 RViz2 中做挥手、步态摆动或空闲动作。
- `lowstate_to_joint_state` 可把 Unitree `unitree_hg/msg/LowState` 转成 `/joint_states`，后续可接 MuJoCo 或真机低层状态。
- `sdk2_lowstate_to_joint_state` 使用 `unitree_sdk2py` 直接读取真机 `rt/lowstate`，适合配合 `src/moduleB/minimal_g1_action.py` 使用。

## 安装编译

```bash
cd /home/a24/project/RobotLearning
colcon build --packages-select ros2_g1_rviz
source install/setup.bash
```

如果系统还没有 RViz2 和 robot_state_publisher：

```bash
sudo apt install ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-robot-state-publisher
```

## 直接看 G1 动起来

默认启动 RViz2，并播放右手挥手动作：

```bash
ros2 launch ros2_g1_rviz display.launch.py
```

切换教学动作：

```bash
ros2 launch ros2_g1_rviz display.launch.py motion:=walk
ros2 launch ros2_g1_rviz display.launch.py motion:=idle
```

只发布模型和 TF，不打开 RViz2：

```bash
ros2 launch ros2_g1_rviz display.launch.py rviz:=false
```

## 使用 Unitree 官方 URDF

包内简化 URDF 只用于快速演示。如果要看官方 mesh 模型，先准备官方 `unitree_ros/robots/g1_description`，然后传入 URDF 绝对路径：

```bash
ros2 launch ros2_g1_rviz display.launch.py \
  urdf:=/path/to/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf
```

`display.launch.py` 会把官方 URDF 中的 `meshes/...` 相对路径转成 `file://` 绝对路径，便于 RViz2 加载 STL。

## 订阅 MuJoCo 或真机 LowState

先 source Unitree ROS2 工作空间，确保能导入 `unitree_hg/msg/LowState`。然后关闭 demo 发布器，打开 LowState 桥接：

```bash
ros2 launch ros2_g1_rviz display.launch.py \
  use_demo:=false \
  use_lowstate:=true \
  lowstate_topic:=lowstate
```

如果你的 Unitree 环境把低层状态发布在其他话题，例如 `lf/lowstate`：

```bash
ros2 launch ros2_g1_rviz display.launch.py \
  use_demo:=false \
  use_lowstate:=true \
  lowstate_topic:=lf/lowstate
```

数据流是：

```text
Unitree G1 / MuJoCo LowState
        |
        v
lowstate_to_joint_state
        |
        v
/joint_states
        |
        v
robot_state_publisher -> /tf
        |
        v
RViz2 RobotModel
```

## 配合 minimal_g1_action.py 显示真机动作

这是最推荐的真机镜像流程。RViz2 进程只读取 G1 的 `rt/lowstate`，不发送控制命令；动作仍然由 `minimal_g1_action.py` 触发。

终端 1：启动 RViz2 真机姿态镜像：

```bash
cd /home/a24/project/RobotLearning
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ros2_g1_rviz display.launch.py \
  use_demo:=false \
  use_sdk2_lowstate:=true \
  iface:=eth0
```

如果连接 G1 的网卡不是 `eth0`，换成 `ip link` 看到的实际网卡名，例如 `enp3s0`。

终端 2：执行真机预置动作：

```bash
cd /home/a24/project/RobotLearning
python3 src/moduleB/minimal_g1_action.py --iface eth0 --action-id 17 --release
```

此时 RViz2 显示的是 G1 实际上报的 29 个关节角，因此会跟随真机动作变化。

## 注意

- RViz2 只负责可视化，不能替代 MuJoCo 的物理仿真。
- `/joint_states` 的关节名按官方 G1 29DOF URDF 命名，例如 `left_hip_pitch_joint`、`waist_yaw_joint`、`right_wrist_yaw_joint`。
- 同一时间只保留一个 `/joint_states` 来源，避免 demo 和 LowState 桥同时发布造成画面抖动。
