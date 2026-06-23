# RobotLearning

本项目用于机器人学习与实训，内容主要围绕 ROS 2、视觉感知、Unitree G1 人形机器人动作接口和教学文档展开。

## 项目内容

- `src/moduleB/`：模块 B 练习代码，包含 OCR 识别、Unitree G1 高层预置动作接口、MuJoCo 仿真检查脚本，以及 Python/C++ 最小动作示例。
- `src/ros2_g1_rviz/`：Unitree G1 的 RViz2 可视化小包，支持 URDF 显示、关节状态 demo 播放和 LowState 到 JointState 桥接。
- `src/yolo_ros/`：基于 ROS 2 的 YOLO 检测、跟踪、分割和 3D 感知相关功能。
- `src/urdf_tutorial/`：URDF 机器人模型练习材料。
- `docs/`：课程文档、环境配置说明、模块 B 教学材料和题目参考说明。
- `third_party/`：外部依赖源码或仿真工具，例如 Unitree SDK、Unitree MuJoCo 等。

## 快速开始

进入项目根目录：

```bash
cd /home/a24/project/RobotLearning
```

查看模块 B 的使用说明：

```bash
less src/moduleB/README.md
```

安装模块 B Python 环境：

```bash
chmod +x src/moduleB/setup_g1_env.sh
src/moduleB/setup_g1_env.sh
```

不连接真机时，可以先用 dry-run 检查动作解析流程：

```bash
python3 src/moduleB/g1_action_interface.py --list
python3 src/moduleB/g1_action_interface.py --dry-run --actions "站立,鼓掌,释放手臂"
```

## 常用文档

- `docs/项目环境与电脑配置要求.md`
- `docs/ROS2多工作空间管理指南.md`
- `docs/模块B-G1高层预置动作机制教学文档.md`
- `docs/模块B-G1动作接口与Unitree-GitHub对应说明.md`

## 注意事项

- 真实连接 Unitree G1 前，请先确认网卡、机器人姿态、急停和周围安全环境。
- `third_party/` 中的依赖可能较大，建议按具体模块 README 的说明安装或更新。
- 不同模块依赖不同运行环境，优先参考对应目录下的 README。
