# 模块 B：Unitree G1 预置动作接口

本目录用于训练《人工智能训练师》模块 B：

- `OCR.py`：识别赛题图片中的“操作说明”文字。
- `g1_action_interface.py`：将中文/英文动作说明映射到 Unitree G1 高层预置动作接口。
- `setup_g1_env.sh`：安装 EasyOCR/OpenCV，并克隆安装 `unitree_sdk2_python`。

配套教学文档：[`模块B-G1高层预置动作机制教学文档.md`](../../docs/模块B-G1高层预置动作机制教学文档.md)。

## 1. 安装环境

使用脚本一键安装：

```bash
chmod +x src/moduleB/setup_g1_env.sh
src/moduleB/setup_g1_env.sh
```

或者手动逐条执行：

```bash
cd /home/a24/project/RobotLearning
python3 --version
python3 -m pip --version
python3 -m pip install --user -r src/moduleB/requirements.txt
mkdir -p third_party
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git third_party/unitree_sdk2_python
python3 -m pip install --user third_party/unitree_sdk2_python
python3 -c "import easyocr; import unitree_sdk2py; print('moduleB python env ok')"
```

如果 `third_party/unitree_sdk2_python` 已存在，更新即可：

```bash
git -C third_party/unitree_sdk2_python pull --ff-only
python3 -m pip install --user third_party/unitree_sdk2_python
```

`unitree_sdk2_python` 需要从 GitHub 克隆安装；若赛场网络受限，请提前把 SDK 放到 `third_party/unitree_sdk2_python`。

## 2. 本地仿真演练

不连接机器人时使用 `--dry-run`，用于检查 OCR 后的动作顺序和接口调用是否正确：

```bash
python3 src/moduleB/g1_action_interface.py --list
python3 src/moduleB/g1_action_interface.py --dry-run --actions "站立,鼓掌,释放手臂"
python3 src/moduleB/g1_action_interface.py --dry-run --instruction-file src/moduleB/samples/operation_instruction.txt --auto-release
```

## 3. 连接 G1 执行

真实执行前先确认机器人周围无障碍物、移动吊架和急停可用，再指定连接 G1 的网卡名：

```bash
ip link
python3 src/moduleB/g1_action_interface.py --iface eth0 --actions "start,clap,release arm"
```

常用动作可直接写中文：`鼓掌`、`击掌`、`拥抱`、`比心`、`举手`、`挥手`、`握手`、`飞吻`、`坐下`、`前进`、`左转`、`停止`。

如果希望执行 `minimal_g1_action.py` 时在 RViz2 中同步看到真机姿态，可先启动 G1 状态镜像窗口：

```bash
cd /home/a24/project/RobotLearning
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros2_g1_rviz display.launch.py use_demo:=false use_sdk2_lowstate:=true iface:=eth0
```

然后另开终端执行真机动作：

```bash
python3 src/moduleB/minimal_g1_action.py --iface eth0 --action-id 17 --release
```

## 4. 官方 G1 MuJoCo 仿真

Unitree 官方提供 `unitree_mujoco`，包含 G1 的 MuJoCo MJCF 模型和 DDS 桥接。它主要支持 `LowCmd`、`LowState`、`SportModeState`、G1 胸部 IMU 等底层仿真消息，适合做低层控制、策略部署和 sim-to-real 验证。

注意：`unitree_mujoco` 当前不是 G1 真机固件高层动作服务的完整替代品，不能直接复现 `G1ArmActionClient.ExecuteAction(17)` 这类“鼓掌/比心”预置手势。模块 B 的高层预置动作仍以真机 SDK 或赛方平台接口为准；MuJoCo 用于先验证 G1 模型、DDS 链路和低层控制程序。

安装并配置 G1 MuJoCo：

使用脚本一键安装：

```bash
chmod +x src/moduleB/setup_g1_mujoco_env.sh src/moduleB/run_g1_mujoco.sh src/moduleB/check_g1_mujoco_env.sh
src/moduleB/setup_g1_mujoco_env.sh
src/moduleB/check_g1_mujoco_env.sh
```

脚本会完成：

- 克隆 `unitree_sdk2` 到 `third_party/unitree_sdk2`，并本地安装到 `third_party/unitree_robotics`。
- 克隆 `unitree_mujoco` 到 `third_party/unitree_mujoco`。
- 下载 MuJoCo `3.3.6` 到 `~/.mujoco/mujoco-3.3.6`。
- 将 `simulate/config.yaml` 配置为 `robot: "g1"`、`robot_scene: "scene_29dof.xml"`、`domain_id: 1`、`interface: "lo"`、`enable_elastic_band: 1`。
- 编译官方 C++ 版 `simulate` 模拟器。

手动逐条执行：

```bash
cd /home/a24/project/RobotLearning

# 1. 安装系统依赖
sudo apt update
sudo apt install -y build-essential cmake git curl tar \
  libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev

# 2. 克隆并本地安装 unitree_sdk2
mkdir -p third_party
git clone https://github.com/unitreerobotics/unitree_sdk2.git third_party/unitree_sdk2
cmake -S third_party/unitree_sdk2 -B third_party/unitree_sdk2/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/a24/project/RobotLearning/third_party/unitree_robotics
cmake --build third_party/unitree_sdk2/build -j"$(nproc)"
cmake --install third_party/unitree_sdk2/build

# 3. 下载并解压 MuJoCo
mkdir -p ~/.mujoco
curl -L https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-x86_64.tar.gz \
  -o /tmp/mujoco-3.3.6-linux-x86_64.tar.gz
tar -xzf /tmp/mujoco-3.3.6-linux-x86_64.tar.gz -C ~/.mujoco

# 4. 克隆 unitree_mujoco 并链接 MuJoCo
git clone https://github.com/unitreerobotics/unitree_mujoco.git third_party/unitree_mujoco
ln -sfn ~/.mujoco/mujoco-3.3.6 third_party/unitree_mujoco/simulate/mujoco

# 5. 写入 G1 仿真配置
cat > third_party/unitree_mujoco/simulate/config.yaml <<'YAML'
robot: "g1"
robot_scene: "scene_29dof.xml"

domain_id: 1
interface: "lo"

use_joystick: 0
joystick_type: "xbox"
joystick_device: "/dev/input/js0"
joystick_bits: 16

print_scene_information: 1

enable_elastic_band: 1
YAML

# 6. 编译官方 C++ MuJoCo 模拟器
cmake -S third_party/unitree_mujoco/simulate -B third_party/unitree_mujoco/simulate/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/home/a24/project/RobotLearning/third_party/unitree_robotics;/home/a24/project/RobotLearning/third_party/unitree_robotics/lib/cmake"
cmake --build third_party/unitree_mujoco/simulate/build -j"$(nproc)"
```

如果上述仓库已经存在，更新命令如下：

```bash
git -C third_party/unitree_sdk2 pull --ff-only
git -C third_party/unitree_mujoco pull --ff-only
```

启动 G1 仿真：

使用脚本启动：

```bash
src/moduleB/run_g1_mujoco.sh
```

手动启动：

```bash
cd /home/a24/project/RobotLearning
export LD_LIBRARY_PATH="/home/a24/project/RobotLearning/third_party/unitree_robotics/lib:/home/a24/project/RobotLearning/third_party/unitree_mujoco/simulate/mujoco/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="/home/a24/project/RobotLearning/third_party/unitree_robotics:/home/a24/project/RobotLearning/third_party/unitree_robotics/lib/cmake:${CMAKE_PREFIX_PATH:-}"
cd third_party/unitree_mujoco/simulate/build
./unitree_mujoco
```

也可以临时覆盖场景：

```bash
src/moduleB/run_g1_mujoco.sh -r g1 -s scene_23dof.xml -i 1 -n lo
```

对应的手动命令：

```bash
cd /home/a24/project/RobotLearning/third_party/unitree_mujoco/simulate/build
./unitree_mujoco -r g1 -s scene_23dof.xml -i 1 -n lo
```

仿真窗口中人形机器人可用虚拟挂带辅助调试：按 `9` 启用/松开挂带，按 `7` 或方向键 `Up` 缩短挂带，按 `8` 或方向键 `Down` 放长挂带。源码只处理单次按下事件，如果高度变化不明显，需要连续多按几次。

如果系统依赖已提前装好，或当前用户没有 sudo 权限，可跳过 apt 步骤：

```bash
SKIP_APT=1 src/moduleB/setup_g1_mujoco_env.sh
```

## 5. OCR 到动作执行

```bash
python3 src/moduleB/OCR.py operation_instruction.jpg --paragraph > /tmp/g1_instruction.txt
python3 src/moduleB/g1_action_interface.py --dry-run --instruction-file /tmp/g1_instruction.txt
```

正式比赛接口以赛方平台为准；本目录对齐的是 Unitree 官方 SDK2 的 `G1ArmActionClient` 与 `LocoClient` 高层动作语义。