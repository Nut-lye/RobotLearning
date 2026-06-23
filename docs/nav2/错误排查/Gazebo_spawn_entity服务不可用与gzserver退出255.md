# Gazebo：spawn_entity 服务不可用与 gzserver 退出 255

## 1. 错误现象

启动 Turtlebot3 Gazebo：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

出现：

```text
[spawn_entity.py] Waiting for service /spawn_entity
[spawn_entity.py] Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?
[spawn_entity.py] Spawn service failed. Exiting.
```

或者更早看到：

```text
[ERROR] [gzserver-1]: process has died [pid ..., exit code 255, cmd 'gzserver ...']
```

## 2. 先理解这两个错误的关系

`/spawn_entity` 服务不是 `spawn_entity.py` 自己提供的，而是 Gazebo 里的 `libgazebo_ros_factory.so` 插件提供的。

所以链路是：

```text
gzserver 正常启动
  -> 加载 libgazebo_ros_factory.so
  -> 发布 /spawn_entity 服务
  -> spawn_entity.py 调用服务
  -> 机器人被插入 Gazebo 世界
```

如果 `gzserver` 没正常启动、启动太慢、卡在模型加载，或者 factory 插件没加载成功，那么 `spawn_entity.py` 必然等不到 `/spawn_entity`。

所以不要只盯着 `spawn_entity.py`。要先看 `gzserver`。

## 3. 常见根因

### 3.1 conda Python 污染 ROS

典型报错：

```text
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension ... cpython-313 ... isn't present
```

原因：

ROS Humble 的 Python 是 3.10，但 conda base 可能把 Python 3.13 放到了前面。

检查：

```bash
which python3
python3 --version
echo $CONDA_PREFIX
```

正确应类似：

```text
/usr/bin/python3
Python 3.10.x
CONDA_PREFIX=
```

修复：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
```

并关闭 conda 自动进入 base：

```bash
conda config --set auto_activate_base false
```

### 3.2 旧 Gazebo 进程或端口残留

典型报错：

```text
Unable to start server[bind: Address already in use]
```

Gazebo 默认 master 端口是 `11345`。如果旧 `gzserver` 没退出，新 Gazebo 会启动失败。

检查：

```bash
pgrep -a gzserver
pgrep -a gzclient
ss -ltnp | grep 11345
```

修复：

```bash
pkill -9 gzserver
pkill -9 gzclient
```

确认：

```bash
pgrep -a gzserver
ss -ltnp | grep 11345
```

没有输出才干净。

### 3.3 Gazebo 找不到本地模型，去线上模型库导致卡住

典型日志：

```text
Getting models from http://models.gazebosim.org
```

这不一定说明网络完全不通。即使：

```bash
curl -I http://models.gazebosim.org
```

返回 `200 OK`，Gazebo 仍可能因为模型路径没配好、线上资源解析慢、WSL 网络/DNS 偶发慢而卡住。

Turtlebot3 world 需要本地找到：

```text
model://turtlebot3_world
model://turtlebot3_common/...
model://ground_plane
model://sun
```

检查：

```bash
echo $GAZEBO_MODEL_PATH
```

如果为空或没有 Turtlebot3 模型路径，就容易出问题。

修复：

```bash
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_MODEL_DATABASE_URI=
```

含义：

- `GAZEBO_MODEL_PATH`：强制 Gazebo 从本地找 Turtlebot3 和 Gazebo 内置模型。
- `GAZEBO_MODEL_DATABASE_URI=`：禁用线上模型库，避免卡在网络模型下载。

### 3.4 WSL/虚拟机图形渲染问题

可以尝试：

```bash
export SVGA_VGPU10=0
```

它常用于绕开部分 WSL/VMware/OpenGL 虚拟图形问题。

注意：

这个变量主要影响 Gazebo 图形/渲染，不是 `/spawn_entity` 的本质来源。但如果图形或模型加载拖住 Gazebo，它也可能间接有帮助。

### 3.5 world 文件里的 `<sim_time>`

网上有人说把 `.world` 文件中的：

```xml
<sim_time>...</sim_time>
```

改成：

```xml
<sim_time>0 0</sim_time>
```

但不是所有 `.world` 文件都有 `<sim_time>`。例如 Turtlebot3 默认：

```text
/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
```

原文件里没有 `<sim_time>`。

所以这个方法不是通用解法。当前案例中，真正有效的是模型路径和禁用线上模型库。

## 4. 推荐稳定启动命令

每次启动 Turtlebot3 Gazebo 前，建议使用：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_MODEL_DATABASE_URI=
export SVGA_VGPU10=0

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

成功时应看到：

```text
[spawn_entity]: Calling service /spawn_entity
[spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [waffle]
```

## 5. 如果还失败，按这个顺序排查

### 5.1 确认没有残留

```bash
pgrep -a gzserver
pgrep -a gzclient
ss -ltnp | grep 11345
```

### 5.2 确认 Python 没被 conda 污染

```bash
which python3
python3 --version
echo $CONDA_PREFIX
```

### 5.3 确认模型路径

```bash
echo $GAZEBO_MODEL_PATH
ls /opt/ros/humble/share/turtlebot3_gazebo/models
ls /usr/share/gazebo-11/models
```

### 5.4 确认 `/spawn_entity` 服务

Gazebo 启动后另开终端：

```bash
source /opt/ros/humble/setup.bash
ros2 service list | grep spawn
```

正常应有：

```text
/spawn_entity
```

### 5.5 看最新日志

```bash
ls -td ~/.ros/log/* | head
```

重点看：

```text
launch.log
gzserver_*.log
python3_*.log
```

如果 `gzserver` 先 exit 255，先修 `gzserver`，不要继续追 `spawn_entity.py`。

## 6. 最终结论

当前案例中，成功启动依赖这一组环境变量：

```bash
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_MODEL_DATABASE_URI=
export SVGA_VGPU10=0
```

这说明问题主要不是 `<sim_time>`，也不是完全没网，而是 Gazebo 在线模型库/模型路径/WSL 环境导致 `gzserver` 没能稳定提供 `/spawn_entity`。

