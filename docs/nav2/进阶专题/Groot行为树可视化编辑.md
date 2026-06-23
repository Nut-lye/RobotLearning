# Groot：离线编辑和理解 Nav2 行为树

> 阅读位置：第四阶段，已经理解普通导航流程后再读。
>
> 前置建议：先读 `理论知识/Nav2行为树与导航服务器.md` 和 `理论知识/行为树黑板机制.md`。
>
> 本章目标：使用 Groot 可视化 Nav2 行为树 XML，降低阅读和修改复杂行为树的难度。

## 本章在导航系统中的位置

Nav2 的 planner、controller、recovery 并不是随便运行的，它们由行为树调度。Groot 帮你把 XML 行为树变成图形结构，适合检查逻辑顺序。

本章对应链路：

```text
行为树 XML
  -> Groot 离线可视化/编辑
  -> Nav2 bt_navigator 加载
  -> 调度规划、控制、恢复动作
```

注意：Humble 中 Groot 主要作为离线编辑器使用，不要把它当作实时监控工具。

---

# 使用 Groot 图形化编辑行为树 (Behavior Tree)

## 1. 核心理论：图形化逻辑编排机制

Nav2 的高级导航逻辑主要由基于 `BehaviorTree.CPP` 库的行为树控制。行为树在物理存储上表现为层级化的 XML 文件（如前文所述的 `follow_dynamic_object.xml`）。

直接编写和审阅复杂的 XML 代码容易引发语法错误与逻辑嵌套混乱。为此，系统引入了官方配套的图形化界面工具 —— **Groot**。 Groot 提供以下三大核心工程功能：

1. **可视化加载 (Visualization)：** 将嵌套的 XML 代码渲染为具有直观数据流向的节点树状图。
2. **可视化编辑 (Editing)：** 通过鼠标拖拽节点、连线的方式，无需触碰代码即可重组导航逻辑流。
3. **节点属性配置 (Properties)：** 快速配置节点的输入/输出端口（Ports）和参数。

> **⚠️ 版本架构变更提示 (Galactic 至 Humble 及后续版本)：** 在早期的 ROS 2 版本中，Groot 支持连接到运行中的机器人进行“实时状态监控”。但由于动态监控对系统稳定性的影响，**从 Humble 版本开始，Nav2 已正式移除 Groot 的实时在线监控功能。** 目前，Groot 仅作为一款纯粹的“离线行为树编辑器”使用。

## 2. 工程实现一：Groot 环境配置与启动

### 2.1 安装 Groot

在 Ubuntu 系统中，可通过 ROS 2 的标准包管理器直接安装 Groot：

```
sudo apt update
sudo apt install ros-humble-groot
```

### 2.2 启动 Groot 编辑器

在终端中直接运行以下指令唤醒图形化界面：

```
Groot
```

启动后，在弹出的初始界面中，必须选择 **`Editor` (编辑器模式)** 进入主工作区。

## 3. 工程实现二：加载与编辑现有行为树

### 3.1 加载节点依赖字典 (Palette)

Groot 作为一款通用的行为树软件，默认情况下并不认识 Nav2 专有的节点（如 `ComputePathToPose` 或 `GoalUpdater`）。在读取 Nav2 的行为树之前，必须先将 Nav2 的节点“字典”导入给 Groot，该字典在 Groot 中被称为 **Palette（托盘/节点库）**。

1. 在 Groot 顶部菜单栏中，点击 **`Load palette from file`**（带有文件夹导入图标的按钮）。
2. 导航至 Nav2 的节点配置库目录，通常位于系统环境内： `/opt/ros/humble/share/nav2_behavior_tree/nav2_tree_nodes.xml`
3. 加载成功后，Groot 左侧的 `TreeNode Palette` 面板将出现大量深蓝色的 Nav2 专用节点。

### 3.2 导入并可视化目标行为树

1. 在 Groot 界面左上角，点击 **`Load tree`**。
2. 选中此前编写的任意行为树文件（例如：`follow_dynamic_object.xml` 或是官方默认的 `Maps_w_replanning_and_recovery.xml`）。
3. **渲染结果：** 主画布将自动生成该行为树的完整拓扑结构图。动作节点 (Action)、控制流节点 (Control) 与装饰器节点 (Decorator) 会通过颜色和形状进行明确的层级区分。

### 3.3 图形化编辑与参数修改

- **添加节点：** 从左侧面板将所需的节点拖拽至中央主画布。
- **逻辑连线：** 点击父节点底部的输出点，拖拽连接至子节点顶部的输入点，即可建立执行顺序。
- **修改端口参数：** 单击画布中的任意节点（如 `TruncatePath`），界面右侧或弹出的属性面板将列出该节点的所有端口（Ports）。开发者可直接在文本框中修改参数值（如将 `distance` 从 `1.0` 修改为 `2.0`）。
- **保存工程：** 修改完毕后，点击顶部菜单栏的保存图标，Groot 会自动将图形化界面重新编译为标准无误的 XML 文件供 Nav2 系统挂载调用。

## 4. 工程实现三：定义并导出多重自定义节点

在大型工程中，我们通常需要创建一系列自定义动作（例如：前进、停止、播放语音等）。编排工程师可以在 C++ 源码编写完成之前，在 Groot 中预先“占位”创建这些节点的接口。

### 4.1 在 GUI 中创建自定义节点接口

1. 在左侧 `TreeNode Palette` 面板中，点击 **`Create new node`**（通常为橙色加号图标）。
2. 录入“前进节点”的工程元数据：
   - **Node Name (节点名称):** `MoveForwardAction`
   - **Node Type (节点类型):** `Action`
   - **Ports (端口定义):** 添加一个 `input_port`，名为 `distance`，用于设定前进距离。点击 `OK` 保存。
3. 再次点击创建按钮，录入“停止节点”的工程元数据：
   - **Node Name (节点名称):** `StopAction`
   - **Node Type (节点类型):** `Action`
   - **Ports (端口定义):** 停止动作通常只需直接下发零速度指令，不需要参数，直接留空即可。点击 `OK` 保存。

### 4.2 导出节点字典配置

为防止自定义节点丢失，必须将其导出至物理文件：

1. 点击左侧面板右上角的 **`Export palette`**（导出图标）。

2. 此时，Groot 会将刚才创建的节点抽象接口打包生成为 XML 结构，如下所示：

   ```
   <root>
       <TreeNodesModel>
           <Action ID="MoveForwardAction">
               <input_port name="distance" default="0.5">前进的距离(米)</input_port>
           </Action>
           <Action ID="StopAction"/>
       </TreeNodesModel>
   </root>
   ```

> **架构实施说明：** 在 Groot 中创建的自定义节点仅包含逻辑接口与参数声明，属于“骨架”。该节点的真正业务逻辑（如具体的电机驱动指令或复杂的数学计算），仍需按照 ROS 2 插件规范，在 C++ 源码层完成独立开发，并在运行时供 `BehaviorTree.CPP` 动态链接调用。

## 5. 工程实现四：C++ 源码编写与插件注册 (实现自定义节点)

当在 Groot 中完成了接口抽象后，必须回到工作空间中编写对应的 C++ 源码。这里我们将展示两种典型的节点类型：**持续执行节点 (Stateful)** 与 **瞬发同步节点 (Sync)**。

### 5.1 编写 C++ 逻辑代码：持续动作 (MoveForwardAction)

“前进”是一个需要耗时的动作，在功能包的 `src` 目录下新建 `move_forward_action.cpp` 文件。

```
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include "behaviortree_cpp_v3/action_node.h"

// 继承自 BT::StatefulActionNode 适合需要持续执行并可能被中断的动作
class MoveForwardAction : public BT::StatefulActionNode
{
public:
  MoveForwardAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  // 【核心约束】此处声明的端口列表必须与 Groot XML 对应
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("distance", 0.5, "前进的距离(米)") };
  }

  // 节点首次激活时调用
  BT::NodeStatus onStart() override
  {
    if (!getInput("distance", target_distance_)) {
      throw BT::RuntimeError("缺少必须的输入参数 [distance]");
    }
    std::cout << "[MoveForwardAction] 开始执行，目标前进距离: " << target_distance_ << " 米" << std::endl;
    current_distance_ = 0.0;
    return BT::NodeStatus::RUNNING; // 返回 RUNNING 告诉树：我还在忙
  }

  // 节点处于 RUNNING 状态时持续调用
  BT::NodeStatus onRunning() override
  {
    double step = 0.1;
    current_distance_ += step;
    
    // 实际工程中，这里会向 cmd_vel 话题发布线速度指令

    if (current_distance_ >= target_distance_) {
      std::cout << "[MoveForwardAction] 执行完成，已前进: " << current_distance_ << " 米" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟计算耗时
    return BT::NodeStatus::RUNNING;
  }
  
  // 节点被中断时调用 (例如被更高优先级的节点抢占)
  void onHalted() override
  {
    std::cout << "[MoveForwardAction] 被强制中断！当前距离: " << current_distance_ << " 米" << std::endl;
  }

private:
  double target_distance_;
  double current_distance_;
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<MoveForwardAction>("MoveForwardAction");
}
```

### 5.2 编写 C++ 逻辑代码：瞬发动作 (StopAction)

“停止”是一个瞬间下发指令的动作，不需要长时间占用系统。在 `src` 目录下新建 `stop_action.cpp` 文件。

```
#include <iostream>
#include "behaviortree_cpp_v3/action_node.h"

// 继承自 BT::SyncActionNode 适合瞬间执行完毕的同步动作
class StopAction : public BT::SyncActionNode
{
public:
  StopAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {}; // 停止动作不需要端口
  }

  // 瞬发动作只需要实现 tick() 函数
  BT::NodeStatus tick() override
  {
    std::cout << "[StopAction] 紧急制动！下发零速度指令。" << std::endl;
    // 实际工程中，这里会向 cmd_vel 发布线速度和角速度均为 0 的指令
    return BT::NodeStatus::SUCCESS; // 瞬间执行完毕，立刻返回 SUCCESS
  }
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<StopAction>("StopAction");
}
```

### 5.3 编译为动态链接库 (.so)

修改功能包下的 `CMakeLists.txt`，将自定义节点分别编译为共享库：

```
# 声明行为树依赖
find_package(behaviortree_cpp_v3 REQUIRED)

# 将自定义节点编译为动态链接库 (前进节点)
add_library(move_forward_action_bt_node SHARED src/move_forward_action.cpp)
ament_target_dependencies(move_forward_action_bt_node behaviortree_cpp_v3)

# 将自定义节点编译为动态链接库 (停止节点)
add_library(stop_action_bt_node SHARED src/stop_action.cpp)
ament_target_dependencies(stop_action_bt_node behaviortree_cpp_v3)

# 安装动态链接库到执行环境
install(TARGETS
  move_forward_action_bt_node
  stop_action_bt_node
  DESTINATION lib
)
```

编译工作空间：`colcon build --packages-select sam_bot_description`

### 5.4 在 Nav2 参数中挂载自定义插件库

底层动态库生成后，需要在 `nav2_params.yaml` 文件中通知 `bt_navigator` 服务器在启动时加载该库。以下是一个完整的、可供复制使用的 `bt_navigator` 配置段落：

```
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20

    # ========================================================
    # 插件加载列表：必须在此处列出所有行为树中用到的节点插件动态库
    # 包含 Nav2 官方插件以及你自己编写的自定义插件
    # ========================================================
    plugin_lib_names:
      # --- Nav2 官方基础 Action 节点 ---
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_clear_costmap_service_bt_node
      
      # --- Nav2 官方控制/装饰/条件节点 ---
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_replan_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node

      # --- 你的自定义插件加载区 ---
      # 注意：这里的名称必须与 CMakeLists.txt 中 add_library 生成的目标名称一致
      - move_forward_action_bt_node 
      - stop_action_bt_node
```

至此，从 Groot 可视化界面设计，到 C++ 源码逻辑实现，再到 Nav2 系统动态加载运行，多个自定义节点的链路已彻底打通。你现在可以在自己编写的 XML 行为树文件中自由组合使用 `<MoveForwardAction distance="1.5"/>` 和 `<StopAction/>` 节点了。