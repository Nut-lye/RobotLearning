# 自定义行为树插件：扩展 Nav2 的任务动作

> 阅读位置：第四阶段进阶内容。
>
> 前置建议：先读 `理论知识/Nav2通信与生命周期.md`、`理论知识/Nav2行为树与导航服务器.md`、`理论知识/行为树黑板机制.md`、`进阶专题/Groot行为树可视化编辑.md`，并理解 ROS 2 Action。
>
> 本章目标：编写继承 `nav2_behavior_tree::BtActionNode<T>` 的 C++ 插件，让行为树可以调用新的 ROS 2 动作。

## 本章在导航系统中的位置

普通参数调优不需要写 BT 插件。只有当现有行为树节点不能表达题目动作时，才需要自定义插件。

本章对应链路：

```text
自定义 ROS 2 Action
  -> BtActionNode 插件
  -> pluginlib 注册
  -> nav2_params.yaml 挂载
  -> 行为树 XML 调用
```

什么时候用：

- 题目要求新增等待、巡检、特殊动作。
- 需要在导航流程中插入自定义逻辑。
- 需要把某个 ROS 2 Action 包装成行为树节点。

---

# 编写 Nav2 行为树动作插件 (基于 ROS 2 动作服务器)

## 1. 核心理论：行为树与 ROS 2 动作服务器的桥接

在基础的 `BehaviorTree.CPP` 框架中，动作节点（Action Node）通常用于直接执行本地的 C++ 函数。但在分布式的 Nav2 导航架构中，真正的复杂任务（如全局路径规划、局部轨迹控制）运行在独立的 ROS 2 生命周期节点中。

因此，Nav2 的行为树节点通常扮演**客户端 (Action Client)** 的角色。为了简化开发，Nav2 官方提供了一个专用的底层模板基类：`nav2_behavior_tree::BtActionNode<T>`。 该基类封装了极其复杂的 ROS 2 动作网络通信底层逻辑，开发者只需继承该基类，即可快速创建一个能够向远程服务器发送目标指令、并实时监控执行状态的自定义行为树插件。

本指南将以 Nav2 中最基础的 `Wait`（等待）节点为例，演示如何从零构建一个标准的 BT 动作插件。

## 2. 接口规范：`BtActionNode` 基类方法解析

继承 `BtActionNode` 后，开发者可以重写一系列虚拟方法来控制节点的生命周期。根据工程需求，以下是各方法的规范定义：

| 方法声明                     | 是否必须重写 | 工程功能描述                                                 |
| ---------------------------- | ------------ | ------------------------------------------------------------ |
| **`Constructor` (构造函数)** | **是**       | 必须初始化并向基类传递：对应的 XML 标签名称、ROS 2 动作服务器的话题名称。 |
| **`providedPorts()`**        | **是**       | 定义该节点在行为树 XML 中支持的输入/输出端口（如参数传递或黑板变量读写）。 |
| **`on_tick()`**              | 否           | 每次该行为树节点被父节点执行 (Tick) 时首先调用。通常用于在向动作服务器发送请求前，从黑板动态提取最新参数并更新目标变量 (`goal_`)。 |
| **`on_wait_for_result()`**   | 否           | 在向动作服务器发送请求后、等待结果返回的阻塞期间高频调用。常用于检查系统是否发生超时，或监听是否有更高优先级的抢占请求。 |
| **`on_success()`**           | 否           | 当 ROS 2 动作服务器成功完成任务时调用。返回该节点向整棵树报告的最终状态（默认返回 `SUCCESS`）。 |
| **`on_aborted()`**           | 否           | 当动作服务器返回“任务中止”时调用（默认返回 `FAILURE`）。     |
| **`on_cancelled()`**         | 否           | 当动作服务器返回“任务被取消”时调用（默认返回 `SUCCESS`）。   |

## 3. 工程实现一：编写插件 C++ 源码

我们将编写一个 `WaitAction` 插件。它的核心职责是：从行为树 XML 中读取指定的等待时间，随后将其作为目标发送给名为 `wait` 的独立 ROS 2 动作服务器。

在功能包的 `src` 目录下新建 `wait_action.cpp` 文件，写入以下规范代码：

```
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

namespace nav2_behavior_tree
{

// 继承 BtActionNode，并严格绑定由 nav2_msgs 提供的 Wait 动作接口类型
class WaitAction : public BtActionNode<nav2_msgs::action::Wait>
{
public:
  // 1. 构造函数
  WaitAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf)
  {
    // 在初始化阶段读取 XML 中硬编码的等待时间参数
    int duration;
    getInput("wait_duration", duration);
    if (duration <= 0) {
      RCLCPP_WARN(
        node_->get_logger(), 
        "等待时间不可为负数或零 (%i)，系统将自动转为绝对值。", duration);
      duration *= -1;
    }
    // 将提取到的参数写入 goal_ 结构体。该结构体将由基类自动发送给远程动作服务器
    goal_.time.sec = duration;
  }

  // 2. 声明端口
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        // 声明一个名为 "wait_duration" 的输入端口，默认值为 1
        BT::InputPort<int>("wait_duration", 1, "指定机器人原地等待的时间(秒)")
      });
  }

  // 3. 勾选时触发的行为
  void on_tick() override
  {
    // 本示例中仅调用父类的状态记录方法。若目标时间为动态黑板变量，可在此处重新读取并覆盖 goal_
    increment_recovery_count();
  }
};

}  // namespace nav2_behavior_tree
```

## 4. 工程实现二：注册节点生成器与导出

由于继承自 `BtActionNode` 的子类构造函数包含特殊的 ROS 2 网络参数（不仅仅是标准的 BT 配置），我们无法使用常规的自动注册宏，必须编写一个自定义的 **NodeBuilder (节点构建器)**。

在 `wait_action.cpp` 文件的末尾追加以下注册代码：

```
#include "behaviortree_cpp_v3/bt_factory.h"

// 宏定义：向行为树工厂导出该动态链接库
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      // 返回一个实例指针。此处的 "wait" 对应的是 ROS 2 Action Server 的实际话题名称
      return std::make_unique<nav2_behavior_tree::WaitAction>(name, "wait", config);
    };

  // 向系统注册该生成器。XML 文件中的 <Wait /> 标签将被映射到此代码逻辑
  factory.registerBuilder<nav2_behavior_tree::WaitAction>("Wait", builder);
}
```

## 5. 工程实现三：编译与系统级挂载

### 5.1 编译为共享库 (CMakeLists.txt)

在 `CMakeLists.txt` 中，将该源码文件编译为动态链接库（.so 文件）：

```
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)

add_library(nav2_wait_action_bt_node SHARED src/wait_action.cpp)
ament_target_dependencies(nav2_wait_action_bt_node 
  behaviortree_cpp_v3 
  nav2_behavior_tree 
  nav2_msgs
)

install(TARGETS
  nav2_wait_action_bt_node
  DESTINATION lib
)
```

使用 `colcon build` 重新编译整个工作空间。

### 5.2 挂载到 Nav2 配置中 (`nav2_params.yaml`)

这是极为关键的一步：底层动态库生成后，必须显式通知 `bt_navigator` 节点在启动时将其加载到系统内存中。

修改核心参数文件 `nav2_params.yaml`，填入**完整无省略**的组件加载列表：

```
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    # 指定默认执行的行为树文件
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    
    # 完整列出 Nav2 所需的基础插件动态库，确保系统核心功能可用
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_clear_costmap_service_bt_node
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
      # --- 下方挂载我们刚刚编译的 Wait 插件库 ---
      - nav2_wait_action_bt_node
```

### 5.3 在行为树 XML 中调用

完成所有工程挂载后，您即可在任何行为树的 XML 逻辑中直接调用该节点。如下提供了一份**完整的默认导航行为树文件**，我们通过硬编码传递参数，在发生故障触发 `RecoveryFallback` 时，调用了我们自定义的 `Wait` 节点：

```
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <!-- 最外层包裹 RecoveryNode，当导航彻底失败时进行至多 6 次的恢复尝试 -->
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      
      <!-- 核心导航管道：边规划边执行 -->
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      
      <!-- 当导航管道发生故障 (FAILURE) 时，触发后备应对措施 -->
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <SequenceStar name="RecoveryActions">
          <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          <Spin spin_dist="1.57"/>
          
          <!-- 调用我们编写的自定义 ROS 2 动作客户端节点：原地等待 5 秒 -->
          <Wait wait_duration="5"/>
          
        </SequenceStar>
      </ReactiveFallback>
      
    </RecoveryNode>
  </BehaviorTree>
</root>
```



----

## 进阶拓展：实现“前进”运动控制 (DriveOnHeading)

在掌握了基础的 `Wait` 插件后，如果我们想要让机器人执行真实的物理移动（如“直线前进”），我们需要调用 Nav2 的运动恢复服务器：`drive_on_heading`。

与“等待”只需传递时间不同，“前进”需要传递**距离 (distance)** 和 **速度 (speed)** 两个参数。以下展示如何基于相同的模板，实现一个 `DriveOnHeadingAction`（沿当前朝向直线行驶）节点：

### 编写“前进”动作的 C++ 源码

在 `src` 下新建 `drive_on_heading_action.cpp`：

```
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"

namespace nav2_behavior_tree
{

// 绑定 DriveOnHeading (直线行驶) 动作消息接口
class DriveOnHeadingAction : public BtActionNode<nav2_msgs::action::DriveOnHeading>
{
public:
  DriveOnHeadingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::DriveOnHeading>(xml_tag_name, action_name, conf)
  {
    // 不在构造函数中读取参数，因为距离和速度可能随环境动态变化，将其推迟到 on_tick 中读取更合理
  }

  // 声明输入端口：包含距离与速度
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("dist", 0.15, "前进的距离(米)"),
        BT::InputPort<double>("speed", 0.1, "直线行驶的速度(m/s)"),
        BT::InputPort<double>("time_allowance", 10.0, "允许的最大耗时(秒)")
      });
  }

  // 每次节点被触发执行时调用
  void on_tick() override
  {
    // 1. 从 XML 端口或黑板中提取最新参数
    double dist, speed, time_allowance;
    getInput("dist", dist);
    getInput("speed", speed);
    getInput("time_allowance", time_allowance);

    // 2. 将参数组装成目标 (goal_) 发送给远端的 ROS 2 控制服务器
    goal_.target.x = dist; 
    goal_.target.y = 0.0;
    goal_.target.z = 0.0;
    goal_.speed = speed;
    goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
    
    increment_recovery_count();
  }
};

}  // namespace nav2_behavior_tree

// 注册工厂
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      // 对应的 ROS 2 Action Server 话题名称为 "drive_on_heading"
      return std::make_unique<nav2_behavior_tree::DriveOnHeadingAction>(name, "drive_on_heading", config);
    };

  factory.registerBuilder<nav2_behavior_tree::DriveOnHeadingAction>("DriveOnHeading", builder);
}
```

按照此步骤将其添加到 `CMakeLists.txt` 编译并在 `nav2_params.yaml` 中挂载后，您就可以在行为树 XML 中随时调用如下节点，强制机器人主动直线前进了：

```
<!-- 强制机器人以 0.2m/s 的速度，笔直向前行驶 1.5 米 -->
<DriveOnHeading dist="1.5" speed="0.2" time_allowance="15.0"/>
```