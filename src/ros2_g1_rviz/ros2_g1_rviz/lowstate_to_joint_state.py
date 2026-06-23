"""Bridge Unitree G1 LowState motor positions to ROS JointState."""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ros2_g1_rviz.g1_joints import G1_29DOF_JOINT_NAMES


class LowStateToJointState(Node):
    """Convert Unitree HG LowState messages into RViz-friendly JointState data."""

    def __init__(self, low_state_msg_type: type[Any]) -> None:
        super().__init__("g1_lowstate_to_joint_state")
        self.declare_parameter("lowstate_topic", "lowstate")
        self.declare_parameter("publish_topic", "joint_states")
        self.declare_parameter("publish_velocity", True)
        self.declare_parameter("joint_count", len(G1_29DOF_JOINT_NAMES))

        lowstate_topic = str(self.get_parameter("lowstate_topic").value)
        publish_topic = str(self.get_parameter("publish_topic").value)
        self.publish_velocity = bool(self.get_parameter("publish_velocity").value)
        self.joint_count = int(self.get_parameter("joint_count").value)

        if self.joint_count < 1 or self.joint_count > len(G1_29DOF_JOINT_NAMES):
            raise ValueError(f"joint_count must be 1..{len(G1_29DOF_JOINT_NAMES)}")

        self.publisher = self.create_publisher(JointState, publish_topic, 10)
        self.create_subscription(low_state_msg_type, lowstate_topic, self._on_lowstate, 10)
        self.get_logger().info(
            f"Bridging {lowstate_topic} -> {publish_topic} with {self.joint_count} G1 joints"
        )

    def _on_lowstate(self, lowstate: Any) -> None:
        motor_state = getattr(lowstate, "motor_state", None)
        if motor_state is None:
            self.get_logger().warning("LowState message has no motor_state field")
            return

        count = min(self.joint_count, len(motor_state), len(G1_29DOF_JOINT_NAMES))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = G1_29DOF_JOINT_NAMES[:count]
        msg.position = [float(getattr(motor_state[i], "q", 0.0)) for i in range(count)]

        if self.publish_velocity:
            msg.velocity = [float(getattr(motor_state[i], "dq", 0.0)) for i in range(count)]

        self.publisher.publish(msg)


def _load_unitree_lowstate_type() -> type[Any]:
    try:
        from unitree_hg.msg import LowState
    except ImportError as exc:
        raise SystemExit(
            "无法导入 unitree_hg.msg.LowState。请先 source Unitree ROS2 工作空间，"
            "或只运行 demo 模式：ros2 launch ros2_g1_rviz display.launch.py use_lowstate:=false"
        ) from exc

    return LowState


def main() -> None:
    rclpy.init()
    node = LowStateToJointState(_load_unitree_lowstate_type())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
