"""Read Unitree SDK2 G1 LowState and publish ROS JointState for RViz2."""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ros2_g1_rviz.g1_joints import G1_29DOF_JOINT_NAMES


class Sdk2LowStateToJointState(Node):
    """Mirror the real G1 joint angles into ROS without controlling the robot."""

    def __init__(self) -> None:
        super().__init__("g1_sdk2_lowstate_to_joint_state")
        self.declare_parameter("iface", "")
        self.declare_parameter("domain_id", 0)
        self.declare_parameter("lowstate_topic", "rt/lowstate")
        self.declare_parameter("publish_topic", "joint_states")
        self.declare_parameter("rate", 60.0)
        self.declare_parameter("joint_count", len(G1_29DOF_JOINT_NAMES))

        self.iface = str(self.get_parameter("iface").value).strip()
        self.domain_id = int(self.get_parameter("domain_id").value)
        self.lowstate_topic = str(self.get_parameter("lowstate_topic").value)
        publish_topic = str(self.get_parameter("publish_topic").value)
        rate = max(1.0, float(self.get_parameter("rate").value))
        self.joint_count = int(self.get_parameter("joint_count").value)

        if not self.iface:
            raise SystemExit("iface 不能为空，例如：iface:=eth0 或 iface:=enp3s0")

        if self.joint_count < 1 or self.joint_count > len(G1_29DOF_JOINT_NAMES):
            raise ValueError(f"joint_count must be 1..{len(G1_29DOF_JOINT_NAMES)}")

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        ChannelFactoryInitialize(self.domain_id, self.iface)

        self.lock = threading.Lock()
        self.latest_position: list[float] | None = None
        self.latest_velocity: list[float] | None = None
        self.frame_count = 0

        self.publisher = self.create_publisher(JointState, publish_topic, 10)
        self.subscriber = ChannelSubscriber(self.lowstate_topic, LowState_)
        self.subscriber.Init(self._on_lowstate, 10)
        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"Reading SDK2 LowState from {self.lowstate_topic} on iface={self.iface}, "
            f"publishing {publish_topic}"
        )

    def _on_lowstate(self, msg) -> None:
        count = min(self.joint_count, len(msg.motor_state), len(G1_29DOF_JOINT_NAMES))
        position = [float(msg.motor_state[i].q) for i in range(count)]
        velocity = [float(msg.motor_state[i].dq) for i in range(count)]
        with self.lock:
            self.latest_position = position
            self.latest_velocity = velocity
            self.frame_count += 1

    def _publish(self) -> None:
        with self.lock:
            if self.latest_position is None:
                return
            position = list(self.latest_position)
            velocity = list(self.latest_velocity or [])

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = G1_29DOF_JOINT_NAMES[: len(position)]
        msg.position = position
        msg.velocity = velocity
        msg.effort = []
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = Sdk2LowStateToJointState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
