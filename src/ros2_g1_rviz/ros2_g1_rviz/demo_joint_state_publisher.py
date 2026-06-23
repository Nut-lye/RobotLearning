"""Publish a simple looping G1 JointState animation for RViz2."""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ros2_g1_rviz.g1_joints import G1_29DOF_JOINT_NAMES
from ros2_g1_rviz.g1_joints import LEG_JOINTS


class DemoJointStatePublisher(Node):
    """Generate a fake, repeatable motion so the RViz2 pipeline can be tested."""

    def __init__(self) -> None:
        super().__init__("g1_demo_joint_state_publisher")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("amplitude", 0.45)
        self.declare_parameter("motion", "wave")

        rate = max(1.0, float(self.get_parameter("rate").value))
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.motion = str(self.get_parameter("motion").value)
        self.start_time = self.get_clock().now()
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"Publishing demo G1 joint_states: motion={self.motion}, rate={rate:.1f} Hz"
        )

    def _publish(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(G1_29DOF_JOINT_NAMES)
        msg.position = self._positions(elapsed)
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = []
        self.publisher.publish(msg)

    def _positions(self, t: float) -> list[float]:
        positions = {name: 0.0 for name in G1_29DOF_JOINT_NAMES}

        if self.motion == "walk":
            self._fill_walk_pose(positions, t)
        elif self.motion == "idle":
            self._fill_idle_pose(positions, t)
        else:
            self._fill_wave_pose(positions, t)

        return [positions[name] for name in G1_29DOF_JOINT_NAMES]

    def _fill_idle_pose(self, positions: dict[str, float], t: float) -> None:
        sway = 0.08 * math.sin(t * 1.2)
        positions["waist_yaw_joint"] = sway
        positions["left_shoulder_roll_joint"] = 0.2 + sway
        positions["right_shoulder_roll_joint"] = -0.2 + sway

    def _fill_wave_pose(self, positions: dict[str, float], t: float) -> None:
        amp = self.amplitude
        wave = math.sin(t * 2.5)
        positions["right_shoulder_pitch_joint"] = -0.8
        positions["right_shoulder_roll_joint"] = -0.75
        positions["right_elbow_joint"] = 1.0
        positions["right_wrist_yaw_joint"] = amp * wave
        positions["right_wrist_roll_joint"] = 0.5 * amp * math.sin(t * 5.0)
        positions["left_shoulder_pitch_joint"] = 0.25 * math.sin(t)
        positions["left_elbow_joint"] = 0.25
        positions["waist_yaw_joint"] = 0.15 * math.sin(t * 0.8)

    def _fill_walk_pose(self, positions: dict[str, float], t: float) -> None:
        amp = self.amplitude
        phase = math.sin(t * 2.0)
        opposite = math.sin(t * 2.0 + math.pi)

        self._fill_leg_swing(positions, LEG_JOINTS[:6], amp, phase)
        self._fill_leg_swing(positions, LEG_JOINTS[6:], amp, opposite)
        positions["waist_yaw_joint"] = 0.18 * phase
        positions["left_shoulder_pitch_joint"] = -0.5 * phase
        positions["right_shoulder_pitch_joint"] = -0.5 * opposite

    @staticmethod
    def _fill_leg_swing(
        positions: dict[str, float],
        joints: Sequence[str],
        amp: float,
        phase: float,
    ) -> None:
        hip_pitch, hip_roll, _hip_yaw, knee, ankle_pitch, ankle_roll = joints
        positions[hip_pitch] = 0.45 * amp * phase
        positions[hip_roll] = 0.08 * phase
        positions[knee] = 0.55 + 0.35 * max(0.0, phase)
        positions[ankle_pitch] = -0.25 - 0.25 * max(0.0, phase)
        positions[ankle_roll] = -positions[hip_roll]


def main() -> None:
    rclpy.init()
    node = DemoJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
