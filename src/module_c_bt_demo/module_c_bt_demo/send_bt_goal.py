import argparse
import math
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw),
    }


class BtGoalSender(Node):
    def __init__(self):
        super().__init__('module_c_bt_goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw, behavior_tree):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        q = yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = q['x']
        goal_msg.pose.pose.orientation.y = q['y']
        goal_msg.pose.pose.orientation.z = q['z']
        goal_msg.pose.pose.orientation.w = q['w']
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        self._client.wait_for_server()
        self.get_logger().info(
            f'Sending BT goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )
        self.get_logger().info(f'Behavior tree: {behavior_tree}')

        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return 1

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f'Goal finished with status: {result.status}')
        return 0

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'distance_remaining='
            f'{feedback.distance_remaining:.2f}, '
            'recoveries='
            f'{feedback.number_of_recoveries}'
        )


def parse_args(argv):
    default_tree = (
        get_package_share_directory('module_c_bt_demo')
        + '/behavior_trees/module_c_demo_replanning.xml'
    )

    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=3.0)
    parser.add_argument('--y', type=float, default=2.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--behavior-tree', default=default_tree)
    parsed, _ = parser.parse_known_args(argv)
    return parsed


def main(args=None):
    parsed = parse_args(sys.argv[1:] if args is None else args)
    rclpy.init()
    node = BtGoalSender()
    try:
        exit_code = node.send_goal(
            parsed.x,
            parsed.y,
            parsed.yaw,
            parsed.behavior_tree,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
