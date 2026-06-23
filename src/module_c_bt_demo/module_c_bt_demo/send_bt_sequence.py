import argparse
import math
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return (
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
    )


def parse_pose(value):
    parts = [float(item.strip()) for item in value.split(',')]
    if len(parts) == 2:
        parts.append(0.0)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            'Pose format must be x,y or x,y,yaw.'
        )
    return parts


class BtSequenceSender(Node):
    def __init__(self):
        super().__init__('module_c_bt_sequence_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters',
        )

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def set_speed(self, speed):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name='FollowPath.max_vel_x',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(speed),
                ),
            ),
            Parameter(
                name='FollowPath.max_speed_xy',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(speed),
                ),
            ),
        ]
        self._param_client.wait_for_service(timeout_sec=2.0)
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Segment speed set to {speed:.2f} m/s')

    def send_one_goal(self, pose_values, speed, behavior_tree, label):
        x, y, yaw = pose_values
        self.set_speed(speed)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(x, y, yaw)
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info(
            f'Sending {label}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )
        self.get_logger().info(f'Behavior tree: {behavior_tree}')
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'{label} rejected.')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f'{label} finished with status: {result.status}')
        return 0

    def send_sequence(self, pose1, pose2, speed1, speed2, wait_tree, goal_tree):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        self._client.wait_for_server()

        first = self.send_one_goal(pose1, speed1, wait_tree, 'waypoint1')
        if first != 0:
            return first
        return self.send_one_goal(pose2, speed2, goal_tree, 'waypoint2')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'distance_remaining='
            f'{feedback.distance_remaining:.2f}, '
            'recoveries='
            f'{feedback.number_of_recoveries}'
        )


def parse_args(argv):
    share = get_package_share_directory('module_c_bt_demo')
    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoint1', type=parse_pose, default='0.0,-2.0,0.0')
    parser.add_argument('--waypoint2', type=parse_pose, default='3.0,-2.0,0.0')
    parser.add_argument('--speed1', type=float, default=0.18)
    parser.add_argument('--speed2', type=float, default=0.35)
    parser.add_argument(
        '--wait-tree',
        default=share + '/behavior_trees/module_c_demo_goal_wait.xml',
    )
    parser.add_argument(
        '--goal-tree',
        default=share + '/behavior_trees/module_c_demo_replanning.xml',
    )
    parsed, _ = parser.parse_known_args(argv)
    return parsed


def main(args=None):
    parsed = parse_args(sys.argv[1:] if args is None else args)
    rclpy.init()
    node = BtSequenceSender()
    try:
        exit_code = node.send_sequence(
            parsed.waypoint1,
            parsed.waypoint2,
            parsed.speed1,
            parsed.speed2,
            parsed.wait_tree,
            parsed.goal_tree,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
