import argparse
import math
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
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


def parse_waypoint(value):
    parts = [float(item.strip()) for item in value.split(',')]
    if len(parts) == 2:
        parts.append(0.0)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            'Waypoint format must be x,y or x,y,yaw.'
        )
    return parts


class BtWaypointsSender(Node):
    def __init__(self):
        super().__init__('module_c_bt_waypoints_sender')
        self._client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
        )
        self._param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters',
        )
        self._segment_speeds = []
        self._total_poses = 0
        self._active_segment = -1

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

    def make_double_parameter(self, name, value):
        return Parameter(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            ),
        )

    def set_segment_speed(self, speed):
        request = SetParameters.Request()
        request.parameters = [
            self.make_double_parameter('FollowPath.max_vel_x', speed),
            self.make_double_parameter('FollowPath.max_speed_xy', speed),
        ]

        if not self._param_client.service_is_ready():
            self._param_client.wait_for_service(timeout_sec=2.0)

        future = self._param_client.call_async(request)
        future.add_done_callback(
            lambda done: self.get_logger().info(
                f'Segment speed set to {speed:.2f} m/s'
            )
        )

    def normalize_speeds(self, speeds, count):
        if not speeds:
            speeds = [0.20, 0.35]
        while len(speeds) < count:
            speeds.append(speeds[-1])
        return speeds[:count]

    def send_goal(self, waypoints, speeds, behavior_tree):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [
            self.make_pose(x, y, yaw)
            for x, y, yaw in waypoints
        ]
        goal_msg.behavior_tree = behavior_tree
        self._segment_speeds = self.normalize_speeds(speeds, len(goal_msg.poses))
        self._total_poses = len(goal_msg.poses)

        self.get_logger().info(
            'Waiting for /navigate_through_poses action server...'
        )
        self._client.wait_for_server()
        self._active_segment = 0
        self.set_segment_speed(self._segment_speeds[0])
        self.get_logger().info(
            f'Sending {len(goal_msg.poses)} waypoint BT goal(s).'
        )
        self.get_logger().info(f'Segment speeds: {self._segment_speeds}')
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
        remaining = feedback.number_of_poses_remaining
        segment_index = min(
            max(self._total_poses - remaining, 0),
            len(self._segment_speeds) - 1,
        )
        if segment_index > self._active_segment:
            self._active_segment = segment_index
            self.set_segment_speed(self._segment_speeds[segment_index])

        self.get_logger().info(
            'distance_remaining='
            f'{feedback.distance_remaining:.2f}, '
            'poses_remaining='
            f'{feedback.number_of_poses_remaining}, '
            'recoveries='
            f'{feedback.number_of_recoveries}'
        )


def parse_args(argv):
    default_tree = (
        get_package_share_directory('module_c_bt_demo')
        + '/behavior_trees/module_c_demo_waypoints.xml'
    )

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--waypoint',
        action='append',
        type=parse_waypoint,
        default=None,
        help='Waypoint in x,y or x,y,yaw format. Repeat for multiple points.',
    )
    parser.add_argument(
        '--speed',
        action='append',
        type=float,
        default=None,
        help='Segment speed in m/s. Repeat once for each waypoint.',
    )
    parser.add_argument('--behavior-tree', default=default_tree)
    parsed, _ = parser.parse_known_args(argv)
    if not parsed.waypoint:
        parsed.waypoint = [
            [1.5, 1.8, 0.0],
            [3.0, 2.0, 0.0],
        ]
    if not parsed.speed:
        parsed.speed = [0.20, 0.35]
    return parsed


def main(args=None):
    parsed = parse_args(sys.argv[1:] if args is None else args)
    rclpy.init()
    node = BtWaypointsSender()
    try:
        exit_code = node.send_goal(
            parsed.waypoint,
            parsed.speed,
            parsed.behavior_tree,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
