#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bt_xml = (
        get_package_share_directory('module_c_bt_demo')
        + '/behavior_trees/module_c_demo_replanning.xml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='3.0'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('behavior_tree', default_value=bt_xml),
        Node(
            package='module_c_bt_demo',
            executable='send_bt_goal',
            name='module_c_bt_goal_sender',
            output='screen',
            arguments=[
                '--x', LaunchConfiguration('x'),
                '--y', LaunchConfiguration('y'),
                '--yaw', LaunchConfiguration('yaw'),
                '--behavior-tree', LaunchConfiguration('behavior_tree'),
            ],
        ),
    ])
