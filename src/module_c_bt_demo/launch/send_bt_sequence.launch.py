#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('module_c_bt_demo')
    wait_tree = share + '/behavior_trees/module_c_demo_goal_wait.xml'
    goal_tree = share + '/behavior_trees/module_c_demo_replanning.xml'

    return LaunchDescription([
        DeclareLaunchArgument('waypoint1', default_value='0.0,-2.0,0.0'),
        DeclareLaunchArgument('waypoint2', default_value='3.0,-2.0,0.0'),
        DeclareLaunchArgument('speed1', default_value='0.18'),
        DeclareLaunchArgument('speed2', default_value='0.35'),
        DeclareLaunchArgument('wait_tree', default_value=wait_tree),
        DeclareLaunchArgument('goal_tree', default_value=goal_tree),
        Node(
            package='module_c_bt_demo',
            executable='send_bt_sequence',
            name='module_c_bt_sequence_sender',
            output='screen',
            arguments=[
                '--waypoint1', LaunchConfiguration('waypoint1'),
                '--waypoint2', LaunchConfiguration('waypoint2'),
                '--speed1', LaunchConfiguration('speed1'),
                '--speed2', LaunchConfiguration('speed2'),
                '--wait-tree', LaunchConfiguration('wait_tree'),
                '--goal-tree', LaunchConfiguration('goal_tree'),
            ],
        ),
    ])
