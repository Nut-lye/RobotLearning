#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bt_xml = (
        get_package_share_directory('module_c_bt_demo')
        + '/behavior_trees/module_c_demo_waypoints.xml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('waypoint1', default_value='1.5,1.8,0.0'),
        DeclareLaunchArgument('waypoint2', default_value='3.0,2.0,0.0'),
        DeclareLaunchArgument('speed1', default_value='0.20'),
        DeclareLaunchArgument('speed2', default_value='0.35'),
        DeclareLaunchArgument('behavior_tree', default_value=bt_xml),
        Node(
            package='module_c_bt_demo',
            executable='send_bt_waypoints',
            name='module_c_bt_waypoints_sender',
            output='screen',
            arguments=[
                '--waypoint', LaunchConfiguration('waypoint1'),
                '--waypoint', LaunchConfiguration('waypoint2'),
                '--speed', LaunchConfiguration('speed1'),
                '--speed', LaunchConfiguration('speed2'),
                '--behavior-tree', LaunchConfiguration('behavior_tree'),
            ],
        ),
    ])
