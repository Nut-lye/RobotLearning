#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

    practice_share = get_package_share_directory('tb3_nav_practice')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )
    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items()
    )

    rviz_cmd = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(practice_share, 'maps', 'map.yaml'),
            description='Local map yaml used by AMCL and Nav2.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                practice_share,
                'config',
                'nav2_params.yaml'
            ),
            description='Local Nav2 parameter file for module C screenshots.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the TurtleBot3 navigation layout.'
        ),
        nav2_bringup_cmd,
        rviz_cmd,
    ])
