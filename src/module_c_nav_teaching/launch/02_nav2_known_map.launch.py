#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_share_file(package_share, subdir, value):
    if os.path.isabs(value):
        return value
    return os.path.join(package_share, subdir, value)


def _launch_setup(context, *args, **kwargs):
    teaching_share = get_package_share_directory('module_c_nav_teaching')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )
    rviz_config = os.path.join(
        teaching_share,
        'rviz',
        'module_c_nav2_view.rviz'
    )

    map_arg = LaunchConfiguration('map').perform(context)
    params_arg = LaunchConfiguration('params_file').perform(context)

    map_file = _resolve_share_file(teaching_share, 'maps', map_arg)
    params_file = _resolve_share_file(teaching_share, 'config', params_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'slam': 'False',
            }.items()
        ),
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='02_open_obstacles.yaml',
            description='Map yaml under maps/ or an absolute map yaml path.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='nav2_params_module_c.yaml',
            description='Nav2 parameter yaml under config/ or an absolute path.'
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
            description='Start RViz.'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
