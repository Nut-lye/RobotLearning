#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_share_file(package_share, subdir, value):
    if os.path.isabs(value):
        return value
    return os.path.join(package_share, subdir, value)


def _launch_setup(context, *args, **kwargs):
    teaching_share = get_package_share_directory('module_c_nav_teaching')
    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

    world_arg = LaunchConfiguration('world').perform(context)
    world_path = _resolve_share_file(teaching_share, 'worlds', world_arg)

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    model_paths = [
        os.path.join(teaching_share, 'models'),
        os.path.join(turtlebot3_share, 'models'),
        '/usr/share/gazebo-11/models',
    ]
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH')
    if existing_model_path:
        model_paths.append(existing_model_path)

    turtlebot3_launch_dir = os.path.join(turtlebot3_share, 'launch')

    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_paths)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('SVGA_VGPU10', '0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items()
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='02_open_obstacles.world',
            description='World filename under worlds/ or an absolute world path.'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial TurtleBot3 x position in Gazebo.'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial TurtleBot3 y position in Gazebo.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
