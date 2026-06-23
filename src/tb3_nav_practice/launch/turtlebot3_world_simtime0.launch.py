#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

    practice_share = get_package_share_directory('tb3_nav_practice')
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch'
    )
    turtlebot3_models = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models'
    )
    practice_models = os.path.join(practice_share, 'models')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_builtin_models = '/usr/share/gazebo-11/models'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    world = LaunchConfiguration('world')

    model_paths = [
        practice_models,
        turtlebot3_models,
        gazebo_builtin_models,
    ]
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH')
    if existing_model_path:
        model_paths.append(existing_model_path)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                practice_share,
                'worlds',
                'turtlebot3_world_simtime0.world'
            ),
            description='Gazebo world file loaded from this teaching package.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='-2.0',
            description='Initial robot x position in Gazebo.'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-0.5',
            description='Initial robot y position in Gazebo.'
        ),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_paths)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('SVGA_VGPU10', '0'),
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
    ])
