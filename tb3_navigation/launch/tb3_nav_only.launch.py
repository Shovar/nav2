#!/usr/bin/env python3
"""Launch only localization + pure pursuit (Gazebo must already be running)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')

def generate_launch_description():
    tb3_nav_dir = get_package_share_directory('tb3_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_map_path = os.path.join(tb3_nav_dir, 'map', 'turtlebot3_house_cust.yaml')
    map_yaml = LaunchConfiguration('map', default=default_map_path)
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        tb3_nav_dir, 'param', 'waffle_pure_pursuit.yaml'))

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    pure_pursuit_navigator = Node(
        package='tb3_navigation',
        executable='pure_pursuit_navigator',
        name='pure_pursuit_navigator',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )

    rviz_config = os.path.join(tb3_nav_dir, 'rviz', 'tb3_navigation2.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        SetParameter('use_sim_time', True),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map_path),
        DeclareLaunchArgument('params_file', default_value=params_file),
        localization,
        pure_pursuit_navigator,
        rviz,
    ])
