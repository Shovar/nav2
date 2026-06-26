#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')

def generate_launch_description():
    tb3_nav_dir = get_package_share_directory('tb3_navigation')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    default_map = os.path.join(tb3_nav_dir, 'map', 'turtlebot3_house_cust.yaml')
    if not os.path.exists(default_map):
        default_map = os.path.join(tb3_nav2_dir, 'map', 'map.yaml')
    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        tb3_nav_dir, 'param', 'waffle_pure_pursuit.yaml'))
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    autostart = LaunchConfiguration('autostart', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')
    declare_map = DeclareLaunchArgument(
        'map', default_value=map_yaml,
        description='Full path to map yaml file')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=params_file,
        description='Full path to params file')
    declare_x = DeclareLaunchArgument(
        'x_pose', default_value=x_pose,
        description='Initial x position')
    declare_y = DeclareLaunchArgument(
        'y_pose', default_value=y_pose,
        description='Initial y position')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup nav2')

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models'))

    # Detectar qué mundo usar según el mapa
    _map = default_map
    if 'house' in _map or 'mi_mapa' in _map:
        _world = 'turtlebot3_house.launch.py'
    else:
        _world = 'turtlebot3_world.launch.py'

    # Gazebo: world + robot spawn + state publisher
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', _world)),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    # Nav2 localization: map_server + AMCL (lifecycle managed)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    # Pure Pursuit navigator (action server + local planner)
    pure_pursuit_navigator = Node(
        package='tb3_navigation',
        executable='pure_pursuit_navigator',
        name='pure_pursuit_navigator',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )

    # RViz with nav2 config
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
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_x,
        declare_y,
        declare_autostart,
        set_gz_resource_path,
        gazebo,
        localization,
        pure_pursuit_navigator,
        rviz,
    ])
