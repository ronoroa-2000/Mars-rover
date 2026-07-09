#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    include_base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rover_description'),
                'launch',
                'my_robot.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim': 'true',
            'use_sim_time': use_sim_time,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
        }.items(),
    )

    weighted_odom_node = Node(
        package='rover_controllers',
        executable='weighted_odometry.py',
        name='weighted_odometry_sim',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/odom', '/rover_controller/odom_median')],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_sim_median',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_nav'), 'config', 'ekf_config_sim_median.yaml'
            ]),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Spawn X'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Spawn Y'),
        DeclareLaunchArgument('spawn_z', default_value='3.0', description='Spawn Z'),

        include_base_stack,
        weighted_odom_node,
        ekf_node,
    ])
