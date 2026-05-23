import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)

from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------

    rover_description_share = get_package_share_directory(
        'rover_description'
    )

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------

    urdf_path = PathJoinSubstitution([
        FindPackageShare('rover_description'),
        'urdf',
        'my_robot.urdf.xacro'
    ])

    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare('rover_description'),
        'config',
        'gazebo_bridge.yaml'
    ])

    gz_gui_config_path = PathJoinSubstitution([
        FindPackageShare('rover_description'),
        'config',
        'gz_gui_config.config'
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare('rover_description'),
        'worlds',
        'rubicon.sdf'
    ])

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Spawn X position'
    )

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Spawn Y position'
    )

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='3.0',
        description='Spawn Z position'
    )

    # ------------------------------------------------------------------
    # Launch configurations
    # ------------------------------------------------------------------

    use_sim_time = LaunchConfiguration('use_sim_time')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    # ------------------------------------------------------------------
    # Gazebo resource path
    # ------------------------------------------------------------------

    gz_resource_path = EnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        default_value=''
    )

    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.dirname(rover_description_share),
            ':',
            rover_description_share,
            '/models:',
            rover_description_share,
            '/worlds:',
            gz_resource_path
        ]
    )

    # ------------------------------------------------------------------
    # Robot description from xacro
    # ------------------------------------------------------------------

    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            urdf_path,
            ' use_sim:=',
            use_sim_time
        ]),
        value_type=str
    )

    # ------------------------------------------------------------------
    # Robot State Publisher
    # ------------------------------------------------------------------

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # ------------------------------------------------------------------
    # Gazebo launch  (sim only)
    # ------------------------------------------------------------------

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [world_path, ' -r']
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    # ------------------------------------------------------------------
    # Spawn robot in Gazebo  (sim only)
    # ------------------------------------------------------------------

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        arguments=[
            '-name',
            'rover_description',

            '-topic',
            'robot_description',

            '-allow_renaming',
            'true',

            '--gui-config',
            gz_gui_config_path,

            '-x',
            spawn_x,

            '-y',
            spawn_y,

            '-z',
            spawn_z
        ],
        condition=IfCondition(use_sim_time)
    )

    # ------------------------------------------------------------------
    # Gazebo ↔ ROS2 bridge  (sim only)
    # ------------------------------------------------------------------

    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[
            {
                'config_file': gazebo_config_path,
                'use_sim_time': use_sim_time
            }
        ],
        condition=IfCondition(use_sim_time)
    )

    # ------------------------------------------------------------------
    # ros2_control node  (real robot only)
    # ------------------------------------------------------------------

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            PathJoinSubstitution([
                FindPackageShare('rover_controllers'),
                'config',
                'diff_controller.yaml'
            ])
        ],
        remappings=[
            ('/rover_controller/odom', '/odom_wheel'),
            ('/rover_controller/cmd_vel', '/cmd_vel_wheel'),
        ],
        condition=UnlessCondition(use_sim_time)
    )

    # ------------------------------------------------------------------
    # Controller spawners
    # ------------------------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    rover_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rover_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    aux_rocker_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'aux_rocker_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
        condition=IfCondition(use_sim_time)
    )

    # ------------------------------------------------------------------
    # Delay controller spawning  (sim: after spawn exits, real: after ros2_control_node starts)
    # ------------------------------------------------------------------

    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                rover_controller_spawner,
                aux_rocker_controller_spawner,
            ]
        ),
        condition=IfCondition(use_sim_time)
    )

    delay_controllers_after_ros2_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster_spawner,
                rover_controller_spawner,
            ]
        ),
        condition=UnlessCondition(use_sim_time)
    )

    # ------------------------------------------------------------------
    # Launch description
    # ------------------------------------------------------------------

    return LaunchDescription([

        # Arguments
        use_sim_time_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,

        # Environment
        set_gz_sim_resource_path,

        # Robot description + TF
        robot_state_publisher_node,

        # --- Simulation only ---
        gazebo_launch,
        spawn_robot_node,
        parameter_bridge_node,
        delay_controllers_after_spawn,

        # --- Real robot only ---
        ros2_control_node,
        delay_controllers_after_ros2_control,
    ])