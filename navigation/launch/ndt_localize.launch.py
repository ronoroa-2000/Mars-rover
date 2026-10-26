from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    # Assumes rover_description's my_robot.launch.py is already running
    # (Gazebo + robot + lidar bridge + EKF). Scan-matches the 3D lidar
    # against a prebuilt PCD map with Autoware's NDT and fuses the result
    # with the local EKF in a map-frame EKF that publishes map -> odom.
    # Do not run it together with the RTAB-Map launch files,
    # which publish the same transform.

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock'
    )

    pcd_map_arg = DeclareLaunchArgument(
        'pcd_map', default_value='/home/zoro/mars-rover/src/point_cloud_data/terrain_world.pcd',
        description='PCD map in the map frame (terrain_world.pcd is the Gazebo world frame)'
    )

    pcd_metadata_arg = DeclareLaunchArgument(
        'pcd_metadata',
        default_value='/home/zoro/mars-rover/src/point_cloud_data/terrain_world_metadata.yaml',
        description='Tile metadata for the map loader'
    )

    scan_cloud_topic_arg = DeclareLaunchArgument(
        'scan_cloud_topic', default_value='/scan/points',
        description='3D lidar PointCloud2 topic (swap this for the real lidar driver on hardware)'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odometry/filtered',
        description='EKF odometry used to predict the pose between scans'
    )

    # Must match the spawn_* arguments given to my_robot.launch.py.
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------
    # PCD map server
    # ------------------------------------------------------------------

    map_loader = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('autoware_map_loader'), 'launch', 'pointcloud_map_loader.launch.xml'
        ])),
        launch_arguments={
            # The packaged launch declares pointcloud_map_* but its param file
            # reads pcd_paths_or_directory / pcd_metadata_path, so pass both.
            'pointcloud_map_path': LaunchConfiguration('pcd_map'),
            'pointcloud_map_metadata_path': LaunchConfiguration('pcd_metadata'),
            'pcd_paths_or_directory': ["'", LaunchConfiguration('pcd_map'), "'"],
            'pcd_metadata_path': LaunchConfiguration('pcd_metadata'),
        }.items(),
    )

    # ------------------------------------------------------------------
    # NDT scan matcher (started deactivated; ndt_ekf_bridge triggers it)
    # ------------------------------------------------------------------

    ndt_params = PathJoinSubstitution([
        FindPackageShare('autoware_ndt_scan_matcher'), 'config', 'ndt_scan_matcher.param.yaml'
    ])

    ndt_node = Node(
        package='autoware_ndt_scan_matcher',
        executable='autoware_ndt_scan_matcher_node',
        name='ndt_scan_matcher',
        output='screen',
        parameters=[ndt_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('points_raw', '/ndt/points_delayed'),
            ('pcd_loader_service', '/service/get_differential_pcd_map'),
            ('trigger_node_srv', '/trigger_node'),
        ],
    )

    # ------------------------------------------------------------------
    # Feeds NDT its pose guess and the NDT pose to the map-frame EKF
    # ------------------------------------------------------------------

    bridge_node = Node(
        package='navigation',
        executable='ndt_ekf_bridge.py',
        name='ndt_ekf_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': LaunchConfiguration('odom_topic'),
            'cloud_in': LaunchConfiguration('scan_cloud_topic'),
            'init_x': LaunchConfiguration('spawn_x'),
            'init_y': LaunchConfiguration('spawn_y'),
            'init_z': LaunchConfiguration('spawn_z'),
            'init_yaw': LaunchConfiguration('spawn_yaw'),
        }],
    )

    # ------------------------------------------------------------------
    # Map-frame EKF: local odometry + NDT pose -> map -> odom
    # ------------------------------------------------------------------

    ekf_map_config = PathJoinSubstitution([
        FindPackageShare('navigation'), 'config', 'ekf_map.yaml'
    ])

    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        parameters=[ekf_map_config, {'use_sim_time': use_sim_time}],
        # The local EKF owns /odometry/filtered.
        remappings=[('odometry/filtered', '/odometry/global')],
    )

    return LaunchDescription([
        use_sim_time_arg,
        pcd_map_arg,
        pcd_metadata_arg,
        scan_cloud_topic_arg,
        odom_topic_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,

        map_loader,
        ndt_node,
        bridge_node,
        ekf_map_node,
    ])
