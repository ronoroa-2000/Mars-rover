from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    # Assumes rover_description's my_robot.launch.py is already running
    # (Gazebo + robot + lidar bridge + EKF + teleop). This launch file
    # only adds the RTAB-Map lidar mapping layer on top.

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='/home/zoro/mars-rover/src/point_cloud_data/lidar_map.db',
        description='Where RTAB-Map stores the mapping session (map graph + scans)'
    )

    delete_db_on_start_arg = DeclareLaunchArgument(
        'delete_db_on_start', default_value='true',
        description='Start a fresh map each run instead of appending to database_path'
    )

    scan_cloud_topic_arg = DeclareLaunchArgument(
        'scan_cloud_topic', default_value='/scan/points',
        description='3D lidar PointCloud2 topic (swap this for the real lidar driver on hardware)'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odometry/filtered',
        description='External odometry topic fed to RTAB-Map (EKF fused wheel+IMU odom)'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu/synced',
        description='IMU topic (orientation) used for the graph gravity constraint'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='base_footprint',
        description='Robot base frame'
    )

    odom_frame_id_arg = DeclareLaunchArgument(
        'odom_frame_id', default_value='odom',
        description='Odometry frame published by the EKF'
    )

    rtabmap_viz_arg = DeclareLaunchArgument(
        'rtabmap_viz', default_value='false',
        description='Also open the rtabmap_viz window (RViz on /rtabmap/cloud_map is lighter)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    scan_cloud_topic = LaunchConfiguration('scan_cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    frame_id = LaunchConfiguration('frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')

    # ------------------------------------------------------------------
    # RTAB-Map SLAM node
    #   delete_db_on_start picks between two otherwise-identical node
    #   definitions since it's a CLI flag, not a ROS parameter.
    # ------------------------------------------------------------------

    rtabmap_config = PathJoinSubstitution([
        FindPackageShare('navigation'), 'config', 'rtabmap_lidar.yaml'
    ])

    rtabmap_params = [rtabmap_config, {
        'frame_id': frame_id,
        'odom_frame_id': odom_frame_id,
        'database_path': database_path,
        'Mem/IncrementalMemory': 'true',
        'use_sim_time': use_sim_time,
    }]
    rtabmap_remappings = [
        ('odom', odom_topic),
        ('scan_cloud', scan_cloud_topic),
        ('imu', imu_topic),
    ]

    rtabmap_node_fresh = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=rtabmap_params,
        remappings=rtabmap_remappings,
        arguments=['--delete_db_on_start'],
        condition=IfCondition(delete_db_on_start),
    )

    rtabmap_node_resume = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=rtabmap_params,
        remappings=rtabmap_remappings,
        condition=UnlessCondition(delete_db_on_start),
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'subscribe_odom_info': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=rtabmap_remappings,
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        delete_db_on_start_arg,
        scan_cloud_topic_arg,
        odom_topic_arg,
        imu_topic_arg,
        frame_id_arg,
        odom_frame_id_arg,
        rtabmap_viz_arg,

        rtabmap_node_fresh,
        rtabmap_node_resume,
        rtabmap_viz_node,
    ])
