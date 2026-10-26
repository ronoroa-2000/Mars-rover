from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    # Assumes rover_description's my_robot.launch.py is already running.
    # Loads the map built by lidar_mapping.launch.py and continuously
    # ICP-matches the live lidar cloud against it, publishing a corrected
    # map -> odom TF. EKF keeps publishing odom -> base_footprint as usual;
    # this node never writes to database_path (Mem/IncrementalMemory:=false).

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='/home/zoro/mars-rover/src/point_cloud_data/lidar_map.db',
        description='Pre-built map database to localize against (from lidar_mapping.launch.py)'
    )

    scan_cloud_topic_arg = DeclareLaunchArgument(
        'scan_cloud_topic', default_value='/scan/points',
        description='3D lidar PointCloud2 topic (swap this for the real lidar driver on hardware)'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odometry/filtered',
        description='External odometry topic fed to RTAB-Map (EKF fused wheel+IMU odom)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='base_footprint',
        description='Robot base frame'
    )

    odom_frame_id_arg = DeclareLaunchArgument(
        'odom_frame_id', default_value='odom',
        description='Odometry frame published by the EKF'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------
    # RTAB-Map in localization-only mode.
    #   Without images there is no global (visual) relocalization, so it
    #   assumes the rover starts where the mapping run started
    #   (RGBD/StartAtOrigin). Otherwise give it a pose with RViz's
    #   "2D Pose Estimate" tool (/rtabmap/initialpose).
    # ------------------------------------------------------------------

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('navigation'), 'config', 'rtabmap_lidar.yaml'
        ]), {
            'frame_id': LaunchConfiguration('frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'database_path': LaunchConfiguration('database_path'),
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'RGBD/StartAtOrigin': 'true',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('odom', LaunchConfiguration('odom_topic')),
            ('scan_cloud', LaunchConfiguration('scan_cloud_topic')),
            ('initialpose', '/initialpose'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        scan_cloud_topic_arg,
        odom_topic_arg,
        frame_id_arg,
        odom_frame_id_arg,

        rtabmap_node,
    ])
