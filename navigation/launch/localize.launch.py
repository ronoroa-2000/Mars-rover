from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    # Assumes rover_description's my_robot.launch.py is already running.
    # Loads the map built by mapping.launch.py and continuously scan-matches
    # the live RGB-D stream against it, publishing a corrected map -> odom
    # TF. EKF keeps publishing odom -> base_footprint as usual; this node
    # never writes to database_path (Mem/IncrementalMemory:=false).

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='/home/zoro/mars-rover/src/point_cloud_data/rtabmap_map.db',
        description='Pre-built map database to localize against (from mapping.launch.py)'
    )

    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic', default_value='/camera/left/image_rect_color',
        description='RGB image topic (swap this for the real camera driver on hardware)'
    )

    rgb_camera_info_topic_arg = DeclareLaunchArgument(
        'rgb_camera_info_topic', default_value='/camera/left/camera_info',
        description='RGB camera_info topic'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/depth/depth_registered',
        description='Depth image topic'
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
    database_path = LaunchConfiguration('database_path')
    rgb_topic = LaunchConfiguration('rgb_topic')
    rgb_camera_info_topic = LaunchConfiguration('rgb_camera_info_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    frame_id = LaunchConfiguration('frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')

    # ------------------------------------------------------------------
    # Sync the RGB image + camera_info + depth image into one rgbd_image
    # ------------------------------------------------------------------

    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('rgb/camera_info', rgb_camera_info_topic),
            ('depth/image', depth_topic),
        ]
    )

    # ------------------------------------------------------------------
    # RTAB-Map in localization-only mode: scan-matches against the stored
    # map and publishes map -> odom, but never modifies database_path.
    # ------------------------------------------------------------------

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'subscribe_rgbd': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_odom_info': False,
            'database_path': database_path,
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('odom', odom_topic),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        rgb_topic_arg,
        rgb_camera_info_topic_arg,
        depth_topic_arg,
        odom_topic_arg,
        frame_id_arg,
        odom_frame_id_arg,

        rgbd_sync_node,
        rtabmap_node,
    ])
