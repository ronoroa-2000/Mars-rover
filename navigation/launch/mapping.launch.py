from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    # Assumes rover_description's my_robot.launch.py is already running
    # (Gazebo + robot + camera bridge + EKF + teleop). This launch file
    # only adds the RTAB-Map mapping layer on top.

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='/home/zoro/mars-rover/src/point_cloud_data/rtabmap_map.db',
        description='Where RTAB-Map stores the mapping session (map graph + point cloud)'
    )

    delete_db_on_start_arg = DeclareLaunchArgument(
        'delete_db_on_start', default_value='true',
        description='Start a fresh map each run instead of appending to database_path'
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
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
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
    # RTAB-Map SLAM node
    #   Odometry comes from the EKF (/odometry/filtered) rather than
    #   RTAB-Map's own visual odometry, so it reuses the existing
    #   wheel+IMU fusion instead of duplicating it.
    #   delete_db_on_start picks between two otherwise-identical node
    #   definitions since it's a CLI flag, not a ROS parameter.
    # ------------------------------------------------------------------

    rtabmap_params = [{
        'frame_id': frame_id,
        'odom_frame_id': odom_frame_id,
        'subscribe_rgbd': True,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_odom_info': False,
        'database_path': database_path,
        'Mem/IncrementalMemory': 'true',
        'use_sim_time': use_sim_time,
    }]
    rtabmap_remappings = [('odom', odom_topic)]

    rtabmap_node_fresh = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
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
        output='screen',
        parameters=rtabmap_params,
        remappings=rtabmap_remappings,
        condition=UnlessCondition(delete_db_on_start),
    )

    # ------------------------------------------------------------------
    # Live view of the map / loop closures while driving
    # ------------------------------------------------------------------

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'subscribe_rgbd': True,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        delete_db_on_start_arg,
        rgb_topic_arg,
        rgb_camera_info_topic_arg,
        depth_topic_arg,
        odom_topic_arg,
        frame_id_arg,
        odom_frame_id_arg,

        rgbd_sync_node,
        rtabmap_node_fresh,
        rtabmap_node_resume,
        rtabmap_viz_node,
    ])
