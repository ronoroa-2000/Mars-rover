import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/zoro/mars-rover/src/point_cloud_data/rtabmap_map.pcd',
        description='Path to the exported RTAB-Map point cloud (from rtabmap-export)'
    )

    pcd_file = LaunchConfiguration('pcd_file')

    rviz_config_path = os.path.join(
        get_package_share_directory('navigation'), 'rviz', 'map_view.rviz'
    )

    # The cloud is already registered in RTAB-Map's optimized map frame,
    # so publish it straight into "map" -- no offset TF hack needed (unlike
    # rover_description's terrain-from-mesh pointcloud_display.launch.xml).
    pcd_publisher_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='rtabmap_pcd_publisher',
        output='screen',
        parameters=[{
            'file_name': pcd_file,
            'frame_id': 'map',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        pcd_file_arg,
        pcd_publisher_node,
        rviz_node,
    ])
