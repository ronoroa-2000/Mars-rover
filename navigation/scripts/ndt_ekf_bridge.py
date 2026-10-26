#!/usr/bin/env python3
"""Glue between Autoware's ndt_scan_matcher and the rover's EKFs.

ndt_scan_matcher estimates the pose of base_link in the map frame and needs a
pose guess in the map frame for every scan. The map-frame EKF (ekf_map, see
config/ekf_map.yaml) fuses the NDT pose and publishes map -> odom. This node
connects the three:

  - publishes the guess  T_map_odom * T_odom_base  on /ekf_pose_with_covariance
    for each odometry message
  - converts each /ndt_pose_with_covariance result from base_link to
    base_footprint (the EKF's base frame) and publishes it on
    /ndt_pose_footprint for ekf_map
  - seeds ekf_map with the spawn pose on /set_pose
  - activates ndt_scan_matcher through its /trigger_node service
  - relays the lidar cloud after a short delay: ndt_scan_matcher interpolates
    its pose guess between messages stamped before and after the scan, so the
    scan must arrive after the next odometry message
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(m):
    t = np.trace(m)
    if t > 0.0:
        s = 2.0 * math.sqrt(t + 1.0)
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    w = (m[k, j] - m[j, k]) / s
    return (q[0], q[1], q[2], w)


def pose_to_T(position, orientation):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(orientation.x, orientation.y, orientation.z, orientation.w)
    T[:3, 3] = (position.x, position.y, position.z)
    return T


def tf_to_T(transform):
    return pose_to_T(transform.translation, transform.rotation)


def fill_pose(pose, T):
    pose.position.x, pose.position.y, pose.position.z = T[:3, 3]
    q = mat_to_quat(T[:3, :3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q


class NdtEkfBridge(Node):
    def __init__(self):
        super().__init__('ndt_ekf_bridge')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.footprint_frame = self.declare_parameter('footprint_frame', 'base_footprint').value
        odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').value
        cloud_in = self.declare_parameter('cloud_in', '/scan/points').value
        cloud_out = self.declare_parameter('cloud_out', '/ndt/points_delayed').value
        self.cloud_delay = Duration(seconds=self.declare_parameter('cloud_delay_s', 0.12).value)
        self.init_x = self.declare_parameter('init_x', 0.0).value
        self.init_y = self.declare_parameter('init_y', 0.0).value
        self.init_z = self.declare_parameter('init_z', 0.0).value
        self.init_yaw = self.declare_parameter('init_yaw', 0.0).value
        self.init_pos_var = self.declare_parameter('init_pos_var', 0.25).value
        self.init_yaw_var = self.declare_parameter('init_yaw_var', 0.1).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.guess_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/ekf_pose_with_covariance', 10)
        self.ndt_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/ndt_pose_footprint', 10)
        self.set_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/set_pose', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/ndt_pose_with_covariance', self.on_ndt, 10)

        self.cloud_buffer = deque()
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_out, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_in, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.01, self.release_clouds)

        # ekf_map has no pose until it gets one: seed it with the spawn pose a
        # few times (it may not be up yet), then start guessing.
        self.seed_count = 0
        self.seeded = False
        self.seed_timer = self.create_timer(1.0, self.seed_ekf_map)

        self.trigger_client = self.create_client(SetBool, '/trigger_node')
        self.trigger_timer = self.create_timer(1.0, self.activate_ndt)

        self.get_logger().info(
            f'Seeding ekf_map at ({self.init_x:.2f}, {self.init_y:.2f}, {self.init_z:.2f}) '
            f'yaw {self.init_yaw:.2f}')

    def seed_ekf_map(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        p = msg.pose.pose
        p.position.x, p.position.y, p.position.z = self.init_x, self.init_y, self.init_z
        p.orientation.z = math.sin(self.init_yaw / 2.0)
        p.orientation.w = math.cos(self.init_yaw / 2.0)
        cov = [0.0] * 36
        for i, v in enumerate((self.init_pos_var,) * 3 + (0.01, 0.01, self.init_yaw_var)):
            cov[i * 7] = v
        msg.pose.covariance = cov
        self.set_pose_pub.publish(msg)
        self.seed_count += 1
        if self.seed_count >= 3:
            self.seeded = True
            self.seed_timer.cancel()

    def activate_ndt(self):
        if not self.trigger_client.service_is_ready():
            return
        req = SetBool.Request()
        req.data = True
        self.trigger_client.call_async(req)
        self.trigger_timer.cancel()
        self.get_logger().info('Activated ndt_scan_matcher')

    def lookup(self, target, source, stamp, timeout_s):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, stamp, timeout=Duration(seconds=timeout_s))
        except Exception:
            return None
        return tf_to_T(tf.transform)

    def on_cloud(self, msg):
        self.cloud_buffer.append(msg)

    def release_clouds(self):
        now = self.get_clock().now()
        while self.cloud_buffer and now - Time.from_msg(
                self.cloud_buffer[0].header.stamp) >= self.cloud_delay:
            self.cloud_pub.publish(self.cloud_buffer.popleft())

    def on_odom(self, odom):
        if not self.seeded:
            return
        # Latest map -> odom (ekf_map publishes it), odom -> base at the odom stamp.
        T_map_odom = self.lookup(self.map_frame, self.odom_frame, Time(), 0.0)
        T_odom_base = self.lookup(
            self.odom_frame, self.base_frame, Time.from_msg(odom.header.stamp), 0.05)
        if T_map_odom is None or T_odom_base is None:
            return
        out = PoseWithCovarianceStamped()
        out.header.stamp = odom.header.stamp
        out.header.frame_id = self.map_frame
        fill_pose(out.pose.pose, T_map_odom @ T_odom_base)
        cov = [0.0] * 36
        for i, v in enumerate((0.1, 0.1, 0.1, 0.01, 0.01, 0.01)):
            cov[i * 7] = v
        out.pose.covariance = cov
        self.guess_pub.publish(out)

    def on_ndt(self, msg):
        T_foot_base = self.lookup(self.footprint_frame, self.base_frame, Time(), 0.1)
        if T_foot_base is None:
            return
        T_map_base = pose_to_T(msg.pose.pose.position, msg.pose.pose.orientation)
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = self.map_frame
        fill_pose(out.pose.pose, T_map_base @ np.linalg.inv(T_foot_base))
        out.pose.covariance = msg.pose.covariance
        self.ndt_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = NdtEkfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
