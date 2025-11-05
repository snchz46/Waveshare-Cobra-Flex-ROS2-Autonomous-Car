#!/usr/bin/env python3
#
# lidar_to_zed_pointcloud.py
#
# Converts LiDAR scans into a PointCloud2 projected in the ZED camera frame.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CameraInfo, PointCloud2, PointField
import numpy as np
import struct
import sensor_msgs_py.point_cloud2 as pc2


class LidarToZEDCloud(Node):
    def __init__(self):
        super().__init__('lidar_to_zed_pointcloud')

        # --- Extrinsics (LiDAR relative to camera) ---
        self.translation = np.array([-0.05, -0.05, 0.05])  # meters (x(forw-backw),y(r-l),z(up-down))
        self.rotation_deg = np.array([90.0, 0.0, -270.0])  # roll, pitch, yaw (degrees)

        self.K = None
        self.width = None
        self.height = None
        self.latest_scan = None

        # Subscriptions
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cb_lidar, 10)
        self.sub_caminfo = self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info', self.cb_caminfo, 10)

        # Publisher
        self.pub_cloud = self.create_publisher(PointCloud2, '/lidar_in_camera_frame', 10)

        self.get_logger().info("Publishing LiDAR → ZED camera frame point cloud as /lidar_in_camera_frame")

    def cb_caminfo(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.width = msg.width
        self.height = msg.height

    def cb_lidar(self, msg: LaserScan):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        valid = np.logical_and(ranges > msg.range_min, ranges < msg.range_max)
        angles = angles[valid]
        ranges = ranges[valid]

        # Convert polar to cartesian (LiDAR frame)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        pts = np.vstack((x, y, z))

        # Apply rotation
        r = np.deg2rad(self.rotation_deg)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(r[0]), -np.sin(r[0])],
                       [0, np.sin(r[0]), np.cos(r[0])]])
        Ry = np.array([[np.cos(r[1]), 0, np.sin(r[1])],
                       [0, 1, 0],
                       [-np.sin(r[1]), 0, np.cos(r[1])]])
        Rz = np.array([[np.cos(r[2]), -np.sin(r[2]), 0],
                       [np.sin(r[2]), np.cos(r[2]), 0],
                       [0, 0, 1]])
        R = Rz @ Ry @ Rx

        # Convert LiDAR frame (x forward, y left, z up) → camera frame (z forward, x right, y down)
        lidar_to_cam = np.array([[0, -1, 0],
                                 [0, 0, -1],
                                 [1, 0, 0]])
        pts_cam = lidar_to_cam @ (R @ pts) + self.translation.reshape(3, 1)

        # Keep only points in front of the camera
        front_mask = pts_cam[2, :] > 0
        pts_cam = pts_cam[:, front_mask]

        # Publish as PointCloud2
        self.publish_cloud(msg.header, pts_cam)

    def publish_cloud(self, header, pts):
        """Publish numpy (3xN) as sensor_msgs/PointCloud2"""
        points = []
        for i in range(pts.shape[1]):
            x, y, z = pts[0, i], pts[1, i], pts[2, i]
            points.append([x, y, z])

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        cloud_msg.header.frame_id = "zed_left_camera_frame"
        self.pub_cloud.publish(cloud_msg)
        self.get_logger().info(f"Published {len(points)} LiDAR points in camera frame.")


def main():
    rclpy.init()
    node = LidarToZEDCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

