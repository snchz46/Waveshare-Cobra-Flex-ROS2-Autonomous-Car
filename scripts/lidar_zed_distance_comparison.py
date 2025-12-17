#!/usr/bin/env python3
#
# lidar_zed_distance_comparison.py
#
# Subscribes to both the RPLIDAR and ZED depth topics, and compares the reported distances.
# It prints the difference and checks if the distances are within an acceptable range.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
import numpy as np

try:
    from cv_bridge import CvBridge
    bridge = CvBridge()
except ImportError:
    bridge = None
    print("[INFO] cv_bridge not found, using manual decoding for ZED depth.")


class DistanceComparison(Node):
    def __init__(self):
        super().__init__('lidar_zed_distance_comparison')

        self.lidar_range = None
        self.depth_range = None

        # Subscribe to the LiDAR topic
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cb_lidar, 10)

        # Subscribe to the ZED depth topic
        self.sub_depth = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.cb_depth, 10)

        # Create a timer to compare every second
        self.timer = self.create_timer(1.0, self.compare)

    def cb_lidar(self, msg: LaserScan):
        # Extract LiDAR distances
        ranges = np.array(msg.ranges)
        valid = np.logical_and(ranges > msg.range_min, ranges < msg.range_max)

        # Take the rear range (180°-360°)
        center_idx = len(ranges) // 2
        # Set a 180° window from the center (for example, take from 180° to 360°)
        width = int((180.0 * np.pi / 180.0) / msg.angle_increment)  # 180°
        window = ranges[center_idx - width:center_idx + width]
        window = window[np.isfinite(window)]  # Filter invalid values

        if len(window) > 0:
            self.lidar_range = np.mean(window)  # Window average

    def cb_depth(self, msg: Image):
        if bridge:
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        else:
            data = np.frombuffer(msg.data, dtype=np.float32)
            depth = data.reshape(msg.height, msg.width)

        # Take the central row
        row = depth[depth.shape[0] // 2, :]
        valid = (row > 0.1) & (row < 20.0)  # Filter invalid values
        row = row[valid]

        if len(row) > 0:
            self.depth_range = np.nanmean(row[depth.shape[1] // 2 - 10 : depth.shape[1] // 2 + 10])  # Average range

    def compare(self):
        # Ensure both sensors have valid data
        if self.lidar_range is None or self.depth_range is None:
            return

        # Calculate the difference
        diff = abs(self.lidar_range - self.depth_range)
        self.get_logger().info(
            f"LIDAR: {self.lidar_range:.2f} m | ZED Depth: {self.depth_range:.2f} m | Δ = {diff:.2f} m"
        )

        # Check whether the difference is acceptable (for example, within a 0.1 m range)
        if diff > 0.1:
            self.get_logger().warn(f"Warning: Distance difference exceeds threshold ({diff:.2f} m)!")
        else:
            self.get_logger().info(f"Distances are consistent (Δ = {diff:.2f} m)")


def main():
    rclpy.init()
    node = DistanceComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
