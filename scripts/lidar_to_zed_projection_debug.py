#!/usr/bin/env python3
#
# lidar_to_zed_projection_debug.py
#
# Projects LiDAR scan points onto ZED camera image with debug info.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
import numpy as np
import cv2

try:
    from cv_bridge import CvBridge
    bridge = CvBridge()
except ImportError:
    bridge = None
    print("[INFO] cv_bridge not found, using manual decoding instead.")


class LidarToZED(Node):
    def __init__(self):
        super().__init__('lidar_to_zed_projection_debug')

        # --- Extrinsics (LiDAR relative to camera) ---
        # LiDAR is 2 cm above the camera, slightly forward (adjust as needed)
        self.translation = np.array([-0.05, -0.05, 0.05])  # meters (x,y,z)
        self.rotation_deg = np.array([0.0, 0.0, 180.0])  # roll, pitch, yaw (degrees)

        self.K = None
        self.width = None
        self.height = None
        self.latest_scan = None

        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cb_lidar, 10)
        self.sub_image = self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.cb_image, 10)
        self.sub_caminfo = self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info', self.cb_caminfo, 10)

        self.get_logger().info("Subscribed to LiDAR and ZED topics for projection test.")

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
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        self.latest_scan = np.vstack((x, y, z))

    def cb_image(self, msg: Image):
        if self.latest_scan is None or self.K is None:
            return

        # Decode image
        if bridge:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        else:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            img = data.reshape(msg.height, msg.width, 3)

        # Compute rotation matrix
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

	# Convert LiDAR (x forward, y left, z up) → Camera (z forward, x right, y down)
        lidar_to_cam = np.array([[0, -1, 0],
                                 [0, 0, -1],
                                 [1, 0, 0]])  # rotates axes into camera frame
        pts_cam = lidar_to_cam @ (R @ self.latest_scan) + self.translation.reshape(3, 1)


        # Only keep points in front of camera
        front_mask = pts_cam[2, :] > 0
        pts_cam = pts_cam[:, front_mask]

        if pts_cam.shape[1] == 0:
            self.get_logger().warn("No LiDAR points in front of camera.")
            return

        # Project to image plane
        uv = self.K @ (pts_cam / pts_cam[2, :])
        u = uv[0, :].astype(int)
        v = uv[1, :].astype(int)

        # Filter visible points
        mask = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)

        self.get_logger().info(f"Projected {mask.sum()} visible LiDAR points onto image.")

        # Draw big visible circles for LiDAR hits
        for x, y in zip(u[mask], v[mask]):
            cv2.circle(img, (x, y), 4, (0, 255, 0), -1)

        # Debug overlay text
        cv2.putText(img, f"{mask.sum()} LiDAR pts visible", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("LiDAR projected on ZED image", img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = LidarToZED()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

