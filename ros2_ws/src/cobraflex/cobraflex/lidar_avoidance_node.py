#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math
import numpy as np


class LidarAvoidance(Node):

    def __init__(self):
        super().__init__('lidar_avoidance_node')

        # Declare ROS parameters
        self.declare_parameter('front_angle_deg', 10.0)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.4)

        # Load parameters
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)

        # Sub / Pub
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, 'front_sector_marker', 10)

        self.get_logger().info("Improved Lidar Avoidance Node started.")

    def scan_callback(self, msg: LaserScan):
        # Convert degrees to radians
        angle_width = math.radians(self.front_angle_deg)

        # Calculate indices for the front ± angle_width region
        angle_min, angle_max = msg.angle_min, msg.angle_max
        angle_inc = msg.angle_increment

        idx_min = int((0 - angle_width - angle_min) / angle_inc)
        idx_max = int((0 + angle_width - angle_min) / angle_inc)

        idx_min = max(idx_min, 0)
        idx_max = min(idx_max, len(msg.ranges) - 1)

        # Extract, clean, and smooth ranges
        front_ranges = np.array(msg.ranges[idx_min:idx_max + 1])
        front_ranges = front_ranges[np.isfinite(front_ranges)]  # remove inf / nan

        if len(front_ranges) == 0:
            self.get_logger().warn("No valid lidar readings!")
            return

        # Smoothing: take median to avoid spikes
        dist = float(np.median(front_ranges))

        # Publish RViz visualization
        self.publish_sector_marker(angle_width)

        # Logging
        self.get_logger().info(f"Front distance: {dist:.2f} m")

        # Movement logic
        twist = Twist()

        if dist < self.safe_distance:
            # Obstacle detected → choose a turn direction
            twist.linear.x = 0.0

            # Compare left vs right ranges
            left = np.median(msg.ranges[len(msg.ranges)//2 + 10 : len(msg.ranges)//2 + 60])
            right = np.median(msg.ranges[len(msg.ranges)//2 - 60 : len(msg.ranges)//2 - 10])

            # Determine safer direction
            if left > right:
                twist.angular.z = self.turn_speed
                self.get_logger().warn("Obstacle ahead! Turning LEFT.")
            else:
                twist.angular.z = -self.turn_speed
                self.get_logger().warn("Obstacle ahead! Turning RIGHT.")

        else:
            # All clear → move forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def publish_sector_marker(self, angle_width):
        """Visual front-sector marker for RViz."""
        marker = Marker()
        marker.header.frame_id = "laser"  # or base_link depending on your TF
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03

        marker.color.a = 0.9
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.1

        marker.points = []

        # Draw arc representing front region
        for angle in np.linspace(-angle_width, angle_width, 20):
            p = self.make_point(math.cos(angle), math.sin(angle))
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def make_point(self, x, y):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        return p


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
