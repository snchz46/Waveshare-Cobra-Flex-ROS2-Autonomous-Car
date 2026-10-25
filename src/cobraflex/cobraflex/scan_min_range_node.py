#!/usr/bin/env python3
"""Minimum range of the RPLIDAR A2M8 applied to the simulated scan.

The gpu_lidar of robot.gazebo renders from 0.015 m, so that the body of the
robot still blocks the rear sector as on the hardware; a larger near clip
would make that sector transparent. The A2M8 discards returns below 0.15 m
(Slamtec LD208) and sllidar_ros2 publishes them as inf. This node applies the
same rule to the bridged scan (scan_gz) and republishes it on /scan with
range_min = 0.15 m.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def apply_min_range(msg, range_min):
    """
    Invalidate the returns of a scan closer than range_min, in place.

    Ranges below range_min become inf and their intensities 0, and range_min
    of the message is raised to that value. NaN and inf readings are kept.
    Returns the number of invalidated rays.
    """
    ranges = np.asarray(msg.ranges, dtype=np.float32)
    short = ranges < range_min
    if short.any():
        ranges[short] = math.inf
        msg.ranges = ranges.tolist()
        if len(msg.intensities) == len(ranges):
            intensities = np.asarray(msg.intensities, dtype=np.float32)
            intensities[short] = 0.0
            msg.intensities = intensities.tolist()
    msg.range_min = max(float(msg.range_min), float(range_min))
    return int(short.sum())


class ScanMinRange(Node):
    """Republish the simulated scan with the minimum range of the A2M8."""

    def __init__(self):
        """Initialize parameters and ROS interfaces."""
        super().__init__("scan_min_range")
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("input_topic", "scan_gz")
        self.declare_parameter("output_topic", "scan")
        self.range_min = float(self.get_parameter("range_min").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.scan_pub = self.create_publisher(LaserScan, output_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, input_topic, self._scan_callback, 10)
        self.get_logger().info(
            f"{input_topic} -> {output_topic}: returns below {self.range_min:.3f} m set to inf"
        )

    def _scan_callback(self, msg):
        """Invalidate the short returns and republish."""
        apply_min_range(msg, self.range_min)
        self.scan_pub.publish(msg)


def main(args=None):
    """Run the scan minimum-range node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ScanMinRange()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Under `ros2 launch`, Ctrl-C arrives as ExternalShutdownException
        # instead of KeyboardInterrupt; catching it gives exit status 0.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # After an external shutdown the context is already down; a second
        # shutdown() call raises an exception.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
