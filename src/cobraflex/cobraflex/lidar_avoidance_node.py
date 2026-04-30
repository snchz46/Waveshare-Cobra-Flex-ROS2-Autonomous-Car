#!/usr/bin/env python3
"""Simple LiDAR-based obstacle avoidance and corridor centering."""

import math

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class AvoidanceWithLights(Node):
    """Drive forward when clear and turn away from nearby obstacles."""

    def __init__(self):
        """Initialize parameters, state and ROS interfaces."""
        super().__init__("cobraflex_avoidance")

        self.declare_parameter("forward_speed", 0.45)
        self.declare_parameter("turn_speed", 3.0)
        self.declare_parameter("safe_distance", 0.55)
        self.declare_parameter("hard_stop_distance", 0.40)
        self.declare_parameter("center_kp", 1.2)
        self.declare_parameter("min_turn_time", 0.9)
        self.declare_parameter("cmd_rate", 15.0)
        self.declare_parameter("front_angle_deg", 20.0)
        self.declare_parameter("side_sample_deg", 40.0)
        self.declare_parameter("lateral_safe_distance", 0.30)

        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.safe_distance = float(self.get_parameter("safe_distance").value)
        self.hard_stop_distance = float(self.get_parameter("hard_stop_distance").value)
        self.center_kp = float(self.get_parameter("center_kp").value)
        self.min_turn_time = float(self.get_parameter("min_turn_time").value)
        self.cmd_rate = float(self.get_parameter("cmd_rate").value)
        self.front_angle_deg = float(self.get_parameter("front_angle_deg").value)
        self.side_sample_deg = float(self.get_parameter("side_sample_deg").value)
        self.lateral_safe_distance = float(
            self.get_parameter("lateral_safe_distance").value
        )

        self.state = "FORWARD"
        self.state_enter_time = self.get_clock().now()
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.last_scan_time = self.get_clock().now()
        self.filt_left = None
        self.filt_right = None

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_timer = self.create_timer(1.0 / self.cmd_rate, self._cmd_timer_cb)

        self.get_logger().info("CobraFlex avoidance node started")

    def _scan_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_scan_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-3
        self.last_scan_time = now

        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        front_rad = math.radians(self.front_angle_deg)
        side_rad = math.radians(self.side_sample_deg)
        center_angle = math.pi

        def sample_range(start_ang, end_ang, samples=8):
            values = []
            for ang in np.linspace(start_ang, end_ang, samples):
                abs_ang = center_angle + ang
                idx = int((abs_ang - angle_min) / angle_inc)
                idx = max(0, min(len(ranges) - 1, idx))
                distance = ranges[idx]
                if math.isfinite(distance):
                    values.append(distance)

            if not values:
                return 5.0

            return float(np.mean(values))

        front = min(
            sample_range(-front_rad, -front_rad * 0.5),
            sample_range(front_rad * 0.5, front_rad),
        )
        left = sample_range(side_rad * 0.5, side_rad)
        right = sample_range(-side_rad, -side_rad * 0.5)

        alpha = 0.25
        if self.filt_left is None:
            self.filt_left = left
            self.filt_right = right
        else:
            self.filt_left = alpha * left + (1.0 - alpha) * self.filt_left
            self.filt_right = alpha * right + (1.0 - alpha) * self.filt_right

        left = self.filt_left
        right = self.filt_right

        if front < self.safe_distance:
            self._handle_obstacle(front, left, right, now)
            return

        if self.state.startswith("TURN"):
            elapsed = (now - self.state_enter_time).nanoseconds / 1e9
            if elapsed < self.min_turn_time:
                return

        self.state = "FORWARD"

        if left < self.lateral_safe_distance:
            self.target_lin = 0.15
            self.target_ang = 1.8
            return

        if right < self.lateral_safe_distance:
            self.target_lin = 0.15
            self.target_ang = -1.8
            return

        center_error = right - left
        centering = self.center_kp * center_error
        centering = max(-1.2, min(1.2, centering))

        self.target_lin = self.forward_speed
        self.target_ang = centering

    def _handle_obstacle(self, front, left, right, now):
        elapsed = (now - self.state_enter_time).nanoseconds / 1e9
        if self.state.startswith("TURN") and elapsed < self.min_turn_time:
            return

        if left > right:
            self.state = "TURN_LEFT"
            self.target_ang = self.turn_speed
        else:
            self.state = "TURN_RIGHT"
            self.target_ang = -self.turn_speed

        self.state_enter_time = now
        self.target_lin = 0.0 if front < self.hard_stop_distance else 0.15

    def _cmd_timer_cb(self):
        twist = Twist()
        twist.linear.x = float(self.target_lin)
        twist.angular.z = float(self.target_ang)
        self.cmd_pub.publish(twist)


def main(args=None):
    """Run the legacy LiDAR avoidance node."""
    rclpy.init(args=args)
    node = None
    try:
        node = AvoidanceWithLights()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
