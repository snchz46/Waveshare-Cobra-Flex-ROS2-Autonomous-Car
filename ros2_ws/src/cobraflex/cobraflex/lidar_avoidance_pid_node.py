#!/usr/bin/env python3
"""
Simple, stable LIDAR avoidance + centering for CobraFlex robot.
FEATURES:
 - Forward driving when clear.
 - Centering in corridors (no zig-zag): right-left distance with smoothing.
 - Obstacle detection with stable state machine:
      FORWARD → TURN_LEFT / TURN_RIGHT
 - Avoid walls laterally (safe lateral distance).
 - Smooth behavior, no oscillation.
 - Publishes clean /cmd_vel for CobraFlex T:13 mode.

Author: Samuel
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
# ================================================================
#                  AVOIDANCE NODE
# ================================================================

class AvoidanceWithLights(Node):

    def __init__(self):
        super().__init__("cobraflex_avoidance")

        # -------------------------
        # PARAMETERS
        # -------------------------
        self.declare_parameter("forward_speed", 0.45)
        self.declare_parameter("turn_speed", 3.0)
        self.declare_parameter("safe_distance", 0.55)
        self.declare_parameter("hard_stop_distance", 0.40)

        self.declare_parameter("center_kp", 1.2)            # centering gain
        self.declare_parameter("min_turn_time", 0.9)        # commit to turn
        self.declare_parameter("cmd_rate", 15.0)

        self.declare_parameter("front_angle_deg", 20.0)
        self.declare_parameter("side_sample_deg", 40.0)
        self.declare_parameter("lateral_safe_distance", 0.30)

        # Load params
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.safe_distance = float(self.get_parameter("safe_distance").value)
        self.hard_stop_distance = float(self.get_parameter("hard_stop_distance").value)
        self.center_kp = float(self.get_parameter("center_kp").value)
        self.min_turn_time = float(self.get_parameter("min_turn_time").value)
        self.cmd_rate = float(self.get_parameter("cmd_rate").value)
        self.front_angle_deg = float(self.get_parameter("front_angle_deg").value)
        self.side_sample_deg = float(self.get_parameter("side_sample_deg").value)
        self.lateral_safe_distance = float(self.get_parameter("lateral_safe_distance").value)

        # -------------------------
        # INTERNAL STATE
        # -------------------------
        self.state = "FORWARD"       # or TURN_LEFT / TURN_RIGHT
        self.state_enter_time = self.get_clock().now()

        # Current command targets
        self.target_lin = 0.0
        self.target_ang = 0.0

        self.last_scan_time = self.get_clock().now()

        # Smoothed lateral distances
        self.filt_left = None
        self.filt_right = None

        # -------------------------
        # ROS I/O
        # -------------------------
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_timer = self.create_timer(1.0 / self.cmd_rate, self.cmd_timer_cb)

        self.get_logger().info("CobraFlex Avoidance node started.")

    # ================================================================
    # LIDAR CALLBACK
    # ================================================================
    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        dt = (now - self.last_scan_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 1e-3
        self.last_scan_time = now

        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # Convert to radians
        front_rad = math.radians(self.front_angle_deg)
        side_rad = math.radians(self.side_sample_deg)

        # Front of robot = pi radians
        center_angle = math.pi

        # Utility: sample mean distance in angular window
        def sample_range(start_ang, end_ang, samples=8):
            vals = []
            for ang in np.linspace(start_ang, end_ang, samples):
                abs_ang = center_angle + ang
                idx = int((abs_ang - angle_min) / angle_inc)
                idx = max(0, min(len(ranges) - 1, idx))
                r = ranges[idx]
                if math.isfinite(r):
                    vals.append(r)
            if not vals:
                return 5.0
            return float(np.mean(vals))

        # FRONT window (two samples averaged)
        front = min(
            sample_range(-front_rad, -front_rad * 0.5),
            sample_range(front_rad * 0.5, front_rad)
        )

        # LEFT and RIGHT averaged
        left = sample_range(side_rad * 0.5, side_rad)
        right = sample_range(-side_rad, -side_rad * 0.5)

        # Smooth with exponential filter
        alpha = 0.25
        if self.filt_left is None:
            self.filt_left = left
            self.filt_right = right
        else:
            self.filt_left = alpha * left + (1 - alpha) * self.filt_left
            self.filt_right = alpha * right + (1 - alpha) * self.filt_right

        left = self.filt_left
        right = self.filt_right

        # ================================================================
        #                         FSM LOGIC
        # ================================================================

        # --- 1) OBSTACLE AHEAD ---
        if front < self.safe_distance:
            self.handle_obstacle(front, left, right, now)
            return

        # --- 2) FORWARD MODE ---
        if self.state.startswith("TURN"):
            elapsed = (now - self.state_enter_time).nanoseconds / 1e9
            if elapsed < self.min_turn_time:
                return  # keep turning
            else:
                self.state = "FORWARD"

        # NORMAL FORWARD with centering
        self.state = "FORWARD"

        # Lateral safety envelope
        too_close_left = left < self.lateral_safe_distance
        too_close_right = right < self.lateral_safe_distance

        if too_close_left:
            self.target_lin = 0.15
            self.target_ang = +1.8
            return

        if too_close_right:
            self.target_lin = 0.15
            self.target_ang = -1.8
            return

        # ---- CENTERING ----
        center_error = right - left
        centering = self.center_kp * center_error
        centering = max(-1.2, min(1.2, centering))

        self.target_lin = self.forward_speed
        self.target_ang = centering

    # ================================================================
    # OBSTACLE HANDLING
    # ================================================================
    def handle_obstacle(self, front, left, right, now):
        elapsed = (now - self.state_enter_time).nanoseconds / 1e9

        # If already turning, don't switch too early
        if self.state.startswith("TURN") and elapsed < self.min_turn_time:
            return

        if left > right:
            self.state = "TURN_LEFT"
            self.target_ang = +self.turn_speed
        else:
            self.state = "TURN_RIGHT"
            self.target_ang = -self.turn_speed

        self.state_enter_time = now

        # Slow or stop
        if front < self.hard_stop_distance:
            self.target_lin = 0.0
        else:
            self.target_lin = 0.15

    # ================================================================
    # CMD_VEL TIMER
    # ================================================================
    def cmd_timer_cb(self):
        twist = Twist()
        twist.linear.x = float(self.target_lin)
        twist.angular.z = float(self.target_ang)
        self.cmd_pub.publish(twist)


# ================================================================
# MAIN
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceWithLights()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
