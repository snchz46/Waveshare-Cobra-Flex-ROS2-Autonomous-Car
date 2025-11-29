#!/usr/bin/env python3
"""
ROS2 node for lidar-based obstacle avoidance with:

- Subscription to /scan (LaserScan from RPLIDAR or similar)
- Full 360° obstacle classification into sectors
- Front safety check with adjustable front angle and safe distance
- PID-based turning toward the safest direction when blocked
- Velocity ramping (acceleration limiting) for smoother motion
- RViz visualization of the front sector

Author: You + ChatGPT :)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker


class LidarAvoidancePID(Node):
    """
    Main node class.

    Responsibilities:
    - Subscribe to /scan and process LaserScan data
    - Classify obstacles in 360° into sectors
    - Decide desired heading (angle) and linear speed
    - Use a PID controller to compute angular velocity
    - Apply acceleration limits (velocity ramping)
    - Publish /cmd_vel Twist messages
    - Publish a marker for RViz to visualize front sector
    """

    def __init__(self):
        super().__init__('lidar_avoidance_pid_node')

        # ---------------------------
        # Parameters
        # ---------------------------
        # Front sector angle (± in degrees) used to decide if "forward" is safe.
        self.declare_parameter('front_angle_deg', 10.0)
        # Minimum allowed distance in the front sector; closer => avoid
        self.declare_parameter('safe_distance', 0.5)
        # Desired maximum forward speed (m/s)
        self.declare_parameter('forward_speed', 0.25)
        # Maximum allowed turn speed (rad/s)
        self.declare_parameter('max_turn_speed', 0.6)

        # PID gains for angular control
        self.declare_parameter('pid_kp', 1.5)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.1)

        # Acceleration limits (for velocity ramping)
        self.declare_parameter('max_lin_accel', 0.4)   # m/s^2
        self.declare_parameter('max_ang_accel', 1.0)   # rad/s^2

        # Number of sectors for 360° classification (e.g., 8 sectors -> 45° each)
        self.declare_parameter('num_sectors', 8)

        # Read parameters
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.max_turn_speed = float(self.get_parameter('max_turn_speed').value)

        self.kp = float(self.get_parameter('pid_kp').value)
        self.ki = float(self.get_parameter('pid_ki').value)
        self.kd = float(self.get_parameter('pid_kd').value)

        self.max_lin_accel = float(self.get_parameter('max_lin_accel').value)
        self.max_ang_accel = float(self.get_parameter('max_ang_accel').value)

        self.num_sectors = int(self.get_parameter('num_sectors').value)

        # ---------------------------
        # Internal state for PID & ramping
        # ---------------------------
        self.angular_error_integral = 0.0   # Integral term for PID
        self.last_angular_error = 0.0       # For derivative term
        self.last_time = self.get_clock().now()

        # Last commanded velocities (for ramping)
        self.last_cmd_lin_x = 0.0
        self.last_cmd_ang_z = 0.0

        # ---------------------------
        # Publishers & Subscribers
        # ---------------------------
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Marker for RViz: visualize front sector
        self.marker_pub = self.create_publisher(Marker, 'front_sector_marker', 10)

        self.get_logger().info("LidarAvoidancePID node started with:")
        self.get_logger().info(f"  front_angle_deg={self.front_angle_deg}")
        self.get_logger().info(f"  safe_distance={self.safe_distance}")
        self.get_logger().info(f"  forward_speed={self.forward_speed}")
        self.get_logger().info(f"  max_turn_speed={self.max_turn_speed}")
        self.get_logger().info(f"  PID gains: kp={self.kp}, ki={self.ki}, kd={self.kd}")
        self.get_logger().info(f"  accel limits: lin={self.max_lin_accel}, ang={self.max_ang_accel}")
        self.get_logger().info(f"  num_sectors={self.num_sectors}")

    # -------------------------------------------------------------------------
    # Main callback: process LaserScan
    # -------------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        """
        Called every time a new LaserScan message arrives.

        Steps:
        1. Compute dt since last callback (for PID & ramping).
        2. Convert ranges to numpy and filter invalid readings.
        3. Perform 360° sector classification.
        4. Compute front clearance (within ±front_angle_deg).
        5. Decide desired heading (angle) and desired linear speed.
        6. Use PID to compute desired angular velocity.
        7. Apply velocity ramping (acceleration limiting).
        8. Publish cmd_vel and RViz marker.
        """

        # ---------------------------------------------------------------------
        # 1. Time delta calculation
        # ---------------------------------------------------------------------
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-3  # avoid divide-by-zero; assume small dt
        self.last_time = now

        # ---------------------------------------------------------------------
        # 2. Convert ranges to numpy & filter
        # ---------------------------------------------------------------------
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment

        # Extract only finite measurements for global checks
        finite_global = np.isfinite(ranges)
        if not np.any(finite_global):
            # No valid readings? Stop the robot for safety.
            self.get_logger().warn("No valid LIDAR readings; stopping.")
            self.publish_cmd_safely(0.0, 0.0, dt)
            return

        # ---------------------------------------------------------------------
        # 3. 360° sector classification
        # ---------------------------------------------------------------------
        sector_info, best_sector_angle = self.classify_sectors(
            ranges,
            angle_min,
            angle_max,
            angle_inc,
            self.num_sectors
        )

        # Optional debug: log min distance per sector
        # (Comment this out if it's too chatty)
        debug_str = "Sectors (min dist each, meters): "
        debug_str += " | ".join(
            [f"{i}: {d['min_dist']:.2f}" for i, d in enumerate(sector_info)]
        )
        self.get_logger().debug(debug_str)

        # ---------------------------------------------------------------------
        # 4. Front clearance (within ±front_angle_deg)
        # ---------------------------------------------------------------------
        front_angle_rad = math.radians(self.front_angle_deg)

        # Compute indices corresponding to the front sector around 0 rad
        idx_front_min = int((0.0 - front_angle_rad - angle_min) / angle_inc)
        idx_front_max = int((0.0 + front_angle_rad - angle_min) / angle_inc)
        idx_front_min = max(0, idx_front_min)
        idx_front_max = min(len(ranges) - 1, idx_front_max)

        front_ranges = ranges[idx_front_min:idx_front_max + 1]
        finite_front = np.isfinite(front_ranges)

        if np.any(finite_front):
            front_clearance = float(np.min(front_ranges[finite_front]))
        else:
            # If no valid front readings, be conservative
            front_clearance = float('inf')

        self.get_logger().info(f"Front clearance: {front_clearance:.2f} m")

        # ---------------------------------------------------------------------
        # 5. Decide desired heading & linear speed
        # ---------------------------------------------------------------------
        # desired_heading: angle (radians) we want the robot to face.
        # 0 rad means "straight ahead".
        if front_clearance > self.safe_distance:
            # Enough space in front -> prefer going straight (0 rad)
            desired_heading = 0.0
        else:
            # Not enough space ahead -> turn toward best sector (most open)
            desired_heading = best_sector_angle
            self.get_logger().warn(
                f"Front blocked (d={front_clearance:.2f} m). "
                f"Turning toward sector at angle {math.degrees(desired_heading):.1f} deg"
            )

        # Decide desired forward speed:
        # - if front is blocked, stop linear movement
        # - if turning a lot, reduce speed
        angle_error = desired_heading  # we want to be at 0 rad
        angle_error_abs = abs(angle_error)

        if front_clearance <= self.safe_distance:
            desired_linear_speed = 0.0
        else:
            # If angle error is small, go faster; if big, go slower
            if angle_error_abs < math.radians(10.0):
                desired_linear_speed = self.forward_speed
            elif angle_error_abs < math.radians(30.0):
                desired_linear_speed = self.forward_speed * 0.5
            else:
                desired_linear_speed = 0.0

        # ---------------------------------------------------------------------
        # 6. PID control for angular velocity
        # ---------------------------------------------------------------------
        desired_angular_speed = self.pid_control(angle_error, dt)
        # Saturate PID output to max_turn_speed
        desired_angular_speed = self.clamp(
            desired_angular_speed, -self.max_turn_speed, self.max_turn_speed
        )

        # ---------------------------------------------------------------------
        # 7. Velocity ramping (acceleration limiting)
        # ---------------------------------------------------------------------
        lin_x_cmd, ang_z_cmd = self.apply_velocity_ramping(
            desired_linear_speed, desired_angular_speed, dt
        )

        # ---------------------------------------------------------------------
        # 8. Publish cmd_vel & RViz marker
        # ---------------------------------------------------------------------
        self.publish_cmd(lin_x_cmd, ang_z_cmd)
        self.publish_front_sector_marker(front_angle_rad)

    # -------------------------------------------------------------------------
    # PID Controller
    # -------------------------------------------------------------------------
    def pid_control(self, error: float, dt: float) -> float:
        """
        Simple PID controller for angular error -> angular velocity.

        error: desired_heading - current_heading (but we assume current_heading = 0).
        dt: time step in seconds.

        Returns:
            angular velocity command (before saturation).
        """
        # Proportional
        p_term = self.kp * error

        # Integral with simple accumulation. Can add clamping if needed.
        self.angular_error_integral += error * dt
        i_term = self.ki * self.angular_error_integral

        # Derivative
        d_error = (error - self.last_angular_error) / dt
        d_term = self.kd * d_error

        # Save for next iteration
        self.last_angular_error = error

        angular_cmd = p_term + i_term + d_term
        return angular_cmd

    # -------------------------------------------------------------------------
    # Velocity ramping / acceleration limiting
    # -------------------------------------------------------------------------
    def apply_velocity_ramping(self, target_lin: float, target_ang: float, dt: float):
        """
        Gradually move from last commanded velocity to target velocity
        respecting maximum linear and angular acceleration.

        This avoids sudden jumps in speed.
        """
        # How much we can change this cycle
        max_lin_delta = self.max_lin_accel * dt
        max_ang_delta = self.max_ang_accel * dt

        # Ramping for linear velocity
        lin_delta = target_lin - self.last_cmd_lin_x
        lin_delta = self.clamp(lin_delta, -max_lin_delta, max_lin_delta)
        new_lin = self.last_cmd_lin_x + lin_delta

        # Ramping for angular velocity
        ang_delta = target_ang - self.last_cmd_ang_z
        ang_delta = self.clamp(ang_delta, -max_ang_delta, max_ang_delta)
        new_ang = self.last_cmd_ang_z + ang_delta

        # Save for next iteration
        self.last_cmd_lin_x = new_lin
        self.last_cmd_ang_z = new_ang

        return new_lin, new_ang

    # -------------------------------------------------------------------------
    # Sector classification (full 360°)
    # -------------------------------------------------------------------------
    def classify_sectors(self, ranges, angle_min, angle_max, angle_inc, num_sectors):
        """
        Divide the 360° (or scan range) into num_sectors angular sectors.

        For each sector, compute:
        - min_dist: minimum valid distance
        - avg_dist: average valid distance

        Returns:
            sector_info: list of dicts with per-sector info
            best_sector_angle: center angle of sector with largest avg_dist
                              (most open direction), in radians.
        """
        sector_info = []

        # Angular width of each sector
        total_angle = angle_max - angle_min
        sector_width = total_angle / float(num_sectors)

        best_avg_dist = -1.0
        best_sector_angle = 0.0

        for i in range(num_sectors):
            # Sector angular bounds
            sector_start = angle_min + i * sector_width
            sector_end = sector_start + sector_width

            # Convert to index
            idx_start = int((sector_start - angle_min) / angle_inc)
            idx_end = int((sector_end - angle_min) / angle_inc)

            idx_start = max(0, idx_start)
            idx_end = min(len(ranges) - 1, idx_end)

            sector_ranges = ranges[idx_start:idx_end + 1]
            finite_mask = np.isfinite(sector_ranges)

            if np.any(finite_mask):
                valid = sector_ranges[finite_mask]
                min_dist = float(np.min(valid))
                avg_dist = float(np.mean(valid))
            else:
                # If no valid measurements, treat as "very bad"
                min_dist = 0.0
                avg_dist = 0.0

            # Center angle of this sector
            center_angle = (sector_start + sector_end) / 2.0

            # Normalize angle to [-pi, pi] (optional, but useful)
            center_angle = self.normalize_angle(center_angle)

            sector_info.append({
                "index": i,
                "start_angle": sector_start,
                "end_angle": sector_end,
                "center_angle": center_angle,
                "min_dist": min_dist,
                "avg_dist": avg_dist,
            })

            # Track sector with largest average distance
            if avg_dist > best_avg_dist:
                best_avg_dist = avg_dist
                best_sector_angle = center_angle

        return sector_info, best_sector_angle

    # -------------------------------------------------------------------------
    # RViz marker (front sector visualization)
    # -------------------------------------------------------------------------
    def publish_front_sector_marker(self, front_angle_rad):
        """
        Publish a LINE_STRIP marker to visualize the front sector (± front_angle_rad)
        in the laser frame. This shows in RViz where we're checking for obstacles.
        """
        marker = Marker()
        marker.header.frame_id = "laser"  # adapt to your TF tree if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "front_sector"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Thickness of the line
        marker.scale.x = 0.03

        # Color (RGBA)
        marker.color.a = 0.9
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.1

        marker.points = []

        # Draw arc for front sector
        num_points = 30
        for angle in np.linspace(-front_angle_rad, front_angle_rad, num_points):
            p = Point()
            # Just draw unit-radius arc; purely visual
            p.x = math.cos(angle)
            p.y = math.sin(angle)
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)

    # -------------------------------------------------------------------------
    # Command publishing helpers
    # -------------------------------------------------------------------------
    def publish_cmd(self, lin_x, ang_z):
        """
        Publish a Twist message with the given linear and angular velocities.
        """
        twist = Twist()
        twist.linear.x = float(lin_x)
        twist.angular.z = float(ang_z)
        self.cmd_pub.publish(twist)

    def publish_cmd_safely(self, lin_x, ang_z, dt):
        """
        Publish a command with ramping even when we want to force stop
        (e.g., no readings). This avoids big jumps if we resume later.
        """
        lin_x_cmd, ang_z_cmd = self.apply_velocity_ramping(lin_x, ang_z, dt)
        self.publish_cmd(lin_x_cmd, ang_z_cmd)

    # -------------------------------------------------------------------------
    # Utility functions
    # -------------------------------------------------------------------------
    @staticmethod
    def clamp(value, vmin, vmax):
        return max(vmin, min(value, vmax))

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


# -----------------------------------------------------------------------------
# Main entry point
# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidancePID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
