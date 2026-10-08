#!/usr/bin/env python3
"""Simple LiDAR-based obstacle avoidance and corridor centering."""

import math

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
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
        # Yaw of the robot forward axis in the angle frame of the scan. 180 deg
        # corresponds to a LiDAR mounted rotated by half a turn, as on this
        # platform (lidar_joint with rpy yaw = pi).
        self.declare_parameter("front_offset_deg", 180.0)
        # Deadman timer: stop when /scan stops. Without it, the command timer
        # keeps publishing the last decision, possibly full forward speed,
        # after a LiDAR disconnection or driver failure.
        self.declare_parameter("scan_timeout", 0.5)

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
        self.front_offset_rad = math.radians(
            float(self.get_parameter("front_offset_deg").value)
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)

        self.state = "FORWARD"
        self.state_enter_time = self.get_clock().now()
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.last_scan_time = None
        self.scan_expired = False
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

    @staticmethod
    def _sector_min(msg, ranges, start_ang, end_ang):
        """Return the closest valid range in an angular sector.

        Angles are relative to the robot forward axis; the caller has already
        added `front_offset_rad`.

        * **Wrap-around.** Indices are taken modulo the ray count, so a sector
          across the seam of the scan (the forward direction on this robot,
          with the LiDAR mounted rotated by 180 deg) includes the rays on both
          sides. Clamping would reduce any out-of-range sector to the last ray
          and yield a constant distance. The wrap-around is valid only for a
          full-circle scanner; the caller verifies this.
        * **Minimum instead of mean.** A mean over the sector suppresses narrow
          obstacles: a table leg two rays wide disappears against the
          surrounding free space.
        """
        n = ranges.size
        if n == 0:
            return float(msg.range_max)

        i0 = int(math.floor((start_ang - msg.angle_min) / msg.angle_increment))
        i1 = int(math.ceil((end_ang - msg.angle_min) / msg.angle_increment))
        if i1 < i0:
            i0, i1 = i1, i0

        values = ranges[np.arange(i0, i1 + 1) % n]
        valid = values[
            np.isfinite(values)
            & (values >= msg.range_min)
            & (values <= msg.range_max)
        ]

        if valid.size == 0:
            # No valid return in the sector: treated as free.
            return float(msg.range_max)

        return float(valid.min())

    def _scan_callback(self, msg):
        """Classify the latest scan into front/left/right minima and decide avoidance."""
        now = self.get_clock().now()
        self.last_scan_time = now

        if self.scan_expired:
            self.scan_expired = False
            self.get_logger().info("/scan recovered, resuming avoidance")

        ranges = np.asarray(msg.ranges, dtype=float)

        # The modulo wrap-around in _sector_min requires a full-circle scanner.
        # With a narrower field of view (bumper LiDAR, cropped scan) the front
        # sector would wrap onto the opposite edge of the field of view.
        span = msg.angle_increment * ranges.size
        if span < 1.9 * math.pi:
            self.get_logger().warning(
                f"Scan spans {math.degrees(span):.0f} deg, not a full circle: "
                "this node assumes a 360 deg lidar, refusing to drive",
                throttle_duration_sec=5.0,
            )
            self.target_lin = 0.0
            self.target_ang = 0.0
            return

        front_rad = math.radians(self.front_angle_deg)
        side_rad = math.radians(self.side_sample_deg)
        off = self.front_offset_rad

        front = self._sector_min(msg, ranges, off - front_rad, off + front_rad)
        left = self._sector_min(msg, ranges, off + side_rad * 0.5, off + side_rad)
        right = self._sector_min(msg, ranges, off - side_rad, off - side_rad * 0.5)

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
        """Choose and latch the avoidance manoeuvre for a detected obstacle."""
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

    def _scan_is_stale(self):
        """True when no scan arrived within `scan_timeout` seconds."""
        if self.scan_timeout <= 0.0 or self.last_scan_time is None:
            return False

        age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        return age > self.scan_timeout

    def _cmd_timer_cb(self):
        """Publish the current (cruise or avoidance) command at the control rate."""
        if self._scan_is_stale():
            if not self.scan_expired:
                self.scan_expired = True
                self.get_logger().warning(
                    f"No /scan for {self.scan_timeout:.2f} s, stopping"
                )

            self.target_lin = 0.0
            self.target_ang = 0.0

        twist = Twist()
        twist.linear.x = float(self.target_lin)
        twist.angular.z = float(self.target_ang)
        self.cmd_pub.publish(twist)

    def _publish_zero(self):
        """Publish a zero Twist (stop)."""
        self.cmd_pub.publish(Twist())

    def destroy_node(self):
        """Stop the robot before shutdown."""
        try:
            self._publish_zero()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    """Run the legacy LiDAR avoidance node."""
    rclpy.init(args=args)
    node = None
    try:
        node = AvoidanceWithLights()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Under `ros2 launch`, Ctrl-C arrives as ExternalShutdownException
        # instead of KeyboardInterrupt; catching it gives exit status 0.
        pass
    finally:
        if node is not None:
            # Publishes a zero Twist before tearing the publisher down.
            node.destroy_node()
        # After an external shutdown the context is already down; a second
        # shutdown() call raises an exception.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
