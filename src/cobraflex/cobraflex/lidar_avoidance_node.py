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
        # Yaw of the robot's forward axis expressed in the scan's own angle
        # frame. 180 deg means the lidar is mounted rotated half a turn, which
        # is how it sits on this platform (lidar_joint carries rpy yaw = pi).
        self.declare_parameter("front_offset_deg", 180.0)
        # Deadman: stop if /scan goes quiet. Without it the command timer keeps
        # republishing the last decision -- possibly full forward speed -- for
        # ever after the lidar unplugs or its driver dies.
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
        """Closest valid return inside an angular sector, measured from the front.

        Angles are relative to the robot's forward axis; the caller has already
        folded in `front_offset_rad`. Two things this has to get right:

        * **Wrapping.** Indices are taken modulo the ray count, so a sector that
          straddles the seam of the scan (which is where "forward" lands on this
          robot, the lidar being mounted rotated 180 deg) still reads the rays
          on both sides of it. Clamping instead -- the previous behaviour --
          silently collapsed any out-of-range sector onto the single last ray,
          so the left distance was a constant and the turn-direction choice ran
          on it. Only safe because this is a full-circle scanner; the caller
          checks that before relying on it.
        * **Minimum, not mean.** Averaging a sector hides exactly what this node
          exists to detect: a table leg two rays wide averages away against the
          open space around it.
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
            # Nothing valid in the sector: treat as clear, not as an obstacle.
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

        # The modulo wrap in _sector_min is only meaningful on a full-circle
        # scanner. Anything narrower (a bumper lidar, a cropped scan) would
        # wrap the front sector onto the far edge of the field of view.
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
        # Under `ros2 launch` a ctrl-c arrives as ExternalShutdownException,
        # not KeyboardInterrupt; letting it escape made the node exit 1.
        pass
    finally:
        if node is not None:
            # Publishes a zero Twist before tearing the publisher down.
            node.destroy_node()
        # Already down when the shutdown came from outside, and calling it
        # twice raises.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
