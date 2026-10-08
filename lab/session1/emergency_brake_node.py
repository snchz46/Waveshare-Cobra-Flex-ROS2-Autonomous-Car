#!/usr/bin/env python3
"""Lab session 1 - emergency braking at constant cruise speed.

Input: /scan (sensor_msgs/LaserScan). Output: /cmd_vel (geometry_msgs/Twist).

The eight gaps, marked A to H, are completed as described in Part 3 of
the workbook. While a gap remains open, Python stops with an error that
names it. Execution, with ROS 2 sourced:

    python3 emergency_brake_node.py                 # physical robot
    python3 emergency_brake_node.py --ros-args -p use_sim_time:=true

Parameters are set at start-up in the same way, for example
``--ros-args -p cruise_speed:=0.4 -p stop_distance:=0.5``. Values are
written as decimal numbers (0.4, 1.0) because the parameters are
declared as floats.

Ctrl-C stops the node, which publishes a zero command before exiting.
"""

import math

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

SCAN_TOPIC = ___A___        # LiDAR topic (string)
CMD_TOPIC = ___B___         # velocity command topic of the driver or Gazebo
FRONT_OFFSET_DEG = ___H___  # scan angle of the forward direction


def front_distance(scan, front_offset_rad, half_width_rad):
    """Return the closest valid range in the forward sector [m]."""
    ranges = np.asarray(scan.ranges, dtype=float)
    n = ranges.size
    if n == 0:
        return float(scan.range_max)

    # Ray i points at angle_min + i * angle_increment.
    # Indices of the first and last ray of the sector.
    start = front_offset_rad - half_width_rad - scan.angle_min
    end = front_offset_rad + half_width_rad - scan.angle_min
    i0 = int(math.floor(start / scan.angle_increment))
    i1 = int(math.ceil(end / scan.angle_increment))

    # The forward direction may lie at the wrap-around between the last
    # and the first ray; indices are therefore taken modulo n.
    sector = ranges[np.arange(i0, i1 + 1) % n]

    # Valid measurements only: finite and within the sensor range.
    valid = sector[
        np.isfinite(sector)
        & (sector >= scan.range_min)
        & (sector <= scan.range_max)
    ]
    if valid.size == 0:
        return float(scan.range_max)  # no return: sector is free

    return float(valid.___D___())  # gap D: reduction to a single distance


class EmergencyBrake(Node):
    """Drive at constant speed and brake below the stop distance."""

    def __init__(self):
        super().__init__('emergency_brake')

        # Cruise speed [m/s], stop distance from the LiDAR [m],
        # half opening angle of the forward sector [deg], deadman [s].
        self.declare_parameter('cruise_speed', 0.20)
        self.declare_parameter('stop_distance', 0.40)
        self.declare_parameter('front_half_width_deg', 15.0)
        self.declare_parameter('scan_timeout', 0.5)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        half_width_deg = self.get_parameter('front_half_width_deg').value
        self.half_width = math.radians(half_width_deg)
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.front_offset = math.radians(FRONT_OFFSET_DEG)

        self.speed = 0.0  # commanded speed; zero until the first scan
        self.braking = False
        self.last_scan_time = None

        self.create_subscription(
            ___C___, SCAN_TOPIC, self.on_scan,  # gap C: message type
            qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.front_pub = self.create_publisher(
            Float32, '/emergency_brake/front_distance', 10)
        self.create_timer(0.05, self.on_timer)  # 20 Hz

        self.get_logger().info(
            f'cruise {self.cruise_speed:.2f} m/s, '
            f'brake below {self.stop_distance:.2f} m')

    def on_scan(self, msg):
        """Set the commanded speed from the latest scan."""
        self.last_scan_time = self.get_clock().now()
        front = front_distance(msg, self.front_offset, self.half_width)
        self.front_pub.publish(Float32(data=front))

        too_close = ___E___  # gap E: True below the stop distance
        if too_close:
            if not self.braking:
                self.get_logger().warning(
                    f'BRAKE: obstacle at {front:.2f} m')
            self.braking = True
            self.speed = 0.0
        else:
            self.braking = False
            self.speed = self.cruise_speed

    def on_timer(self):
        """Publish the command at 20 Hz; stop when /scan times out."""
        if self.last_scan_time is not None:
            age = self.get_clock().now() - self.last_scan_time
            if age.nanoseconds / 1e9 > self.scan_timeout:
                self.speed = ___F___  # gap F: speed without a recent scan
        cmd = Twist()
        cmd.___G___ = self.speed  # gap G: forward velocity field
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        """Publish a zero command before the node is destroyed."""
        try:
            self.cmd_pub.publish(Twist())  # all fields zero
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Run the node until Ctrl-C."""
    rclpy.init(args=args)
    node = None
    try:
        node = EmergencyBrake()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
