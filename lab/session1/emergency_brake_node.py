#!/usr/bin/env python3
"""Lab session 1 - emergency braking: drive straight, stop in time.

/scan (sensor_msgs/LaserScan) in, /cmd_vel (geometry_msgs/Twist) out.

Fill the eight gaps, marked A to H in the code, as described in the
workbook (Part 3). Until every gap is filled, Python stops with an
error that names the first gap still open. Then run, with ROS 2
sourced:

    python3 emergency_brake_node.py                 # on the car
    python3 emergency_brake_node.py --ros-args -p use_sim_time:=true

Parameters are changed at start-up the same way, for example
``--ros-args -p cruise_speed:=0.4 -p stop_distance:=0.5``. Write them
as decimal numbers (0.4, 1.0), because they are declared as floats.

Ctrl-C stops the node; it sends a zero command before it exits.
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

SCAN_TOPIC = ___A___        # topic the LiDAR publishes on (a string)
CMD_TOPIC = ___B___         # topic the driver (or Gazebo) listens to
FRONT_OFFSET_DEG = ___H___  # scan angle that points straight ahead


def front_distance(scan, front_offset_rad, half_width_rad):
    """Closest valid return in the sector straight ahead [m]."""
    ranges = np.asarray(scan.ranges, dtype=float)
    n = ranges.size
    if n == 0:
        return float(scan.range_max)

    # Ray i points at the angle  angle_min + i * angle_increment.
    # First and last ray of the sector:
    start = front_offset_rad - half_width_rad - scan.angle_min
    end = front_offset_rad + half_width_rad - scan.angle_min
    i0 = int(math.floor(start / scan.angle_increment))
    i1 = int(math.ceil(end / scan.angle_increment))

    # Straight ahead may sit where the scan wraps around from its last
    # ray back to its first, so the ray numbers are taken modulo n.
    sector = ranges[np.arange(i0, i1 + 1) % n]

    # Keep real measurements only: finite and inside the sensor's range.
    valid = sector[
        np.isfinite(sector)
        & (sector >= scan.range_min)
        & (sector <= scan.range_max)
    ]
    if valid.size == 0:
        return float(scan.range_max)  # nothing seen: sector is clear

    return float(valid.___D___())  # gap D: one number for the sector


class EmergencyBrake(Node):
    """Cruise at constant speed; brake when an obstacle is too close."""

    def __init__(self):
        super().__init__('emergency_brake')

        # cruise speed [m/s], stop distance measured from the LiDAR [m],
        # half the opening of the front sector [deg], deadman [s]
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

        self.speed = 0.0  # what the timer sends; 0 until a scan
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
        """Decide the speed from the newest scan."""
        self.last_scan_time = self.get_clock().now()
        front = front_distance(msg, self.front_offset, self.half_width)
        self.front_pub.publish(Float32(data=front))

        too_close = ___E___  # gap E: True if the obstacle is too close
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
        """Send the decision at 20 Hz; stop if /scan went quiet."""
        if self.last_scan_time is not None:
            age = self.get_clock().now() - self.last_scan_time
            if age.nanoseconds / 1e9 > self.scan_timeout:
                self.speed = ___F___  # gap F: no fresh scan, so ...
        cmd = Twist()
        cmd.___G___ = self.speed  # gap G: the field that drives forward
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        """Stop the car before the node goes away."""
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
