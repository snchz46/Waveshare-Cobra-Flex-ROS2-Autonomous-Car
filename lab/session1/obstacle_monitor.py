"""
Lab session 1 - report the nearest obstacle that the LiDAR sees.

Node name: obstacle_monitor.
Input: /scan (sensor_msgs/LaserScan).
Output: /obstacle_distance (std_msgs/Float32), one value per scan, and
a log line once per second with the distance and the scan angle of the
nearest obstacle.
Parameter: warn_distance [m]. Closer obstacles are logged as warnings.

The five gaps, marked F to J, are completed as described in Part 5 of
the workbook; ``ros2 interface show`` lists the fields of a message.
While a gap remains open, Python stops with an error that names it. The
file is copied into the package cobraflex_lab (Part 2) and run from
there:

    ros2 run cobraflex_lab obstacle_monitor --ros-args -p use_sim_time:=true
    ros2 run cobraflex_lab obstacle_monitor       # on the car
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class ObstacleMonitor(Node):
    """Find the nearest valid range of every scan."""

    def __init__(self):
        super().__init__('obstacle_monitor')

        # Obstacles closer than this are logged as warnings [m]
        self.declare_parameter('warn_distance', 0.5)

        # Sensor data QoS (best effort): a lost scan is not sent again,
        # the next one is already on its way.
        self.create_subscription(
            ___F___, '/scan', self.on_scan,  # gap F: message type of /scan
            qos_profile_sensor_data)
        self.distance_pub = self.create_publisher(
            Float32, ___G___, 10)  # gap G: output topic (string)

    def on_scan(self, scan):
        """Publish and log the nearest obstacle of one scan."""
        nearest = math.inf
        nearest_angle = 0.0
        for i, r in enumerate(scan.ranges):
            # Valid readings lie within the sensor limits; inf and nan do not
            # gap H: field of the scan with the shortest valid range
            if scan.___H___ <= r <= scan.range_max and r < nearest:
                nearest = r
                # Ray i points at angle_min + i * (angle between two rays)
                # gap I: field of the scan with the angle between two rays
                nearest_angle = scan.angle_min + i * scan.___I___

        if math.isinf(nearest):
            self.get_logger().info(
                'no obstacle in range', throttle_duration_sec=1.0)
            return

        # gap J: name of the only field of std_msgs/msg/Float32
        self.distance_pub.publish(Float32(___J___=nearest))

        text = (f'nearest obstacle {nearest:.2f} m '
                f'at {math.degrees(nearest_angle):.0f} deg')
        if nearest < self.get_parameter('warn_distance').value:
            self.get_logger().warning(text, throttle_duration_sec=1.0)
        else:
            self.get_logger().info(text, throttle_duration_sec=1.0)


def main(args=None):
    """Run the node until Ctrl-C."""
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleMonitor()
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
