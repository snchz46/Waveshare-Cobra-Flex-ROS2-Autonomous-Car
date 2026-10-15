"""
Lab session 1 - drive the Cobra Flex in a circle.

Node name: circle_driver.
Output: /cmd_vel (geometry_msgs/Twist), ten times per second.
Parameters: speed [m/s] and radius [m], read again on every tick, so
``ros2 param set`` changes the circle while the node runs. Values are
written as decimal numbers (0.3, 1.0): the parameters are floats.

The five gaps, marked A to E, are completed as described in Part 4 of
the workbook. While a gap remains open, Python stops with an error that
names it. The file is copied into the package cobraflex_lab (Part 2)
and run from there:

    ros2 run cobraflex_lab circle_driver --ros-args -p use_sim_time:=true
    ros2 run cobraflex_lab circle_driver          # on the car

Ctrl-C stops the node, which sends a zero command before it exits.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class CircleDriver(Node):
    """Publish one velocity command every 0.1 s."""

    def __init__(self):
        super().__init__(___A___)  # gap A: node name (string)

        # Forward speed [m/s] and radius of the circle [m]
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('radius', 0.5)

        # gap B: topic of the velocity command (string)
        self.cmd_pub = self.create_publisher(Twist, ___B___, 10)
        # gap C: period of the timer [s] for ten commands per second
        self.create_timer(___C___, self.on_timer)

        self.get_logger().info('driving in a circle, Ctrl-C stops the car')

    def on_timer(self):
        """Read the parameters and publish one command."""
        speed = self.get_parameter('speed').value
        radius = self.get_parameter('radius').value

        cmd = Twist()  # every field starts at zero
        cmd.___D___ = speed  # gap D: field of the forward speed
        cmd.angular.z = ___E___  # gap E: turn rate [rad/s] on this circle
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        """Stop the car before the node goes away."""
        try:
            self.cmd_pub.publish(Twist())  # all fields zero
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Run the node until Ctrl-C, then stop the car."""
    # Ctrl-C raises KeyboardInterrupt and leaves ROS running, so that the
    # node can still send its stop command. Gazebo keeps the last command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CircleDriver()
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
