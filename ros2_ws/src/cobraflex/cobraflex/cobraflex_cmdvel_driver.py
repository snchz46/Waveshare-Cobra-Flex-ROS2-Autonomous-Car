#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json


class CobraFlexDriver(Node):
    """
    Node based on your original template, adapted to
    convert /cmd_vel -> JSON for the Cobraflex.
    """

    def __init__(self):
        super().__init__("cobraflex_cmdvel_driver")

        # -----------------------------
        # Configurable parameters
        # -----------------------------
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_speed_value", 100)   # range [0–100]
        self.declare_parameter("max_linear", 1.0)        # m/s
        self.declare_parameter("max_angular", 1.0)       # rad/s

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        # Normalization scales
        self.max_speed_val = float(self.get_parameter("max_speed_value").value)
        self.max_lin = float(self.get_parameter("max_linear").value)
        self.max_ang = float(self.get_parameter("max_angular").value)

        # -----------------------------
        # Open serial port
        # -----------------------------
        self.ser = serial.Serial(port, baud, timeout=0.05)

        # -----------------------------
        # Subscribe to /cmd_vel
        # -----------------------------
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.get_logger().info("Cobraflex driver initialized and listening on /cmd_vel.")


    # ---------------------------------------------------------------------
    # Velocity normalization
    # ---------------------------------------------------------------------
    def scale_to_motor(self, lin_x, ang_z):
        """
        Convert ROS 2 velocities -> Cobraflex scale.
        lin_x: m/s
        ang_z: rad/s

        Returns L and R within [-max_speed_val, max_speed_val].
        """

        # Normalize linear and angular inputs
        lin_norm = max(-1.0, min(1.0, lin_x / self.max_lin))
        ang_norm = max(-1.0, min(1.0, ang_z / self.max_ang))

        # Differential mix (tank-drive style)
        left = lin_norm - ang_norm
        right = lin_norm + ang_norm

        # Normalize mix to [-1, 1]
        max_abs = max(abs(left), abs(right), 1)
        left /= max_abs
        right /= max_abs

        # Scale to Cobraflex range
        L_val = int(left * self.max_speed_val)
        R_val = int(right * self.max_speed_val)

        return L_val, R_val


    # ---------------------------------------------------------------------
    # /cmd_vel callback
    # ---------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        lin = msg.linear.x
        ang = msg.angular.z

        # Convert velocities
        L, R = self.scale_to_motor(lin, ang)

        # Build final JSON according to the template
        data = {
            "T": 1,
            "L": L,
            "R": R
        }

        json_data = (json.dumps(data) + "\n").encode("utf-8")

        try:
            self.ser.write(json_data)
            self.get_logger().info(f"Sent: {json_data}")
        except Exception as e:
            self.get_logger().error(f"ERROR sending to the Cobraflex: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CobraFlexDriver()

    # Matches your template: spin to ensure timers / callbacks run
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
