#!/usr/bin/env python3
"""ROS 2 serial driver for the CobraFlex chassis."""

import json

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32, String


class CobraFlexROSDriver(Node):
    """Bridge `/cmd_vel` to the CobraFlex JSON serial protocol."""

    def __init__(self):
        """Initialize parameters, ROS I/O and the serial connection."""
        super().__init__("cobraflex_ros_driver")

        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_linear", 0.53)
        self.declare_parameter("max_angular", 6.0)
        self.declare_parameter("turn_threshold", 0.3)

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)

        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.turn_threshold = float(self.get_parameter("turn_threshold").value)

        self.last_vx = 0.0
        self.last_wz = 0.0
        self.ser = None

        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
        except Exception as exc:
            raise RuntimeError(f"Failed to open serial port {port}: {exc}") from exc

        self.get_logger().info(f"Connected to {port}")

        self.feedback_pub = self.create_publisher(String, "/cobraflex/feedback", 10)
        self.battery_pub = self.create_publisher(Float32, "/cobraflex/battery", 10)
        self.wheels_pub = self.create_publisher(Twist, "/cobraflex/wheel_speeds", 10)

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._cmd_callback,
            10,
        )

        self._turn_lights(True, True)
        self.read_timer = self.create_timer(0.02, self._read_serial)
        self.cmd_timer = self.create_timer(0.05, self._resend_last_cmd)
        self._enable_feedback_stream()

    def _turn_lights(self, left=True, right=True):
        left_value = 255 if left else 0
        right_value = 255 if right else 0
        self._send_json({"T": 132, "IO1": left_value, "IO2": right_value})

    def _update_lights(self, wz):
        if wz > self.turn_threshold:
            self._turn_lights(True, False)
        elif wz < -self.turn_threshold:
            self._turn_lights(False, True)
        else:
            self._turn_lights(True, True)

    def _cmd_callback(self, msg):
        vx = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        wz = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        self.last_vx = vx
        self.last_wz = wz
        self._update_lights(wz)

    def _resend_last_cmd(self):
        self._send_json({"T": 13, "X": self.last_vx, "Z": self.last_wz})

    def _stop_robot(self):
        self.last_vx = 0.0
        self.last_wz = 0.0
        self._send_json({"T": 13, "X": 0.0, "Z": 0.0})

    def _send_json(self, data):
        if self.ser is None:
            return

        try:
            line = (json.dumps(data) + "\n").encode("utf-8")
            self.ser.write(line)
        except Exception as exc:
            self.get_logger().error(f"Serial write failed: {exc}")

    def _enable_feedback_stream(self):
        self._send_json({"T": 131, "cmd": 1})
        self.get_logger().info("Requested feedback stream (T=131)")

    def _read_serial(self):
        if self.ser is None:
            return

        try:
            raw = self.ser.readline().decode("utf-8").strip()
            if not raw:
                return

            data = json.loads(raw)
            self.feedback_pub.publish(String(data=json.dumps(data)))

            t_code = data.get("T", -1)
            if t_code == 1001:
                battery = float(data.get("v", 0.0))
                self.battery_pub.publish(Float32(data=battery))

                twist = Twist()
                twist.linear.x = float(data.get("odl", 0.0))
                twist.linear.y = float(data.get("odr", 0.0))
                self.wheels_pub.publish(twist)
        except Exception as exc:
            self.get_logger().warning(f"Serial parse error: {exc}")

    def destroy_node(self):
        """Stop the robot and release the serial device on shutdown."""
        try:
            self._stop_robot()
            self._turn_lights(False, False)
        except Exception:
            pass

        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    """Run the CobraFlex serial driver node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CobraFlexROSDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
