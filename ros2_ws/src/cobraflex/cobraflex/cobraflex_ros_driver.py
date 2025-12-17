#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
import json
import serial
import sys
import signal

class CobraFlexROSDriver(Node):
    def __init__(self):
        super().__init__("cobraflex_ros_driver")

        # Parameters
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_linear", 0.53)
        self.declare_parameter("max_angular", 6.0)
        self.declare_parameter("turn_threshold", 0.3)   # NEW: configurable turn threshold

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)

        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.turn_threshold = float(self.get_parameter("turn_threshold").value)

        # Last cmd_vel saved for stable command output
        self.last_vx = 0.0
        self.last_wz = 0.0

        # Connect serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            self.get_logger().info(f"Connected to {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            sys.exit(1)

        # ROS publishers for feedback
        self.feedback_pub = self.create_publisher(String, "/cobraflex/feedback", 10)
        self.battery_pub = self.create_publisher(Float32, "/cobraflex/battery", 10)
        self.wheels_pub = self.create_publisher(Twist, "/cobraflex/wheel_speeds", 10)

        # Subscriber
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

        # Activate headlights ON boot
        self.turn_lights(True, True)

        # Timers: read serial and resend last cmd_vel
        self.read_timer = self.create_timer(0.02, self.read_serial)   # 50 Hz read
        self.cmd_timer = self.create_timer(0.05, self.resend_last_cmd)  # 20 Hz resend

        self.register_signal_handlers()

        # Ask chasis to start sending feedback stream
        self.enable_feedback_stream()

    # ---------------- HEADLIGHT CONTROL ----------------

    def turn_lights(self, left=True, right=True):
        L = 255 if left else 0
        R = 255 if right else 0
        msg = {"T":132, "IO1":L, "IO2":R}
        self.send_json(msg)

    def update_lights(self, wz):
        if wz > self.turn_threshold:
            self.turn_lights(True, False)
        elif wz < -self.turn_threshold:
            self.turn_lights(False, True)
        else:
            self.turn_lights(True, True)

    # ---------------- COMMAND OUTPUT ----------------

    def cmd_callback(self, msg):
        vx = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        wz = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        self.last_vx = vx
        self.last_wz = wz

        self.update_lights(wz)

    def resend_last_cmd(self):
        """Send last known command at stable rate."""
        data = {"T":13, "X":self.last_vx, "Z":self.last_wz}
        self.send_json(data)

    def stop_robot(self):
        self.last_vx = 0.0
        self.last_wz = 0.0
        msg = {"T":13, "X":0.0, "Z":0.0}
        self.send_json(msg)

    # ---------------- SERIAL WRITE ----------------

    def send_json(self, data):
        try:
            line = (json.dumps(data) + "\n").encode("utf-8")
            self.ser.write(line)
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ---------------- ENABLE FEEDBACK STREAM ----------------

    def enable_feedback_stream(self):
        """Tells chassis to push continuous feedback (T=131)."""
        self.send_json({"T":131, "cmd":1})
        self.get_logger().info("Requested feedback stream (T=131)")

    # ---------------- SERIAL READER ----------------

    def read_serial(self):
        try:
            raw = self.ser.readline().decode("utf-8").strip()
            if not raw:
                return

            data = json.loads(raw)

            # Publish raw data for debug
            self.feedback_pub.publish(String(data=json.dumps(data)))

            T_code = data.get("T", -1)

            # ---------------- FEEDBACK BASE INFO (1001) ----------------
            if T_code == 1001:
                battery = float(data.get("v", 0.0))
                self.battery_pub.publish(Float32(data=battery))

                twist = Twist()
                twist.linear.x = float(data.get("odl", 0.0))
                twist.linear.y = float(data.get("odr", 0.0))
                self.wheels_pub.publish(twist)

            # ---------------- IMU DATA (1002) ----------------
            elif T_code == 1002:
                # You can publish into IMU topic here  
                pass

        except Exception as e:
            self.get_logger().warn(f"Serial parse error: {e}")

    # ---------------- SIGNAL HANDLERS ----------------

    def register_signal_handlers(self):
        def handler(signum, frame):
            self.get_logger().info("CTRL+C → STOP robot + lights OFF")
            self.stop_robot()
            self.turn_lights(False, False)
            self.ser.close()
            sys.exit(0)

        signal.signal(signal.SIGINT, handler)

    # ---------------- CLEANUP ----------------

    def destroy_node(self):
        self.get_logger().info("Stopping robot before shutdown...")
        self.stop_robot()
        try:
            self.ser.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CobraFlexROSDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
