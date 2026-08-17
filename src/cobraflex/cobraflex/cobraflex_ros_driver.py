#!/usr/bin/env python3
"""ROS 2 serial driver for the CobraFlex chassis."""

import json

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
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
        # Deadman. `_resend_last_cmd` exists to defeat the firmware's own
        # command timeout, so without this the last velocity is re-sent for
        # ever: if the publisher of /cmd_vel dies, its process is killed or the
        # DDS link drops, the physical robot keeps driving at that velocity
        # until someone cuts the power. Zero the command after this many
        # seconds without a fresh /cmd_vel. Set to 0.0 to disable (bench only).
        self.declare_parameter("cmd_timeout", 0.5)
        # Cap on feedback lines drained per read tick, so a chatty firmware
        # cannot starve the keep-alive timer that shares this executor thread.
        self.declare_parameter("max_lines_per_read", 20)
        # Stiction floor for turning on the spot. The firmware maps a twist to
        # wheel RPM linearly (`rosCtrl` in movtion_module.h) with no deadband
        # compensation of its own, so a small yaw command with zero forward
        # speed asks for a wheel RPM the motors cannot break static friction
        # with: the robot buzzes and does not move, and nothing reports an
        # error. Waveshare's own ugv_bringup lifts such commands to 0.2 rad/s,
        # which is the value used here. Applies ONLY when linear.x is exactly
        # zero, so lane following and the RL policy (both always driving
        # forward) never see it, and a full stop (wz == 0.0) is never lifted.
        # Set to 0.0 to disable.
        self.declare_parameter("min_angular_in_place", 0.2)

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)

        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.turn_threshold = float(self.get_parameter("turn_threshold").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.max_lines_per_read = int(self.get_parameter("max_lines_per_read").value)
        self.min_angular_in_place = float(self.get_parameter("min_angular_in_place").value)

        self.last_vx = 0.0
        self.last_wz = 0.0
        self.last_cmd_time = None
        self.cmd_expired = False
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
        """Toggle the indicator LEDs via the serial JSON protocol."""
        left_value = 255 if left else 0
        right_value = 255 if right else 0
        self._send_json({"T": 132, "IO1": left_value, "IO2": right_value})

    def _update_lights(self, wz):
        """Drive the indicators from the commanded yaw rate."""
        if wz > self.turn_threshold:
            self._turn_lights(True, False)
        elif wz < -self.turn_threshold:
            self._turn_lights(False, True)
        else:
            self._turn_lights(True, True)

    def _lift_in_place_yaw(self, vx, wz):
        """Raise a small pure-rotation yaw command to the stiction floor.

        See the `min_angular_in_place` declaration for why this exists. Guarded
        so it can only ever affect a turn on the spot: a zero yaw stays zero,
        and any command with forward speed passes through untouched.
        """
        if self.min_angular_in_place <= 0.0 or vx != 0.0 or wz == 0.0:
            return wz

        if abs(wz) < self.min_angular_in_place:
            return self.min_angular_in_place if wz > 0.0 else -self.min_angular_in_place

        return wz

    def _cmd_callback(self, msg):
        """Translate an incoming /cmd_vel into the serial JSON drive command."""
        vx = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        wz = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        wz = self._lift_in_place_yaw(vx, wz)

        self.last_vx = vx
        self.last_wz = wz
        self.last_cmd_time = self.get_clock().now()

        if self.cmd_expired:
            self.cmd_expired = False
            self.get_logger().info("/cmd_vel recovered, resuming drive commands")

        self._update_lights(wz)

    def _cmd_is_stale(self):
        """True when no /cmd_vel arrived within `cmd_timeout` seconds."""
        if self.cmd_timeout <= 0.0 or self.last_cmd_time is None:
            return False

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        return age > self.cmd_timeout

    def _resend_last_cmd(self):
        """Re-send the last drive command (keep-alive against the firmware timeout)."""
        if self._cmd_is_stale():
            if not self.cmd_expired:
                self.cmd_expired = True
                self.get_logger().warning(
                    f"No /cmd_vel for {self.cmd_timeout:.2f} s, stopping the robot"
                )
                self._turn_lights(True, True)

            self.last_vx = 0.0
            self.last_wz = 0.0

        self._send_json({"T": 13, "X": self.last_vx, "Z": self.last_wz})

    def _stop_robot(self):
        """Send the stop command and turn the lights off."""
        self.last_vx = 0.0
        self.last_wz = 0.0
        self._send_json({"T": 13, "X": 0.0, "Z": 0.0})

    def _send_json(self, data):
        """Serialise one JSON command over the serial port."""
        if self.ser is None:
            return

        try:
            line = (json.dumps(data) + "\n").encode("utf-8")
            self.ser.write(line)
        except Exception as exc:
            self.get_logger().error(f"Serial write failed: {exc}")

    def _enable_feedback_stream(self):
        """Ask the firmware to stream feedback (battery, encoders)."""
        self._send_json({"T": 131, "cmd": 1})
        self.get_logger().info("Requested feedback stream (T=131)")

    def _publish_feedback(self, raw):
        """Parse one feedback line and republish it on the ROS topics.

        The T=1001 frame is built by `base_info_feedback()` in the stock
        Cobra_Flex firmware (Cobra_Driver/ugv_advance.h). What that build
        actually puts on the wire, which is less than the protocol comment in
        json_cmd.h advertises:

          odl, odr  cumulative distance travelled by each side, as
                    `(long int)(en_odom_l * 100)` -- INTEGER CENTIMETRES, and
                    monotonic. They are odometers, not speeds (see below).
          v         battery, as `(int)(loadVoltage_V * 100)` -- CENTIVOLTS.
          M1..M4    per-motor feedback, but `ddsm_fb_*` is initialised to 0 and
                    the line that would refresh it is commented out upstream,
                    so these are always 0. Do not build anything on them.

        The IMU fields (gx/gy/gz, ax/ay/az, mx/my/mz) that json_cmd.h documents
        in this frame are commented out in the shipped build, as is the whole
        T=1002 frame. The chassis does carry an ICM-20948, so it is a recompile
        away, not a wiring problem -- but nothing arrives today.

        The firmware rate-limits this frame to `feedbackFlowExtraDelay` = 50 ms,
        i.e. 20 Hz. `_read_serial` polls at 50 Hz only so the OS buffer never
        backs up; it does not make the data any fresher.
        """
        data = json.loads(raw)
        self.feedback_pub.publish(String(data=json.dumps(data)))

        if data.get("T", -1) != 1001:
            return

        # Centivolts -> volts. Publishing the raw field put ~1180 on a topic
        # named `battery`, where 11.80 V was meant.
        battery = float(data.get("v", 0.0)) / 100.0
        self.battery_pub.publish(Float32(data=battery))

        # NOT speeds, despite the topic name: these are the two odometers
        # described above, republished raw (integer centimetres, cumulative).
        # Nothing subscribes to this topic today. Converting them into a real
        # nav_msgs/Odometry is deliberately still open -- it needs the wheel
        # geometry question settled first (see parameters.md 1.4), and the 1 cm
        # quantisation makes them a poor speed source without filtering.
        twist = Twist()
        twist.linear.x = float(data.get("odl", 0.0))
        twist.linear.y = float(data.get("odr", 0.0))
        self.wheels_pub.publish(twist)

    def _read_serial(self):
        """Drain and parse whatever feedback the OS buffer already holds."""
        if self.ser is None:
            return

        # Gated on in_waiting instead of calling readline() unconditionally.
        # readline() blocks for the port's full timeout (20 ms) whenever the
        # firmware is quiet, and this node runs on a single-threaded executor
        # shared with the 20 Hz keep-alive timer -- a silent link used to eat a
        # whole timer period on every one of these 50 Hz ticks.
        lines = 0
        try:
            while self.ser.in_waiting and lines < self.max_lines_per_read:
                raw = self.ser.readline().decode("utf-8", errors="replace").strip()
                lines += 1
                if not raw:
                    continue

                try:
                    self._publish_feedback(raw)
                except (ValueError, TypeError) as exc:
                    self.get_logger().warning(f"Serial parse error: {exc}")
        except Exception as exc:
            self.get_logger().warning(f"Serial read failed: {exc}")

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
    except (KeyboardInterrupt, ExternalShutdownException):
        # Under `ros2 launch` a ctrl-c arrives as ExternalShutdownException, not
        # KeyboardInterrupt. Letting it escape aborted main() before the motors
        # were stopped and made the node exit 1 on every shutdown.
        pass
    finally:
        if node is not None:
            # Stops the motors and closes the port. Safe after an external
            # shutdown: it only touches the serial device, not the ROS context.
            node.destroy_node()
        # Already down when the shutdown came from outside; calling it twice
        # raises and would again mask the clean exit.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
