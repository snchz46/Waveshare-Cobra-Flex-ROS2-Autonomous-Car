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
        # Deadman timer. `_resend_last_cmd` overrides the firmware command
        # timeout by re-sending the last velocity. Without this timer, a failed
        # /cmd_vel publisher (terminated process, lost DDS link) leaves the
        # physical robot driving at the last velocity. The command is set to
        # zero after this many seconds without a new /cmd_vel. 0.0 disables
        # the timer (bench use only).
        self.declare_parameter("cmd_timeout", 0.5)
        # Maximum number of feedback lines read per tick, so that a high
        # feedback rate cannot delay the keep-alive timer on the same executor
        # thread.
        self.declare_parameter("max_lines_per_read", 20)
        # Minimum yaw rate for in-place rotation (static friction). Disabled
        # by default.
        #
        # The firmware maps a twist linearly to wheel RPM (`rosCtrl` in
        # movtion_module.h) without deadband compensation. A small yaw command
        # with zero forward speed therefore requests an RPM below the static
        # friction of the motors: the robot does not move and no error is
        # reported. Waveshare ugv_bringup raises such commands to 0.2 rad/s.
        #
        # In this stack a stall can be an intended command.
        # `safe_action_to_cmd_2d` (cobraflex_rl/cage_bridge) derives linear_x
        # and angular_z independently: a throttle below `throttle_deadband`
        # gives linear_x == 0.0 while the steering is still mapped through
        # `steering_to_yaw_rate_gain` (0.8). A C-04 attenuation to standstill,
        # which SR-009 requires to be commandable, therefore arrives here as
        # vx == 0 with |wz| < 0.2 for any steering within a quarter of its
        # range. Raising it would rotate a robot that the cage has stopped.
        #
        # Recommended values: 0.2 for Nav2 bring-up or teleoperation, where
        # `rotate_to_goal` otherwise stalls without an error; 0.0 whenever the
        # cage or the RL policy drives.
        self.declare_parameter("min_angular_in_place", 0.0)

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

        Rationale: see the `min_angular_in_place` declaration. Only in-place
        rotations are affected: a zero yaw remains zero, and any command with
        forward speed is passed through unchanged.
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
        Cobra_Flex firmware (Cobra_Driver/ugv_advance.h). The published build
        transmits fewer fields than the protocol comment in json_cmd.h lists:

          odl, odr  Cumulative distance per side,
                    `(long int)(en_odom_l * 100)`: integer centimetres,
                    monotonic. These are odometers, not speeds.
          v         Battery voltage, `(int)(loadVoltage_V * 100)`: centivolts.
          M1..M4    Per-motor feedback. `ddsm_fb_*` is initialised to 0 and
                    its update is commented out in the firmware; the values
                    are always 0 and are not used.

        The IMU fields (gx/gy/gz, ax/ay/az, mx/my/mz) documented in json_cmd.h
        for this frame, and the complete T=1002 frame, are commented out in the
        published build. The chassis carries an ICM-20948; IMU data requires a
        firmware recompilation.

        The firmware limits this frame to `feedbackFlowExtraDelay` = 50 ms
        (20 Hz). `_read_serial` polls at 50 Hz to keep the OS buffer empty; the
        polling rate does not increase the data rate.
        """
        data = json.loads(raw)
        self.feedback_pub.publish(String(data=json.dumps(data)))

        if data.get("T", -1) != 1001:
            return

        # Conversion from centivolts to volts (raw value ~1180 for 11.80 V).
        battery = float(data.get("v", 0.0)) / 100.0
        self.battery_pub.publish(Float32(data=battery))

        # Despite the topic name, these values are the two odometers described
        # above, republished unchanged (integer centimetres, cumulative). The
        # topic currently has no subscriber. Conversion into nav_msgs/Odometry
        # is open: it requires the wheel geometry to be settled first
        # (parameters.md 1.4), and the 1 cm quantisation makes the values
        # unsuitable as a speed source without filtering.
        twist = Twist()
        twist.linear.x = float(data.get("odl", 0.0))
        twist.linear.y = float(data.get("odr", 0.0))
        self.wheels_pub.publish(twist)

    def _read_serial(self):
        """Drain and parse whatever feedback the OS buffer already holds."""
        if self.ser is None:
            return

        # Reads only when in_waiting reports data. An unconditional readline()
        # blocks for the full port timeout (20 ms) when the firmware sends
        # nothing; on the single-threaded executor shared with the 20 Hz
        # keep-alive timer, this would consume a full timer period on every
        # 50 Hz tick.
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
        # Under `ros2 launch`, Ctrl-C arrives as ExternalShutdownException
        # instead of KeyboardInterrupt. Catching it ensures that the motors are
        # stopped and the node exits with status 0.
        pass
    finally:
        if node is not None:
            # Stops the motors and closes the port. Valid after an external
            # shutdown, as it accesses only the serial device.
            node.destroy_node()
        # After an external shutdown the context is already down; a second
        # shutdown() call raises an exception.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
