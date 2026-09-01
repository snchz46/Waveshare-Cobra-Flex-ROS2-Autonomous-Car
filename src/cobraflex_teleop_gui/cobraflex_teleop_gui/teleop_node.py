"""ROS 2 side of the teleoperation GUI: one Twist publisher and a watchdog.

Kept apart from the Qt widgets deliberately. The widgets live in Qt's event
loop and this node spins in a thread of its own, so the only state they share
is the velocity pair below, behind one lock, in one file.
"""

import threading
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node


class TeleopNode(Node):
    """Publish the velocity the window last asked for, and stop if it stalls."""

    def __init__(self):
        """Declare the parameters, create the publisher and start the timer."""
        super().__init__("cobraflex_teleop_gui")

        # cmd_vel is the right topic and the SI one: linear.x in m/s, angular.z
        # in rad/s, which is what cobraflex_ros_driver and the DiffDrive plugin
        # both read.
        #
        # Do NOT point this at /raw_action to "go through the safety cage". That
        # chain speaks a different contract - the cage takes linear.x as a
        # throttle in [-1, 1] and angular.z as a normalised steering, and
        # vehicle_control_node is what turns those back into m/s and rad/s. A
        # 0.3 arriving there means 30 % throttle, not 0.3 m/s. The cage is also
        # a lane-following cage: it decides on /state_obs lateral offsets, which
        # say nothing about a human driving by hand.
        #
        # What actually protects the robot here is the driver's own cmd_timeout
        # deadman, plus the ui_watchdog below.
        self.declare_parameter("cmd_vel_topic", "cmd_vel")

        # The envelope the rest of the stack plans inside (nav2_params.yaml's
        # max_vel_x and max_vel_theta), NOT what the driver will accept - it
        # clamps at 0.53 m/s and 6.0 rad/s. The smaller pair on purpose: a
        # slider promising a speed the robot then silently refuses is worse
        # than one that stays inside what everything else already agrees on.
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 2.0)

        self.declare_parameter("publish_rate", 15.0)

        # A Qt event loop that wedges does not stop the ROS thread: without
        # this, the timer below would happily republish the last non-zero
        # velocity forever while the window is frozen, and the driver's
        # cmd_timeout would never fire because commands ARE still arriving.
        # So the window pets this node from inside its own event loop, and a
        # pet that goes stale zeroes the command. Same idea, and the same
        # default, as vehicle_control_node's safe_action_timeout_s.
        #
        # 0 disables it, which is only ever right when working on the GUI
        # itself with no robot attached.
        self.declare_parameter("ui_watchdog", 0.5)

        self.max_linear = abs(float(self.get_parameter("max_linear").value))
        self.max_angular = abs(float(self.get_parameter("max_angular").value))
        self._ui_watchdog = float(self.get_parameter("ui_watchdog").value)

        self._lock = threading.Lock()
        self._linear = 0.0
        self._angular = 0.0
        self._last_pet = time.monotonic()
        self._stalled = False
        self._spin_thread = None

        self._topic = str(self.get_parameter("cmd_vel_topic").value)
        self._publisher = self.create_publisher(Twist, self._topic, 10)

        rate = float(self.get_parameter("publish_rate").value)
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"teleop GUI publishing on {self._topic} at {rate:.0f} Hz, "
            f"limits {self.max_linear:.2f} m/s and {self.max_angular:.2f} rad/s"
        )

    def set_velocity(self, linear, angular):
        """Set the commanded velocity, clamped to the configured limits."""
        with self._lock:
            self._linear = max(-self.max_linear, min(self.max_linear, float(linear)))
            self._angular = max(-self.max_angular, min(self.max_angular, float(angular)))

    def stop(self):
        """Command zero velocity."""
        self.set_velocity(0.0, 0.0)

    def publish_stop(self):
        """Command zero and put it on the wire now, without waiting a tick.

        Used on the way out: the timer is about to be destroyed, so a stop
        that only sets the state would never reach the robot.
        """
        self.stop()
        self._publisher.publish(Twist())

    def current_command(self):
        """Return the (linear, angular) pair currently being published."""
        with self._lock:
            return self._linear, self._angular

    def pet(self):
        """Report that the Qt event loop is still running.

        Called from a timer inside the GUI thread. See ui_watchdog.
        """
        with self._lock:
            self._last_pet = time.monotonic()

    def set_topic(self, name):
        """Republish on a different topic from now on, stopping first."""
        name = name.strip()
        if not name or name == self._topic:
            return
        self.stop()
        self._publisher.publish(Twist())
        self.destroy_publisher(self._publisher)
        self._topic = name
        self._publisher = self.create_publisher(Twist, self._topic, 10)
        self.get_logger().info(f"now publishing on {self._topic}")

    def start_spinning(self):
        """Spin this node in a daemon thread, leaving the main thread to Qt."""
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self):
        import rclpy

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _publish(self):
        msg = Twist()
        with self._lock:
            stale = (self._ui_watchdog > 0.0
                     and time.monotonic() - self._last_pet > self._ui_watchdog)
            if stale:
                self._linear = 0.0
                self._angular = 0.0
            msg.linear.x = self._linear
            msg.angular.z = self._angular
            was_stalled, self._stalled = self._stalled, stale

        if stale and not was_stalled:
            self.get_logger().warn(
                f"no UI heartbeat for {self._ui_watchdog:.2f} s - the window is "
                "frozen or gone; commanding zero"
            )
        elif was_stalled and not stale:
            self.get_logger().info("UI heartbeat back")

        self._publisher.publish(msg)
