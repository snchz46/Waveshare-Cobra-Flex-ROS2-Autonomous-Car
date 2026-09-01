"""Entry point: bring up the ROS node, then give the main thread to Qt."""

import signal
import sys

from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.executors import ExternalShutdownException

from cobraflex_teleop_gui.main_window import MainWindow
from cobraflex_teleop_gui.teleop_node import TeleopNode


def main(args=None):
    """Run the teleoperation window."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = TeleopNode()
        node.start_spinning()

        # Qt installs a SIGINT handler and then does nothing with it, so a
        # ctrl-c in the terminal that launched the window would be swallowed.
        # Handing the signal back to Python's default makes it kill the
        # process, which is what a ctrl-c should do here.
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        app = QApplication(sys.argv)
        window = MainWindow(node)
        window.show()
        exit_code = app.exec_()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Under `ros2 launch` a ctrl-c arrives as ExternalShutdownException,
        # not KeyboardInterrupt; letting it escape made the node exit 1.
        pass
    finally:
        if node is not None:
            # A zero Twist has to go out before the publisher dies, or the
            # last velocity commanded is the last one the driver saw.
            node.publish_stop()
            node.destroy_node()
        # Already down when the shutdown came from outside, and calling it
        # twice raises.
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
