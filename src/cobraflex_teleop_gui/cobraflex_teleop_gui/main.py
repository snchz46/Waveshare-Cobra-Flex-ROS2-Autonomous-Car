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

        # Qt installs a SIGINT handler that does not act on the signal, so
        # Ctrl-C in the launching terminal would have no effect. Restoring the
        # default handler makes Ctrl-C terminate the process.
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        app = QApplication(sys.argv)
        window = MainWindow(node)
        window.show()
        exit_code = app.exec_()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Under `ros2 launch`, Ctrl-C arrives as ExternalShutdownException
        # instead of KeyboardInterrupt; catching it gives exit status 0.
        pass
    finally:
        if node is not None:
            # A zero Twist is published before the publisher is destroyed;
            # otherwise the last commanded velocity remains the last command
            # received by the driver.
            node.publish_stop()
            node.destroy_node()
        # After an external shutdown the context is already down; a second
        # shutdown() call raises an exception.
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
