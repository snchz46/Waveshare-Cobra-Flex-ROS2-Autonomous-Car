"""Entry point: discover launch files, bring up ROS, then give the main thread to Qt."""

import argparse
from pathlib import Path
import shutil
import signal
import sys

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication

from cobraflex_launch_gui.backend import MockBackend, RosBackend
from cobraflex_launch_gui.discovery import (
    build_launches,
    discover_installed,
    discover_source,
)
from cobraflex_launch_gui.main_window import LaunchGui
from cobraflex_launch_gui.meta import load_meta
from cobraflex_launch_gui.ros_interface import MockRosInterface, RosInterface
from cobraflex_launch_gui.state import UserState

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
except ImportError:  # mock mode on a machine without ROS
    rclpy = None

    class ExternalShutdownException(Exception):
        """Placeholder so that the except clause below is valid without rclpy."""


_QSS = Path(__file__).resolve().parent / "style.qss"


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="launch_gui",
        description="CobraFlex launch manager: launch files, ROS graph and TF tree.")
    parser.add_argument("--mock", action="store_true",
                        help="simulate runs and the ROS graph; no process is started")
    parser.add_argument("--workspace", type=Path, default=None,
                        help="scan <workspace>/src for launch files instead of the "
                             "ament index of the sourced workspaces")
    parser.add_argument("--all", action="store_true",
                        help="also list the launch files of the packages under /opt/ros")
    # --ros-args and its parameters belong to rclpy.
    args, _unknown = parser.parse_known_args(argv)
    return args


def _discover(args):
    if args.workspace is not None:
        return discover_source(args.workspace.expanduser().resolve())
    try:
        return discover_installed(include_system=args.all)
    except ImportError:
        # ament_index_python is missing outside a sourced ROS environment;
        # the source tree that contains this package is the fallback.
        return discover_source(Path(__file__).resolve().parents[3])


def main(args=None):
    """Run the launch manager window."""
    argv = sys.argv[1:] if args is None else list(args)
    options = _parse_args(argv)
    mock = options.mock or rclpy is None or shutil.which("ros2") is None
    if mock and not options.mock:
        print("[launch_gui] ROS 2 is not sourced: starting in mock mode.", file=sys.stderr)

    files = _discover(options)
    state = UserState()
    launches = build_launches(files, load_meta(), state)
    workspaces = sorted({f.workspace for f in files if f.workspace})

    ros = None
    exit_code = 0
    try:
        if mock:
            ros, backend = MockRosInterface(), MockBackend()
        else:
            rclpy.init(args=["launch_gui", *argv])
            ros, backend = RosInterface(), RosBackend()

        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
        app = QApplication([sys.argv[0]])
        app.setApplicationName("cobraflex_launch_gui")
        try:
            app.setStyleSheet(_QSS.read_text(encoding="utf-8"))
        except OSError:
            pass
        window = LaunchGui(backend, launches, ros, state, workspaces)
        window.show()

        # The launches run in sessions of their own and do not receive the
        # Ctrl-C of this terminal. SIGINT and SIGTERM therefore close the
        # window, which stops every launch. Python runs signal handlers only
        # between bytecodes, so a timer wakes the interpreter from the Qt
        # event loop several times per second.
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_a: window.force_close())
        wake = QTimer()
        wake.timeout.connect(lambda: None)
        wake.start(200)

        exit_code = app.exec_()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if ros is not None:
            ros.shutdown()
        # After an external shutdown the context is already down; a second
        # shutdown() call raises an exception.
        if rclpy is not None and not mock and rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
