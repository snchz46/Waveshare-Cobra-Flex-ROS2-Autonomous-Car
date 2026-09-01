"""The teleoperation window: a topic field, a mode selector and three pads."""

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from cobraflex_teleop_gui.widgets.joystick_mode import JoystickMode
from cobraflex_teleop_gui.widgets.keyboard_mode import KeyboardMode
from cobraflex_teleop_gui.widgets.slider_mode import SliderMode

#: How often the window tells the node it is still alive, in milliseconds.
#: Well inside the node's ui_watchdog (0.5 s by default) so that ordinary
#: scheduling jitter never looks like a freeze.
_HEARTBEAT_MS = 100


class MainWindow(QMainWindow):
    """Main teleoperation window."""

    def __init__(self, node):
        """Build the window around an already-spinning TeleopNode."""
        super().__init__()
        self._node = node
        self.setWindowTitle("CobraFlex Teleop")
        self.setMinimumSize(420, 520)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel("Topic:"))
        self._topic_edit = QLineEdit(node.get_parameter("cmd_vel_topic").value)
        topic_layout.addWidget(self._topic_edit)
        set_btn = QPushButton("Set")
        set_btn.clicked.connect(self._on_set_topic)
        topic_layout.addWidget(set_btn)
        layout.addLayout(topic_layout)

        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["Buttons", "Virtual joystick", "Sliders"])
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self._mode_combo)
        layout.addLayout(mode_layout)

        self._stack = QStackedWidget()
        self._keyboard_mode = KeyboardMode(node)
        self._joystick_mode = JoystickMode(node)
        self._slider_mode = SliderMode(node)
        for widget in (self._keyboard_mode, self._joystick_mode, self._slider_mode):
            self._stack.addWidget(widget)
        layout.addWidget(self._stack)

        # The readout shows what the NODE is publishing, not what the widget
        # asked for. Those differ whenever a limit clamps, and a slider that
        # silently disagrees with the wire is how you end up debugging the
        # wrong end of the stack.
        self.statusBar().showMessage("linear 0.00 m/s   angular 0.00 rad/s")

        # Doubles as the watchdog heartbeat: it runs in Qt's event loop, so it
        # stops the moment the loop does, which is exactly the condition the
        # node's ui_watchdog is there to catch.
        self._heartbeat = QTimer(self)
        self._heartbeat.timeout.connect(self._on_heartbeat)
        self._heartbeat.start(_HEARTBEAT_MS)

    def _on_heartbeat(self):
        self._node.pet()
        linear, angular = self._node.current_command()
        self.statusBar().showMessage(
            f"linear {linear:+.2f} m/s   angular {angular:+.2f} rad/s"
        )

    def _on_set_topic(self):
        self._node.set_topic(self._topic_edit.text())

    def _on_mode_changed(self, index):
        # Stop first: the pad being left behind may well be holding a non-zero
        # command, and nothing in the pad taking over would ever clear it.
        self._node.stop()
        self._slider_mode.reset()
        self._stack.setCurrentIndex(index)

    def closeEvent(self, event):
        """Stop the robot before the window goes away."""
        self._heartbeat.stop()
        self._node.publish_stop()
        event.accept()
