"""Button pad: a 3x3 grid that drives while a button is held down."""

from PyQt5.QtWidgets import QGridLayout, QPushButton, QVBoxLayout, QWidget

from cobraflex_teleop_gui.widgets.scale import ThrottleScale


class KeyboardMode(QWidget):
    """Eight direction buttons around a stop, held to drive."""

    #: (label, linear sign, angular sign). Angular follows the ROS convention:
    #: positive is counter-clockwise, so the left-hand column is positive.
    _BUTTONS = [
        ("↖", 1.0, 1.0), ("↑", 1.0, 0.0), ("↗", 1.0, -1.0),
        ("←", 0.0, 1.0), ("STOP", 0.0, 0.0), ("→", 0.0, -1.0),
        ("↙", -1.0, 1.0), ("↓", -1.0, 0.0), ("↘", -1.0, -1.0),
    ]

    def __init__(self, node):
        """Build the pad against `node`."""
        super().__init__()
        self._node = node

        layout = QVBoxLayout(self)
        self._scale = ThrottleScale(node)
        layout.addWidget(self._scale)

        grid = QGridLayout()
        grid.setSpacing(4)
        for idx, (label, lin, ang) in enumerate(self._BUTTONS):
            btn = QPushButton(label)
            btn.setMinimumSize(80, 60)
            btn.setStyleSheet("font-size: 16px; font-weight: bold;")
            btn.pressed.connect(self._make_handler(lin, ang))
            # Releasing always stops, including on the middle button, so a
            # pointer dragged off a held button cannot leave the robot driving.
            btn.released.connect(self._node.stop)
            grid.addWidget(btn, *divmod(idx, 3))
        layout.addLayout(grid)
        layout.addStretch()

    def _make_handler(self, lin_sign, ang_sign):
        def handler():
            linear, angular = self._scale.velocities()
            self._node.set_velocity(lin_sign * linear, ang_sign * angular)
        return handler
