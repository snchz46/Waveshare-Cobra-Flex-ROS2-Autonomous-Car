"""Sliders: set a velocity and it holds until you change it or stop."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

#: Sliders are integers, so the SI values are carried in hundredths.
_STEPS_PER_UNIT = 100


class _VelocityRow(QWidget):
    """One slider plus spin box plus reset, over a symmetric SI range."""

    def __init__(self, limit, step, on_change):
        """Build a row spanning [-limit, +limit], calling `on_change` on edits."""
        super().__init__()
        self._on_change = on_change
        self._span = int(round(limit * _STEPS_PER_UNIT))

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(-self._span, self._span)
        self._slider.setValue(0)
        self._slider.setTickInterval(max(1, self._span // 10))
        self._slider.setTickPosition(QSlider.TicksBelow)
        layout.addWidget(self._slider)

        self._spin = QDoubleSpinBox()
        self._spin.setRange(-limit, limit)
        self._spin.setSingleStep(step)
        self._spin.setDecimals(2)
        self._spin.setValue(0.0)
        layout.addWidget(self._spin)

        reset = QPushButton("Reset")
        reset.clicked.connect(self.reset)
        layout.addWidget(reset)

        self._slider.valueChanged.connect(self._from_slider)
        self._spin.valueChanged.connect(self._from_spin)

    def value(self):
        """Return the current value in SI units."""
        return self._spin.value()

    def reset(self):
        """Return the row to zero."""
        self._slider.setValue(0)

    def _from_slider(self, raw):
        self._spin.blockSignals(True)
        self._spin.setValue(raw / _STEPS_PER_UNIT)
        self._spin.blockSignals(False)
        self._on_change()

    def _from_spin(self, value):
        self._slider.blockSignals(True)
        self._slider.setValue(int(round(value * _STEPS_PER_UNIT)))
        self._slider.blockSignals(False)
        self._on_change()


class SliderMode(QWidget):
    """Two velocity rows and a stop button."""

    def __init__(self, node):
        """Build the rows over the limits `node` was configured with."""
        super().__init__()
        self._node = node

        layout = QVBoxLayout(self)

        layout.addWidget(QLabel(f"Linear velocity, m/s (max {node.max_linear:.2f}):"))
        self._linear = _VelocityRow(node.max_linear, 0.05, self._publish)
        layout.addWidget(self._linear)

        layout.addWidget(
            QLabel(f"Angular velocity, rad/s (max {node.max_angular:.2f}, "
                   "positive turns left):")
        )
        self._angular = _VelocityRow(node.max_angular, 0.10, self._publish)
        layout.addWidget(self._angular)

        stop = QPushButton("STOP")
        stop.setStyleSheet(
            "QPushButton { background-color: #cc0000; color: white; "
            "font-size: 18px; font-weight: bold; padding: 12px; }"
            "QPushButton:pressed { background-color: #990000; }"
        )
        stop.clicked.connect(self.reset)
        layout.addWidget(stop)
        layout.addStretch()

    def reset(self):
        """Zero both rows and stop the robot."""
        self._linear.reset()
        self._angular.reset()
        self._node.stop()

    def _publish(self):
        # No sign flip on the angular channel. The widget this was adapted from
        # negated it here and nowhere else, so its slider drove the opposite way
        # round from its own joystick and button pad, and against the ROS
        # convention its label claimed to follow.
        self._node.set_velocity(self._linear.value(), self._angular.value())
