"""Throttle slider shared by the button pad and the joystick.

One percentage, applied separately to the linear and angular limits of the
node. A single value in m/s for both channels, as in the original widget,
interprets 0.5 as 0.5 m/s and 0.5 rad/s, which are different fractions of the
limits: this chassis plans within 0.35 m/s and 2.0 rad/s, so a single SI value
would give either very slow straight motion or very little rotation.
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QSlider, QWidget


class ThrottleScale(QWidget):
    """A percentage slider that reports the velocities it corresponds to."""

    def __init__(self, node, default_percent=40):
        """Build the slider against the limits `node` was configured with."""
        super().__init__()
        self._node = node

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QLabel("Throttle:"))

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(5, 100)
        self._slider.setValue(default_percent)
        self._slider.setTickInterval(10)
        self._slider.setTickPosition(QSlider.TicksBelow)
        layout.addWidget(self._slider)

        self._label = QLabel()
        self._label.setMinimumWidth(150)
        layout.addWidget(self._label)

        self._slider.valueChanged.connect(self._refresh)
        self._refresh()

    def factor(self):
        """Return the current scale as a fraction in [0.05, 1.0]."""
        return self._slider.value() / 100.0

    def velocities(self):
        """Return the (linear, angular) limits at the current scale."""
        f = self.factor()
        return self._node.max_linear * f, self._node.max_angular * f

    def _refresh(self):
        linear, angular = self.velocities()
        self._label.setText(f"{linear:.2f} m/s   {angular:.2f} rad/s")
