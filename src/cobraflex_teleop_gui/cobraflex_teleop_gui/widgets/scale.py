"""The throttle slider the button pad and the joystick both carry.

One percentage, applied to the node's linear and angular limits separately.
Driving both channels off a single figure in m/s - the way the widget this was
adapted from does it - reads 0.5 as 0.5 m/s AND 0.5 rad/s, which are not the
same fraction of anything: this chassis plans inside 0.35 m/s and 2.0 rad/s,
so one number in SI would either crawl in a straight line or barely turn.
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
