"""Virtual joystick: drag a handle, release to stop."""

import math

from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QBrush, QColor, QLinearGradient, QPainter, QPen
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from cobraflex_teleop_gui.widgets.scale import ThrottleScale


class JoystickPad(QWidget):
    """A round pad reporting a handle position normalised to the unit disc."""

    def __init__(self, parent=None):
        """Create the pad with the handle centred and no callbacks attached."""
        super().__init__(parent)
        self.setMinimumSize(250, 250)
        self._handle = QPointF(0.0, 0.0)
        self._dragging = False
        self.on_move = None
        self.on_release = None

    def paintEvent(self, event):
        """Draw the dial, the crosshair and the handle."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        radius = side / 2.0 - 10

        painter.setBrush(QBrush(QColor(50, 50, 50)))
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        painter.setPen(QPen(QColor(100, 100, 100), 1, Qt.DashLine))
        painter.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        painter.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))

        hx = cx + self._handle.x() * radius
        hy = cy + self._handle.y() * radius
        handle_r = 20
        gradient = QLinearGradient(hx - handle_r, hy - handle_r,
                                   hx + handle_r, hy + handle_r)
        gradient.setColorAt(0, QColor(100, 150, 255))
        gradient.setColorAt(1, QColor(30, 80, 200))
        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(QColor(200, 200, 255), 2))
        painter.drawEllipse(QPointF(hx, hy), handle_r, handle_r)

    def mousePressEvent(self, event):
        """Grab the handle."""
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._update_handle(event.pos())

    def mouseMoveEvent(self, event):
        """Follow the pointer while the handle is held."""
        if self._dragging:
            self._update_handle(event.pos())

    def mouseReleaseEvent(self, event):
        """Drop the handle back to the centre and report the release."""
        if event.button() == Qt.LeftButton:
            self._dragging = False
            self._handle = QPointF(0.0, 0.0)
            self.update()
            if self.on_release:
                self.on_release()

    def _update_handle(self, pos):
        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        radius = side / 2.0 - 10

        dx = (pos.x() - cx) / radius
        dy = (pos.y() - cy) / radius
        dist = math.hypot(dx, dy)
        if dist > 1.0:
            dx /= dist
            dy /= dist

        self._handle = QPointF(dx, dy)
        self.update()
        if self.on_move:
            # Screen y grows downwards, so up on the pad has to become forward,
            # and right has to become a negative (clockwise) yaw rate.
            self.on_move(-dy, -dx)


class JoystickMode(QWidget):
    """The joystick pad, scaled by a throttle slider."""

    def __init__(self, node):
        """Build the joystick against `node`."""
        super().__init__()
        self._node = node

        layout = QVBoxLayout(self)
        self._scale = ThrottleScale(node)
        layout.addWidget(self._scale)

        self._pad = JoystickPad()
        self._pad.on_move = self._on_move
        self._pad.on_release = self._on_release
        layout.addWidget(self._pad, stretch=1)

        values = QHBoxLayout()
        self._linear_label = QLabel("linear 0.00 m/s")
        self._angular_label = QLabel("angular 0.00 rad/s")
        for label in (self._linear_label, self._angular_label):
            label.setStyleSheet("font-size: 14px;")
            values.addWidget(label)
        layout.addLayout(values)

    def _on_move(self, forward, left):
        max_linear, max_angular = self._scale.velocities()
        linear = forward * max_linear
        angular = left * max_angular
        self._node.set_velocity(linear, angular)
        self._linear_label.setText(f"linear {linear:+.2f} m/s")
        self._angular_label.setText(f"angular {angular:+.2f} rad/s")

    def _on_release(self):
        self._node.stop()
        self._linear_label.setText("linear 0.00 m/s")
        self._angular_label.setText("angular 0.00 rad/s")
