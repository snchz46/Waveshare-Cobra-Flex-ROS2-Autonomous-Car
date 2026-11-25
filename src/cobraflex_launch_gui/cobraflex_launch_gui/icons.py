"""Icons and the running mark, painted with QPainter in the colours of the theme.

Unicode symbols (▶ ■ ☰ ★) change shape with the installed fonts and differ
between the PC and the Jetson. These paths look the same on every machine
and need no SVG module. Each icon is drawn in a 24 x 24 box.
"""

import math
from typing import Callable, Dict, Optional
import weakref

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QIcon, QPainter, QPainterPath, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import QWidget

from cobraflex_launch_gui.density import px
from cobraflex_launch_gui.theme import PALETTE

#: Pixmaps are drawn at twice their size, so that they stay sharp at the
#: 1.25 factor of the touch density and on high-DPI screens.
_OVERSAMPLE = 2.0


def _pen(color: QColor, width: float = 2.0) -> QPen:
    pen = QPen(color, width)
    pen.setCapStyle(Qt.RoundCap)
    pen.setJoinStyle(Qt.RoundJoin)
    return pen


def _filled(p: QPainter, color: QColor) -> None:
    p.setPen(Qt.NoPen)
    p.setBrush(color)


def _stroked(p: QPainter, color: QColor, width: float = 2.0) -> None:
    p.setPen(_pen(color, width))
    p.setBrush(Qt.NoBrush)


def _play(p: QPainter, c: QColor) -> None:
    _filled(p, c)
    p.drawPolygon(QPolygonF([QPointF(7.5, 5.0), QPointF(7.5, 19.0), QPointF(19.0, 12.0)]))


def _stop(p: QPainter, c: QColor) -> None:
    _filled(p, c)
    p.drawRoundedRect(QRectF(6.5, 6.5, 11.0, 11.0), 1.5, 1.5)


def _restart(p: QPainter, c: QColor) -> None:
    # Clockwise open circle, gap at the upper left, arrowhead pointing up.
    _stroked(p, c, 2.2)
    rect = QRectF(4.0, 4.0, 16.0, 16.0)
    path = QPainterPath()
    path.arcMoveTo(rect, 120.0)
    path.arcTo(rect, 120.0, -300.0)
    p.drawPath(path)
    _filled(p, c)
    p.drawPolygon(QPolygonF([QPointF(4.0, 8.2), QPointF(7.6, 13.0), QPointF(0.4, 13.0)]))


def _logs(p: QPainter, c: QColor) -> None:
    _stroked(p, c)
    for y in (7.0, 12.0, 17.0):
        p.drawLine(QPointF(5.0, y), QPointF(19.0, y))


def _search(p: QPainter, c: QColor) -> None:
    _stroked(p, c, 2.2)
    p.drawEllipse(QPointF(10.5, 10.5), 5.5, 5.5)
    p.drawLine(QPointF(14.6, 14.6), QPointF(19.5, 19.5))


def _star_polygon() -> QPolygonF:
    points = []
    for i in range(10):
        r = 8.6 if i % 2 == 0 else 3.7
        a = math.radians(-90.0 + i * 36.0)
        points.append(QPointF(12.0 + r * math.cos(a), 12.6 + r * math.sin(a)))
    return QPolygonF(points)


def _star(p: QPainter, c: QColor) -> None:
    _stroked(p, c, 1.6)
    p.drawPolygon(_star_polygon())


def _star_filled(p: QPainter, c: QColor) -> None:
    p.setPen(_pen(c, 1.6))
    p.setBrush(c)
    p.drawPolygon(_star_polygon())


def _chevron_right(p: QPainter, c: QColor) -> None:
    _stroked(p, c, 2.4)
    p.drawPolyline(QPolygonF([QPointF(9.0, 6.0), QPointF(15.0, 12.0), QPointF(9.0, 18.0)]))


def _chevron_down(p: QPainter, c: QColor) -> None:
    _stroked(p, c, 2.4)
    p.drawPolyline(QPolygonF([QPointF(6.0, 9.0), QPointF(12.0, 15.0), QPointF(18.0, 9.0)]))


def _check(p: QPainter, c: QColor) -> None:
    _stroked(p, c, 2.4)
    p.drawPolyline(QPolygonF([QPointF(5.0, 12.5), QPointF(10.0, 17.5), QPointF(19.0, 7.5)]))


def _close(p: QPainter, c: QColor) -> None:
    _stroked(p, c, 2.4)
    p.drawLine(QPointF(6.5, 6.5), QPointF(17.5, 17.5))
    p.drawLine(QPointF(17.5, 6.5), QPointF(6.5, 17.5))


def _dot(p: QPainter, c: QColor) -> None:
    _filled(p, c)
    p.drawEllipse(QPointF(12.0, 12.0), 3.5, 3.5)


_DRAW: Dict[str, Callable[[QPainter, QColor], None]] = {
    "play": _play,
    "stop": _stop,
    "restart": _restart,
    "logs": _logs,
    "search": _search,
    "star": _star,
    "star_filled": _star_filled,
    "chevron_right": _chevron_right,
    "chevron_down": _chevron_down,
    "check": _check,
    "close": _close,
    "dot": _dot,
}


def pixmap(name: str, color: str, size: int) -> QPixmap:
    """Return icon name drawn in color, size x size logical pixels."""
    pm = QPixmap(int(size * _OVERSAMPLE), int(size * _OVERSAMPLE))
    pm.fill(Qt.transparent)
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.scale(size * _OVERSAMPLE / 24.0, size * _OVERSAMPLE / 24.0)
    _DRAW[name](painter, QColor(color))
    painter.end()
    pm.setDevicePixelRatio(_OVERSAMPLE)
    return pm


def icon(name: str, color: str, size: int = 16, disabled: Optional[str] = None) -> QIcon:
    """Return icon name as a QIcon; disabled is the colour of its disabled state."""
    result = QIcon(pixmap(name, color, px(size)))
    result.addPixmap(pixmap(name, disabled or PALETTE["c_faint"], px(size)), QIcon.Disabled)
    return result


class TrackGlyph(QWidget):
    """Running mark: a strip of the black circuit whose dashed lane line moves.

    The robot drives on a black track with white dashed lines; a running
    launch shows that line moving, a stopping one shows it still and amber.
    A single timer drives every instance.
    """

    _glyphs: "weakref.WeakSet[TrackGlyph]" = weakref.WeakSet()
    _timer: Optional[QTimer] = None
    _phase = 0

    def __init__(self, moving: bool = True):
        """Create the mark; moving is False for a launch that is stopping."""
        super().__init__()
        self._moving = moving
        self.setFixedSize(px(22), px(10))
        TrackGlyph._glyphs.add(self)
        if moving and TrackGlyph._timer is None:
            TrackGlyph._timer = QTimer()
            TrackGlyph._timer.timeout.connect(TrackGlyph._advance)
            TrackGlyph._timer.start(120)

    @classmethod
    def _advance(cls) -> None:
        cls._phase = (cls._phase + 1) % 8
        for glyph in list(cls._glyphs):
            try:
                if glyph._moving and glyph.isVisible():
                    glyph.update()
            except RuntimeError:
                # The widget was deleted on the C++ side.
                cls._glyphs.discard(glyph)

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        """Draw the strip of track and its lane line."""
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect())
        outline = QPainterPath()
        outline.addRoundedRect(rect, rect.height() / 3.0, rect.height() / 3.0)
        p.setClipPath(outline)
        p.fillRect(rect, QColor(PALETTE["c_tyre"]))
        dash = QColor(PALETTE["c_on_dark"] if self._moving else PALETTE["c_log_warn"])
        period = rect.width() / 3.0
        height = max(1.5, rect.height() / 5.0)
        x = -period + (TrackGlyph._phase / 8.0 * period if self._moving else 0.0)
        while x < rect.width():
            p.fillRect(QRectF(x, (rect.height() - height) / 2.0, period * 0.55, height), dash)
            x += period
        p.end()
