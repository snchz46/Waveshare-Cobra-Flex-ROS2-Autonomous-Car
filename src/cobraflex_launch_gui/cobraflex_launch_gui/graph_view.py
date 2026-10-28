"""Native views of the ROS graph and the TF tree.

Both pages draw with QGraphicsView on top of graph_layout. A page refreshes
only while it is visible, and computes a new layout only when the structure
(the set of items and edges) changes; rates, ages and colours are updated in
place, so that the drawing does not move under the cursor.
"""

import html
import math
import shutil
import time
from typing import Callable, Dict, List, Optional, Set, Tuple

from PyQt5.QtCore import pyqtSignal, QPointF, QProcess, QRectF, Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QFont, QFontMetricsF, QPainter, QPainterPath, QPen, \
    QPolygonF
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFrame,
    QGraphicsItem,
    QGraphicsPathItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from cobraflex_launch_gui.constants import (
    GRAPH_ALWAYS_HIDDEN_TOPICS,
    GRAPH_BACKGROUND,
    GRAPH_COLORS,
    GRAPH_EDGE_COLOR,
    GRAPH_EDGE_HIGHLIGHT,
    GRAPH_INFRASTRUCTURE_TOPICS,
    GRAPH_REFRESH_MS,
    TF_REFRESH_MS,
    TF_STALE_S,
)
from cobraflex_launch_gui.graph_layout import EdgeRoute, layout_graph, LayoutEdge, LayoutNode
from cobraflex_launch_gui.model import Launch
from cobraflex_launch_gui.ros_interface import analyse_tf

_PAD_X = 12.0
_PAD_Y = 6.0
_ARROW = 7.0
_MAX_FIT_SCALE = 1.4


def ros_package_available(pkg: str) -> bool:
    """Return whether a ROS package is installed in a sourced workspace."""
    try:
        from ament_index_python.packages import get_package_prefix

        get_package_prefix(pkg)
        return shutil.which("ros2") is not None
    except Exception:
        return False


# --- Scene items ------------------------------------------------------------


class GraphItem(QGraphicsItem):
    """A node, topic or frame box with a centred label."""

    def __init__(self, item_id: str, label: str, kind: str, size: Tuple[float, float],
                 font: QFont):
        """Create the box; kind is node, topic or frame."""
        super().__init__()
        self.item_id = item_id
        self.label = label
        self.kind = kind
        self.state = "normal"
        self._w, self._h = size
        self._font = font
        self._highlight = False
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setZValue(1)

    def set_state(self, state: str, tooltip: str) -> None:
        """Set the state (normal, warn, error) and the tooltip."""
        if state != self.state:
            self.state = state
            self.update()
        self.setToolTip(tooltip)

    def set_emphasis(self, emphasis: int) -> None:
        """Set 1 for highlighted, 0 for normal, -1 for dimmed."""
        self._highlight = emphasis > 0
        self.setOpacity(0.25 if emphasis < 0 else 1.0)
        self.update()

    def boundingRect(self) -> QRectF:  # noqa: N802 - Qt API
        """Return the box with room for the highlight border."""
        return QRectF(-self._w / 2 - 2, -self._h / 2 - 2, self._w + 4, self._h + 4)

    def paint(self, painter: QPainter, _option, _widget=None) -> None:
        """Draw the box and its label."""
        fill, border, text = GRAPH_COLORS[self.state if self.state != "normal" else self.kind]
        rect = QRectF(-self._w / 2, -self._h / 2, self._w, self._h)
        radius = 4.0 if self.kind == "topic" else self._h / 2
        border_color = QColor(GRAPH_EDGE_HIGHLIGHT if self._highlight else border)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(border_color, 2.2 if self._highlight else 1.2))
        painter.setBrush(QBrush(QColor(fill)))
        painter.drawRoundedRect(rect, radius, radius)
        painter.setPen(QColor(text))
        painter.setFont(self._font)
        painter.drawText(rect, Qt.AlignCenter, self.label)


def _route_path(route: EdgeRoute) -> Tuple[QPainterPath, QPointF, QPointF]:
    """Return the path of an edge and the two points that orient its arrow."""
    pts = [QPointF(x, y) for x, y in route.points]
    path = QPainterPath(pts[0])
    if route.spline and len(pts) >= 4:
        for i in range(1, len(pts) - 2, 3):
            path.cubicTo(pts[i], pts[i + 1], pts[i + 2])
        tail = pts[-2]
    else:
        # Catmull-Rom through the polyline points, as cubic segments.
        tail = pts[0]
        for i in range(len(pts) - 1):
            p0 = pts[i - 1] if i > 0 else pts[i]
            p1, p2 = pts[i], pts[i + 1]
            p3 = pts[i + 2] if i + 2 < len(pts) else p2
            c1 = p1 + (p2 - p0) / 6.0
            c2 = p2 - (p3 - p1) / 6.0
            path.cubicTo(c1, c2, p2)
            tail = c2
    tip = pts[-1]
    if (tip - tail).manhattanLength() < 0.5 and len(pts) >= 2:
        tail = pts[-2]
    return path, tail, tip


class _LabelItem(QGraphicsItem):
    """Edge label centred on its position, on a background patch.

    Drawn with QPainter.drawText: QGraphicsSimpleTextItem fills the glyph
    outlines as paths, which renders small text without hinting.
    """

    def __init__(self, text: str, font: QFont, parent: QGraphicsItem):
        super().__init__(parent)
        self._text = text
        self._font = font
        self._fm = QFontMetricsF(font)
        self._color = QColor("#8b949e")
        self.setZValue(2)

    def set_text(self, text: str) -> None:
        if text != self._text:
            self.prepareGeometryChange()
            self._text = text
            self.update()

    def set_color(self, color: str) -> None:
        self._color = QColor(color)
        self.update()

    def boundingRect(self) -> QRectF:  # noqa: N802 - Qt API
        w = self._fm.horizontalAdvance(self._text) + 6
        h = self._fm.height() + 2
        return QRectF(-w / 2, -h / 2, w, h)

    def paint(self, painter: QPainter, _option, _widget=None) -> None:
        rect = self.boundingRect()
        background = QColor(GRAPH_BACKGROUND)
        background.setAlpha(220)
        painter.setPen(Qt.NoPen)
        painter.setBrush(background)
        painter.drawRoundedRect(rect, 3, 3)
        painter.setFont(self._font)
        painter.setPen(self._color)
        painter.drawText(rect, Qt.AlignCenter, self._text)


class EdgeItem(QGraphicsPathItem):
    """A routed edge with an arrowhead and an optional label."""

    def __init__(self, route: EdgeRoute, font: QFont):
        """Build the path, the arrowhead and the label from a route."""
        path, tail, tip = _route_path(route)
        super().__init__(path)
        self.src, self.dst = route.src, route.dst
        dx, dy = tip.x() - tail.x(), tip.y() - tail.y()
        angle = math.atan2(dy, dx)
        left = QPointF(tip.x() - _ARROW * math.cos(angle - 0.4),
                       tip.y() - _ARROW * math.sin(angle - 0.4))
        right = QPointF(tip.x() - _ARROW * math.cos(angle + 0.4),
                        tip.y() - _ARROW * math.sin(angle + 0.4))
        self._arrow = QGraphicsPolygonItem(QPolygonF([tip, left, right]), self)
        self._label: Optional[_LabelItem] = None
        if route.label_pos is not None:
            self._label = _LabelItem(route.label, font, self)
            self._label.setPos(*route.label_pos)
        self.set_style(GRAPH_EDGE_COLOR)

    def set_label(self, text: str) -> None:
        """Replace the label text, keeping it centred on its position."""
        if self._label is not None:
            self._label.set_text(text)

    def set_style(self, color: str, width: float = 1.2, dashed: bool = False,
                  opacity: float = 1.0) -> None:
        """Set colour, width, dash and opacity of line, arrow and label."""
        pen = QPen(QColor(color), width)
        if dashed:
            pen.setStyle(Qt.DashLine)
        self.setPen(pen)
        self._arrow.setPen(QPen(QColor(color), 1.0))
        self._arrow.setBrush(QBrush(QColor(color)))
        if self._label is not None:
            self._label.set_color("#8b949e" if color == GRAPH_EDGE_COLOR else color)
        self.setOpacity(opacity)


class GraphCanvas(QGraphicsView):
    """Zoomable, pannable scene; a click emits the id of the item under it."""

    item_clicked = pyqtSignal(str)

    def __init__(self):
        """Create an empty scene."""
        super().__init__()
        self.setScene(QGraphicsScene(self))
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QBrush(QColor(GRAPH_BACKGROUND)))
        self.setFrameShape(QFrame.NoFrame)
        self.item_font = QFont(self.font())
        self.item_font.setPointSizeF(9.0)
        self.label_font = QFont(self.font())
        self.label_font.setPointSizeF(7.5)
        self.user_zoomed = False
        self._press = None

    def item_size(self, label: str) -> Tuple[float, float]:
        """Return the box size that fits a label."""
        fm = QFontMetricsF(self.item_font)
        return fm.horizontalAdvance(label) + 2 * _PAD_X, fm.height() + 2 * _PAD_Y

    def show_message(self, text: str) -> None:
        """Replace the drawing with a centred message."""
        self.scene().clear()
        item = self.scene().addSimpleText(text, self.item_font)
        item.setBrush(QBrush(QColor("#8b949e")))
        rect = item.boundingRect()
        item.setPos(-rect.width() / 2, -rect.height() / 2)
        self.scene().setSceneRect(QRectF(-400, -200, 800, 400))
        self.resetTransform()
        self.centerOn(0, 0)

    def fit(self) -> None:
        """Fit the drawing in the view, without enlarging small graphs."""
        rect = self.scene().itemsBoundingRect()
        if rect.isEmpty():
            return
        self.scene().setSceneRect(rect.adjusted(-40, -40, 40, 40))
        self.fitInView(rect.adjusted(-20, -20, 20, 20), Qt.KeepAspectRatio)
        if self.transform().m11() > _MAX_FIT_SCALE:
            self.resetTransform()
            self.scale(_MAX_FIT_SCALE, _MAX_FIT_SCALE)
            self.centerOn(rect.center())
        self.user_zoomed = False

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Zoom around the cursor."""
        factor = 1.15 ** (event.angleDelta().y() / 120.0)
        scale = self.transform().m11() * factor
        if 0.08 <= scale <= 5.0:
            self.scale(factor, factor)
            self.user_zoomed = True

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Remember where a press started, to tell a click from a drag."""
        self._press = event.pos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Emit item_clicked for a release close to its press."""
        super().mouseReleaseEvent(event)
        if self._press is None or (event.pos() - self._press).manhattanLength() > 4:
            return
        self._press = None
        item = self.itemAt(event.pos())
        while item is not None and not isinstance(item, GraphItem):
            item = item.parentItem()
        self.item_clicked.emit(item.item_id if item is not None else "")

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Fit the drawing on a double click."""
        self.fit()


class _GraphPage(QWidget):
    """Toolbar, canvas and periodic refresh shared by both pages."""

    def __init__(self, interval_ms: int):
        super().__init__()
        self.setObjectName("root")
        self.canvas = GraphCanvas()
        self.canvas.item_clicked.connect(self._on_clicked)
        self.items: Dict[str, GraphItem] = {}
        self.edges: List[EdgeItem] = []
        self.selected = ""
        self._signature = None
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._interval = interval_ms

        self.toolbar = QFrame()
        self.toolbar.setObjectName("filterBar")
        self.toolbar_layout = QHBoxLayout(self.toolbar)
        self.toolbar_layout.setContentsMargins(24, 10, 24, 10)
        self.toolbar_layout.setSpacing(10)
        self.status = QLabel("")
        self.status.setObjectName("countPill")

    def _finish_toolbar(self, external: List[QPushButton]) -> None:
        self.toolbar_layout.addStretch()
        self.toolbar_layout.addWidget(self.status)
        fit = QPushButton("Fit")
        fit.setObjectName("btnGhost")
        fit.setToolTip("Fit the drawing in the view (also: double click)")
        fit.clicked.connect(self.canvas.fit)
        self.toolbar_layout.addWidget(fit)
        for b in external:
            self.toolbar_layout.addWidget(b)

    @staticmethod
    def _external_button(label: str, pkg: str, argv: List[str], mock: bool) -> QPushButton:
        b = QPushButton(label)
        b.setObjectName("btnGhost")
        available = not mock and ros_package_available(pkg)
        b.setEnabled(available)
        b.setToolTip(f"Start: ros2 {' '.join(argv)}" if available
                     else f"Package {pkg} is not installed in a sourced workspace")
        b.clicked.connect(lambda: QProcess.startDetached("ros2", argv))
        return b

    def set_active(self, active: bool) -> None:
        """Refresh periodically while the page is visible, not otherwise."""
        if active:
            self.refresh()
            self._timer.start(self._interval)
        else:
            self._timer.stop()

    def refresh(self) -> None:
        """Read new data and redraw; implemented by the pages."""

    def _draw(self, nodes: Dict[str, Tuple[str, str]], edges: List[Tuple[str, str, str]],
              rankdir: str) -> None:
        """Lay out and draw; nodes maps id to (label, kind)."""
        scene = self.canvas.scene()
        scene.clear()
        self.items, self.edges = {}, []
        if not nodes:
            return
        layout_nodes = [LayoutNode(i, *self.canvas.item_size(lbl))
                        for i, (lbl, _k) in nodes.items()]
        layout = layout_graph(layout_nodes, [LayoutEdge(s, d, lbl) for s, d, lbl in edges],
                              rankdir)
        for i, (lbl, kind) in nodes.items():
            item = GraphItem(i, lbl, kind, self.canvas.item_size(lbl), self.canvas.item_font)
            item.setPos(*layout.centers[i])
            scene.addItem(item)
            self.items[i] = item
        for route in layout.routes:
            if len(route.points) < 2:
                continue
            edge = EdgeItem(route, self.canvas.label_font)
            scene.addItem(edge)
            self.edges.append(edge)
        self.engine = layout.engine
        if not self.canvas.user_zoomed:
            self.canvas.fit()

    def _on_clicked(self, item_id: str) -> None:
        self.selected = item_id if item_id in self.items else ""
        self._apply_emphasis()

    def _apply_emphasis(self) -> None:
        if not self.selected:
            for item in self.items.values():
                item.set_emphasis(0)
            for e in self.edges:
                self._style_edge(e, False, False)
            return
        near = {self.selected}
        for e in self.edges:
            if self.selected in (e.src, e.dst):
                near.update((e.src, e.dst))
        for i, item in self.items.items():
            item.set_emphasis(1 if i == self.selected else (0 if i in near else -1))
        for e in self.edges:
            touching = self.selected in (e.src, e.dst)
            self._style_edge(e, touching, not touching)

    def _style_edge(self, edge: EdgeItem, highlight: bool, dim: bool) -> None:
        edge.set_style(GRAPH_EDGE_HIGHLIGHT if highlight else GRAPH_EDGE_COLOR,
                       2.0 if highlight else 1.2, opacity=0.2 if dim else 1.0)


def _legend(entries: List[Tuple[str, str]]) -> QWidget:
    row = QWidget()
    h = QHBoxLayout(row)
    h.setContentsMargins(24, 0, 24, 6)
    h.setSpacing(14)
    for state, text in entries:
        fill, border, _t = GRAPH_COLORS[state]
        swatch = QLabel()
        swatch.setFixedSize(12, 12)
        swatch.setStyleSheet(f"background:{fill}; border:1px solid {border}; border-radius:3px;")
        label = QLabel(text)
        label.setStyleSheet("color:#8b949e; font-size:11px;")
        h.addWidget(swatch)
        h.addWidget(label)
    h.addStretch()
    return row


# --- ROS graph --------------------------------------------------------------


class RosGraphPage(_GraphPage):
    """Nodes and topics of the live ROS graph."""

    def __init__(self, ros, launches: Callable[[], List[Launch]]):
        """Build the page; launches returns the launches for the filter."""
        super().__init__(GRAPH_REFRESH_MS)
        self._ros = ros
        self._launches = launches
        self._snap = None
        self.engine = ""

        self.search = QLineEdit()
        self.search.setObjectName("graphSearch")
        self.search.setPlaceholderText("Filter nodes or topics…")
        self.search.setMinimumWidth(220)
        self.search.textChanged.connect(lambda _t: self.refresh())
        self.toolbar_layout.addWidget(self.search)

        self.launch_combo = QComboBox()
        self.launch_combo.setObjectName("argEnum")
        self.launch_combo.setMinimumWidth(200)
        self.launch_combo.setToolTip(
            "Show only the nodes of one launch. Nodes are attributed through the logger\n"
            "and process names in the launch output; a node that never logs and whose\n"
            "name differs from its executable is not attributed.")
        self.launch_combo.addItem("All nodes", "")
        self.launch_combo.currentIndexChanged.connect(lambda _i: self.refresh())
        self.toolbar_layout.addWidget(self.launch_combo)

        self.topics_box = QCheckBox("Topics as boxes")
        self.topics_box.setChecked(True)
        self.topics_box.setToolTip("Unchecked: node-to-node arrows labelled with the topic")
        self.topics_box.toggled.connect(lambda _c: self.refresh())
        self.toolbar_layout.addWidget(self.topics_box)

        self.infra_box = QCheckBox("Hide /clock, /tf")
        self.infra_box.setChecked(True)
        self.infra_box.setToolTip("/clock and /tf reach almost every node; the TF tab shows /tf")
        self.infra_box.toggled.connect(lambda _c: self.refresh())
        self.toolbar_layout.addWidget(self.infra_box)

        self._finish_toolbar([self._external_button(
            "rqt_graph", "rqt_graph", ["run", "rqt_graph", "rqt_graph"], ros.is_mock)])

        self.details = QLabel()
        self.details.setObjectName("graphDetails")
        self.details.setWordWrap(True)
        self.details.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.details.setTextInteractionFlags(Qt.TextSelectableByMouse)
        details_scroll = QScrollArea()
        details_scroll.setObjectName("graphDetailsScroll")
        details_scroll.setWidgetResizable(True)
        details_scroll.setWidget(self.details)
        details_scroll.setFixedWidth(320)
        self.details_panel = details_scroll
        self.details_panel.setVisible(False)

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        body.addWidget(self.canvas, 1)
        body.addWidget(self.details_panel)

        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        v.addWidget(self.toolbar)
        v.addWidget(_legend([("node", "node"), ("topic", "topic"),
                             ("warn", "topic without publisher or subscriber"),
                             ("error", "several nodes with the same name")]))
        v.addLayout(body, 1)

    def _update_launch_combo(self) -> None:
        current = self.launch_combo.currentData()
        wanted = [la for la in self._launches()
                  if la.state in ("running", "stopping") or la.node_hints or la.process_hints]
        ids = [""] + [la.uid for la in wanted]
        if ids == [self.launch_combo.itemData(i) for i in range(self.launch_combo.count())]:
            return
        self.launch_combo.blockSignals(True)
        self.launch_combo.clear()
        self.launch_combo.addItem("All nodes", "")
        for la in wanted:
            self.launch_combo.addItem(f"{la.pkg} / {la.name}", la.uid)
        index = self.launch_combo.findData(current)
        self.launch_combo.setCurrentIndex(max(index, 0))
        self.launch_combo.blockSignals(False)

    def refresh(self) -> None:
        """Query the graph, filter it and redraw if its structure changed."""
        self._update_launch_combo()
        snap = self._snap = self._ros.graph_snapshot()
        nodes, edges, states, n_topics = self._filtered(snap)
        n_nodes = sum(1 for i in nodes if i.startswith("n:"))
        signature = (tuple(sorted(nodes.items())), tuple(sorted(edges)))
        if signature != self._signature:
            self._signature = signature
            self._draw(nodes, edges, "LR")
            if not nodes:
                self.canvas.show_message("No nodes match." if snap.nodes
                                         else "No ROS nodes found on this ROS_DOMAIN_ID.")
            if self.selected not in self.items:
                self.selected = ""
        for i, (state, tip) in states.items():
            if i in self.items:
                self.items[i].set_state(state, tip)
        self._apply_emphasis()
        self._show_details()
        engine = f" · layout: {self.engine}" if self.engine else ""
        self.status.setText(f"{n_nodes} nodes · {n_topics} topics · "
                            f"{time.strftime('%H:%M:%S')}{engine}")

    def _filtered(self, snap):
        """Return (items, edges, states, topic count) after the toolbar filters."""
        hidden = set(GRAPH_ALWAYS_HIDDEN_TOPICS)
        if self.infra_box.isChecked():
            hidden |= GRAPH_INFRASTRUCTURE_TOPICS
        topics = {t: info for t, info in snap.topics.items() if t not in hidden}

        node_set: Set[str] = set(snap.nodes)
        launch_id = self.launch_combo.currentData()
        if launch_id:
            launch = next((la for la in self._launches() if la.uid == launch_id), None)
            if launch is not None:
                node_set = {n for n in node_set if launch.owns_node(n)}
        topic_set = {t for t, info in topics.items()
                     if any(n in node_set for n in info.publishers + info.subscribers)}

        query = self.search.text().strip().lower()
        if query:
            seed_nodes = {n for n in node_set if query in n.lower()}
            seed_topics = {t for t in topic_set if query in t.lower()}
            keep_topics = seed_topics | {
                t for n in seed_nodes
                for t in snap.nodes[n].publishes + snap.nodes[n].subscribes if t in topic_set}
            keep_nodes = seed_nodes | {n for t in keep_topics
                                       for n in topics[t].publishers + topics[t].subscribers
                                       if n in node_set}
            node_set, topic_set = keep_nodes, keep_topics

        items: Dict[str, Tuple[str, str]] = {}
        states: Dict[str, Tuple[str, str]] = {}
        edges: List[Tuple[str, str, str]] = []
        for n in node_set:
            info = snap.nodes[n]
            items["n:" + n] = (n, "node")
            tip = f"{n}\npublishes {len(set(info.publishes))} topic(s), " \
                  f"subscribes {len(set(info.subscribes))}"
            states["n:" + n] = ("error", tip + "\nSeveral nodes have this name.") \
                if info.duplicate else ("normal", tip)
        if self.topics_box.isChecked():
            for t in topic_set:
                info = topics[t]
                items["t:" + t] = (t, "topic")
                types = ", ".join(sorted(info.types)) or "unknown type"
                tip = f"{t}\n{types}\n{len(info.publishers)} publisher(s), " \
                      f"{len(info.subscribers)} subscriber(s)"
                if not info.publishers:
                    states["t:" + t] = ("warn", tip + "\nNo publisher.")
                elif not info.subscribers:
                    states["t:" + t] = ("warn", tip + "\nNo subscriber.")
                else:
                    states["t:" + t] = ("normal", tip)
                edges += [("n:" + p, "t:" + t, "") for p in set(info.publishers) if p in node_set]
                edges += [("t:" + t, "n:" + s, "") for s in set(info.subscribers) if s in node_set]
        else:
            for t in topic_set:
                info = topics[t]
                edges += [("n:" + p, "n:" + s, t) for p in set(info.publishers)
                          for s in set(info.subscribers)
                          if p in node_set and s in node_set and p != s]
        return items, sorted(set(edges)), states, len(topic_set)

    def _on_clicked(self, item_id: str) -> None:
        super()._on_clicked(item_id)
        self._show_details()

    def _show_details(self) -> None:
        snap = self._snap
        if not self.selected or snap is None:
            self.details_panel.setVisible(False)
            return
        kind, name = self.selected.split(":", 1)
        esc = html.escape
        if kind == "n" and name in snap.nodes:
            info = snap.nodes[name]
            body = [f"<p class='k'>NODE</p><p><b>{esc(name)}</b></p>"]
            for title, topics in (("Publishes", info.publishes), ("Subscribes", info.subscribes)):
                uniq = sorted(set(topics))
                body.append(f"<p class='k'>{title.upper()} ({len(uniq)})</p>")
                body += [f"<p>{esc(t)}<br><span class='t'>{esc(self._type(t))}</span></p>"
                         for t in uniq]
            cli = [f"ros2 node info {name}"]
        elif kind == "t" and name in snap.topics:
            info = snap.topics[name]
            body = [f"<p class='k'>TOPIC</p><p><b>{esc(name)}</b><br>"
                    f"<span class='t'>{esc(', '.join(sorted(info.types)))}</span></p>"]
            for title, nodes in (("Publishers", info.publishers),
                                 ("Subscribers", info.subscribers)):
                uniq = sorted(set(nodes))
                body.append(f"<p class='k'>{title.upper()} ({len(uniq)})</p>")
                body += [f"<p>{esc(n)}</p>" for n in uniq] or ["<p class='t'>none</p>"]
            cli = [f"ros2 topic info {name} -v", f"ros2 topic echo {name}",
                   f"ros2 topic hz {name}"]
        else:
            self.details_panel.setVisible(False)
            return
        body.append("<p class='k'>COMMAND LINE</p>")
        body += [f"<p><code>{esc(c)}</code></p>" for c in cli]
        style = ("<style>p{margin:2px 0 6px 0;} .k{color:#8b949e;font-size:10px;"
                 "letter-spacing:1px;margin-top:10px;} .t{color:#8b949e;font-size:11px;}"
                 "code{color:#79c0ff;}</style>")
        self.details.setText(style + "".join(body))
        self.details_panel.setVisible(True)

    def _type(self, topic: str) -> str:
        info = self._snap.topics.get(topic) if self._snap else None
        return ", ".join(sorted(info.types)) if info else ""


# --- TF tree ----------------------------------------------------------------


class TfTreePage(_GraphPage):
    """Frames and transforms seen on /tf and /tf_static, with diagnostics."""

    def __init__(self, ros):
        """Build the page around a RosInterface or MockRosInterface."""
        super().__init__(TF_REFRESH_MS)
        self._ros = ros
        self.engine = ""
        self._edge_items: Dict[Tuple[str, str], EdgeItem] = {}

        clear = QPushButton("Clear")
        clear.setObjectName("btnGhost")
        clear.setToolTip("Forget the transforms received so far")
        clear.clicked.connect(self._clear)
        self.toolbar_layout.addWidget(clear)
        self._finish_toolbar([self._external_button("RViz", "rviz2", ["run", "rviz2", "rviz2"],
                                                    ros.is_mock)])

        self.diagnostics = QLabel()
        self.diagnostics.setObjectName("tfDiagnostics")
        self.diagnostics.setWordWrap(True)
        self.diagnostics.setTextInteractionFlags(Qt.TextSelectableByMouse)

        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        v.addWidget(self.toolbar)
        v.addWidget(_legend([("frame", "frame"), ("warn", "no update for "
                                                  f"{TF_STALE_S:.0f} s (stale)"),
                             ("error", "frame with more than one parent")]))
        v.addWidget(self.canvas, 1)
        v.addWidget(self.diagnostics)

    def set_active(self, active: bool) -> None:
        """Subscribe to /tf on first use, then refresh while visible."""
        if active:
            self._ros.start_tf()
        super().set_active(active)

    def _clear(self) -> None:
        self._ros.clear_tf()
        self._signature = None
        self.refresh()

    def refresh(self) -> None:
        """Read the TF store and redraw or update in place."""
        edges = self._ros.tf_snapshot()
        conflicts, stale, roots, frames = analyse_tf(edges)
        signature = tuple(sorted((e.parent, e.child) for e in edges))
        changed = signature != self._signature
        if changed:
            self._signature = signature
            nodes = {"f:" + f: (f, "frame") for f in frames}
            # A placeholder label makes the layout reserve room for the rate.
            self._draw(nodes, [("f:" + e.parent, "f:" + e.child, "00.0 Hz") for e in edges], "TB")
            self._edge_items = {(e.src[2:], e.dst[2:]): e for e in self.edges}
            if not edges:
                self.canvas.show_message("No transforms received on /tf or /tf_static yet.")

        for f in frames:
            item = self.items.get("f:" + f)
            if item is None:
                continue
            if f in conflicts:
                item.set_state("error", f"{f}\nparents: {', '.join(sorted(conflicts[f]))}")
            elif any(c == f for (_p, c) in stale):
                item.set_state("warn", f"{f}\nits transform is stale")
            else:
                item.set_state("normal", f)
        for e in edges:
            item = self._edge_items.get((e.parent, e.child))
            if item is None:
                continue
            key = (e.parent, e.child)
            if e.static:
                item.set_label("static")
            elif key in stale:
                item.set_label(f"stale {e.age_s:.0f} s")
            else:
                item.set_label(f"{e.rate_hz:.1f} Hz")
            if e.child in conflicts:
                item.set_style(GRAPH_COLORS["error"][1], 2.0)
            elif key in stale:
                item.set_style(GRAPH_COLORS["warn"][1], 1.4, dashed=True)
            else:
                item.set_style(GRAPH_EDGE_COLOR)
        self._update_diagnostics(edges, conflicts, stale, roots, frames)
        if changed and edges and not self.canvas.user_zoomed:
            # The diagnostics text changes the canvas height; fit once laid out.
            QTimer.singleShot(0, self.canvas.fit)
        engine = f" · layout: {self.engine}" if self.engine else ""
        self.status.setText(f"{len(frames)} frames · {len(edges)} transforms · "
                            f"{time.strftime('%H:%M:%S')}{engine}")

    def _apply_emphasis(self) -> None:
        # Edge colours carry the diagnostics here; selection only dims frames.
        near = {self.selected}
        for e in self.edges:
            if self.selected in (e.src, e.dst):
                near.update((e.src, e.dst))
        for i, item in self.items.items():
            item.set_emphasis(0 if not self.selected or i in near else -1)
        if self.selected:
            self.items[self.selected].set_emphasis(1)

    def _update_diagnostics(self, edges, conflicts, stale, roots, frames) -> None:
        esc = html.escape
        lines = []
        for child, parents in sorted(conflicts.items()):
            lines.append(
                f"<span style='color:#f85149'>●</span> <b>{esc(child)}</b> has "
                f"{len(parents)} parents ({esc(', '.join(sorted(parents)))}). Each frame has "
                "exactly one parent; more than one node publishes a transform to this frame.")
        for parent, child in sorted(stale):
            age = next(e.age_s for e in edges if (e.parent, e.child) == (parent, child))
            lines.append(f"<span style='color:#d29922'>●</span> {esc(parent)} → "
                         f"{esc(child)}: no update for {age:.1f} s.")
        if len(roots) > 1:
            lines.append(
                f"<span style='color:#d29922'>●</span> {len(roots)} separate trees "
                f"(roots: {esc(', '.join(roots))}). No transform exists between frames of "
                "different trees.")
        if not lines:
            lines.append(f"<span style='color:#56d364'>●</span> One tree, {len(frames)} "
                         "frames, no conflicts." if edges else
                         "<span style='color:#8b949e'>●</span> Waiting for transforms "
                         "on /tf and /tf_static.")
        self.diagnostics.setText("<br>".join(lines))
