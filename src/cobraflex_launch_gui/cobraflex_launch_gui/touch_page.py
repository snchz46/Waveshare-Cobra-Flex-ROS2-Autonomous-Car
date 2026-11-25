"""Launches page of the touch density: the list of launches beside the detail of the selected one.

On the 7 inch screen of the robot the cards of the desktop page leave room
for two or three launches, and their arguments open inside the card. Here a
row of the list only selects; the action button sits beside the name of the
selected launch, and its arguments, output and command fill the tabs of the
detail pane.
"""

import time
from typing import Dict, List, Optional

from PyQt5.QtCore import pyqtSignal, QEvent, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QMouseEvent, QPainter, QPainterPath
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QStyle,
    QStyleOptionButton,
    QStylePainter,
    QVBoxLayout,
    QWidget,
)

from cobraflex_launch_gui.constants import FILTER_CHIPS, FILTER_CHIPS_SHORT, HOLD_TO_RUN_MS
from cobraflex_launch_gui.density import margins, px, size
from cobraflex_launch_gui.model import build_command, Launch
from cobraflex_launch_gui.theme import PALETTE
from cobraflex_launch_gui.widgets import (
    enable_kinetic_scroll,
    ExpandableSection,
    fill_status_row,
    LogsPanel,
    make_state_dot,
    restyle,
    SectionHeader,
    set_icon,
    set_letter_spacing,
    set_star,
)


class ElidedLabel(QLabel):
    """Single-line label that ends with an ellipsis when its text does not fit."""

    def __init__(self, text: str = "", name: str = ""):
        """Create the label; name is its object name for the style sheet."""
        super().__init__()
        if name:
            self.setObjectName(name)
        self._full = ""
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.set_full_text(text)

    def set_full_text(self, text: str) -> None:
        """Set the text, elided to the width of the label."""
        self._full = text
        self._elide()

    def _elide(self) -> None:
        text = self.fontMetrics().elidedText(self._full, Qt.ElideRight, max(0, self.width()))
        if text != self.text():
            self.setText(text)

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Elide again for the new width."""
        super().resizeEvent(event)
        self._elide()

    def changeEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Elide again when the style sheet changes the font."""
        super().changeEvent(event)
        if event.type() in (QEvent.FontChange, QEvent.StyleChange):
            self._elide()


# --- List -------------------------------------------------------------------


class LaunchRow(QFrame):
    """One launch of the list: state dot, name, and a line with the state or the description."""

    selected_signal = pyqtSignal(str)

    def __init__(self, launch: Launch):
        """Build the row for one launch."""
        super().__init__()
        self.launch = launch
        self.setObjectName("launchRow")
        self.setProperty("selected", "false")
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(size("row_h"))
        h = QHBoxLayout(self)
        h.setContentsMargins(*margins(14, 6, 12, 6))
        h.setSpacing(px(12))
        holder = QWidget()
        holder.setFixedWidth(px(16))
        self._dot_layout = QHBoxLayout(holder)
        self._dot_layout.setContentsMargins(0, 0, 0, 0)
        self._dot: Optional[QWidget] = None
        self._dot_state = ""
        h.addWidget(holder)
        col = QVBoxLayout()
        col.setSpacing(px(2))
        self.name = ElidedLabel(launch.name, "rowName")
        self.sub = ElidedLabel("", "rowSub")
        col.addWidget(self.name)
        col.addWidget(self.sub)
        h.addLayout(col, 1)
        self.refresh()

    def refresh(self) -> None:
        """Show the state: dot, and running time, exit code or description."""
        la = self.launch
        if la.state != self._dot_state:
            if self._dot is not None:
                self._dot.setParent(None)
            self._dot = make_state_dot(la.state)
            self._dot_layout.addWidget(self._dot)
            self._dot_state = la.state
        if la.state == "running":
            mm, ss = divmod(la.uptime, 60)
            text = f"Running {mm:02d}:{ss:02d} · {la.pkg}"
        elif la.state == "stopping":
            text = f"Stopping… · {la.pkg}"
        elif la.state == "failed":
            code = f" · exit {la.exit_code}" if la.exit_code is not None else ""
            text = f"Failed{code} · {la.pkg}"
        else:
            text = f"{la.pkg} · {la.desc}"
        self.sub.set_full_text(text)
        if self.sub.property("state") != la.state:
            self.sub.setProperty("state", la.state)
            restyle(self.sub)

    def set_selected(self, selected: bool) -> None:
        """Mark the row as the launch shown in the detail pane."""
        self.setProperty("selected", "true" if selected else "false")
        restyle(self)

    def mousePressEvent(self, e: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """Accept the press, so that its release reaches the row."""
        e.accept()

    def mouseReleaseEvent(self, e: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """Select the row when a press ends inside it; a kinetic scroll ends outside."""
        if self.rect().contains(e.pos()):
            self.selected_signal.emit(self.launch.uid)


class LaunchList(QFrame):
    """Filter buttons above the scrolling list of launches, pinned ones first.

    The filter selects one tag at a time: one tap shows the launches of the
    robot, where the multiple-choice chips of the desktop need four.
    """

    selected_signal = pyqtSignal(str)

    def __init__(self):
        """Build an empty list; rebuild() fills it."""
        super().__init__()
        self.setObjectName("listPane")
        self.setFixedWidth(size("list_w"))
        #: Tag shown; empty for every launch.
        self.filter_tag = ""
        self.rows: Dict[str, LaunchRow] = {}
        self._launches: List[Launch] = []
        self._selected = ""
        self._filter_buttons: Dict[str, QPushButton] = {}

        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        seg_wrap = QFrame()
        seg_wrap.setObjectName("filterSegWrap")
        self._seg_layout = QHBoxLayout(seg_wrap)
        self._seg_layout.setContentsMargins(*margins(10, 8, 10, 8))
        self._seg_layout.setSpacing(0)
        v.addWidget(seg_wrap)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        body = QWidget()
        body.setObjectName("root")
        self._body = QVBoxLayout(body)
        self._body.setContentsMargins(*margins(0, 0, 0, 12))
        self._body.setSpacing(0)
        self.scroll.setWidget(body)
        enable_kinetic_scroll(self.scroll.viewport())
        v.addWidget(self.scroll, 1)

    def rebuild(self, launches: List[Launch], selected: str) -> None:
        """Fill the filter and the rows from scratch."""
        self._launches = launches
        self._selected = selected
        self._build_filter()
        self._fill()

    def _build_filter(self) -> None:
        while self._seg_layout.count():
            w = self._seg_layout.takeAt(0).widget()
            if w is not None:
                w.setParent(None)
        tags = [(tag, FILTER_CHIPS_SHORT.get(tag, label)) for tag, label in FILTER_CHIPS
                if any(tag in la.tags for la in self._launches)]
        if self.filter_tag not in [t for t, _l in tags]:
            self.filter_tag = ""
        self._filter_buttons = {}
        for tag, label in [("", "All")] + tags:
            b = QPushButton(label)
            b.setObjectName("filterSeg")
            b.setCheckable(True)
            b.setChecked(tag == self.filter_tag)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _ck, t=tag: self._set_filter(t))
            self._filter_buttons[tag] = b
            self._seg_layout.addWidget(b, 1)

    def _set_filter(self, tag: str) -> None:
        self.filter_tag = tag
        for t, b in self._filter_buttons.items():
            b.setChecked(t == tag)
        self._fill()
        self.scroll.verticalScrollBar().setValue(0)

    def _fill(self) -> None:
        while self._body.count():
            w = self._body.takeAt(0).widget()
            if w is not None:
                w.setParent(None)
        self.rows = {}
        shown = [la for la in self._launches if not self.filter_tag or self.filter_tag in la.tags]
        pinned = [la for la in shown if la.starred]
        rest = [la for la in shown if not la.starred]
        if not shown:
            empty = QLabel("No launches with this tag.")
            empty.setObjectName("empty")
            empty.setWordWrap(True)
            self._body.addWidget(empty)
        if pinned:
            self._add_header("PINNED", len(pinned), "star_filled")
            for la in pinned:
                self._add_row(la)
        for pkg in sorted({la.pkg for la in rest}):
            group = [la for la in rest if la.pkg == pkg]
            self._add_header(pkg.upper(), len(group))
            for la in group:
                self._add_row(la)
        self._body.addStretch()

    def _add_header(self, label: str, count: int, icon_name: str = "") -> None:
        holder = QWidget()
        h = QHBoxLayout(holder)
        h.setContentsMargins(*margins(14, 10, 12, 0))
        h.addWidget(SectionHeader(label, count, icon_name))
        self._body.addWidget(holder)

    def _add_row(self, la: Launch) -> None:
        row = LaunchRow(la)
        row.set_selected(la.uid == self._selected)
        row.selected_signal.connect(self.selected_signal)
        self.rows[la.uid] = row
        self._body.addWidget(row)

    def set_selected(self, uid: str) -> None:
        """Mark the row of uid as selected and scroll it into view."""
        previous = self.rows.get(self._selected)
        if previous is not None:
            previous.set_selected(False)
        self._selected = uid
        row = self.rows.get(uid)
        if row is not None:
            row.set_selected(True)
            self.scroll.ensureWidgetVisible(row, 0, px(24))

    def refresh(self, launch: Launch) -> None:
        """Redraw the row of one launch after a change of state or of its uptime."""
        row = self.rows.get(launch.uid)
        if row is not None:
            row.refresh()


# --- Detail -----------------------------------------------------------------


class HoldButton(QPushButton):
    """Button that fires only once held down for HOLD_TO_RUN_MS; a fill shows the progress.

    For the launches that drive the robot: a tap that brushes the screen must
    not start them. Releasing early, or sliding off the button, cancels.
    """

    held = pyqtSignal()
    released_early = pyqtSignal()

    def __init__(self, text: str, hold_ms: int = HOLD_TO_RUN_MS):
        """Create the button; hold_ms is the press duration that fires it."""
        super().__init__(text)
        self._hold_ms = hold_ms
        self._progress = 0.0
        self._t0 = 0.0
        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._step)

    def mousePressEvent(self, event: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """Start the fill."""
        super().mousePressEvent(event)
        if event.button() == Qt.LeftButton and self.isEnabled():
            self._t0 = time.monotonic()
            self._progress = 0.0
            self._timer.start()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """Cancel when the finger slides off the button."""
        super().mouseMoveEvent(event)
        if self._timer.isActive() and not self.rect().contains(event.pos()):
            self._cancel()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """Cancel a hold released before the end."""
        super().mouseReleaseEvent(event)
        if self._timer.isActive():
            self._cancel()

    def _cancel(self) -> None:
        self._timer.stop()
        self._progress = 0.0
        self.update()
        self.released_early.emit()

    def _step(self) -> None:
        self._progress = min(1.0, (time.monotonic() - self._t0) * 1000.0 / self._hold_ms)
        self.update()
        if self._progress >= 1.0:
            self._timer.stop()
            self._progress = 0.0
            self.update()
            self.held.emit()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Draw the button and the fill of the hold, then the label again over the fill."""
        super().paintEvent(event)
        if self._progress <= 0.0:
            return
        whole = QRectF(self.rect())
        fill = QRectF(whole)
        fill.setWidth(whole.width() * self._progress)
        painter = QStylePainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        outline = QPainterPath()
        outline.addRoundedRect(whole, px(6), px(6))
        painter.setClipPath(outline)
        # A darkening rather than a colour of its own: while the finger rests
        # on the button it is hovered, and the hover colour is the dark green.
        painter.fillRect(fill, QColor(0, 0, 0, 100))
        # The fill covers the label: draw the label again, over the fill only.
        painter.setClipRect(fill, Qt.IntersectClip)
        option = QStyleOptionButton()
        self.initStyleOption(option)
        option.rect = self.style().subElementRect(QStyle.SE_PushButtonContents, option, self)
        painter.drawControl(QStyle.CE_PushButtonLabel, option)
        painter.end()


class LaunchDetail(QFrame):
    """Name, state and action of the selected launch, with Arguments, Output and Command tabs."""

    run_signal = pyqtSignal(str)
    stop_signal = pyqtSignal(str)
    restart_signal = pyqtSignal(str)
    star_signal = pyqtSignal(str)
    presets_changed = pyqtSignal(object)
    #: Text of a hint, for a hold released before it started the launch.
    hold_hint = pyqtSignal(str)

    TABS = ("Arguments", "Output", "Command")

    def __init__(self, logs_panel: LogsPanel):
        """Build the pane around the log panel of the Output tab."""
        super().__init__()
        self.setObjectName("detail")
        self.launch: Optional[Launch] = None
        self.section: Optional[ExpandableSection] = None
        v = QVBoxLayout(self)
        v.setContentsMargins(*margins(18, 10, 14, 0))
        v.setSpacing(px(6))

        top = QHBoxLayout()
        top.setSpacing(px(10))
        titles = QVBoxLayout()
        titles.setSpacing(px(2))
        self.name = ElidedLabel("", "detailName")
        self.meta = ElidedLabel("", "detailMeta")
        self.status_row = QHBoxLayout()
        self.status_row.setSpacing(px(6))
        titles.addWidget(self.name)
        titles.addWidget(self.meta)
        titles.addLayout(self.status_row)
        top.addLayout(titles, 1)

        on_dark = PALETTE["c_on_dark"]
        self.run_btn = self._action_button("Run", "btnRun", "Start the launch")
        set_icon(self.run_btn, "play", on_dark, 16)
        self.run_btn.clicked.connect(lambda: self._emit(self.run_signal))
        self.hold_btn = HoldButton("Hold to run")
        self.hold_btn.setObjectName("btnRun")
        self.hold_btn.setCursor(Qt.PointingHandCursor)
        self.hold_btn.setMinimumWidth(px(150))
        self.hold_btn.setToolTip("This launch drives the robot: hold the button for 1 s")
        set_icon(self.hold_btn, "play", on_dark, 16)
        self.hold_btn.held.connect(lambda: self._emit(self.run_signal))
        self.hold_btn.released_early.connect(lambda: self.hold_hint.emit(
            "Hold the button for 1 s: this launch drives the robot"))
        self.stop_btn = self._action_button("Stop", "btnStop", "Stop the launch (SIGINT)")
        set_icon(self.stop_btn, "stop", on_dark, 16)
        self.stop_btn.clicked.connect(lambda: self._emit(self.stop_signal))
        self.restart_btn = self._action_button("", "btnRestart", "Restart the launch")
        set_icon(self.restart_btn, "restart", PALETTE["c_tyre"], 20)
        # Square: an icon-only button still needs a finger-wide target.
        self.restart_btn.setMinimumWidth(px(56))
        self.restart_btn.clicked.connect(lambda: self._emit(self.restart_signal))
        for b in (self.run_btn, self.hold_btn, self.stop_btn, self.restart_btn):
            top.addWidget(b, 0, Qt.AlignTop)
        v.addLayout(top)

        self.err = QLabel()
        self.err.setObjectName("cardErrSnippet")
        self.err.setWordWrap(True)
        self.err.setVisible(False)
        v.addWidget(self.err)

        tabs = QHBoxLayout()
        tabs.setSpacing(px(2))
        self.tab_buttons: List[QPushButton] = []
        for i, label in enumerate(self.TABS):
            b = QPushButton(label)
            b.setObjectName("tab")
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _ck, idx=i: self.show_tab(idx))
            self.tab_buttons.append(b)
            tabs.addWidget(b)
        tabs.addStretch()
        self.star_btn = QPushButton()
        self.star_btn.setObjectName("cardStar")
        self.star_btn.setCursor(Qt.PointingHandCursor)
        self.star_btn.setToolTip("Pin / unpin")
        self.star_btn.clicked.connect(lambda: self._emit(self.star_signal))
        tabs.addWidget(self.star_btn)
        v.addLayout(tabs)
        rule = QFrame()
        rule.setObjectName("sectionRule")
        rule.setFixedHeight(1)
        v.addWidget(rule)

        self.stack = QStackedWidget()
        self.args_scroll = QScrollArea()
        self.args_scroll.setWidgetResizable(True)
        self.args_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        enable_kinetic_scroll(self.args_scroll.viewport())
        self.stack.addWidget(self.args_scroll)
        self.logs_panel = logs_panel
        self.stack.addWidget(logs_panel)
        self.stack.addWidget(self._build_command_page())
        v.addWidget(self.stack, 1)
        self.show_tab(0)
        self.set_launch(None)

    def _action_button(self, text: str, name: str, tip: str) -> QPushButton:
        b = QPushButton(text)
        b.setObjectName(name)
        b.setCursor(Qt.PointingHandCursor)
        b.setToolTip(tip)
        b.setMinimumWidth(px(150))
        return b

    def _build_command_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("detailPage")
        v = QVBoxLayout(page)
        v.setContentsMargins(*margins(0, 12, 8, 12))
        v.setSpacing(px(10))
        head = QHBoxLayout()
        label = QLabel("COMMAND")
        label.setObjectName("expandLabel")
        set_letter_spacing(label, 1.2)
        head.addWidget(label)
        head.addStretch()
        copy = QPushButton("Copy")
        copy.setObjectName("btnGhost")
        copy.setCursor(Qt.PointingHandCursor)
        copy.setToolTip("Copy the command line to the clipboard")
        copy.clicked.connect(self._copy_command)
        head.addWidget(copy)
        v.addLayout(head)
        self.cmd_box = QLabel()
        self.cmd_box.setObjectName("cmdBox")
        self.cmd_box.setWordWrap(True)
        self.cmd_box.setTextInteractionFlags(Qt.TextSelectableByMouse)
        v.addWidget(self.cmd_box)
        self.path_label = QLabel()
        self.path_label.setObjectName("cardWs")
        self.path_label.setWordWrap(True)
        self.path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        v.addWidget(self.path_label)
        v.addStretch()
        return page

    def _emit(self, signal) -> None:
        if self.launch is not None:
            signal.emit(self.launch.uid)

    def _copy_command(self) -> None:
        if self.launch is not None:
            QApplication.clipboard().setText(build_command(self.launch))

    def _refresh_command(self) -> None:
        if self.launch is not None:
            self.cmd_box.setText(build_command(self.launch))
            self.path_label.setText(str(self.launch.file_path))

    def show_tab(self, index: int) -> None:
        """Show the Arguments (0), Output (1) or Command (2) tab."""
        for i, b in enumerate(self.tab_buttons):
            b.setChecked(i == index)
        self.stack.setCurrentIndex(index)

    def set_launch(self, launch: Optional[Launch]) -> None:
        """Show another launch: header, argument editors and command."""
        self.launch = launch
        old = self.args_scroll.takeWidget()
        if old is not None:
            old.deleteLater()
        self.section = None
        self.setEnabled(launch is not None)
        if launch is None:
            self.name.set_full_text("No launch selected")
            self.meta.set_full_text("")
            for b in (self.run_btn, self.hold_btn, self.stop_btn, self.restart_btn):
                b.setVisible(False)
            return
        page = QWidget()
        page.setObjectName("detailPage")
        pv = QVBoxLayout(page)
        pv.setContentsMargins(*margins(0, 10, 8, 0))
        pv.setSpacing(px(6))
        desc = QLabel(launch.desc)
        desc.setObjectName("cardDesc")
        desc.setWordWrap(True)
        pv.addWidget(desc)
        self.section = ExpandableSection(launch, embedded=True)
        self.section.presets_changed.connect(self.presets_changed)
        self.section.command_changed.connect(self._refresh_command)
        pv.addWidget(self.section)
        pv.addStretch()
        self.args_scroll.setWidget(page)
        self.name.set_full_text(launch.name)
        fmt = f" · {launch.file_format}" if launch.file_format != "py" else ""
        tags = ", ".join("Robot" if t == "Hardware" else t for t in launch.tags)
        self.meta.set_full_text(f"{launch.pkg}{fmt} · {tags}")
        self._refresh_command()
        self.refresh()

    def refresh(self) -> None:
        """Show the state: status line, error, pin, and the button of the next action."""
        la = self.launch
        if la is None:
            return
        fill_status_row(self.status_row, la)
        show_err = la.last_error and (la.state == "failed" or la.died)
        self.err.setText(la.last_error or "")
        self.err.setVisible(bool(show_err))
        active = la.state in ("running", "stopping")
        stopping = la.state == "stopping"
        verb = "Retry" if la.state == "failed" else "Run"
        self.run_btn.setText(verb)
        self.run_btn.setObjectName("btnRetry" if la.state == "failed" else "btnRun")
        restyle(self.run_btn)
        self.hold_btn.setText(f"Hold to {verb.lower()}")
        self.run_btn.setVisible(not active and not la.moves_robot)
        self.hold_btn.setVisible(not active and la.moves_robot)
        self.stop_btn.setText("Stopping…" if stopping else "Stop")
        self.stop_btn.setEnabled(not stopping)
        self.stop_btn.setVisible(active)
        self.restart_btn.setEnabled(not stopping)
        self.restart_btn.setVisible(active)
        set_star(self.star_btn, la.starred)

    def show_missing(self) -> None:
        """Open the Arguments tab on the first required argument without a value."""
        self.show_tab(0)
        if self.section is not None:
            row = self.section.mark_missing()
            if row is not None:
                self.args_scroll.ensureWidgetVisible(row)


# --- Page -------------------------------------------------------------------


class TouchLaunchesPage(QWidget):
    """List of launches beside the detail of the selected one."""

    selected_signal = pyqtSignal(str)
    run_signal = pyqtSignal(str)
    stop_signal = pyqtSignal(str)
    restart_signal = pyqtSignal(str)
    star_signal = pyqtSignal(str)
    presets_changed = pyqtSignal(object)
    hold_hint = pyqtSignal(str)

    def __init__(self, logs_panel: LogsPanel):
        """Build the page; logs_panel becomes the Output tab of the detail pane."""
        super().__init__()
        self.setObjectName("root")
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(0)
        self.launch_list = LaunchList()
        self.detail = LaunchDetail(logs_panel)
        h.addWidget(self.launch_list)
        h.addWidget(self.detail, 1)
        self.launch_list.selected_signal.connect(self.selected_signal)
        self.detail.run_signal.connect(self.run_signal)
        self.detail.stop_signal.connect(self.stop_signal)
        self.detail.restart_signal.connect(self.restart_signal)
        self.detail.star_signal.connect(self.star_signal)
        self.detail.presets_changed.connect(self.presets_changed)
        self.detail.hold_hint.connect(self.hold_hint)

    def rebuild(self, launches: List[Launch], selected: str) -> None:
        """Fill the list again (pins changed) and show the selected launch."""
        self.launch_list.rebuild(launches, selected)
        launch = next((la for la in launches if la.uid == selected), None)
        if launch is not self.detail.launch:
            self.detail.set_launch(launch)
        else:
            self.detail.refresh()

    def select(self, launch: Launch) -> None:
        """Show another launch in the detail pane."""
        self.launch_list.set_selected(launch.uid)
        self.detail.set_launch(launch)

    def refresh_launch(self, launch: Launch) -> None:
        """Redraw a launch after a change of state or of its uptime."""
        self.launch_list.refresh(launch)
        if launch is self.detail.launch:
            self.detail.refresh()

    def show_missing(self) -> None:
        """Point at the required arguments of the selected launch that have no value."""
        self.detail.show_missing()
