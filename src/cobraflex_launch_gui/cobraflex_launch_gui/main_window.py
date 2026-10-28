"""Main window: environment strip, header, and the Launches, ROS Graph and TF tabs."""

import os
import socket
from typing import Dict, List, Optional, Set

from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QShortcut,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from cobraflex_launch_gui.backend import Backend
from cobraflex_launch_gui.constants import (
    DEFAULT_WINDOW_HEIGHT,
    DEFAULT_WINDOW_WIDTH,
    FILTER_CHIPS,
    MIN_WINDOW_HEIGHT,
    MIN_WINDOW_WIDTH,
    UPTIME_TICK_MS,
)
from cobraflex_launch_gui.graph_view import RosGraphPage, TfTreePage
from cobraflex_launch_gui.model import Launch, LogEntry, missing_required
from cobraflex_launch_gui.state import UserState
from cobraflex_launch_gui.widgets import (
    CommandPalette,
    LaunchCard,
    LogsPanel,
    make_green_dot,
    restyle,
    SectionHeader,
    set_letter_spacing,
    Toast,
)


class LaunchGui(QMainWindow):
    """Launch manager window."""

    def __init__(self, backend: Backend, launches: List[Launch], ros, state: UserState,
                 workspaces: List[str]):
        """Build the window around a backend, the discovered launches and a ROS interface."""
        super().__init__()
        self.setWindowTitle("CobraFlex · Launch Manager")
        self.setMinimumSize(MIN_WINDOW_WIDTH, MIN_WINDOW_HEIGHT)
        self.resize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT)

        self.backend = backend
        self.backend.on_state_changed = self._on_backend_state
        self.backend.on_log_line = self._on_backend_log
        self.launches = launches
        self.state = state
        self._workspaces = workspaces
        self.query = ""
        self.active_chips: Set[str] = {tag for tag, _l in FILTER_CHIPS}
        self.selected_id = launches[0].uid if launches else ""
        self.card_widgets: Dict[str, LaunchCard] = {}
        self._toasts: List[Toast] = []
        self._palette: Optional[CommandPalette] = None
        self._previous_state: Dict[str, str] = {la.uid: la.state for la in launches}
        self._force_close = False

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._build_env_strip())
        outer.addWidget(self._build_header())
        outer.addWidget(self._build_tab_bar())

        launches_page = QWidget()
        launches_page.setObjectName("root")
        lp = QVBoxLayout(launches_page)
        lp.setContentsMargins(0, 0, 0, 0)
        lp.setSpacing(0)
        lp.addWidget(self._build_filter())
        lp.addWidget(self._build_list(), 1)
        self.logs_panel = LogsPanel()
        self.logs_panel.clear_requested.connect(self._clear_logs)
        lp.addWidget(self.logs_panel)

        self.graph_page = RosGraphPage(ros, lambda: self.launches)
        self.tf_page = TfTreePage(ros)
        self.stack = QStackedWidget()
        for page in (launches_page, self.graph_page, self.tf_page):
            self.stack.addWidget(page)
        outer.addWidget(self.stack, 1)

        self._rebuild_list()
        self._refresh_logs_subject()

        self.uptime_timer = QTimer(self)
        self.uptime_timer.timeout.connect(self._tick_uptime)
        self.uptime_timer.start(UPTIME_TICK_MS)
        self._install_shortcuts()

    # Lifecycle --------------------------------------------------------------

    def force_close(self) -> None:
        """Close without asking, stopping every launch (SIGINT, SIGTERM)."""
        self._force_close = True
        self.close()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Ask before closing over running launches, then stop them all."""
        active = [la for la in self.launches if la.state in ("running", "stopping")]
        if active and not self._force_close:
            names = "\n".join(f"  {la.pkg} / {la.name}" for la in active)
            answer = QMessageBox.question(
                self, "Running launches",
                f"{len(active)} launch(es) still running:\n{names}\n\nStop them and quit?",
                QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        self.graph_page.set_active(False)
        self.tf_page.set_active(False)
        self.backend.shutdown()
        super().closeEvent(event)

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt API
        """Keep the toasts in the bottom right corner."""
        super().resizeEvent(event)
        self._layout_toasts()

    # Environment strip and header ------------------------------------------

    def _build_env_strip(self) -> QWidget:
        f = QFrame()
        f.setObjectName("envStrip")
        h = QHBoxLayout(f)
        h.setContentsMargins(24, 4, 24, 4)
        h.setSpacing(8)
        domain = os.environ.get("ROS_DOMAIN_ID", "")
        entries = [
            ("ROS_DISTRO", os.environ.get("ROS_DISTRO", "not sourced"), None),
            ("ROS_DOMAIN_ID", domain or "0 (unset)",
             None if domain not in ("", "0") else
             "Lab rule: each robot and its PC use their own ROS_DOMAIN_ID (1-N), never 0."),
            ("RMW", os.environ.get("RMW_IMPLEMENTATION", "default"), None),
            ("HOST", socket.gethostname(), None),
        ]
        for i, (key, value, warning) in enumerate(entries):
            if i:
                h.addWidget(self._env_label("·", "envSep"))
            h.addWidget(self._env_label(key, "envLabel"))
            lbl = self._env_label(value, "envWarn" if warning else "envValue")
            if warning:
                lbl.setToolTip(warning)
            h.addWidget(lbl)
        h.addStretch()
        h.addWidget(self._env_label("WORKSPACES", "envLabel"))
        h.addWidget(self._env_label(", ".join(self._workspaces) or "none", "envValue"))
        return f

    @staticmethod
    def _env_label(text: str, name: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setObjectName(name)
        return lbl

    def _build_header(self) -> QWidget:
        f = QFrame()
        f.setObjectName("hdr")
        h = QHBoxLayout(f)
        h.setContentsMargins(24, 14, 24, 12)
        h.setSpacing(14)
        left = QVBoxLayout()
        left.setSpacing(2)
        distro = os.environ.get("ROS_DISTRO", "")
        eye = self._env_label("COBRAFLEX · ROS 2 " + distro.upper(), "hdrEyebrow")
        set_letter_spacing(eye, 1.6)
        left.addWidget(eye)
        left.addWidget(self._env_label("Launch Manager", "hdrTitle"))
        h.addLayout(left)
        h.addStretch()
        if self.backend.mode_label:
            badge = self._env_label(self.backend.mode_label, "modeBadge")
            badge.setToolTip("Simulated runs: no process is started")
            h.addWidget(badge)
        self.running_pill = QFrame()
        self.running_pill.setObjectName("hdrRunning")
        rp = QHBoxLayout(self.running_pill)
        rp.setContentsMargins(10, 4, 12, 4)
        rp.setSpacing(6)
        rp.addWidget(make_green_dot())
        self.running_text = self._env_label("0 running", "hdrRunningText")
        rp.addWidget(self.running_text)
        h.addWidget(self.running_pill)
        return f

    def _build_tab_bar(self) -> QWidget:
        f = QFrame()
        f.setObjectName("tabBar")
        h = QHBoxLayout(f)
        h.setContentsMargins(20, 0, 20, 0)
        h.setSpacing(2)
        self.tab_buttons: List[QPushButton] = []
        for i, label in enumerate(["Launches", "ROS Graph", "TF Tree"]):
            b = QPushButton(label)
            b.setObjectName("tab")
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setChecked(i == 0)
            b.clicked.connect(lambda _ck, idx=i: self._on_tab_changed(idx))
            self.tab_buttons.append(b)
            h.addWidget(b)
        h.addStretch()
        return f

    def _on_tab_changed(self, idx: int) -> None:
        for i, b in enumerate(self.tab_buttons):
            b.setChecked(i == idx)
        self.stack.setCurrentIndex(idx)
        self.graph_page.set_active(idx == 1)
        self.tf_page.set_active(idx == 2)

    # Filter bar and list ----------------------------------------------------

    def _build_filter(self) -> QWidget:
        f = QFrame()
        f.setObjectName("filterBar")
        h = QHBoxLayout(f)
        h.setContentsMargins(24, 10, 24, 10)
        h.setSpacing(12)

        self.search_wrap = QFrame()
        self.search_wrap.setObjectName("searchWrap")
        self.search_wrap.setProperty("focused", "false")
        self.search_wrap.setFixedHeight(32)
        self.search_wrap.setMaximumWidth(320)
        sw = QHBoxLayout(self.search_wrap)
        sw.setContentsMargins(10, 0, 10, 0)
        sw.setSpacing(8)
        sw.addWidget(self._env_label("⌕", "searchIcon"))
        self.search_input = QLineEdit()
        self.search_input.setObjectName("searchInput")
        self.search_input.setPlaceholderText("Filter launches…  ( / )")
        self.search_input.textChanged.connect(self._on_query_changed)
        self.search_input.installEventFilter(self)
        sw.addWidget(self.search_input)
        h.addWidget(self.search_wrap)

        self.chip_buttons: Dict[str, QPushButton] = {}
        for tag, label in FILTER_CHIPS:
            count = sum(1 for la in self.launches if tag in la.tags)
            b = QPushButton(f"{label} {count}")
            b.setObjectName("chip")
            b.setCursor(Qt.PointingHandCursor)
            b.setCheckable(True)
            b.setChecked(True)
            b.setVisible(count > 0)
            b.clicked.connect(lambda checked, t=tag: self._on_chip(t, checked))
            self.chip_buttons[tag] = b
            h.addWidget(b)
        h.addStretch()

        self.stop_all_btn = QPushButton("■  Stop all")
        self.stop_all_btn.setObjectName("btnStopAll")
        self.stop_all_btn.setCursor(Qt.PointingHandCursor)
        self.stop_all_btn.clicked.connect(self._on_stop_all)
        self.stop_all_btn.setVisible(False)
        h.addWidget(self.stop_all_btn)
        self.count_pill = self._env_label("", "countPill")
        h.addWidget(self.count_pill)
        return f

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802 - Qt API
        """Highlight the search field frame while it has the focus."""
        if obj is self.search_input and ev.type() in (QEvent.FocusIn, QEvent.FocusOut):
            self.search_wrap.setProperty("focused",
                                         "true" if ev.type() == QEvent.FocusIn else "false")
            restyle(self.search_wrap)
        return super().eventFilter(obj, ev)

    def _build_list(self) -> QWidget:
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.list_container = QWidget()
        self.list_container.setObjectName("root")
        self.list_layout = QVBoxLayout(self.list_container)
        self.list_layout.setContentsMargins(24, 16, 24, 12)
        self.list_layout.setSpacing(8)
        self.scroll.setWidget(self.list_container)
        return self.scroll

    def _rebuild_list(self) -> None:
        while self.list_layout.count():
            w = self.list_layout.takeAt(0).widget()
            if w is not None:
                w.setParent(None)
        self.card_widgets = {}
        filtered = self._filter_launches()
        starred = [la for la in filtered if la.starred]
        rest = [la for la in filtered if not la.starred]
        if not self.launches:
            self._add_empty("No launch files found. Build and source a workspace "
                            "(colcon build, source install/setup.bash) and start again.")
        elif not filtered:
            self._add_empty("No launches match the current filters.")
        else:
            if starred:
                self.list_layout.addWidget(SectionHeader("PINNED", len(starred), "★"))
                for la in starred:
                    self._add_card(la)
                self.list_layout.addSpacing(10)
            for pkg in sorted({la.pkg for la in rest}):
                group = [la for la in rest if la.pkg == pkg]
                self.list_layout.addWidget(SectionHeader(pkg.upper(), len(group)))
                for la in group:
                    self._add_card(la)
        self.list_layout.addStretch()
        self.count_pill.setText(f"✓  {len(filtered)} of {len(self.launches)} launches")
        self._refresh_running_pill()

    def _add_empty(self, text: str) -> None:
        empty = self._env_label(text, "empty")
        empty.setAlignment(Qt.AlignCenter)
        empty.setWordWrap(True)
        self.list_layout.addWidget(empty)

    def _add_card(self, launch: Launch) -> None:
        card = LaunchCard(launch)
        card.set_selected(launch.uid == self.selected_id)
        card.selected_signal.connect(self._on_select)
        card.run_signal.connect(self._on_run)
        card.stop_signal.connect(self._on_stop)
        card.restart_signal.connect(self._on_restart)
        card.star_signal.connect(self._on_star)
        card.presets_changed.connect(self._on_presets_changed)
        self.card_widgets[launch.uid] = card
        self.list_layout.addWidget(card)

    def _filter_launches(self) -> List[Launch]:
        q = self.query.strip().lower()
        return [la for la in self.launches
                if any(t in self.active_chips for t in la.tags)
                and (not q or q in f"{la.pkg} {la.name} {la.desc}".lower())]

    def _refresh_running_pill(self) -> None:
        n = sum(1 for la in self.launches if la.state == "running")
        self.running_text.setText(f"{n} running")
        self.stop_all_btn.setVisible(n > 0)
        self.stop_all_btn.setText(f"■  Stop all ({n})")

    # Logs -------------------------------------------------------------------

    def _launch(self, launch_id: str) -> Optional[Launch]:
        return next((la for la in self.launches if la.uid == launch_id), None)

    def _refresh_logs_subject(self) -> None:
        self.logs_panel.set_launch(self._launch(self.selected_id))

    def _clear_logs(self) -> None:
        sel = self._launch(self.selected_id)
        if sel:
            sel.logs.clear()
        self._refresh_logs_subject()

    # Backend callbacks ------------------------------------------------------

    def _on_backend_state(self, launch: Launch) -> None:
        card = self.card_widgets.get(launch.uid)
        if card is not None:
            card.refresh()
        self._refresh_running_pill()
        if launch.uid == self.selected_id:
            self.logs_panel.refresh_live_pill()
        previous = self._previous_state.get(launch.uid)
        if previous != launch.state:
            title = f"{launch.pkg} / {launch.name}"
            if launch.state == "running":
                self.show_toast("ok", f"{title} started", f"PID {launch.pid}")
            elif launch.state == "failed":
                detail = f"exit {launch.exit_code}" if launch.exit_code is not None else ""
                if launch.died:
                    detail = f"{launch.died} process(es) died" + (f", {detail}" if detail else "")
                self.show_toast("err", f"{title} failed", detail or None)
            elif launch.state == "stopped" and previous in ("running", "stopping"):
                self.show_toast("stop", f"{title} stopped")
        self._previous_state[launch.uid] = launch.state

    def _on_backend_log(self, launch: Launch, entry: LogEntry) -> None:
        if launch.uid == self.selected_id:
            self.logs_panel.append_entry(entry)

    # User actions -----------------------------------------------------------

    def _on_query_changed(self, text: str) -> None:
        self.query = text
        self._rebuild_list()

    def _on_chip(self, tag: str, checked: bool) -> None:
        if checked:
            self.active_chips.add(tag)
        else:
            self.active_chips.discard(tag)
        self._rebuild_list()

    def _on_select(self, launch_id: str) -> None:
        if self.selected_id == launch_id:
            return
        previous = self.card_widgets.get(self.selected_id)
        if previous is not None:
            previous.set_selected(False)
        self.selected_id = launch_id
        card = self.card_widgets.get(launch_id)
        if card is not None:
            card.set_selected(True)
        self._refresh_logs_subject()

    def _on_run(self, launch_id: str) -> None:
        launch = self._launch(launch_id)
        if launch is None:
            return
        self._on_select(launch_id)
        missing = missing_required(launch.args)
        if missing:
            card = self.card_widgets.get(launch_id)
            if card is not None:
                card.toggle_expand(force_open=True)
                card.expandable_section.mark_missing()
            self.show_toast("err", f"{launch.pkg} / {launch.name} not started",
                            "Required argument(s) without a value: " + ", ".join(missing))
            return
        self.backend.run(launch)

    def _on_stop(self, launch_id: str) -> None:
        launch = self._launch(launch_id)
        if launch is not None:
            self.backend.stop(launch)

    def _on_restart(self, launch_id: str) -> None:
        launch = self._launch(launch_id)
        if launch is not None:
            self.backend.restart(launch)
            self.show_toast("info", f"{launch.pkg} / {launch.name} restarting")

    def _on_star(self, launch_id: str) -> None:
        launch = self._launch(launch_id)
        if launch is None:
            return
        launch.starred = not launch.starred
        self.state.set_starred(launch.key, launch.starred)
        self._rebuild_list()

    def _on_presets_changed(self, launch: Launch) -> None:
        self.state.set_presets(launch.key, [p for p in launch.presets if p.get("user")])

    def _on_stop_all(self) -> None:
        running = [la for la in self.launches if la.state == "running"]
        for la in running:
            self.backend.stop(la)
        if running:
            self.show_toast("stop", f"Stopping {len(running)} launch(es)")

    # Toasts -----------------------------------------------------------------

    def show_toast(self, kind: str, title: str, subtitle: Optional[str] = None) -> None:
        """Show a notification in the bottom right corner."""
        toast = Toast(kind, title, subtitle, parent=self)
        toast.dismissed.connect(lambda t=toast: self._remove_toast(t))
        self._toasts.append(toast)
        toast.show()
        self._layout_toasts()

    def _remove_toast(self, toast: Toast) -> None:
        if toast in self._toasts:
            self._toasts.remove(toast)
        self._layout_toasts()

    def _layout_toasts(self) -> None:
        y = self.height() - 16
        for toast in reversed(self._toasts):
            toast.adjustSize()
            y -= toast.height()
            toast.move(self.width() - toast.width() - 16, y)
            y -= 8
            toast.raise_()

    # Command palette and shortcuts -----------------------------------------

    def _open_palette(self) -> None:
        if self._palette is None:
            self._palette = CommandPalette(self)
            self._palette.triggered.connect(self._on_palette_action)
        self._palette.open_with(self.launches)

    def _on_palette_action(self, data: dict) -> None:
        kind = data.get("kind")
        if kind == "action" and data.get("action_id") == "stop-all":
            self._on_stop_all()
        elif kind == "action" and data.get("action_id") == "clear-logs":
            self._clear_logs()
        elif kind == "run":
            self._on_run(data.get("launch_id", ""))
        elif kind == "stop":
            self._on_stop(data.get("launch_id", ""))

    def _install_shortcuts(self) -> None:
        # Shortcuts carry a modifier: a plain letter would also fire while the
        # read-only log pane has the focus and start a launch by accident.
        QShortcut(QKeySequence("Ctrl+K"), self, activated=self._open_palette)
        QShortcut(QKeySequence("/"), self, activated=self._focus_search)
        QShortcut(QKeySequence("Ctrl+R"), self, activated=self._run_selected)
        QShortcut(QKeySequence("Ctrl+E"), self, activated=self._stop_selected)
        for i in range(3):
            QShortcut(QKeySequence(f"Ctrl+{i + 1}"), self,
                      activated=lambda idx=i: self._on_tab_changed(idx))

    def _focus_search(self) -> None:
        if self.stack.currentIndex() == 0:
            self.search_input.setFocus()
            self.search_input.selectAll()

    def _run_selected(self) -> None:
        la = self._launch(self.selected_id)
        if la is not None and la.state not in ("running", "stopping"):
            self._on_run(la.uid)

    def _stop_selected(self) -> None:
        la = self._launch(self.selected_id)
        if la is not None and la.state == "running":
            self._on_stop(la.uid)

    def _tick_uptime(self) -> None:
        for la in self.launches:
            if la.state in ("running", "stopping"):
                la.uptime += 1
                card = self.card_widgets.get(la.uid)
                if card is not None:
                    card.refresh()
