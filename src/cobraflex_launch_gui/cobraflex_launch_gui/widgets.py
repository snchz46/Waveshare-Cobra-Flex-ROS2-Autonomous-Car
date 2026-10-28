"""Widgets of the launch list: cards, argument editors, logs, toasts, palette."""

import html
from typing import Dict, List, Optional

from PyQt5.QtCore import pyqtSignal, QEvent, Qt, QTimer
from PyQt5.QtGui import QFont, QMouseEvent
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QFrame,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMenu,
    QPushButton,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from cobraflex_launch_gui.constants import (
    LOG_LEVEL_COLORS,
    LOG_LEVELS,
    LOGS_MAX_PER_LAUNCH,
    LOGS_PANEL_MAX_HEIGHT,
    PALETTE_HEIGHT,
    PALETTE_WIDTH,
    TOAST_DURATION_MS,
)
from cobraflex_launch_gui.model import build_command, Launch, LogEntry, missing_required


# --- Helpers ----------------------------------------------------------------


def _dot(name: str, halo: Optional[str] = None) -> QFrame:
    """Return a status dot, optionally inside a halo frame."""
    dot = QFrame()
    dot.setObjectName(name)
    dot.setFixedSize(8, 8)
    if halo is None:
        return dot
    frame = QFrame()
    frame.setObjectName(halo)
    frame.setFixedSize(14, 14)
    dot.setParent(frame)
    dot.move(3, 3)
    return frame


def make_green_dot() -> QFrame:
    """Return the green dot of a running launch."""
    return _dot("dotGreen", "dotHalo")


def set_letter_spacing(widget: QWidget, value: float) -> None:
    """Set absolute letter spacing on a widget's font."""
    f = widget.font()
    f.setLetterSpacing(QFont.AbsoluteSpacing, value)
    widget.setFont(f)


def restyle(widget: QWidget) -> None:
    """Re-apply the style sheet after a dynamic property changed."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)


def _label(text: str, name: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setObjectName(name)
    return lbl


def _button(text: str, name: str, tooltip: str = "") -> QPushButton:
    b = QPushButton(text)
    b.setObjectName(name)
    b.setCursor(Qt.PointingHandCursor)
    if tooltip:
        b.setToolTip(tooltip)
    return b


def _clear_layout(layout) -> None:
    while layout.count():
        w = layout.takeAt(0).widget()
        if w is not None:
            w.setParent(None)


class SectionHeader(QFrame):
    """Small caps section title with a count and a rule."""

    def __init__(self, label: str, count: int, glyph: str = ""):
        """Build the header row."""
        super().__init__()
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 6, 0, 6)
        h.setSpacing(8)
        if glyph:
            h.addWidget(_label(glyph, "sectionGlyph"))
        lab = _label(label, "sectionLabel")
        set_letter_spacing(lab, 1.4)
        h.addWidget(lab)
        h.addWidget(_label(str(count), "sectionCount"))
        rule = QFrame()
        rule.setObjectName("sectionRule")
        rule.setFixedHeight(1)
        rule.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        h.addWidget(rule, 1)


# --- Argument editor --------------------------------------------------------


class ArgRow(QFrame):
    """Editor of one launch argument; kind is bool, enum, number or string."""

    value_changed = pyqtSignal()

    def __init__(self, arg: dict):
        """Build the editor for one argument dictionary (edited in place)."""
        super().__init__()
        self.arg = arg
        self._suppress = False
        self._bool_buttons: Dict[str, QPushButton] = {}
        self.setObjectName("argRow")

        h = QHBoxLayout(self)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(10)
        key = _label(arg["key"], "argKey")
        key.setMinimumWidth(170)
        h.addWidget(key)

        kind = arg.get("kind", "string")
        if kind == "bool":
            self.editor = self._make_bool_editor()
            h.addWidget(self.editor)
            h.addStretch(1)
        else:
            self.editor = self._make_enum_editor() if kind == "enum" else self._make_text_editor()
            h.addWidget(self.editor, 1)

        if arg.get("required"):
            h.addWidget(_label("required", "argRequired"))
        self.mod_indicator = _label("", "argMod")
        self.mod_indicator.setFixedWidth(14)
        self.mod_indicator.setAlignment(Qt.AlignCenter)
        self.mod_indicator.setToolTip("Differs from the default: passed as key:=value")
        h.addWidget(self.mod_indicator)

        tip = [arg["key"]]
        if arg.get("help"):
            tip.append(arg["help"])
        if arg.get("hint"):
            tip.append(f"Default computed by the launch file: {arg['hint']}")
        self.setToolTip("\n".join(tip))
        self._refresh_modified()

    def _make_bool_editor(self) -> QWidget:
        wrap = QFrame()
        wrap.setObjectName("boolSeg")
        h = QHBoxLayout(wrap)
        h.setContentsMargins(2, 2, 2, 2)
        h.setSpacing(2)
        for val in ("true", "false"):
            b = _button(val, "boolSegBtn")
            b.setCheckable(True)
            b.setChecked(self.arg["value"] == val)
            b.clicked.connect(lambda _ck, v=val: self._set_bool(v))
            self._bool_buttons[val] = b
            h.addWidget(b)
        return wrap

    def _make_enum_editor(self) -> QComboBox:
        cb = QComboBox()
        cb.setObjectName("argEnum")
        options = list(self.arg.get("options", []))
        if self.arg.get("default", "") == "" and "" not in options:
            # The file computes its default; an empty entry keeps it.
            options.insert(0, "")
        cb.addItems(options)
        if options and options[0] == "":
            cb.setItemText(0, "(default of the launch file)")
        cb.setCurrentIndex(max(options.index(self.arg["value"]), 0)
                           if self.arg["value"] in options else 0)
        cb.currentIndexChanged.connect(lambda i: self._set_value(options[i]))
        self._options = options
        return cb

    def _make_text_editor(self) -> QLineEdit:
        le = QLineEdit(self.arg["value"])
        le.setObjectName("argText")
        if self.arg.get("required"):
            le.setPlaceholderText("value required")
        elif self.arg.get("hint"):
            le.setPlaceholderText(f"default: {self.arg['hint']}")
        le.textChanged.connect(self._set_value)
        return le

    def _set_bool(self, v: str) -> None:
        for k, btn in self._bool_buttons.items():
            btn.setChecked(k == v)
        self._set_value(v)

    def _set_value(self, v: str) -> None:
        if self._suppress:
            return
        self.arg["value"] = v
        self._refresh_modified()
        self.value_changed.emit()

    def _refresh_modified(self) -> None:
        value = self.arg["value"]
        modified = value != "" and value != self.arg.get("default", "")
        self.mod_indicator.setText("~" if modified else "")
        if isinstance(self.editor, QLineEdit):
            self.editor.setProperty("missing", "true" if self.arg.get("required")
                                    and value == "" else "false")
            restyle(self.editor)

    def refresh_from_arg(self) -> None:
        """Show the current value of the argument without emitting signals."""
        self._suppress = True
        try:
            kind = self.arg.get("kind", "string")
            v = self.arg["value"]
            if kind == "bool":
                for k, btn in self._bool_buttons.items():
                    btn.setChecked(k == v)
            elif kind == "enum":
                self.editor.setCurrentIndex(self._options.index(v) if v in self._options else 0)
            else:
                self.editor.setText(v)
            self._refresh_modified()
        finally:
            self._suppress = False


class ExpandableSection(QFrame):
    """Expanded part of a card: command preview, argument editors, presets.

    Edits change launch.args in place; the next run uses them.
    """

    presets_changed = pyqtSignal(object)

    def __init__(self, launch: Launch):
        """Build the section for one launch."""
        super().__init__()
        self.launch = launch
        self.setObjectName("expandable")
        v = QVBoxLayout(self)
        v.setContentsMargins(14, 12, 14, 14)
        v.setSpacing(10)

        cmd_hd = QHBoxLayout()
        cmd_label = _label(">_  COMMAND", "expandLabel")
        set_letter_spacing(cmd_label, 1.2)
        cmd_hd.addWidget(cmd_label)
        cmd_hd.addStretch()
        copy_btn = _button("Copy", "btnGhost", "Copy the command line to the clipboard")
        copy_btn.clicked.connect(self._copy_command)
        cmd_hd.addWidget(copy_btn)
        v.addLayout(cmd_hd)

        self.cmd_box = _label(build_command(launch), "cmdBox")
        self.cmd_box.setWordWrap(True)
        self.cmd_box.setTextInteractionFlags(Qt.TextSelectableByMouse)
        v.addWidget(self.cmd_box)
        path = _label(str(launch.file_path), "cardWs")
        path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        v.addWidget(path)

        self.arg_rows: List[ArgRow] = []
        if not launch.args:
            empty = QLabel("(no declared arguments)")
            empty.setStyleSheet("color: #6e7681; font-size: 11px; font-style: italic;")
            v.addWidget(empty)
            return

        args_hd = QHBoxLayout()
        args_label = _label("◆  ARGUMENTS", "expandLabel")
        set_letter_spacing(args_label, 1.2)
        args_hd.addWidget(args_label)
        args_hd.addStretch()
        reset_btn = _button("Reset", "btnGhost", "Restore every default")
        reset_btn.clicked.connect(self._reset_args)
        args_hd.addWidget(reset_btn)
        v.addLayout(args_hd)
        for a in launch.args:
            row = ArgRow(a)
            row.value_changed.connect(self._on_arg_changed)
            self.arg_rows.append(row)
            v.addWidget(row)

        presets_label = _label("⚡  PRESETS", "expandLabel")
        set_letter_spacing(presets_label, 1.2)
        v.addWidget(presets_label)
        self.preset_row = QHBoxLayout()
        self.preset_row.setSpacing(6)
        v.addLayout(self.preset_row)
        self._rebuild_presets()

    def _rebuild_presets(self) -> None:
        _clear_layout(self.preset_row)
        for p in self.launch.presets:
            args = " ".join(f"{k}:={v}" for k, v in p["args"].items()) or "(defaults)"
            tip = args + ("\nRight click to delete" if p.get("user") else "")
            pb = _button(p["name"], "presetPill", tip)
            pb.clicked.connect(lambda _ck, preset=p: self._apply_preset(preset))
            if p.get("user"):
                pb.setContextMenuPolicy(Qt.CustomContextMenu)
                pb.customContextMenuRequested.connect(
                    lambda pos, preset=p, b=pb: self._preset_menu(preset, b, pos))
            self.preset_row.addWidget(pb)
        save_btn = _button("+ Save current", "presetSave",
                           "Store the modified arguments as a preset of this user")
        save_btn.clicked.connect(self._save_preset)
        self.preset_row.addWidget(save_btn)
        self.preset_row.addStretch()

    def _on_arg_changed(self) -> None:
        self.cmd_box.setText(build_command(self.launch))

    def _copy_command(self) -> None:
        QApplication.clipboard().setText(build_command(self.launch))

    def _reset_args(self) -> None:
        for a in self.launch.args:
            a["value"] = a.get("default", "")
        for row in self.arg_rows:
            row.refresh_from_arg()
        self._on_arg_changed()

    def _apply_preset(self, preset: dict) -> None:
        for a in self.launch.args:
            a["value"] = preset["args"].get(a["key"], a.get("default", ""))
        for row in self.arg_rows:
            row.refresh_from_arg()
        self._on_arg_changed()

    def _save_preset(self) -> None:
        name, ok = QInputDialog.getText(self, "Save preset", "Name of the new preset:")
        name = name.strip()
        if not ok or not name:
            return
        args = {a["key"]: a["value"] for a in self.launch.args
                if a["value"] != "" and a["value"] != a.get("default", "")}
        existing = next((p for p in self.launch.presets
                         if p["name"] == name and p.get("user")), None)
        if existing is not None:
            existing["args"] = args
        else:
            self.launch.presets.append({"name": name, "args": args, "user": True})
        self._rebuild_presets()
        self.presets_changed.emit(self.launch)

    def _preset_menu(self, preset: dict, button: QPushButton, pos) -> None:
        menu = QMenu(self)
        delete = menu.addAction(f"Delete preset “{preset['name']}”")
        if menu.exec_(button.mapToGlobal(pos)) is delete:
            self.launch.presets.remove(preset)
            self._rebuild_presets()
            self.presets_changed.emit(self.launch)

    def mark_missing(self) -> None:
        """Move the focus to the first required argument without a value."""
        missing = set(missing_required(self.launch.args))
        for row in self.arg_rows:
            if row.arg["key"] in missing:
                row.editor.setFocus()
                return


# --- Launch card ------------------------------------------------------------


class LaunchCard(QFrame):
    """One launch file: name, description, state, buttons, expandable editor."""

    selected_signal = pyqtSignal(str)
    run_signal = pyqtSignal(str)
    stop_signal = pyqtSignal(str)
    restart_signal = pyqtSignal(str)
    star_signal = pyqtSignal(str)
    presets_changed = pyqtSignal(object)

    def __init__(self, launch: Launch):
        """Build the card for one launch."""
        super().__init__()
        self.launch = launch
        self.setProperty("role", "card")
        self.setProperty("selected", "false")
        self.setProperty("expanded", "false")
        self.setCursor(Qt.PointingHandCursor)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        top_widget = QWidget()
        top = QHBoxLayout(top_widget)
        top.setContentsMargins(14, 14, 14, 12)
        top.setSpacing(16)
        col = QVBoxLayout()
        col.setSpacing(3)

        l1 = QHBoxLayout()
        l1.setSpacing(8)
        self.expand_btn = _button("▸", "expandBtn", "Show command and arguments")
        self.expand_btn.setProperty("open", "false")
        self.expand_btn.setFlat(True)
        self.expand_btn.clicked.connect(self.toggle_expand)
        l1.addWidget(self.expand_btn)
        self.star_btn = _button("", "cardStar", "Pin / unpin")
        self.star_btn.setFlat(True)
        self.star_btn.clicked.connect(lambda: self.star_signal.emit(self.launch.uid))
        l1.addWidget(self.star_btn)
        l1.addWidget(_label(launch.pkg, "cardPkg"))
        l1.addWidget(_label("/", "cardSlash"))
        name = _label(launch.name, "cardName")
        name.setToolTip(str(launch.file_path))
        l1.addWidget(name)
        if launch.file_format != "py":
            l1.addWidget(_label(launch.file_format, "cardFmt"))
        if launch.args:
            l1.addWidget(_label(f"{len(launch.args)} args", "cardArgsBadge"))
        l1.addStretch()
        if launch.workspace:
            ws = _label(launch.workspace, "cardWs")
            ws.setToolTip("Workspace of this launch file")
            l1.addWidget(ws)
        col.addLayout(l1)

        desc = _label(launch.desc, "cardDesc")
        desc.setWordWrap(True)
        col.addWidget(desc)

        self.err_snippet = _label("", "cardErrSnippet")
        self.err_snippet.setWordWrap(True)
        self.err_snippet.setVisible(False)
        col.addWidget(self.err_snippet)

        self.status_row = QHBoxLayout()
        self.status_row.setSpacing(6)
        self.status_row.setContentsMargins(0, 3, 0, 0)
        col.addLayout(self.status_row)
        top.addLayout(col, 1)

        btns_widget = QWidget()
        btns_widget.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        btns = QHBoxLayout(btns_widget)
        btns.setSpacing(6)
        btns.setContentsMargins(0, 0, 0, 0)
        self.run_btn = _button("▶  Run", "btnRun")
        self.run_btn.clicked.connect(self._on_run_click)
        self.stop_btn = _button("■  Stop", "btnStop")
        self.stop_btn.clicked.connect(lambda: self.stop_signal.emit(self.launch.uid))
        self.logs_btn = _button("☰  Logs", "btnLogs", "Show the output of this launch")
        self.logs_btn.clicked.connect(lambda: self.selected_signal.emit(self.launch.uid))
        for b in (self.run_btn, self.stop_btn, self.logs_btn):
            btns.addWidget(b)
        top.addWidget(btns_widget, 0, Qt.AlignTop)
        outer.addWidget(top_widget)

        self.expand_container = QFrame()
        self.expand_container.setObjectName("expandContainer")
        self.expand_container.setVisible(False)
        ec = QVBoxLayout(self.expand_container)
        ec.setContentsMargins(0, 0, 0, 0)
        self.expandable_section = ExpandableSection(launch)
        self.expandable_section.presets_changed.connect(self.presets_changed)
        ec.addWidget(self.expandable_section)
        outer.addWidget(self.expand_container)

        self.refresh()

    def toggle_expand(self, force_open: bool = False) -> None:
        """Show or hide the command and argument editor."""
        new_open = True if force_open is True else not self.expand_container.isVisible()
        self.expand_container.setVisible(new_open)
        self.expand_btn.setText("▾" if new_open else "▸")
        self.expand_btn.setProperty("open", "true" if new_open else "false")
        restyle(self.expand_btn)
        self.setProperty("expanded", "true" if new_open else "false")
        restyle(self)

    def _on_run_click(self) -> None:
        if self.launch.state == "running":
            self.restart_signal.emit(self.launch.uid)
        else:
            self.run_signal.emit(self.launch.uid)

    def mousePressEvent(self, e: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """Select the card on click."""
        self.selected_signal.emit(self.launch.uid)
        super().mousePressEvent(e)

    def set_selected(self, selected: bool) -> None:
        """Mark the card as the subject of the log panel."""
        self.setProperty("selected", "true" if selected else "false")
        restyle(self)

    def refresh(self) -> None:
        """Redraw state-dependent parts: dot, status text, buttons, error."""
        la = self.launch
        _clear_layout(self.status_row)
        self.star_btn.setText("★" if la.starred else "☆")
        self.star_btn.setProperty("starred", "true" if la.starred else "false")
        restyle(self.star_btn)

        show_err = la.last_error and (la.state == "failed" or la.died)
        self.err_snippet.setText(la.last_error or "")
        self.err_snippet.setVisible(bool(show_err))

        row = self.status_row
        if la.state in ("running", "stopping"):
            stopping = la.state == "stopping"
            row.addWidget(_dot("dotAmber") if stopping else make_green_dot())
            row.addWidget(_label("stopping…" if stopping else "running",
                                 "statusStopping" if stopping else "statusRunning"))
            row.addWidget(_label("·", "metaSep"))
            row.addWidget(_label(f"PID {la.pid}", "metaMono"))
            row.addWidget(_label("·", "metaSep"))
            mm, ss = divmod(la.uptime, 60)
            row.addWidget(_label(f"{mm:02d}:{ss:02d}", "metaMono"))
            if la.died:
                row.addWidget(_label("·", "metaSep"))
                row.addWidget(_label(f"{la.died} process(es) died", "cardDied"))
            self.run_btn.setText("↻  Restart")
            self.run_btn.setObjectName("btnRestart")
            self.run_btn.setEnabled(not stopping)
            self.stop_btn.setEnabled(not stopping)
            running, failed = "true", "false"
        elif la.state == "failed":
            row.addWidget(_dot("dotRed", "dotRedHalo"))
            row.addWidget(_label("failed", "statusFailed"))
            if la.exit_code is not None:
                row.addWidget(_label("·", "metaSep"))
                row.addWidget(_label(f"exit {la.exit_code}", "metaMono"))
            if la.died:
                row.addWidget(_label("·", "metaSep"))
                row.addWidget(_label(f"{la.died} process(es) died", "cardDied"))
            self.run_btn.setText("▶  Retry")
            self.run_btn.setObjectName("btnRetry")
            self.run_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            running, failed = "false", "true"
        else:
            row.addWidget(_dot("dotGrey"))
            row.addWidget(_label("stopped", "statusStopped"))
            self.run_btn.setText("▶  Run")
            self.run_btn.setObjectName("btnRun")
            self.run_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            running, failed = "false", "false"
        row.addStretch()
        self.setProperty("running", running)
        self.setProperty("failed", failed)
        restyle(self.run_btn)
        restyle(self)


# --- Toast ------------------------------------------------------------------


class Toast(QFrame):
    """Auto-dismissing notification in the bottom right corner."""

    dismissed = pyqtSignal()

    GLYPHS = {"ok": "▶", "err": "✕", "stop": "■", "info": "·"}
    _GLYPH_OBJ_NAME = {"ok": "toastGlyphOk", "err": "toastGlyphErr",
                       "stop": "toastGlyphStop", "info": "toastGlyphInfo"}

    def __init__(self, kind: str, title: str, subtitle: Optional[str] = None,
                 duration_ms: int = TOAST_DURATION_MS, parent=None):
        """Build the toast; kind is ok, err, stop or info."""
        super().__init__(parent)
        self.setObjectName("toast")
        self.setProperty("kind", kind)
        self.setCursor(Qt.PointingHandCursor)
        h = QHBoxLayout(self)
        h.setContentsMargins(12, 10, 12, 10)
        h.setSpacing(10)
        h.addWidget(_label(self.GLYPHS.get(kind, "·"),
                           self._GLYPH_OBJ_NAME.get(kind, "toastGlyphInfo")))
        col = QVBoxLayout()
        col.setSpacing(1)
        t = _label(title, "toastTitle")
        t.setWordWrap(True)
        col.addWidget(t)
        if subtitle:
            s = _label(subtitle, "toastSub")
            s.setWordWrap(True)
            col.addWidget(s)
        h.addLayout(col, 1)
        h.addWidget(_label("×", "toastX"))
        self.setMinimumWidth(280)
        self.setMaximumWidth(380)
        QTimer.singleShot(duration_ms, self.dismiss)

    def mousePressEvent(self, e: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """Dismiss on click."""
        self.dismiss()
        super().mousePressEvent(e)

    def dismiss(self) -> None:
        """Hide and delete the toast."""
        if self.isVisible():
            self.dismissed.emit()
            self.hide()
            self.deleteLater()


# --- Command palette --------------------------------------------------------


class CommandPalette(QDialog):
    """Ctrl+K dialog: run or stop any launch by typing part of its name."""

    triggered = pyqtSignal(dict)

    def __init__(self, parent=None):
        """Build the dialog; open_with() fills it."""
        super().__init__(parent)
        self.setObjectName("paletteRoot")
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        self.setModal(True)
        self._items: List[dict] = []

        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        input_wrap = QFrame()
        input_wrap.setObjectName("paletteInputWrap")
        ih = QHBoxLayout(input_wrap)
        ih.setContentsMargins(16, 14, 16, 14)
        ih.setSpacing(12)
        ih.addWidget(_label("⌕", "paletteIcon"))
        self.input = QLineEdit()
        self.input.setObjectName("paletteInput")
        self.input.setPlaceholderText("Run a launch, search…")
        self.input.textChanged.connect(self._refilter)
        self.input.returnPressed.connect(self._activate_current)
        self.input.installEventFilter(self)
        ih.addWidget(self.input, 1)
        ih.addWidget(_label("esc", "paletteKbd"))
        v.addWidget(input_wrap)

        self.list = QListWidget()
        self.list.setObjectName("paletteList")
        self.list.itemActivated.connect(lambda _i: self._activate_current())
        v.addWidget(self.list, 1)
        foot = _label("↑↓ navigate     ↵ select     esc close", "paletteFoot")
        foot.setAlignment(Qt.AlignCenter)
        v.addWidget(foot)
        self.resize(PALETTE_WIDTH, PALETTE_HEIGHT)

    def open_with(self, launches: List[Launch]) -> None:
        """Fill the list from the current launches and show the dialog."""
        self._items = self._build_items(launches)
        self.input.clear()
        self._refilter("")
        parent = self.parentWidget()
        if parent is not None:
            geo = parent.geometry()
            self.move(geo.x() + (geo.width() - self.width()) // 2, geo.y() + 120)
        self.show()
        self.raise_()
        self.input.setFocus()

    @staticmethod
    def _build_items(launches: List[Launch]) -> List[dict]:
        items: List[dict] = []
        running = [la for la in launches if la.state == "running"]
        if running:
            items.append({"kind": "action", "action_id": "stop-all", "glyph": "■",
                          "title": f"Stop all running ({len(running)})", "subtitle": ""})
        items.append({"kind": "action", "action_id": "clear-logs", "glyph": "✕",
                      "title": "Clear logs", "subtitle": ""})
        for la in launches:
            verb = "Retry" if la.state == "failed" else "Run"
            items.append({"kind": "run", "launch_id": la.uid, "glyph": "▶",
                          "title": f"{verb}  {la.pkg} / {la.name}", "subtitle": la.desc,
                          "disabled": la.state in ("running", "stopping")})
            if la.state == "running":
                items.append({"kind": "stop", "launch_id": la.uid, "glyph": "■",
                              "title": f"Stop  {la.pkg} / {la.name}",
                              "subtitle": f"PID {la.pid}"})
        return items

    def _refilter(self, query: str) -> None:
        query = (query or "").strip().lower()
        self.list.clear()
        for it in self._items:
            if query and query not in f"{it['title']} {it.get('subtitle', '')}".lower():
                continue
            text = f"{it['glyph']}   {it['title']}"
            if it.get("subtitle"):
                text += f"   ·   {it['subtitle']}"
            row = QListWidgetItem(text)
            row.setData(Qt.UserRole, it)
            if it.get("disabled"):
                row.setFlags(row.flags() & ~Qt.ItemIsEnabled)
            self.list.addItem(row)
        for i in range(self.list.count()):
            if self.list.item(i).flags() & Qt.ItemIsEnabled:
                self.list.setCurrentRow(i)
                break

    def _activate_current(self) -> None:
        item = self.list.currentItem()
        if item is None or not item.flags() & Qt.ItemIsEnabled:
            return
        self.triggered.emit(item.data(Qt.UserRole))
        self.close()

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802 - Qt API
        """Move the list selection with the arrow keys while typing."""
        if obj is self.input and ev.type() == QEvent.KeyPress:
            if ev.key() == Qt.Key_Down:
                self.list.setCurrentRow(min(self.list.currentRow() + 1, self.list.count() - 1))
                return True
            if ev.key() == Qt.Key_Up:
                self.list.setCurrentRow(max(self.list.currentRow() - 1, 0))
                return True
            if ev.key() == Qt.Key_Escape:
                self.close()
                return True
        return super().eventFilter(obj, ev)


# --- Log panel --------------------------------------------------------------


class LogsPanel(QFrame):
    """Output of the selected launch, with level filter, search, follow, wrap."""

    clear_requested = pyqtSignal()

    def __init__(self):
        """Build the panel with no launch selected."""
        super().__init__()
        self.setObjectName("logsPanel")
        self.setMaximumHeight(LOGS_PANEL_MAX_HEIGHT)
        self._launch: Optional[Launch] = None
        self._level_filter = "ALL"
        self._search_query = ""
        self._follow = True

        v = QVBoxLayout(self)
        v.setContentsMargins(24, 12, 24, 16)
        v.setSpacing(8)
        head = QHBoxLayout()
        head.setSpacing(8)
        key = _label("LOGS", "logsKey")
        set_letter_spacing(key, 1.4)
        head.addWidget(key)
        head.addWidget(_label("·", "logsSep"))
        self.subject_label = _label("—", "logsSubject")
        head.addWidget(self.subject_label)
        self.live_pill = QFrame()
        self.live_pill.setObjectName("logsLive")
        lp = QHBoxLayout(self.live_pill)
        lp.setContentsMargins(8, 2, 10, 2)
        lp.setSpacing(5)
        lp.addWidget(make_green_dot())
        live = _label("LIVE", "logsLiveText")
        set_letter_spacing(live, 1.0)
        lp.addWidget(live)
        self.live_pill.setVisible(False)
        head.addWidget(self.live_pill)
        head.addStretch()

        self.level_buttons: Dict[str, QPushButton] = {}
        for lvl in LOG_LEVELS:
            b = QPushButton(lvl)
            b.setProperty("logLevel", lvl)
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setChecked(lvl == "ALL")
            b.clicked.connect(lambda _ck, lv=lvl: self._set_level(lv))
            self.level_buttons[lvl] = b
            head.addWidget(b)
        self.search_input = QLineEdit()
        self.search_input.setObjectName("logSearch")
        self.search_input.setPlaceholderText("Search…")
        self.search_input.textChanged.connect(self._on_search_changed)
        head.addWidget(self.search_input)
        self.follow_btn = _button("✓ Follow", "btnGhost", "Keep the newest line in view")
        self.follow_btn.setCheckable(True)
        self.follow_btn.setChecked(True)
        self.follow_btn.clicked.connect(self._toggle_follow)
        head.addWidget(self.follow_btn)
        self.wrap_btn = _button("✓ Wrap", "btnGhost", "Wrap long lines")
        self.wrap_btn.setCheckable(True)
        self.wrap_btn.setChecked(True)
        self.wrap_btn.clicked.connect(self._toggle_wrap)
        head.addWidget(self.wrap_btn)
        clear_btn = _button("Clear", "btnGhost", "Clear the output of this launch")
        clear_btn.clicked.connect(self.clear_requested.emit)
        head.addWidget(clear_btn)
        v.addLayout(head)

        self.body = QTextEdit()
        self.body.setObjectName("logsBody")
        self.body.setReadOnly(True)
        self.body.setLineWrapMode(QTextEdit.WidgetWidth)
        self.body.document().setMaximumBlockCount(LOGS_MAX_PER_LAUNCH)
        v.addWidget(self.body, 1)

    def set_launch(self, launch: Optional[Launch]) -> None:
        """Show the output of another launch."""
        self._launch = launch
        self.subject_label.setText(f"{launch.pkg} / {launch.name}" if launch else "—")
        self.refresh_live_pill()
        self._render_all()

    def refresh_live_pill(self) -> None:
        """Show the LIVE pill while the selected launch runs."""
        self.live_pill.setVisible(bool(self._launch) and self._launch.state == "running")

    def _update_counts(self) -> None:
        counts = dict.fromkeys(LOG_LEVELS, 0)
        if self._launch:
            for entry in self._launch.logs:
                counts["ALL"] += 1
                if entry.level in counts:
                    counts[entry.level] += 1
        for lvl, btn in self.level_buttons.items():
            btn.setText(f"{lvl} {counts[lvl]}")

    def _matches(self, entry: LogEntry) -> bool:
        if self._level_filter != "ALL" and entry.level != self._level_filter:
            return False
        q = self._search_query
        return not q or q in entry.msg.lower() or q in entry.node.lower()

    def _render_all(self) -> None:
        self.body.setUpdatesEnabled(False)
        self.body.clear()
        self._update_counts()
        if self._launch:
            for entry in self._launch.logs:
                if self._matches(entry):
                    self.body.append(self._entry_html(entry))
        self.body.setUpdatesEnabled(True)
        self._scroll_to_end()

    def append_entry(self, entry: LogEntry) -> None:
        """Append one live line of the selected launch."""
        if not self._launch:
            return
        self._update_counts()
        if self._matches(entry):
            self.body.append(self._entry_html(entry))
            self._scroll_to_end()

    def _scroll_to_end(self) -> None:
        if self._follow:
            sb = self.body.verticalScrollBar()
            sb.setValue(sb.maximum())

    @staticmethod
    def _entry_html(entry: LogEntry) -> str:
        color = LOG_LEVEL_COLORS.get(entry.level, "#56d364")
        node = f'<span style="color:#79c0ff;">[{html.escape(entry.node)}]</span> ' \
            if entry.node else ""
        return (f'<span style="color:#6e7681;">{entry.fmt_time()}</span> '
                f'<span style="color:{color};">[{entry.level}]</span> {node}'
                f'<span style="color:#c9d1d9;">{html.escape(entry.msg)}</span>')

    def _set_level(self, lvl: str) -> None:
        self._level_filter = lvl
        for k, btn in self.level_buttons.items():
            btn.setChecked(k == lvl)
        self._render_all()

    def _on_search_changed(self, text: str) -> None:
        self._search_query = text.strip().lower()
        self._render_all()

    def _toggle_follow(self) -> None:
        self._follow = self.follow_btn.isChecked()
        self.follow_btn.setText(("✓ " if self._follow else "") + "Follow")

    def _toggle_wrap(self) -> None:
        wrap = self.wrap_btn.isChecked()
        self.body.setLineWrapMode(QTextEdit.WidgetWidth if wrap else QTextEdit.NoWrap)
        self.wrap_btn.setText(("✓ " if wrap else "") + "Wrap")
