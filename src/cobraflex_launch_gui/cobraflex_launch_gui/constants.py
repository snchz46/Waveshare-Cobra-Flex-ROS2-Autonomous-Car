"""Timing values, buffer sizes and colour tables shared by the widgets."""

#: Lifetime of a toast notification, in milliseconds.
TOAST_DURATION_MS = 4200

#: Time between SIGINT and the SIGTERM sent by the window. ros2 launch escalates
#: on its own after SIGINT (SIGTERM after 5 s, SIGKILL after 5 s more); this
#: value lets that sequence run first and acts only as a backstop.
FORCE_KILL_TIMEOUT_MS = 8000

#: Time between the window's SIGTERM and SIGKILL.
SIGKILL_TIMEOUT_MS = 3000

#: QProcess.waitForStarted timeout.
PROCESS_START_TIMEOUT_MS = 3000

#: Log buffer per launch; older entries are evicted first.
LOGS_MAX_PER_LAUNCH = 2000

#: Command palette size.
PALETTE_WIDTH = 560
PALETTE_HEIGHT = 420

#: Maximum height of the log panel pinned to the bottom of the launch list.
LOGS_PANEL_MAX_HEIGHT = 280

#: Window size. The minimum fits a 1366 x 768 lab monitor.
DEFAULT_WINDOW_WIDTH = 1240
DEFAULT_WINDOW_HEIGHT = 860
MIN_WINDOW_WIDTH = 960
MIN_WINDOW_HEIGHT = 640

#: Tick intervals, in milliseconds.
UPTIME_TICK_MS = 1000
MOCK_LOG_TICK_MS = 2200
GRAPH_REFRESH_MS = 2000
TF_REFRESH_MS = 1000

#: A dynamic transform without an update for this long is drawn as stale.
TF_STALE_S = 2.0

#: Window over which two parents of one frame count as a conflict, and over
#: which the publication rate of a transform is averaged.
TF_WINDOW_S = 5.0

#: Topics never drawn: present on every node and carry no structure.
GRAPH_ALWAYS_HIDDEN_TOPICS = frozenset({"/rosout", "/parameter_events"})

#: Topics hidden by the "infrastructure" toggle: in simulation every node
#: subscribes to /clock, and /tf has a publisher or subscriber in most nodes.
#: Both would collapse the drawing into two hubs; the TF tab covers /tf.
GRAPH_INFRASTRUCTURE_TOPICS = frozenset({"/clock", "/tf", "/tf_static"})

#: Filter chips (tag, label), in display order. "Other" collects launches
#: without a tag, so that no discovered launch file is ever unreachable.
FILTER_CHIPS = (
    ("Sim", "Simulation"),
    ("Hardware", "Robot"),
    ("RL", "RL / evaluation"),
    ("Lab", "Lab"),
    ("Tools", "Tools"),
    ("Other", "Other"),
)
UNTAGGED = "Other"

#: Log levels offered by the log panel filter.
LOG_LEVELS = ("ALL", "INFO", "WARN", "ERROR")
LOG_LEVEL_COLORS = {
    "INFO": "#56d364",
    "WARN": "#f0a040",
    "ERROR": "#f85149",
    "DEBUG": "#79c0ff",
}

#: Graph colours: (fill, border, text) per item kind and state.
GRAPH_COLORS = {
    "node": ("#10233d", "#3b82c4", "#cfe3ff"),
    "topic": ("#0f2a24", "#2f9e7a", "#c8f5e4"),
    "frame": ("#1d1b33", "#7c6fd6", "#e0dcff"),
    "warn": ("#2e2410", "#d29922", "#f5dfa8"),
    "error": ("#3a1414", "#f85149", "#ffd2cf"),
}
GRAPH_EDGE_COLOR = "#4b5a6e"
GRAPH_EDGE_HIGHLIGHT = "#79c0ff"
GRAPH_BACKGROUND = "#0d1117"
