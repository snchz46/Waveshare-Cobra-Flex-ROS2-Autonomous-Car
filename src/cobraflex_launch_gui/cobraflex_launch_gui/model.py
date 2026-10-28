"""Domain model: launches, log entries and the helpers around them.

Free of Qt imports, so that the parsers are testable without a display.
"""

from dataclasses import dataclass, field
from pathlib import Path
import re
import shlex
import time
from typing import List, Optional, Set

_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

#: rcutils severities, folded onto the four levels of the log panel.
_LEVELS = {
    "DEBUG": "DEBUG",
    "INFO": "INFO",
    "WARN": "WARN",
    "WARNING": "WARN",
    "ERROR": "ERROR",
    "FATAL": "ERROR",
}

#: Output of a node under ros2 launch: process name with its index, then the
#: default rcutils format. Example:
#: [robot_state_publisher-1] [INFO] [1730000000.12] [robot_state_publisher]: msg
_NODE_LINE_RE = re.compile(
    r"^\[(?P<proc>[^\]]+)\]\s+\[(?P<level>[A-Z]+)\]\s+\[(?P<stamp>[0-9.]+)\]\s+"
    r"\[(?P<logger>[^\]]+)\]:\s?(?P<msg>.*)$"
)
#: Messages of ros2 launch itself, about the launch or one of its processes.
#: Example: [ERROR] [ekf_node-5]: process has died [pid 4242, exit code 1, ...]
_LAUNCH_LINE_RE = re.compile(r"^\[(?P<level>[A-Z]+)\]\s+\[(?P<src>[^\]]+)\]:\s?(?P<msg>.*)$")
#: Free text of a process: print(), tracebacks, Gazebo's own logger.
_RAW_PROC_RE = re.compile(r"^\[(?P<proc>[^\]]+)\]\s(?P<msg>.*)$")
#: First line of a Python traceback, or its final "SomethingError: ..." line.
_TRACEBACK_RE = re.compile(
    r"^(Traceback \(most recent call last\)|([A-Za-z_][\w.]*)?(Error|Exception)(: |$))"
)
#: Gazebo's console prefixes.
_GZ_LEVELS = {"[Err]": "ERROR", "[Wrn]": "WARN", "[Dbg]": "DEBUG", "[Msg]": "INFO"}

PROCESS_STARTED_RE = re.compile(r"process started with pid \[(\d+)\]")
PROCESS_DIED_RE = re.compile(r"process has died")
_PROCESS_INDEX_RE = re.compile(r"-\d+$")


@dataclass
class LogEntry:
    """One line of launch output, classified by level and origin."""

    time: float
    level: str
    node: str
    msg: str
    raw: str = ""
    #: Process name with its launch index ("ekf_node-5"), when known.
    process: str = ""
    #: rcutils logger name ("ns.node.child"), only for formatted node output.
    logger: str = ""

    @classmethod
    def parse(cls, line: str) -> "LogEntry":
        """Classify one output line of ros2 launch.

        Unrecognised lines are kept as INFO, so that they stay visible and
        searchable.
        """
        line = _ANSI_RE.sub("", line).rstrip()
        now = time.time()

        m = _NODE_LINE_RE.match(line)
        if m and m.group("level") in _LEVELS:
            return cls(now, _LEVELS[m.group("level")], m.group("logger"), m.group("msg"),
                       line, process=m.group("proc"), logger=m.group("logger"))

        m = _LAUNCH_LINE_RE.match(line)
        if m and m.group("level") in _LEVELS:
            return cls(now, _LEVELS[m.group("level")], m.group("src"), m.group("msg"), line)

        m = _RAW_PROC_RE.match(line)
        if m:
            msg = m.group("msg")
            level = _GZ_LEVELS.get(msg[:5], "INFO")
            if level == "INFO" and _TRACEBACK_RE.match(msg):
                level = "ERROR"
            return cls(now, level, m.group("proc"), msg, line, process=m.group("proc"))

        return cls(now, "INFO", "", line, line)

    def fmt_time(self) -> str:
        """Return the wall-clock time of the entry as HH:MM:SS."""
        return time.strftime("%H:%M:%S", time.localtime(self.time))


@dataclass(eq=False)
class Launch:
    """A launch file and its run state in the window."""

    pkg: str
    name: str
    file_path: Path
    file_format: str = "py"
    desc: str = ""
    #: Label of the workspace the file was found in ("ros2_ws", "lab_ws").
    workspace: str = ""
    tags: List[str] = field(default_factory=list)
    starred: bool = False
    #: Each argument: {key, default, value, kind, options?, help?, required?,
    #: hint?}. kind is one of bool, enum, number, string. An empty value means
    #: that the argument is not passed and the launch file applies its default.
    args: List[dict] = field(default_factory=list)
    #: Each preset: {name, args: {key: value}, user: bool}.
    presets: List[dict] = field(default_factory=list)
    #: stopped | running | stopping | failed
    state: str = "stopped"
    pid: Optional[int] = None
    uptime: int = 0
    exit_code: Optional[int] = None
    last_error: Optional[str] = None
    #: Processes of the launch that exited with an error while it ran.
    died: int = 0
    logs: List[LogEntry] = field(default_factory=list)
    #: Fully qualified node names derived from the loggers in the output.
    node_hints: Set[str] = field(default_factory=set)
    #: Process names from "process started" lines, without the index.
    process_hints: Set[str] = field(default_factory=set)

    @property
    def key(self) -> str:
        """Metadata key, "<package>/<name without extension>"."""
        return f"{self.pkg}/{self.name}"

    @property
    def uid(self) -> str:
        """Unique identifier, "<package>/<file name>"."""
        return f"{self.pkg}/{self.file_path.name}"

    def owns_node(self, fqn: str) -> bool:
        """Return whether a graph node is attributed to this launch.

        ROS 2 does not expose which process hosts a node, so the attribution
        uses the launch output: logger names (a node logs as "ns.node", child
        loggers as "ns.node.child") and process names, which equal the node
        name unless the launch file renames the node.
        """
        return fqn in self.node_hints or fqn.rsplit("/", 1)[-1] in self.process_hints


def strip_process_index(name: str) -> str:
    """Return a launch process name without its "-N" suffix."""
    return _PROCESS_INDEX_RE.sub("", name)


def logger_node_candidates(logger: str) -> Set[str]:
    """Return the node names a logger can belong to.

    "ns.node.child" yields /ns, /ns/node and /ns/node/child: the dots stand
    both for namespace separators and for child loggers, and only the graph
    can tell which prefix is a node.
    """
    parts = [p for p in logger.split(".") if p]
    return {"/" + "/".join(parts[:i]) for i in range(1, len(parts) + 1)}


def args_to_overrides(args: List[dict]) -> List[str]:
    """Return the key:=value pairs of the arguments that differ from default."""
    out = []
    for a in args:
        value = a.get("value", "")
        if value != "" and value != a.get("default", ""):
            out.append(f"{a['key']}:={value}")
    return out


def missing_required(args: List[dict]) -> List[str]:
    """Return the keys of required arguments that have no value."""
    return [a["key"] for a in args if a.get("required") and a.get("value", "") == ""]


def build_command(launch: Launch) -> str:
    """Compose the ros2 launch command line equivalent to a run."""
    parts = ["ros2", "launch", launch.pkg, launch.file_path.name]
    parts.extend(shlex.quote(o) for o in args_to_overrides(launch.args))
    return " ".join(parts)


def instantiate_args(spec: List[dict]) -> List[dict]:
    """Copy an argument specification and seed each value from its default."""
    out = []
    for a in spec:
        b = dict(a)
        b["value"] = b.get("default", "")
        out.append(b)
    return out
