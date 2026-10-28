"""Process backends: RosBackend runs ros2 launch, MockBackend simulates it.

RosBackend starts every launch in a session of its own (util-linux setsid),
so that one SIGINT to its process group reaches all of its processes, the
same signal a Ctrl-C in a terminal delivers. Processes that leave the group
(gz sim calls setsid() itself) are found through /proc and signalled
separately. Linux only; the mock backend runs anywhere.
"""

import os
import random
import shutil
import signal
import time
from typing import Callable, Dict, List, Optional, Set

from PyQt5.QtCore import QProcess, QProcessEnvironment, QTimer

from cobraflex_launch_gui.constants import (
    FORCE_KILL_TIMEOUT_MS,
    LOGS_MAX_PER_LAUNCH,
    MOCK_LOG_TICK_MS,
    PROCESS_START_TIMEOUT_MS,
    SIGKILL_TIMEOUT_MS,
)
from cobraflex_launch_gui.model import (
    args_to_overrides,
    build_command,
    Launch,
    LogEntry,
    logger_node_candidates,
    PROCESS_DIED_RE,
    PROCESS_STARTED_RE,
    strip_process_index,
)


def collect_descendants(pid: int) -> List[int]:
    """Return all descendant PIDs of a process, from one scan of /proc."""
    children: Dict[int, List[int]] = {}
    try:
        entries = list(os.scandir("/proc"))
    except OSError:
        return []
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            with open(f"{entry.path}/stat") as fh:
                stat = fh.read()
            # "pid (comm) state ppid ...": comm may contain spaces and
            # parentheses, so the fields are counted from the last ")".
            ppid = int(stat[stat.rfind(")") + 2:].split()[1])
            children.setdefault(ppid, []).append(int(entry.name))
        except (OSError, ValueError, IndexError):
            continue
    out: List[int] = []
    queue = list(children.get(pid, []))
    while queue:
        p = queue.pop(0)
        out.append(p)
        queue.extend(children.get(p, []))
    return out


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


class Backend:
    """Common interface of the backends.

    The window sets two callbacks: on_state_changed(launch) after a change of
    state, PID or exit code, and on_log_line(launch, entry) for every line.
    """

    mode_label = ""

    def __init__(self):
        """Install no-op callbacks until the window replaces them."""
        self.on_state_changed: Callable[[Launch], None] = lambda _l: None
        self.on_log_line: Callable[[Launch, LogEntry], None] = lambda _l, _e: None

    def run(self, launch: Launch) -> None:
        """Start a launch."""

    def stop(self, launch: Launch) -> None:
        """Ask a launch to stop, escalating if it does not."""

    def restart(self, launch: Launch) -> None:
        """Stop a launch and start it again once it has exited."""

    def shutdown(self) -> None:
        """Stop every launch and wait for it; called when the window closes."""

    def _record(self, launch: Launch, entry: LogEntry) -> None:
        """Store a log line and update the state derived from the output."""
        launch.logs.append(entry)
        if len(launch.logs) > LOGS_MAX_PER_LAUNCH:
            del launch.logs[0]
        if entry.level == "ERROR":
            launch.last_error = entry.raw or entry.msg
        if entry.logger:
            launch.node_hints.update(logger_node_candidates(entry.logger))
        if PROCESS_STARTED_RE.search(entry.msg):
            launch.process_hints.add(strip_process_index(entry.node))
        notify = False
        # Processes interrupted by a stop also report "process has died"
        # (exit code -2); only deaths while running count as failures.
        if PROCESS_DIED_RE.search(entry.msg) and launch.state == "running":
            launch.died += 1
            notify = True
        self.on_log_line(launch, entry)
        if notify:
            self.on_state_changed(launch)

    def _log(self, launch: Launch, level: str, msg: str) -> None:
        self._record(launch, LogEntry.parse(f"[{level}] [launch_gui]: {msg}"))

    @staticmethod
    def _reset_run_state(launch: Launch) -> None:
        launch.uptime = 0
        launch.exit_code = None
        launch.last_error = None
        launch.died = 0
        launch.node_hints.clear()
        launch.process_hints.clear()


# --- RosBackend -------------------------------------------------------------


class _Running:
    """Process handle and bookkeeping of one running launch."""

    def __init__(self, process: QProcess, pgid: Optional[int]):
        self.process = process
        #: Process group of the launch (its PID, set by setsid), or None. It
        #: outlives ros2 launch while any of its processes is still alive.
        self.pgid = pgid
        self.buffer = ""
        self.stopping = False
        self.restart = False
        #: Descendants outside the launch's process group, found at stop().
        self.strays: List[int] = []


class RosBackend(Backend):
    """Run launch files with ros2 launch through QProcess."""

    def __init__(self):
        """Locate setsid; without it, stop() signals the launch process only."""
        super().__init__()
        self._setsid: Optional[str] = shutil.which("setsid")
        self._running: Dict[str, _Running] = {}
        #: Stray PIDs not yet terminated, also of runs that have exited.
        self._pending_strays: Set[int] = set()

    def run(self, launch: Launch) -> None:
        """Start `ros2 launch <pkg> <file> key:=value ...`."""
        if launch.uid in self._running:
            return
        argv = ["ros2", "launch", launch.pkg, launch.file_path.name,
                *args_to_overrides(launch.args)]
        proc = QProcess()
        env = QProcessEnvironment.systemEnvironment()
        # Through a pipe, Python nodes buffer print() until exit and rcutils
        # may colour its output; both break the live log pane.
        env.insert("PYTHONUNBUFFERED", "1")
        env.insert("RCUTILS_COLORIZED_OUTPUT", "0")
        proc.setProcessEnvironment(env)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        if self._setsid:
            proc.setProgram(self._setsid)
            proc.setArguments(argv)
        else:
            proc.setProgram(argv[0])
            proc.setArguments(argv[1:])

        self._reset_run_state(launch)
        proc.start()
        if not proc.waitForStarted(PROCESS_START_TIMEOUT_MS):
            self._log(launch, "ERROR", f"failed to start: {proc.errorString()}")
            launch.state = "failed"
            self.on_state_changed(launch)
            return

        launch.pid = int(proc.processId())
        running = _Running(proc, launch.pid if self._setsid else None)
        self._running[launch.uid] = running
        proc.readyReadStandardOutput.connect(lambda: self._on_output(launch, running))
        proc.finished.connect(lambda code, status: self._on_finished(launch, code, status))
        launch.state = "running"
        self._log(launch, "INFO", f"{build_command(launch)}  (PID {launch.pid})")
        self.on_state_changed(launch)

    def stop(self, launch: Launch) -> None:
        """Send SIGINT to the launch, then escalate after a grace period."""
        running = self._running.get(launch.uid)
        if running is None or running.stopping:
            return
        running.stopping = True
        launch.state = "stopping"
        self.on_state_changed(launch)
        n = self._interrupt(running, launch.pid or 0)
        extra = f" and {n} process(es) outside its group" if n else ""
        self._log(launch, "INFO", f"SIGINT sent to the launch{extra}")
        # The timer is bound to this run: after a restart, the new run of the
        # same launch must not receive the escalation meant for the old one.
        QTimer.singleShot(FORCE_KILL_TIMEOUT_MS, lambda: self._escalate(launch, running))

    def restart(self, launch: Launch) -> None:
        """Stop the launch and start it again from _on_finished."""
        running = self._running.get(launch.uid)
        if running is None:
            self.run(launch)
            return
        running.restart = True
        self.stop(launch)

    def shutdown(self) -> None:
        """Stop every launch, blocking until all have exited."""
        self.on_state_changed = lambda _l: None
        self.on_log_line = lambda _l, _e: None
        items = list(self._running.values())
        for r in items:
            r.restart = False
            if not r.stopping:
                r.stopping = True
                self._interrupt(r, r.process.processId())
        deadline = time.monotonic() + FORCE_KILL_TIMEOUT_MS / 1000.0
        for r in items:
            remaining = max(0.0, deadline - time.monotonic())
            r.process.waitForFinished(int(remaining * 1000))
        for r in items:
            if r.process.state() != QProcess.NotRunning:
                self._signal_tree(r.process.processId(), signal.SIGTERM)
                if not r.process.waitForFinished(SIGKILL_TIMEOUT_MS):
                    self._signal_tree(r.process.processId(), signal.SIGKILL)
                    r.process.waitForFinished(1000)
        # Processes orphaned inside the group, e.g. a gz sim that ignored
        # SIGINT after ros2 launch terminated only its sh wrapper.
        for r in items:
            if not self._wait_group(r.pgid, max(0.0, deadline - time.monotonic())):
                self._signal_group(r.pgid, signal.SIGTERM)
                if not self._wait_group(r.pgid, SIGKILL_TIMEOUT_MS / 1000.0):
                    self._signal_group(r.pgid, signal.SIGKILL)
        self._terminate_strays(self._pending_strays)

    # Signals ----------------------------------------------------------------

    @staticmethod
    def _signal_tree(pid: int, sig: int) -> str:
        """Signal the process group of a launch, or the process alone.

        The group is used only when the launch leads its own group (setsid
        took effect). Signalling the group of the window itself would reach
        the window and every other launch.
        """
        if not pid:
            return ""
        try:
            pgid = os.getpgid(pid)
            if pgid == pid and pgid != os.getpgrp():
                os.killpg(pgid, sig)
                return f"process group {pgid}"
            os.kill(pid, sig)
            return f"PID {pid}"
        except OSError:
            return ""

    @staticmethod
    def _signal_group(pgid: Optional[int], sig: int) -> bool:
        """Signal a launch's process group; False if it has no process left.

        Valid after ros2 launch has exited: Linux does not reuse a PID while
        it is the ID of a process group with members. Signal 0 only tests.
        """
        if not pgid or pgid == os.getpgrp():
            return False
        try:
            os.killpg(pgid, sig)
            return True
        except OSError:
            return False

    def _wait_group(self, pgid: Optional[int], timeout_s: float) -> bool:
        """Block until a process group is empty; return whether it is."""
        end = time.monotonic() + timeout_s
        while self._signal_group(pgid, 0):
            if time.monotonic() >= end:
                return False
            time.sleep(0.1)
        return True

    def _interrupt(self, running: _Running, pid: int) -> int:
        """SIGINT the launch group and the descendants outside of it.

        Returns the number of descendants outside the group. Descendants
        inside it are not signalled twice: a second SIGINT interrupts the
        shutdown handlers of Python nodes.
        """
        try:
            pgid = os.getpgid(pid)
        except OSError:
            pgid = None
        strays = []
        for d in collect_descendants(pid):
            try:
                if os.getpgid(d) != pgid:
                    strays.append(d)
            except OSError:
                continue
        running.strays = strays
        self._pending_strays.update(strays)
        self._signal_tree(pid, signal.SIGINT)
        for d in strays:
            try:
                os.kill(d, signal.SIGINT)
            except OSError:
                pass
        return len(strays)

    def _terminate_strays(self, pids) -> int:
        killed = 0
        for pid in list(pids):
            self._pending_strays.discard(pid)
            if _alive(pid):
                os.kill(pid, signal.SIGTERM)
                killed += 1
        return killed

    def _escalate(self, launch: Launch, running: _Running) -> None:
        """SIGTERM what survived the grace period; SIGKILL after a second one.

        ros2 launch may have exited already while a process of its group is
        still alive: it signals only its direct children, so a process behind
        a wrapper (sh -c ruby gz sim) can outlive it.
        """
        killed = self._terminate_strays(running.strays)
        if killed:
            self._log(launch, "WARN", f"SIGTERM sent to {killed} remaining process(es)")
        if self._launch_alive(launch, running):
            target = self._signal_tree(launch.pid or 0, signal.SIGTERM)
            self._log(launch, "WARN", f"still running after SIGINT; SIGTERM sent to {target}")
        elif self._signal_group(running.pgid, signal.SIGTERM):
            self._log(launch, "WARN", f"process group {running.pgid} still running after "
                                      "ros2 launch exited; SIGTERM sent")
        else:
            return
        QTimer.singleShot(SIGKILL_TIMEOUT_MS, lambda: self._kill(launch, running))

    def _kill(self, launch: Launch, running: _Running) -> None:
        if self._launch_alive(launch, running):
            self._signal_tree(launch.pid or 0, signal.SIGKILL)
            running.process.kill()
            self._log(launch, "WARN", "SIGKILL sent")
        elif self._signal_group(running.pgid, signal.SIGKILL):
            self._log(launch, "WARN", f"SIGKILL sent to process group {running.pgid}")

    def _launch_alive(self, launch: Launch, running: _Running) -> bool:
        """Return whether ros2 launch of this run (not of a restart) still runs."""
        return (self._running.get(launch.uid) is running
                and running.process.state() != QProcess.NotRunning)

    # Process output ---------------------------------------------------------

    def _on_output(self, launch: Launch, running: _Running) -> None:
        data = bytes(running.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        # A read can end in the middle of a line; the tail waits for the next.
        *lines, running.buffer = (running.buffer + data).split("\n")
        for line in lines:
            if line.strip():
                self._record(launch, LogEntry.parse(line))

    def _on_finished(self, launch: Launch, code: int, status) -> None:
        running = self._running.pop(launch.uid, None)
        if running is None:
            return
        self._on_output(launch, running)
        if running.buffer.strip():
            self._record(launch, LogEntry.parse(running.buffer))
        crashed = status == QProcess.CrashExit
        self._log(launch, "INFO", f"ros2 launch exited (code {code})")
        if running.stopping:
            launch.state, launch.exit_code = "stopped", None
        elif code != 0 or crashed or launch.died:
            # ros2 launch exits with 0 when a process died and the others ended.
            launch.state = "failed"
            launch.exit_code = code if code != 0 or crashed else None
        else:
            launch.state, launch.exit_code = "stopped", None
        launch.pid = None
        launch.uptime = 0
        self.on_state_changed(launch)
        running.process.deleteLater()
        if running.restart:
            self.run(launch)


# --- MockBackend ------------------------------------------------------------


_MOCK_LINES = (
    ("INFO", "{node}", "publishing at 20.0 Hz"),
    ("INFO", "{node}", "frame OK, 2 lane candidates"),
    ("INFO", "{node}", "steering {:+.3f} rad, throttle {:.2f}"),
    ("WARN", "{node}", "frame 312: low contrast, confidence 0.61"),
)


class MockBackend(Backend):
    """Simulate runs for development without ROS: fake PIDs and log lines.

    Launches tagged Hardware fail after a moment with the error a PC without
    the robot's serial port produces.
    """

    mode_label = "MOCK"

    def __init__(self):
        """Start with no simulated launch running."""
        super().__init__()
        self._timers: Dict[str, QTimer] = {}

    def run(self, launch: Launch) -> None:
        """Pretend to start the launch."""
        if launch.state in ("running", "stopping"):
            return
        self._reset_run_state(launch)
        launch.state = "running"
        launch.pid = random.randint(10000, 99999)
        self._log(launch, "INFO", f"{build_command(launch)}  (PID {launch.pid})")
        node = f"{launch.name}_node"
        self._record(launch, LogEntry.parse(
            f"[INFO] [{node}-1]: process started with pid [{launch.pid + 1}]"))
        self.on_state_changed(launch)
        if "Hardware" in launch.tags:
            QTimer.singleShot(1200, lambda: self._fail(launch, node))
            return
        timer = QTimer()
        timer.timeout.connect(lambda: self._tick(launch, node))
        timer.start(MOCK_LOG_TICK_MS)
        self._timers[launch.uid] = timer

    def stop(self, launch: Launch) -> None:
        """Pretend to stop the launch, with a short shutdown delay."""
        if launch.state != "running":
            return
        self._stop_timer(launch)
        launch.state = "stopping"
        self._log(launch, "INFO", "SIGINT sent to the launch")
        self.on_state_changed(launch)
        QTimer.singleShot(800, lambda: self._finish(launch))

    def restart(self, launch: Launch) -> None:
        """Stop, then start again once the simulated shutdown is over."""
        if launch.state != "running":
            self.run(launch)
            return
        self.stop(launch)
        QTimer.singleShot(900, lambda: self.run(launch))

    def shutdown(self) -> None:
        """Stop all simulated log timers."""
        for timer in self._timers.values():
            timer.stop()
        self._timers.clear()

    def _stop_timer(self, launch: Launch) -> None:
        timer = self._timers.pop(launch.uid, None)
        if timer is not None:
            timer.stop()

    def _finish(self, launch: Launch) -> None:
        self._log(launch, "INFO", "ros2 launch exited (code 0)")
        launch.state = "stopped"
        launch.pid = None
        launch.uptime = 0
        self.on_state_changed(launch)

    def _fail(self, launch: Launch, node: str) -> None:
        if launch.state != "running":
            return
        self._record(launch, LogEntry.parse(
            f"[{node}-1] [ERROR] [{time.time():.3f}] [{node}]: "
            "could not open /dev/ttyACM1: [Errno 2] No such file or directory"))
        self._record(launch, LogEntry.parse(
            f"[ERROR] [{node}-1]: process has died [pid {launch.pid + 1}, exit code 1]"))
        self._log(launch, "INFO", "ros2 launch exited (code 1)")
        launch.state = "failed"
        launch.exit_code = 1
        launch.pid = None
        self.on_state_changed(launch)

    def _tick(self, launch: Launch, node: str) -> None:
        level, _node, msg = random.choice(_MOCK_LINES)
        msg = msg.format(random.uniform(-0.1, 0.1), random.uniform(0.2, 0.5))
        self._record(launch, LogEntry.parse(
            f"[{node}-1] [{level}] [{time.time():.3f}] [{node}]: {msg}"))
