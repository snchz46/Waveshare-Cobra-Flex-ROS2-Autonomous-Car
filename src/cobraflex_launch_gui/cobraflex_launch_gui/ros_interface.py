"""ROS side of the window: graph introspection and a TF listener.

One hidden node spins in a daemon thread; the widgets read snapshots from the
Qt thread. Graph queries read the rmw graph cache of this process, which DDS
discovery keeps current, so no ros2 CLI process is started. The TF
subscriptions are created only when the TF tab is first opened.

MockRosInterface offers the same methods with a fixed graph and TF tree of
the simulated robot, for development without ROS.
"""

from collections import deque
from dataclasses import dataclass, field
import threading
import time
from typing import Deque, Dict, List, Optional, Set, Tuple

from cobraflex_launch_gui.constants import TF_STALE_S, TF_WINDOW_S

#: Name of the window's own node. The leading underscore marks it hidden, as
#: for the ros2cli daemon, and the graph view skips hidden nodes.
NODE_NAME = "_cobraflex_launch_gui"


def is_hidden(name: str) -> bool:
    """Return whether a node or topic name has a hidden component."""
    return any(part.startswith("_") for part in name.split("/") if part)


@dataclass
class NodeInfo:
    """A node and the topics it uses."""

    name: str
    publishes: List[str] = field(default_factory=list)
    subscribes: List[str] = field(default_factory=list)
    #: More than one node in the graph has this name.
    duplicate: bool = False


@dataclass
class TopicInfo:
    """A topic, its types and its endpoints."""

    name: str
    types: Set[str] = field(default_factory=set)
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)


@dataclass
class GraphSnapshot:
    """Nodes and topics of the ROS graph at one instant."""

    nodes: Dict[str, NodeInfo] = field(default_factory=dict)
    topics: Dict[str, TopicInfo] = field(default_factory=dict)


@dataclass
class TfEdge:
    """One parent -> child transform as seen on /tf or /tf_static."""

    parent: str
    child: str
    static: bool
    rate_hz: float
    age_s: float


def build_snapshot(entries: List[Tuple[str, List[Tuple[str, List[str]]],
                                       List[Tuple[str, List[str]]]]]) -> GraphSnapshot:
    """Assemble a snapshot from (node, publishers, subscribers) tuples.

    Publishers and subscribers are (topic, [types]) pairs, as returned by the
    rclpy graph API. Hidden nodes and topics are dropped.
    """
    snap = GraphSnapshot()
    for fqn, pubs, subs in entries:
        if is_hidden(fqn):
            continue
        info = snap.nodes.get(fqn)
        if info is None:
            info = snap.nodes[fqn] = NodeInfo(fqn)
        else:
            info.duplicate = True
        for topic, types in pubs:
            if not is_hidden(topic):
                info.publishes.append(topic)
                t = snap.topics.setdefault(topic, TopicInfo(topic))
                t.types.update(types)
                t.publishers.append(fqn)
        for topic, types in subs:
            if not is_hidden(topic):
                info.subscribes.append(topic)
                t = snap.topics.setdefault(topic, TopicInfo(topic))
                t.types.update(types)
                t.subscribers.append(fqn)
    return snap


def analyse_tf(edges) -> Tuple[Dict[str, Set[str]], Set[Tuple[str, str]], List[str], List[str]]:
    """Return (conflicts, stale edges, roots, frames) of a list of TfEdge.

    conflicts maps a frame to its parents when it has more than one current
    parent; a tree has exactly one parent per frame.
    """
    parents: Dict[str, Set[str]] = {}
    stale: Set[Tuple[str, str]] = set()
    frames: Set[str] = set()
    children: Set[str] = set()
    for e in edges:
        frames.update((e.parent, e.child))
        children.add(e.child)
        if not e.static and e.age_s > TF_STALE_S:
            stale.add((e.parent, e.child))
        if e.static or e.age_s <= TF_WINDOW_S:
            parents.setdefault(e.child, set()).add(e.parent)
    conflicts = {c: p for c, p in parents.items() if len(p) > 1}
    roots = sorted(frames - children)
    return conflicts, stale, roots, sorted(frames)


class _TfRecord:
    def __init__(self, static: bool):
        self.static = static
        self.times: Deque[float] = deque(maxlen=400)

    def edge(self, parent: str, child: str, now: float) -> TfEdge:
        last = self.times[-1] if self.times else now
        recent = [t for t in self.times if now - t <= TF_WINDOW_S]
        rate = 0.0
        if not self.static and len(recent) >= 2:
            rate = (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)
        return TfEdge(parent, child, self.static, rate, now - last)


class TfStore:
    """Thread-safe record of the transforms received, keyed by edge."""

    def __init__(self):
        """Start with no transform recorded."""
        self._lock = threading.Lock()
        self._records: Dict[Tuple[str, str], _TfRecord] = {}

    def add(self, parent: str, child: str, static: bool, stamp: Optional[float] = None) -> None:
        """Record the arrival of one transform."""
        key = (parent.lstrip("/"), child.lstrip("/"))
        now = time.monotonic() if stamp is None else stamp
        with self._lock:
            rec = self._records.get(key)
            if rec is None:
                rec = self._records[key] = _TfRecord(static)
            rec.static = rec.static or static
            rec.times.append(now)

    def snapshot(self, now: Optional[float] = None) -> List[TfEdge]:
        """Return every edge seen so far, with its rate and age."""
        now = time.monotonic() if now is None else now
        with self._lock:
            return [rec.edge(p, c, now) for (p, c), rec in sorted(self._records.items())]

    def clear(self) -> None:
        """Forget every transform seen so far."""
        with self._lock:
            self._records.clear()


class RosInterface:
    """Hidden rclpy node: graph queries and the TF listener."""

    is_mock = False

    def __init__(self):
        """Create the node and spin it in a daemon thread (rclpy initialised)."""
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        self._node = rclpy.create_node(NODE_NAME)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._stop = threading.Event()
        self._tf = TfStore()
        self._tf_started = False
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while not self._stop.is_set() and self._rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                # Raised while the context shuts down; nothing left to spin.
                break

    def graph_snapshot(self) -> GraphSnapshot:
        """Query nodes and their publishers and subscribers."""
        entries = []
        for name, ns in self._node.get_node_names_and_namespaces():
            fqn = (ns.rstrip("/") + "/" + name) if ns else "/" + name
            if is_hidden(fqn):
                continue
            try:
                pubs = self._node.get_publisher_names_and_types_by_node(name, ns)
                subs = self._node.get_subscriber_names_and_types_by_node(name, ns)
            except Exception:
                # The node left the graph between the two queries.
                continue
            entries.append((fqn, pubs, subs))
        return build_snapshot(entries)

    def start_tf(self) -> None:
        """Subscribe to /tf and /tf_static (idempotent)."""
        if self._tf_started:
            return
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_msgs.msg import TFMessage

        self._tf_started = True
        # /tf_static is latched: transient-local durability delivers the
        # transforms that were published before this subscription existed.
        static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self._node.create_subscription(TFMessage, "/tf", lambda m: self._on_tf(m, False), 100)
        self._node.create_subscription(TFMessage, "/tf_static",
                                       lambda m: self._on_tf(m, True), static_qos)

    def _on_tf(self, msg, static: bool) -> None:
        for t in msg.transforms:
            self._tf.add(t.header.frame_id, t.child_frame_id, static)

    def tf_snapshot(self) -> List[TfEdge]:
        """Return the transforms received so far."""
        return self._tf.snapshot()

    def clear_tf(self) -> None:
        """Forget the transforms received so far."""
        self._tf.clear()

    def shutdown(self) -> None:
        """Stop the spin thread and destroy the node."""
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._executor.shutdown(timeout_sec=0.5)
            self._node.destroy_node()
        except Exception:
            # The context may already be shut down by a signal.
            pass


class MockRosInterface:
    """Fixed graph and TF tree of the simulated robot, without ROS."""

    is_mock = True

    _GRAPH = (
        ("/robot_state_publisher",
         [("/robot_description", ["std_msgs/msg/String"]),
          ("/tf_static", ["tf2_msgs/msg/TFMessage"]), ("/tf", ["tf2_msgs/msg/TFMessage"])],
         [("/joint_states", ["sensor_msgs/msg/JointState"]),
          ("/clock", ["rosgraph_msgs/msg/Clock"])]),
        ("/ros_gz_bridge",
         [("/clock", ["rosgraph_msgs/msg/Clock"]), ("/scan_gz", ["sensor_msgs/msg/LaserScan"]),
          ("/odom", ["nav_msgs/msg/Odometry"]), ("/joint_states", ["sensor_msgs/msg/JointState"]),
          ("/imu", ["sensor_msgs/msg/Imu"]),
          ("/camera/image_raw_lane", ["sensor_msgs/msg/Image"]),
          ("/tf", ["tf2_msgs/msg/TFMessage"])],
         [("/cmd_vel", ["geometry_msgs/msg/Twist"])]),
        ("/scan_min_range_node",
         [("/scan", ["sensor_msgs/msg/LaserScan"])],
         [("/scan_gz", ["sensor_msgs/msg/LaserScan"]), ("/clock", ["rosgraph_msgs/msg/Clock"])]),
        ("/ekf_filter_node",
         [("/odometry/filtered", ["nav_msgs/msg/Odometry"])],
         [("/odom", ["nav_msgs/msg/Odometry"]), ("/imu", ["sensor_msgs/msg/Imu"]),
          ("/clock", ["rosgraph_msgs/msg/Clock"])]),
        ("/lane_keeper_gazebo_node",
         [("/cmd_vel", ["geometry_msgs/msg/Twist"]),
          ("/lane_debug_image", ["sensor_msgs/msg/Image"])],
         [("/camera/image_raw_lane", ["sensor_msgs/msg/Image"]),
          ("/clock", ["rosgraph_msgs/msg/Clock"])]),
        ("/rviz2", [],
         [("/scan", ["sensor_msgs/msg/LaserScan"]), ("/tf", ["tf2_msgs/msg/TFMessage"]),
          ("/robot_description", ["std_msgs/msg/String"]),
          ("/lane_debug_image", ["sensor_msgs/msg/Image"]),
          ("/clock", ["rosgraph_msgs/msg/Clock"])]),
    )

    _TF = (
        ("odom", "base_footprint", False, 50.0),
        ("base_footprint", "base_link", True, 0.0),
        ("base_link", "body_link", True, 0.0),
        ("base_link", "lidar_link", True, 0.0),
        ("base_link", "imu_link", True, 0.0),
        ("body_link", "camera_link", True, 0.0),
        ("camera_link", "camera_link_optical", True, 0.0),
        ("body_link", "camera_link_lane", True, 0.0),
        ("camera_link_lane", "camera_link_optical_lane", True, 0.0),
        ("base_link", "front_left_wheel", False, 30.0),
        ("base_link", "front_right_wheel", False, 30.0),
        ("base_link", "rear_left_wheel", False, 30.0),
        ("base_link", "rear_right_wheel", False, 30.0),
    )

    def graph_snapshot(self) -> GraphSnapshot:
        """Return the fixed graph."""
        return build_snapshot(list(self._GRAPH))

    def start_tf(self) -> None:
        """Do nothing; the mock tree needs no subscription."""

    def tf_snapshot(self) -> List[TfEdge]:
        """Return the fixed TF tree, as if every transform were current."""
        return [TfEdge(p, c, s, r, 0.0 if s else 1.0 / r) for p, c, s, r in self._TF]

    def clear_tf(self) -> None:
        """Do nothing; the mock tree is fixed."""

    def shutdown(self) -> None:
        """Do nothing."""
