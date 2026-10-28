"""Unit tests of the Qt-free parts: log parsing, argument extraction, layout, TF analysis."""

from pathlib import Path

from cobraflex_launch_gui.discovery import (
    extract_description,
    parse_python_args,
    parse_xml_args,
    split_launch_name,
    workspace_label,
)
from cobraflex_launch_gui.graph_layout import (
    build_dot,
    layout_layered,
    LayoutEdge,
    LayoutNode,
    parse_plain,
)
from cobraflex_launch_gui.meta import LaunchMeta
from cobraflex_launch_gui.model import (
    args_to_overrides,
    build_command,
    instantiate_args,
    Launch,
    LogEntry,
    logger_node_candidates,
    missing_required,
)
from cobraflex_launch_gui.ros_interface import analyse_tf, build_snapshot, TfEdge, TfStore


# --- Log lines --------------------------------------------------------------


def test_node_line_keeps_level_and_logger():
    """A formatted node line yields its level, logger and process."""
    e = LogEntry.parse("[ekf_node-5] [ERROR] [1730000000.5] [ekf_filter_node]: no odometry")
    assert (e.level, e.node, e.process, e.msg) == \
        ("ERROR", "ekf_filter_node", "ekf_node-5", "no odometry")


def test_launch_line_and_severity_folding():
    """Lines of ros2 launch itself and WARNING/FATAL fold onto the panel levels."""
    e = LogEntry.parse("[ERROR] [rviz2-4]: process has died [pid 42, exit code -6]")
    assert (e.level, e.node) == ("ERROR", "rviz2-4")
    assert LogEntry.parse("[x-1] [WARNING] [1.0] [x]: m").level == "WARN"
    assert LogEntry.parse("[x-1] [FATAL] [1.0] [x]: m").level == "ERROR"


def test_raw_process_output():
    """Tracebacks and Gazebo errors are errors; other free text is INFO."""
    assert LogEntry.parse("[my_node-1] Traceback (most recent call last):").level == "ERROR"
    assert LogEntry.parse("[my_node-1] ModuleNotFoundError: No module named 'x'").level == "ERROR"
    assert LogEntry.parse("[gz-1] [Err] [SystemLoader.cc:92] failed").level == "ERROR"
    assert LogEntry.parse("[my_node-1] Error handling done: ok").level == "INFO"


def test_ansi_codes_are_stripped():
    """Colour escape sequences do not break the classification."""
    e = LogEntry.parse("\x1b[33m[x-1] [WARN] [1.0] [x]: slow\x1b[0m")
    assert (e.level, e.msg) == ("WARN", "slow")


def test_logger_candidates_cover_namespace_and_children():
    """Dotted loggers map to every prefix as a node name."""
    assert logger_node_candidates("robot1.planner_server.costmap") == \
        {"/robot1", "/robot1/planner_server", "/robot1/planner_server/costmap"}


# --- Arguments --------------------------------------------------------------

_PY = """
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    return [
        DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz."),
        DeclareLaunchArgument("checkpoint", description="Model to load."),
        DeclareLaunchArgument("world", default_value=os.path.join(share, "a.world")),
        DeclareLaunchArgument("mode", default_value="enforcement",
                              choices=["enforcement", "monitoring"]),
        DeclareLaunchArgument(name="rate", default_value="10.0"),
    ]
"""

_XML = """<launch>
    <!-- Layer 1 of the robot: model and driver. Second sentence. -->
    <arg name="use_rviz" default="true"/>
    <arg name="urdf" default="$(find-pkg-share cobraflex)/urdf/a.urdf"/>
    <arg name="port"/>
    <include file="$(find-pkg-share cobraflex)/launch/other.launch.xml">
        <arg name="use_rviz" value="$(var use_rviz)"/>
    </include>
</launch>
"""


def test_python_args():
    """Kinds, required arguments, computed defaults and choices are recognised."""
    args = {a["key"]: a for a in parse_python_args(_PY)}
    assert list(args) == ["use_rviz", "checkpoint", "world", "mode", "rate"]
    assert args["use_rviz"]["kind"] == "bool" and args["use_rviz"]["help"] == "Start RViz."
    assert args["checkpoint"].get("required") is True
    assert args["world"]["default"] == "" and "os.path.join" in args["world"]["hint"]
    assert args["mode"]["kind"] == "enum"
    assert args["mode"]["options"] == ["enforcement", "monitoring"]
    assert args["rate"]["kind"] == "number"


def test_xml_args_skip_include_values():
    """Only top-level declarations count; <arg value=> inside <include> does not."""
    args = parse_xml_args(_XML)
    assert [a["key"] for a in args] == ["use_rviz", "urdf", "port"]
    assert "$(find-pkg-share" in args[1]["hint"]
    assert args[2].get("required") is True


def test_description_and_names(tmp_path: Path):
    """First sentence of the leading comment; both file naming conventions."""
    f = tmp_path / "robot.launch.xml"
    f.write_text(_XML, encoding="utf-8")
    assert extract_description(f, "xml", "robot") == "Layer 1 of the robot: model and driver"
    g = tmp_path / "demo.launch.py"
    g.write_text('"""demo.launch.py - Demo of the stack. More."""\n', encoding="utf-8")
    assert extract_description(g, "py", "demo") == "Demo of the stack"
    assert split_launch_name("a.launch.py") == ("a", "py")
    assert split_launch_name("b_launch.xml") == ("b", "xml")
    assert split_launch_name("notes.txt") is None
    assert workspace_label(Path("/home/u/lab_ws/install/cobraflex_lab"), "cobraflex_lab") \
        == "lab_ws"
    assert workspace_label(Path("/home/u/ros2_ws/install"), "cobraflex") == "ros2_ws"


def test_metadata_cannot_add_arguments():
    """Refinements apply to declared arguments only."""
    declared = parse_python_args(_PY)
    refined = LaunchMeta.refine_args(declared, {"rate": {"options": ["5", "10"]},
                                                "ghost": {"options": ["x"]}})
    assert [a["key"] for a in refined] == [a["key"] for a in declared]
    assert next(a for a in refined if a["key"] == "rate")["kind"] == "enum"


def test_overrides_and_command():
    """Only changed, non-empty values are passed, and they are shell-quoted."""
    launch = Launch("cobraflex", "gazebo", Path("gazebo.launch.py"))
    launch.args = instantiate_args(parse_python_args(_PY))
    assert missing_required(launch.args) == ["checkpoint"]
    for a in launch.args:
        if a["key"] == "use_rviz":
            a["value"] = "false"
        if a["key"] == "checkpoint":
            a["value"] = "/tmp/my model.zip"
    assert args_to_overrides(launch.args) == ["use_rviz:=false", "checkpoint:=/tmp/my model.zip"]
    assert build_command(launch) == ("ros2 launch cobraflex gazebo.launch.py use_rviz:=false "
                                     "'checkpoint:=/tmp/my model.zip'")
    assert missing_required(launch.args) == []


def test_node_attribution():
    """A launch owns the nodes named in its loggers or after its processes."""
    launch = Launch("p", "n", Path("n.launch.py"))
    launch.node_hints = logger_node_candidates("ekf_filter_node")
    launch.process_hints = {"robot_state_publisher"}
    assert launch.owns_node("/ekf_filter_node")
    assert launch.owns_node("/robot_state_publisher")
    assert not launch.owns_node("/rviz2")


# --- Graph and layout -------------------------------------------------------


def test_snapshot_marks_duplicates_and_hides_hidden_names():
    """Two nodes with one name are flagged; hidden nodes and topics are dropped."""
    snap = build_snapshot([
        ("/talker", [("/chatter", ["std_msgs/msg/String"])], []),
        ("/talker", [("/chatter", ["std_msgs/msg/String"])], []),
        ("/listener", [], [("/chatter", ["std_msgs/msg/String"])]),
        ("/_hidden", [("/chatter", ["std_msgs/msg/String"])], []),
        ("/x", [("/_action/status", ["a"])], []),
    ])
    assert snap.nodes["/talker"].duplicate
    assert "/_hidden" not in snap.nodes and "/_action/status" not in snap.topics
    assert snap.topics["/chatter"].subscribers == ["/listener"]


def test_layered_layout_places_every_node_and_routes_cycles():
    """The built-in layout handles cycles and long edges."""
    nodes = [LayoutNode(c, 60, 20) for c in "abcd"]
    edges = [LayoutEdge("a", "b"), LayoutEdge("b", "c"), LayoutEdge("c", "a"),
             LayoutEdge("a", "d"), LayoutEdge("a", "c", "label")]
    layout = layout_layered(nodes, edges, "LR")
    assert set(layout.centers) == set("abcd")
    assert len(layout.routes) == len(edges)
    for r in layout.routes:
        # Each route starts at its source box and ends at its destination box.
        sx, sy = layout.centers[r.src]
        dx, dy = layout.centers[r.dst]
        assert abs(r.points[0][0] - sx) <= 30.5 and abs(r.points[-1][0] - dx) <= 30.5
        assert abs(r.points[0][1] - sy) <= 10.5 and abs(r.points[-1][1] - dy) <= 10.5


def test_dot_plain_output_is_parsed():
    """Coordinates are converted from inches (y up) to points (y down)."""
    nodes = [LayoutNode("a", 72, 36), LayoutNode("b", 72, 36)]
    edges = [LayoutEdge("a", "b", "/topic name")]
    source, names = build_dot(nodes, edges, "LR")
    assert 'n0 -> n1 [label="/topic name"];' in source
    plain = ("graph 1 3 1\n"
             "node n0 0.5 0.5 1 0.5 \"\" solid box black lightgrey\n"
             "node n1 2.5 0.5 1 0.5 \"\" solid box black lightgrey\n"
             "edge n0 n1 4 1 0.5 1.5 0.5 1.5 0.5 2 0.5 \"/topic name\" 1.5 0.7 solid black\n"
             "stop\n")
    layout = parse_plain(plain, names, edges)
    assert layout.centers == {"a": (36.0, 36.0), "b": (180.0, 36.0)}
    route = layout.routes[0]
    assert route.spline and route.points[0] == (72.0, 36.0) and route.points[-1] == (144.0, 36.0)
    assert route.label == "/topic name" and route.label_pos == (108.0, 21.6)


# --- TF ---------------------------------------------------------------------


def test_tf_rate_and_conflicts():
    """Rates come from arrival times; two current parents of a frame conflict."""
    store = TfStore()
    for i in range(11):
        store.add("odom", "base_footprint", False, stamp=100.0 + 0.1 * i)
    store.add("base_footprint", "base_link", True, stamp=100.0)
    edge = {(e.parent, e.child): e for e in store.snapshot(now=101.0)}[("odom", "base_footprint")]
    assert abs(edge.rate_hz - 10.0) < 1e-6

    edges = [TfEdge("odom", "base_footprint", False, 50.0, 0.1),
             TfEdge("odom_gt", "base_footprint", False, 30.0, 0.2),
             TfEdge("base_footprint", "base_link", True, 0.0, 9.0),
             TfEdge("map", "odom", False, 0.0, 3.0)]
    conflicts, stale, roots, frames = analyse_tf(edges)
    assert conflicts == {"base_footprint": {"odom", "odom_gt"}}
    assert stale == {("map", "odom")}
    assert roots == ["map", "odom_gt"]
    assert len(frames) == 5
