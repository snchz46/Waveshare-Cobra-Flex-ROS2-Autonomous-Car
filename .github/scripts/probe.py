#!/usr/bin/env python3
"""Wait for a ROS 2 condition: a message on a topic, or a TF transform.

    probe.py --topic /scan [--expr "len(msg.ranges) > 0"] [--print "msg.header.frame_id"]
    probe.py --topic /camera/image_raw_lane --save frame.png
    probe.py --tf odom base_footprint

The subscription copies the reliability and durability of the first publisher,
so sensor topics (best effort) and latched topics (transient local) are both
received. --save writes the matching sensor_msgs/Image (rgb8, bgr8 or mono8)
to a PNG file. Exit status 0 when the condition holds before the timeout.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def save_image(msg, path):
    """Write a sensor_msgs/Image to an image file."""
    import cv2
    import numpy as np

    channels = 1 if msg.encoding == "mono8" else 3
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    img = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, img)


def wait_topic(node, topic, expr, pexpr, deadline, save=None):
    """Return the printed value when a message satisfies expr, else None."""
    typ = None
    pubs = []
    while time.monotonic() < deadline and not (typ and pubs):
        for name, types in node.get_topic_names_and_types():
            if name == topic:
                typ = types[0]
        pubs = node.get_publishers_info_by_topic(topic) if typ else []
        rclpy.spin_once(node, timeout_sec=0.5)
    if not (typ and pubs):
        print(f"no publisher on {topic}")
        return None
    qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
    qos.reliability = pubs[0].qos_profile.reliability
    qos.durability = pubs[0].qos_profile.durability
    if qos.reliability not in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
    if qos.durability not in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
        qos.durability = DurabilityPolicy.VOLATILE
    result = {}

    def cb(msg):
        if "value" in result:
            return
        try:
            if eval(expr, {"msg": msg, "math": math}):
                result["value"] = eval(pexpr, {"msg": msg, "math": math}) if pexpr else "ok"
                if save:
                    save_image(msg, save)
        except Exception as exc:  # noqa: BLE001
            result.setdefault("error", repr(exc))

    node.create_subscription(get_message(typ), topic, cb, qos)
    while time.monotonic() < deadline and "value" not in result:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "error" in result and "value" not in result:
        print("expression error:", result["error"])
    return result.get("value")


def wait_tf(node, parent, child, deadline):
    """Return the translation of parent -> child, else None."""
    from tf2_ros import Buffer, TransformListener

    buf = Buffer()
    TransformListener(buf, node)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            t = buf.lookup_transform(parent, child, rclpy.time.Time())
            tr = t.transform.translation
            return f"{tr.x:.3f} {tr.y:.3f} {tr.z:.3f}"
        except Exception:  # noqa: BLE001
            pass
    return None


def main():
    """Parse arguments and run the probe."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic")
    ap.add_argument("--expr", default="True")
    ap.add_argument("--print", dest="pexpr")
    ap.add_argument("--save", metavar="PNG")
    ap.add_argument("--tf", nargs=2, metavar=("PARENT", "CHILD"))
    ap.add_argument("--timeout", type=float, default=120.0)
    a = ap.parse_args()
    rclpy.init()
    node = Node("ci_probe")
    deadline = time.monotonic() + a.timeout
    if a.tf:
        value = wait_tf(node, a.tf[0], a.tf[1], deadline)
    else:
        value = wait_topic(node, a.topic, a.expr, a.pexpr, deadline, a.save)
    node.destroy_node()
    rclpy.shutdown()
    if value is None:
        return 1
    print(value)
    return 0


if __name__ == "__main__":
    sys.exit(main())
