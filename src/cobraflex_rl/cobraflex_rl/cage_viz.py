"""cage_viz — real-time RViz view of what the cage and the agent perceive.

During training/eval the cage state and the policy observation are computed
in-process and never reach the ROS bus (only the Gazebo sensors and ``/cmd_vel``
are published). This optional helper publishes, each control cycle:

* ``/cage/markers`` (``visualization_msgs/MarkerArray``) — the road-centre
  centerline, the painted road **edges** (the off-road boundary, road centre ±
  road_width/2), the reward (right-lane) target line, the vehicle marker
  colour-coded by state (green = on-road / orange = cage intervention / red =
  emergency or off-road), and a status text label.
* ``/agent/obs_image`` (``sensor_msgs/Image``, mono8) — the exact 84×84 grayscale
  frame the CNN policy sees this step.

ROS message types are imported lazily inside ``__init__`` so the module (and the
package) stays importable without ROS for the pure-Python tests.
"""

from __future__ import annotations

import array
from typing import Optional, Sequence, Tuple

import numpy as np


class CageViz:
    """Publishes the cage/agent runtime view for RViz (fixed frame ``odom``)."""

    def __init__(
        self,
        node,
        reward_centerline: np.ndarray,
        road_centerline: Optional[np.ndarray],
        road_width: float,
        frame_id: str = "odom",
    ) -> None:
        from visualization_msgs.msg import MarkerArray
        from sensor_msgs.msg import Image

        self._MarkerArray = MarkerArray
        self._Image = Image
        self.node = node
        self.frame_id = frame_id
        self.half_width = float(road_width) * 0.5
        self.reward_cl = np.asarray(reward_centerline, dtype=float)
        self.road_cl = (
            np.asarray(road_centerline, dtype=float)
            if road_centerline is not None
            else None
        )
        self.markers_pub = node.create_publisher(MarkerArray, "/cage/markers", 1)
        self.obs_pub = node.create_publisher(Image, "/agent/obs_image", 1)
        self._edges = (
            self._compute_edges(self.road_cl) if self.road_cl is not None else None
        )

    def _compute_edges(self, c: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Left/right road edge polylines = centre ± half_width along the normal."""
        tangent = np.gradient(c, axis=0)
        normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
        normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-9)
        return c + normal * self.half_width, c - normal * self.half_width

    # ------------------------------------------------------------- markers
    def _stamp(self):
        # Zero stamp => RViz resolves the marker's frame against the *latest*
        # available transform. The train node runs on the wall clock (it is not
        # launched with use_sim_time) while RViz runs on sim time, so a real
        # timestamp here is ~decades in RViz's future and the TF lookup fails,
        # making the markers glitch/streak. A zero stamp sidesteps that.
        from rclpy.time import Time

        return Time().to_msg()

    def _line(self, mid: int, pts: np.ndarray, rgba, width: float = 0.008):
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self._stamp()
        m.ns = "cage"
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = width
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.01) for p in pts]
        return m

    def _sphere(self, mid: int, x: float, y: float, rgba, size: float = 0.07):
        from visualization_msgs.msg import Marker

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self._stamp()
        m.ns = "cage"
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _text(self, mid: int, x: float, y: float, text: str, rgba):
        from visualization_msgs.msg import Marker

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self._stamp()
        m.ns = "cage"
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.2
        m.pose.orientation.w = 1.0
        m.scale.z = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.text = text
        return m

    def publish(
        self,
        pose: Tuple[float, float, float],
        off_road: bool,
        emergency: bool,
        interventions: Sequence[str],
        obs: Optional[np.ndarray] = None,
    ) -> None:
        ma = self._MarkerArray()
        if self.road_cl is not None:
            ma.markers.append(self._line(0, self.road_cl, (0.7, 0.7, 0.7, 0.8)))
            ma.markers.append(self._line(1, self._edges[0], (1.0, 0.85, 0.0, 0.95)))
            ma.markers.append(self._line(2, self._edges[1], (1.0, 0.85, 0.0, 0.95)))
        ma.markers.append(self._line(3, self.reward_cl, (0.2, 0.8, 0.35, 0.7)))

        x, y, _ = pose
        if off_road:
            rgba, status = (1.0, 0.0, 0.0, 1.0), "OFF-ROAD"
        elif emergency:
            rgba, status = (1.0, 0.25, 0.0, 1.0), "EMERGENCY"
        elif interventions:
            rgba = (1.0, 0.6, 0.0, 1.0)
            status = "CAGE " + ",".join(sorted(set(interventions)))
        else:
            rgba, status = (0.2, 0.9, 0.35, 1.0), "OK"
        ma.markers.append(self._sphere(4, x, y, rgba))
        ma.markers.append(self._text(5, x, y, status, rgba))
        self.markers_pub.publish(ma)

        if obs is not None:
            self._publish_obs(obs)

    def _publish_obs(self, obs: np.ndarray) -> None:
        arr = np.asarray(obs)
        if arr.ndim == 3:
            arr = arr[:, :, 0]
        if arr.ndim != 2:
            return
        arr = arr.astype(np.uint8)
        msg = self._Image()
        msg.header.frame_id = "camera_link_optical_lane"
        msg.header.stamp = self._stamp()
        msg.height, msg.width = int(arr.shape[0]), int(arr.shape[1])
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = int(arr.shape[1])
        # array.array('B', ...): the only fast path through rclpy's uint8[]
        # setter — see csi_camera_node._on_tick for the measurement.
        msg.data = array.array("B", arr.tobytes())
        self.obs_pub.publish(msg)
