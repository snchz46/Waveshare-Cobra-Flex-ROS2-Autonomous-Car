"""cobraflex_rl — RL training/eval package for the CobraFlex lane-following task.

Top-level names are imported lazily (PEP 562 ``__getattr__``): importing
``cobraflex_rl`` itself must stay cheap and ROS2-free so that pure-Python
consumers (pytest, the campaign runner) can use submodules like ``rewards``
or ``polyline_tracker`` without pulling in ``rclpy``/Gazebo, which only the
``GazeboLaneEnv``/``RosGazeboInterface`` path needs.
"""

from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any

__all__ = [
    "GazeboLaneEnv",
    "PolylineTracker",
    "RosGazeboInterface",
    "TrackState",
]

_LAZY_ATTRS = {
    "GazeboLaneEnv": ("cobraflex_rl.gazebo_lane_env", "GazeboLaneEnv"),
    "PolylineTracker": ("cobraflex_rl.polyline_tracker", "PolylineTracker"),
    "TrackState": ("cobraflex_rl.polyline_tracker", "TrackState"),
    "RosGazeboInterface": ("cobraflex_rl.ros_interface", "RosGazeboInterface"),
}


def __getattr__(name: str) -> Any:
    try:
        module_name, attr_name = _LAZY_ATTRS[name]
    except KeyError as exc:
        raise AttributeError(f"module 'cobraflex_rl' has no attribute {name!r}") from exc
    module = import_module(module_name)
    value = getattr(module, attr_name)
    globals()[name] = value
    return value


if TYPE_CHECKING:
    from .gazebo_lane_env import GazeboLaneEnv
    from .polyline_tracker import PolylineTracker, TrackState
    from .ros_interface import RosGazeboInterface
