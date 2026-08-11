"""
camera_geometry — deterministic pinhole ground-plane projection for track 'E'.

The cage's CV lane-estimator (D-43) converts detected lane-line pixels into
metric lateral offsets, so it needs the camera's pixel↔ground mapping. The
camera is pitch-only (no roll, no yaw relative to the vehicle), which makes the
mapping closed-form and auditable:

* each image row ``v`` below the horizon corresponds to a single ground
  distance ``X(v)`` ahead of the camera, and
* within that row, the column ``u`` maps linearly to the lateral ground
  coordinate ``Y``.

Frames: vehicle/ground frame has X forward, Y left, origin at the camera's
vertical ground projection; the optical frame is the usual x-right / y-down /
z-forward. All parameters mirror the Gazebo sensor + URDF mount (the single
source of truth for the physical numbers):

* intrinsics from the ``Lane Cam`` sensor (640x360, HFOV 1.5707963 rad —
  mirroring the real IMX219-160 as configured in ``lane_keeper_node.py``),
* extrinsics from the URDF chain (camera height above ground, pitch-down).

Pure numpy/stdlib — host-testable without ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

# URDF-derived defaults (my_robot_gazebo_mesh.urdf — the variant every
# train/eval launch includes via gazebo_mesh.launch.py): base_link at
# wheel_radius 0.03725, body_link at +body_height/2+chassis_height/2-0.01
# (0.05+0.03-0.01, mesh-variant body offset), camera_link_lane at -0.03
# → 0.07725 m above ground; pitch 0.30 rad (matches the camera_link_lane joint
# rpy="0 0.30 0" in my_robot_gazebo_mesh.urdf — re-run
# tools/validate_cv_estimator.py if it changes). NOTE the non-mesh URDF
# variants currently lack the -0.01 body offset (camera 1 cm higher).
DEFAULT_CAMERA_HEIGHT_M = 0.03725 + 0.05 + 0.03 - 0.01 - 0.03
DEFAULT_CAMERA_PITCH_RAD = 0.30
DEFAULT_HFOV_RAD = 1.5707963
DEFAULT_WIDTH_PX = 640
DEFAULT_HEIGHT_PX = 360


@dataclass(frozen=True)
class CameraModel:
    """Pitch-only pinhole camera over a flat ground plane."""

    height_m: float = DEFAULT_CAMERA_HEIGHT_M
    pitch_rad: float = DEFAULT_CAMERA_PITCH_RAD
    hfov_rad: float = DEFAULT_HFOV_RAD
    width_px: int = DEFAULT_WIDTH_PX
    height_px: int = DEFAULT_HEIGHT_PX

    def __post_init__(self) -> None:
        if self.height_m <= 0:
            raise ValueError("height_m must be > 0")
        if not 0.0 < self.hfov_rad < math.pi:
            raise ValueError("hfov_rad must be in (0, pi)")
        if self.width_px < 2 or self.height_px < 2:
            raise ValueError("image dimensions must be >= 2 px")

    @property
    def fx(self) -> float:
        return (self.width_px / 2.0) / math.tan(self.hfov_rad / 2.0)

    @property
    def fy(self) -> float:
        return self.fx  # square pixels (Gazebo camera default)

    @property
    def cx(self) -> float:
        return self.width_px / 2.0

    @property
    def cy(self) -> float:
        return self.height_px / 2.0

    @property
    def horizon_row(self) -> float:
        """Image row of the horizon (ground at infinity): v where the ray is
        parallel to the ground, i.e. yo = -tan(pitch)."""
        return self.cy - self.fy * math.tan(self.pitch_rad)

    def row_to_distance(self, v: float) -> float:
        """Ground distance X (m, forward) seen by image row ``v``.

        Pitch-only mounting makes X independent of the column. Rows at or
        above the horizon have no ground intersection → ValueError.
        """
        yo = (v - self.cy) / self.fy
        denom = math.sin(self.pitch_rad) + yo * math.cos(self.pitch_rad)
        if denom <= 1e-9:
            raise ValueError(f"row {v} is at/above the horizon (no ground point)")
        t = self.height_m / denom
        return t * (math.cos(self.pitch_rad) - yo * math.sin(self.pitch_rad))

    def distance_to_row(self, x_m: float) -> float:
        """Inverse of :meth:`row_to_distance` (X forward → image row)."""
        if x_m <= 0:
            raise ValueError("x_m must be > 0 (ahead of the camera)")
        sp, cp = math.sin(self.pitch_rad), math.cos(self.pitch_rad)
        # yo solves x = h(c - yo*s)/(s + yo*c)
        yo = (self.height_m * cp - x_m * sp) / (x_m * cp + self.height_m * sp)
        return self.cy + self.fy * yo

    def pixel_to_ground(self, u: float, v: float) -> Tuple[float, float]:
        """(u, v) → ground (X forward, Y left) in metres."""
        x = self.row_to_distance(v)
        yo = (v - self.cy) / self.fy
        denom = math.sin(self.pitch_rad) + yo * math.cos(self.pitch_rad)
        t = self.height_m / denom
        xo = (u - self.cx) / self.fx
        y = -t * xo
        return x, y

    def ground_to_pixel(self, x_m: float, y_m: float) -> Tuple[float, float]:
        """Ground (X forward, Y left) → image (u, v)."""
        if x_m <= 0:
            raise ValueError("x_m must be > 0 (ahead of the camera)")
        sp, cp = math.sin(self.pitch_rad), math.cos(self.pitch_rad)
        # Vector camera→point in vehicle frame: (x, y, -h); optical components:
        xo_n = -y_m                              # · x_opt axis (0,-1,0)
        yo_n = -x_m * sp + self.height_m * cp    # · y_opt axis (-s,0,-c)
        zo_n = x_m * cp + self.height_m * sp     # · z_opt axis (c,0,-s)
        u = self.cx + self.fx * xo_n / zo_n
        v = self.cy + self.fy * yo_n / zo_n
        return u, v

    def lateral_resolution(self, v: float) -> float:
        """Metres of lateral ground per pixel at image row ``v``."""
        yo = (v - self.cy) / self.fy
        denom = math.sin(self.pitch_rad) + yo * math.cos(self.pitch_rad)
        if denom <= 1e-9:
            raise ValueError(f"row {v} is at/above the horizon")
        t = self.height_m / denom
        return t / self.fx
