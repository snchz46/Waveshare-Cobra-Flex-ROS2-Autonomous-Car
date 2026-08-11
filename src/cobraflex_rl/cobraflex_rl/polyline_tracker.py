"""Centerline tracking: project a world pose onto a 2D polyline.

Given the lane centerline as an ``(N, 2)`` polyline, :class:`PolylineTracker`
computes the Frenet-style quantities the controllers and the RL env consume:
lateral error ``ey`` (signed, + = left of travel direction), heading error
``epsi``, arc-length ``s``, and a smoothed track heading. Closed polylines
(the F2 oval) are detected automatically and wrap indices/arc-length modulo
the perimeter. Pure NumPy — no ROS2 dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Tuple

import numpy as np


# Window radius (segments) used to smooth track_heading. With a discretised
# centerline polyline (~0.1 m / segment on the F2 oval) the per-segment
# heading is piecewise-constant and steps by ~7 deg on the R=0.8 m curve.
# Averaging over the 2*radius+1 neighbouring segments yields a heading
# that varies continuously with arc length, which is what a path-tracking
# PD needs for its heading-error finite difference to be meaningful.
_HEADING_SMOOTHING_RADIUS = 2

# Search radius (in segments) for the local nearest-segment search after
# the tracker has acquired a previous best index. The global O(N) sweep
# can leapfrog to a non-adjacent segment whenever two segments happen to
# be near-equidistant — on the F2 right-lane oval this manifests as a
# multi-segment jump in segment_index between consecutive cycles, which
# slams a step into ey/epsi and saturates the PD. Constraining the search
# to the immediate neighbourhood of the previous index enforces continuity
# at the cost of an explicit reset when the robot is teleported.
_LOCAL_SEARCH_RADIUS = 2

# Hysteresis margin (m²) for segment selection. A candidate segment that
# differs from the previous best must beat it by this margin to be accepted.
# Prevents the tracker from oscillating between equidistant adjacent segments
# at polyline joints (especially on tight curves where consecutive segments
# project the car to the same endpoint with different ey signs). At 1e-5 m²
# the linear equivalent is ~3.2 mm — large enough to absorb floating-point
# ties at joints, small enough that genuine advancement along the path is
# never suppressed.
_HYSTERESIS_M2 = 1e-5


def wrap_angle(angle: float) -> float:
    """Wrap an angle (rad) to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class TrackState:
    """Result of projecting a pose onto the centerline (one track() call).

    ``ey``: signed lateral error (m), + = left of travel direction.
    ``epsi``: heading error vs the smoothed track heading (rad).
    ``s``: arc-length along the polyline at the projection point (m).
    ``segment_index`` / ``segment_fraction``: where on the polyline the
    projection landed; ``closest_point`` is that point in world coords.
    """

    ey: float
    epsi: float
    s: float
    segment_index: int
    segment_fraction: float
    track_heading: float
    closest_point: Tuple[float, float]


class PolylineTracker:
    """Stateful nearest-segment tracker along a fixed centerline polyline.

    Stateful because, after the first global search, subsequent ``track()``
    calls only search a small neighbourhood of the previous best segment
    (continuity + hysteresis, see module constants). Call
    :meth:`reset_tracking` whenever the tracked entity teleports.
    """

    def __init__(self, polyline: np.ndarray, max_advance_m: Optional[float] = None):
        points = np.asarray(polyline, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("Polyline must have shape (N, 2).")
        if points.shape[0] < 2:
            raise ValueError("Polyline must contain at least two points.")

        # Auto-close a circuit whose generator left the seam point un-duplicated.
        # The closed-loop convention here is points[-1] == points[0] (the F2 oval
        # repeats its first point, gap == 0). The complex_b offset right-lane
        # centreline instead ends ~one segment short of the start (gap 0.060 m vs
        # 0.052 m mean segment), so it would be mis-detected as *open* and the
        # stateful nearest-segment search cannot wrap at the start/finish line:
        # on lap 2+ it gets pinned near the final segment and ey explodes (this
        # produced the spurious 1.68 m mean |ey| in the new-cam CV eval). Append
        # the first point to close the loop when the endpoints sit within ~one
        # segment of each other — a genuinely open path has its ends far apart,
        # and the already-closed oval (gap == 0) is left untouched.
        if points.shape[0] >= 3:
            seam_gap = float(np.linalg.norm(points[0] - points[-1]))
            median_seg = float(
                np.median(np.linalg.norm(points[1:] - points[:-1], axis=1))
            )
            if 1e-6 < seam_gap < 1.75 * median_seg:
                points = np.vstack([points, points[0]])

        self.points = points
        self.segment_vectors = self.points[1:] - self.points[:-1]
        self.segment_lengths = np.linalg.norm(self.segment_vectors, axis=1)

        if np.any(self.segment_lengths < 1e-9):
            raise ValueError("Polyline contains a zero-length segment.")

        self.segment_headings = np.arctan2(
            self.segment_vectors[:, 1], self.segment_vectors[:, 0]
        )
        self.cumulative_lengths = np.concatenate(
            ([0.0], np.cumsum(self.segment_lengths))
        )

        # Closed polylines (oval, lap) wrap segment indices modulo N.
        self._closed = bool(
            np.allclose(self.points[0], self.points[-1], atol=1e-6)
        )
        self._prev_best_index: Optional[int] = None
        # Progress-bounded projection (opt-in). When set, the projection's
        # arc-length may advance at most this many metres per ``track`` call, so
        # the nearest-point search cannot leap onto a different, self-approaching
        # section of the circuit when the agent leaves its lane (see ``track``).
        # ``None`` => legacy unbounded behaviour (the convex F-track oval).
        self.max_advance_m: Optional[float] = (
            float(max_advance_m) if max_advance_m is not None else None
        )
        self._prev_s: Optional[float] = None

    @property
    def closed(self) -> bool:
        return self._closed

    def reset_tracking(self) -> None:
        """Drop the cached best_index so the next track() call does a
        global search. Call after warping the tracked entity (e.g. lap
        reset) so we do not constrain the search to a stale neighbourhood.
        """
        self._prev_best_index = None
        self._prev_s = None

    def distance_to(self, x: float, y: float) -> float:
        """Global minimum distance (m) from ``(x, y)`` to the polyline.

        Stateless full sweep with per-segment projection (clipped to the
        segment). Unlike the stateful :meth:`track` cross-track error, this is
        immune to a self-approaching circuit folding back near the query point
        — it answers "how far is the vehicle from the *nearest* road point",
        which is the right primitive for an off-road (left the painted road)
        containment test (verified: catches 28/28 straight-off departures to
        the grass on complex_b, 0 false positives in-lane).
        """
        position = np.array([x, y], dtype=float)
        starts = self.points[:-1]
        rel = position - starts
        ll = np.maximum(self.segment_lengths ** 2, 1e-12)
        t = np.clip((rel * self.segment_vectors).sum(axis=1) / ll, 0.0, 1.0)
        projections = starts + t[:, None] * self.segment_vectors
        return float(np.linalg.norm(position - projections, axis=1).min())

    def pose_at_arclength(
        self, s: float, lateral_offset: float = 0.0
    ) -> Tuple[float, float, float]:
        """World pose ``(x, y, heading)`` at centerline arc-length ``s`` (m).

        Used by the F4 scenario executor to spawn the vehicle at a scenario's
        ``track.start_s_m`` (e.g. 0.0 straight start vs 1.5 curve entry) instead
        of always at the first centerline point. ``s`` wraps modulo the perimeter
        for a closed polyline (the oval) and is clamped to ``[0, length]`` for an
        open one. ``lateral_offset`` (m) shifts the spawn along the left normal
        (+ = left of the travel direction) so a scenario can start the car
        off-centre; heading is the segment tangent (add a heading error on top).
        """
        total = float(self.cumulative_lengths[-1])
        if self._closed and total > 0.0:
            s = s % total
        else:
            s = float(np.clip(s, 0.0, total))
        idx = int(np.searchsorted(self.cumulative_lengths, s, side="right") - 1)
        idx = max(0, min(idx, len(self.segment_vectors) - 1))
        frac = (s - float(self.cumulative_lengths[idx])) / float(self.segment_lengths[idx])
        point = self.points[idx] + frac * self.segment_vectors[idx]
        heading = float(self.segment_headings[idx])
        if lateral_offset:
            normal = np.array([-math.sin(heading), math.cos(heading)], dtype=float)
            point = point + lateral_offset * normal
        return float(point[0]), float(point[1]), heading

    def track(self, x: float, y: float, yaw: float) -> TrackState:
        """Project pose ``(x, y, yaw)`` onto the polyline and return errors.

        Searches globally on the first call (or after :meth:`reset_tracking`),
        otherwise only the ±`_LOCAL_SEARCH_RADIUS` segments around the previous
        best index, with `_HYSTERESIS_M2` favouring the incumbent segment.
        """
        position = np.array([x, y], dtype=float)
        n_segments = len(self.segment_vectors)

        if self._prev_best_index is None:
            indices: Iterable[int] = range(n_segments)
        elif self._closed:
            base = self._prev_best_index
            indices = [
                (base + k) % n_segments
                for k in range(-_LOCAL_SEARCH_RADIUS, _LOCAL_SEARCH_RADIUS + 1)
            ]
        else:
            lo = max(0, self._prev_best_index - _LOCAL_SEARCH_RADIUS)
            hi = min(n_segments, self._prev_best_index + _LOCAL_SEARCH_RADIUS + 1)
            indices = range(lo, hi)

        # Progress-bounded forward cap (opt-in via ``max_advance_m``). On a
        # circuit that approaches itself (the complex_b kidney loop), the nearest
        # centerline point can leap to a *different* track section when the agent
        # leaves its lane, collapsing |ey| and defeating the off-road
        # termination (verified: even a global nearest-point search does this).
        # Limiting how far the projection's arc-length may advance per call to
        # the real along-track travel keeps the projection on the lane the agent
        # abandoned, so ey reflects the true departure and off-road fires.
        if self.max_advance_m is not None and self._prev_s is not None:
            s_limit = self._prev_s + self.max_advance_m
            bounded = [i for i in indices if self.cumulative_lengths[i] <= s_limit]
            if bounded:
                indices = bounded

        best_index = 0
        best_fraction = 0.0
        best_projection = self.points[0]
        best_error = np.zeros(2, dtype=float)
        best_distance_sq = float("inf")

        for index in indices:
            start = self.points[index]
            vector = self.segment_vectors[index]
            length = self.segment_lengths[index]

            relative = position - start
            projection_fraction = float(
                np.dot(relative, vector) / max(length * length, 1e-12)
            )
            projection_fraction = float(np.clip(projection_fraction, 0.0, 1.0))
            projection = start + projection_fraction * vector
            error = position - projection
            distance_sq = float(np.dot(error, error))

            # Apply hysteresis: a segment other than the previous best must
            # beat it by _HYSTERESIS_M2 to win. This prevents oscillation
            # between equidistant adjacent segments at polyline joints.
            if self._prev_best_index is not None and index != self._prev_best_index:
                distance_sq += _HYSTERESIS_M2

            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_index = index
                best_fraction = projection_fraction
                best_projection = projection
                best_error = error

        self._prev_best_index = best_index

        segment_vector = self.segment_vectors[best_index]
        segment_length = self.segment_lengths[best_index]
        track_heading = self._smoothed_heading(best_index)
        cross_z = segment_vector[0] * best_error[1] - segment_vector[1] * best_error[0]
        ey = float(cross_z / segment_length)
        epsi = wrap_angle(yaw - track_heading)
        s = float(self.cumulative_lengths[best_index] + best_fraction * segment_length)
        self._prev_s = s

        return TrackState(
            ey=ey,
            epsi=epsi,
            s=s,
            segment_index=best_index,
            segment_fraction=best_fraction,
            track_heading=track_heading,
            closest_point=(float(best_projection[0]), float(best_projection[1])),
        )

    def curvature_ahead(self, segment_index: int, lookahead_segments: int = 5) -> float:
        """Mean signed curvature (1/m) over the next ``lookahead_segments``.

        Computed as heading change divided by arc length from ``segment_index``
        forward; + = left turn. Returns 0.0 when the window degenerates.
        """
        n = len(self.segment_headings)
        if n < 2 or lookahead_segments <= 0:
            return 0.0

        start = int(segment_index)
        if self._closed:
            start %= n
            steps = min(int(lookahead_segments), n - 1)
            end = (start + steps) % n
            arc = float(
                sum(self.segment_lengths[(start + k) % n] for k in range(steps))
            )
        else:
            start = max(0, min(n - 1, start))
            end = min(n - 1, start + int(lookahead_segments))
            arc = float(self.cumulative_lengths[end] - self.cumulative_lengths[start])

        if arc <= 1e-6 or end == start:
            return 0.0

        dpsi = wrap_angle(
            float(self.segment_headings[end]) - float(self.segment_headings[start])
        )
        return dpsi / arc

    def arclength_at_curvature(
        self, target_kappa: float, lookahead_segments: int = 5
    ) -> float:
        """Arc-length ``s`` of the track location whose local curvature magnitude
        is closest to ``|target_kappa|`` (1/m).

        Used by SC-EDGE-05's ``kappa_seed_rad_m`` grid anchors so the vehicle
        spawns on a curve of the requested curvature, exercising C-04's
        curvature-parameterised speed ceiling for real geometry rather than a
        synthetic seed. Scans every segment's :meth:`curvature_ahead` and returns
        the cumulative arc-length at the best match (0.0 on a degenerate track)."""
        n = len(self.segment_headings)
        if n < 2:
            return 0.0
        target = abs(float(target_kappa))
        best_index, best_err = 0, float("inf")
        for i in range(n):
            err = abs(abs(self.curvature_ahead(i, lookahead_segments)) - target)
            if err < best_err:
                best_err, best_index = err, i
        return float(self.cumulative_lengths[best_index])

    def _smoothed_heading(self, segment_index: int) -> float:
        """Circular mean of segment headings within `_HEADING_SMOOTHING_RADIUS`."""
        n = len(self.segment_headings)
        if self._closed:
            indices = [
                (segment_index + k) % n
                for k in range(-_HEADING_SMOOTHING_RADIUS, _HEADING_SMOOTHING_RADIUS + 1)
            ]
            headings = self.segment_headings[indices]
        else:
            lo = max(0, segment_index - _HEADING_SMOOTHING_RADIUS)
            hi = min(n, segment_index + _HEADING_SMOOTHING_RADIUS + 1)
            headings = self.segment_headings[lo:hi]
        cs = float(np.cos(headings).sum())
        sn = float(np.sin(headings).sum())
        return math.atan2(sn, cs)
