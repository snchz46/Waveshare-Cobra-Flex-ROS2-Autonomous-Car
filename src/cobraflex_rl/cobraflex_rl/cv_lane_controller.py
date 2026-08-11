"""
cv_lane_controller — the logical (non-learned) camera lane-keeper.

Shared by the deployment node (cobraflex/lane_keeper_gazebo_node.py) and the
scored evaluation (cobraflex_rl/eval_cv_controller.py) so both drive with the
*identical* control law — the single source of truth for the RL baseline.

It is the fair, classical counterpart to the RL camera agent: it reads the same
deterministic CV lane estimator the safety cage uses (`CvLaneEstimator`, D-43),
which projects the white markings to the ground plane with the calibrated
camera model and returns the lane-centre polynomial. The controller closes a
**pure-pursuit** loop on it: aim at the lane centre a look-ahead distance ahead
and command the pursuit yaw rate (see :meth:`CVLaneController.compute`).

Sign conventions (shared with PolylineTracker): ey + = left of lane centre,
epsi + = yawed left, κ + = left bend; Twist.angular.z + = turn left.

This supersedes the earlier PD + curvature-feedforward law, which under-steered
tight curves: its feedforward needed an explicit curvature estimate, and
monocular curvature on a short arc is irrecoverably noisy. Pure pursuit needs
none — it reads the lane-centre offset *within* the observed band (stable) and
turns exactly as much as the curve there demands. On the nominal oval at
0.2 m/s it tracks to RMSE ~10 mm (req < 50 mm), on par with the RL agent — a
like-for-like reference.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import numpy as np

from .cv_lane_estimator import CvLaneEstimator

# Tunable gains; recorded in each run's metadata so the law that drove it is
# exactly reproducible.
CONTROLLER_DEFAULTS: Dict[str, float] = {
    # Pure-pursuit look-ahead (the whole control law, see compute()).
    "look_ahead_m": 0.40,   # aim-point distance ahead [m]; inside the camera's
                            # ~0.7 m curve band so the lane centre there is an
                            # interpolation (robust), never an extrapolation, and
                            # short enough to hold the turn through the apex.
    "pursuit_gain": 1.0,    # scales the pure-pursuit yaw-rate command.
    "max_angular_z": 0.90,
}


class CVLaneController:
    """Pure-pursuit CV lane-keeper. ``compute(frame_bgr) -> (angular_z, detected)``.

    The control law aims at the lane-centre point a fixed *look-ahead* distance
    ``L`` ahead and commands the pure-pursuit yaw rate

        angular_z = pursuit_gain · v · 2·y_L / (L² + y_L²)

    where ``y_L`` (+left) is the lane-centre lateral offset at ``X = L``, read
    from the estimator's lane-centre polynomial. This **supersedes the earlier
    PD + curvature-feedforward law**, which could not hold tight curves: that law
    needed an explicit curvature estimate for its feedforward, and monocular
    curvature on a short arc is irrecoverably noisy (it swung sign frame-to-
    frame, so the car under-steered and ran wide — see the dumped
    ``cv_ctrl_eval_20260618T175028Z`` frames). Pure pursuit needs **no curvature
    estimate**: ``y_L`` is read *within* the observed band, where the lane fit is
    stable even when its bare coefficients are not, and on a bend ``y_L`` grows
    naturally and commands exactly the turn the curve requires. It regulates
    lateral offset and heading through the same single aim point.

    ``angular_z`` is clipped to [-max_angular_z, max_angular_z]; the env / node
    publishes it as Twist.angular.z. No usable lane this frame → (0.0, False).
    Sign conventions: y_L + = lane centre to the left ⇒ angular_z + = turn left.

    Legacy gain kwargs (``kp_ey``/``kd_epsi``/``kff_curv``) are still accepted
    (the deployment node passes them) but ignored.
    """

    def __init__(self, speed: float = 0.2, estimator: Optional[CvLaneEstimator] = None,
                 **params: Any) -> None:
        self.p = {**CONTROLLER_DEFAULTS, **params}
        self.speed = float(speed)
        self.estimator = estimator or CvLaneEstimator()
        self.dbg: Dict[str, Any] = {}

    def compute(self, frame_bgr: Optional[np.ndarray]) -> Tuple[float, bool]:
        if frame_bgr is None or getattr(frame_bgr, "size", 0) == 0:
            self.dbg = {"ok": False, "reason": "no_frame"}
            return 0.0, False
        est = self.estimator.estimate(frame_bgr)
        if not est.ok:
            self.dbg = {"ok": False, "reason": est.reason}
            return 0.0, False
        p = self.p
        L = float(p["look_ahead_m"])
        c0, c1, c2 = est.center_coeffs
        y_l = float(c0) + float(c1) * L + float(c2) * L * L  # lane centre at look-ahead
        ld2 = L * L + y_l * y_l
        kappa_cmd = 2.0 * y_l / ld2 if ld2 > 1e-9 else 0.0
        angular = float(p["pursuit_gain"]) * self.speed * kappa_cmd
        m = float(p["max_angular_z"])
        self.dbg = {"ok": True, "ey": round(est.ey, 4), "epsi": round(est.epsi, 4),
                    "y_l": round(y_l, 4), "kappa_cmd": round(kappa_cmd, 3),
                    "nL": est.n_lines}
        return float(np.clip(angular, -m, m)), True
