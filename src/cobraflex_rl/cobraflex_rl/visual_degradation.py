"""
visual_degradation — pure-numpy front-camera image degradations for track 'E'.

Track 'E' (end-to-end front-camera, decisions D-41/D-43) stresses the camera
with visual degradations (hazard H-10, verified by SR-012 over the scenarios
SC-PERT-04..06; plus the H-11/H-12 stressors of SC-PERT-07/08). Per D-43 the
degraded frame reaches **both consumers** — the policy's CNN and the cage's CV
lane-estimator — the accepted common-cause trade-off: a camera fault can blind
both at once, and the designed answer is the open-loop controlled stop
(SR-013/SR-014 → C-05 Trigger 8).

Each primitive takes a uint8 image (grayscale ``(H, W)`` or ``(H, W, C)``) and a
``level`` in ``[0, 1]`` (0 = identity, 1 = strongest) and returns a uint8 image of
the same shape. The functions are **deterministic** (no RNG): the training-time
domain randomisation samples the mode and level; these primitives are the
deterministic kernels it draws from.

Mode strings match ``perturbations.mode`` in
``scenarios/perturbed/sc_pert_04..08.yaml``:

* ``"glare_overexposure"``      — SC-PERT-04 (H-10)
* ``"low_light_underexposure"`` — SC-PERT-05 (H-10)
* ``"motion_blur"``             — SC-PERT-06 (H-10)
* ``"occlusion"``               — SC-PERT-07 (H-11; perception loss)
* ``"false_lane"``              — SC-PERT-08 (H-12; misleading marking)

``MODES`` is the H-10 trio the training-side domain randomisation draws from
(`visual_domain_randomization`, SR-012 mitigation); occlusion and false-lane
are **eval stressors** — training on them would teach the policy to ignore
exactly the cues whose loss must trigger the SR-013/SR-014 stop — and are
exposed separately via ``EVAL_ONLY_MODES`` / ``ALL_MODES``.

Pure (numpy only, no ROS), host-testable on any platform. The Gazebo camera sensor
and the runtime injector that calls these primitives live on the Ubuntu+Jazzy host.
"""
from __future__ import annotations

import numpy as np

GLARE = "glare_overexposure"
LOW_LIGHT = "low_light_underexposure"
MOTION_BLUR = "motion_blur"
OCCLUSION = "occlusion"
FALSE_LANE = "false_lane"
MODES = (GLARE, LOW_LIGHT, MOTION_BLUR)
EVAL_ONLY_MODES = (OCCLUSION, FALSE_LANE)
ALL_MODES = MODES + EVAL_ONLY_MODES

_MAX = 255.0


def _check(img: np.ndarray, level: float) -> None:
    if not isinstance(img, np.ndarray):
        raise TypeError("img must be a numpy ndarray")
    if img.dtype != np.uint8:
        raise ValueError(f"img must be uint8, got {img.dtype}")
    if img.ndim not in (2, 3):
        raise ValueError(f"img must be (H,W) or (H,W,C), got shape {img.shape}")
    if not 0.0 <= float(level) <= 1.0:
        raise ValueError(f"level must be in [0, 1], got {level}")


def apply_glare(img: np.ndarray, level: float, *,
                max_gain: float = 2.0, max_bias: float = 160.0) -> np.ndarray:
    """Over-exposure / glare: multiplicative gain plus an additive wash toward white.

    ``level=0`` is the identity; ``level=1`` applies ``gain=max_gain`` and
    ``bias=max_bias`` (strong wash-out). Models sun glare / specular highlights
    saturating the sensor.
    """
    _check(img, level)
    gain = 1.0 + level * (max_gain - 1.0)
    bias = level * max_bias
    out = img.astype(np.float32) * gain + bias
    return np.clip(out, 0.0, _MAX).astype(np.uint8)


def apply_low_light(img: np.ndarray, level: float, *,
                    min_gain: float = 0.15, contrast_drop: float = 0.5) -> np.ndarray:
    """Under-exposure / low light: brightness gain down + contrast compression
    toward the per-image mean.

    ``level=0`` is the identity; ``level=1`` darkens to ``min_gain`` of the original
    brightness and compresses contrast by ``contrast_drop``. Models dusk / deep
    shadow reducing lane contrast.
    """
    _check(img, level)
    gain = 1.0 - level * (1.0 - min_gain)
    contrast = 1.0 - level * contrast_drop
    f = img.astype(np.float32)
    mean = float(f.mean())
    out = ((f - mean) * contrast + mean) * gain
    return np.clip(out, 0.0, _MAX).astype(np.uint8)


def apply_motion_blur(img: np.ndarray, level: float, *,
                      max_kernel: int = 15, axis: int = 1) -> np.ndarray:
    """Directional motion blur: a box average over a window that grows with ``level``.

    ``level=0`` is the identity (window 1); ``level=1`` uses ``max_kernel``. ``axis=1``
    blurs along columns (horizontal motion, the dominant case for a forward camera on
    a turning vehicle). Models motion blur / rolling-shutter smear at speed.
    """
    _check(img, level)
    k = 1 + int(round(level * (max_kernel - 1)))
    if k <= 1:
        return img.copy()
    return _box_blur_1d(img, k, axis)


def _box_blur_1d(img: np.ndarray, k: int, axis: int) -> np.ndarray:
    """Moving average of window ``k`` along ``axis`` (edge-padded), pure numpy via a
    cumulative-sum sliding window."""
    f = img.astype(np.float32)
    left = k // 2
    right = k - 1 - left
    pad = [(0, 0)] * f.ndim
    pad[axis] = (left, right)
    fp = np.pad(f, pad, mode="edge")
    csum = np.cumsum(fp, axis=axis)
    zero_shape = list(csum.shape)
    zero_shape[axis] = 1
    csum = np.concatenate([np.zeros(zero_shape, dtype=csum.dtype), csum], axis=axis)
    n = f.shape[axis]
    hi = [slice(None)] * f.ndim
    lo = [slice(None)] * f.ndim
    hi[axis] = slice(k, k + n)
    lo[axis] = slice(0, n)
    out = (csum[tuple(hi)] - csum[tuple(lo)]) / float(k)
    return np.clip(out, 0.0, _MAX).astype(np.uint8)


def apply_occlusion(img: np.ndarray, level: float, *,
                    fill_value: int = 25, max_height_frac: float = 0.75) -> np.ndarray:
    """Occlusion / perception loss (SC-PERT-07, H-11): an opaque dark patch
    grows from the bottom of the frame — where the near-field lane features
    live — covering ``level * max_height_frac`` of the image height across the
    full width. Models debris on the lens, a tarp, or a deep cast shadow
    swallowing the markings. ``level=0`` is the identity.
    """
    _check(img, level)
    out = img.copy()
    rows = int(round(img.shape[0] * level * max_height_frac))
    if rows > 0:
        out[img.shape[0] - rows:, ...] = fill_value
    return out


def apply_false_lane(img: np.ndarray, level: float, *,
                     brightness: int = 235, width_frac: float = 0.02,
                     base_col_frac: float = 0.55, top_col_frac: float = 0.95) -> np.ndarray:
    """False lane marking (SC-PERT-08, H-12): paint a bright line that starts
    near the bottom centre (right of the true centre) and slants toward the
    image edge — a plausible-but-wrong feature (fork, old paint, tar seam) a
    lane detector can lock onto. ``level`` blends the line's opacity from
    invisible (0) to fully painted (1). Geometry is fixed and deterministic so
    a given level is exactly reproducible.
    """
    _check(img, level)
    if level <= 0.0:
        return img.copy()
    h, w = img.shape[0], img.shape[1]
    out = img.astype(np.float32)
    half = max(1, int(round(w * width_frac / 2.0)))
    # Line runs from (row h-1, col base) up to (row h//2, col top): only the
    # lower half of the frame, where ground features are metrically plausible.
    rows = np.arange(h // 2, h)
    frac = (rows - h // 2) / max(1, (h - 1) - h // 2)  # 0 at mid, 1 at bottom
    cols = (top_col_frac + (base_col_frac - top_col_frac) * frac) * (w - 1)
    for r, c in zip(rows, cols):
        lo = max(0, int(round(c)) - half)
        hi = min(w, int(round(c)) + half + 1)
        if lo < hi:
            out[r, lo:hi, ...] = (
                (1.0 - level) * out[r, lo:hi, ...] + level * float(brightness)
            )
    return np.clip(out, 0.0, _MAX).astype(np.uint8)


def degrade(img: np.ndarray, mode: str, level: float) -> np.ndarray:
    """Dispatch to the degradation primitive for ``mode`` (one of :data:`ALL_MODES`)."""
    if mode == GLARE:
        return apply_glare(img, level)
    if mode == LOW_LIGHT:
        return apply_low_light(img, level)
    if mode == MOTION_BLUR:
        return apply_motion_blur(img, level)
    if mode == OCCLUSION:
        return apply_occlusion(img, level)
    if mode == FALSE_LANE:
        return apply_false_lane(img, level)
    raise ValueError(f"unknown degradation mode {mode!r}; expected one of {ALL_MODES}")
