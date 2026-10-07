# 06 · Lane perception and control

> **In one sentence:** a calibrated camera model turns white lane markings into
> metric lane geometry, and a pure-pursuit law turns that geometry into a yaw
> rate — two classical controllers do this, a histogram tracker on the car and a
> ground-plane CV estimator with pure pursuit in Gazebo.

[← 05 Localisation and navigation](05_localization_and_navigation.md) · [Concepts](README.md) · Next: [07 Reinforcement learning →](07_reinforcement_learning.md)

---

## Concept

### From pixels to the ground

A pinhole camera maps a 3-D point to pixel $(u, v)$ through the focal lengths
$f_x, f_y$ and the principal point $(c_x, c_y)$. If the camera sits at height
$h$, looks forward and is only pitched down by $\theta$ (no roll, no yaw), every
image row below the horizon corresponds to one distance on the ground. With
$a = (v - c_y)/f_y$ and $b = (u - c_x)/f_x$, a pixel lands on the ground at

$$
X(v) = h\,\frac{\cos\theta - a\sin\theta}{\sin\theta + a\cos\theta},
\qquad
Y(u, v) = -\,\frac{h\, b}{\sin\theta + a\cos\theta}
$$

($X$ forward, $Y$ left, origin below the camera). $X$ depends only on the row,
and within a row $Y$ is linear in the column — which is what makes a lane
estimate in metres possible from a single camera.

### Classical lane detection

The usual chain, all of it inspectable:

1. **Region of interest** — keep the lower part of the image, where the road is.
2. **Filtering** — blur to suppress noise; morphology to close gaps.
3. **Segmentation** — threshold brightness (or low saturation and high value in
   HSV) to get a binary mask of the white paint. Edge detectors (gradients,
   Canny) are the alternative.
4. **Line finding** — column histograms of horizontal bands, or line fits to
   candidate points.
5. **Lane state** — lateral offset $e_y$, heading error $e_\psi$, lane width,
   curvature.

### Pure pursuit

Pick the point on the lane centre a look-ahead distance $L$ ahead, at lateral
offset $y_L$. The circular arc that starts tangent to the vehicle's heading and
passes through that point has curvature

$$
\kappa = \frac{2\,y_L}{L^2 + y_L^2},
\qquad
\omega = k \cdot v \cdot \kappa
$$

A short $L$ reacts fast but oscillates; a long $L$ is smooth but cuts corners.

---

## In this repository

### The lane camera

| | Car | Gazebo |
| --- | --- | --- |
| Sensor | IMX219-160 on CSI | `Lane Cam` sensor in the URDF |
| Pipeline | `nvarguscamerasrc` 1280×720 at 60 fps, area-resized to 640×360 `bgr8` | 640×360 `R8G8B8`, Gaussian noise σ = 0.007 |
| Field of view | 90° horizontal | 1.5708 rad horizontal |
| Topic, rate | `camera/image_raw_lane`, 20 Hz | `camera/image_raw_lane`, 20 Hz, via the bridge |
| Frame | `camera_link_optical_lane` | `camera_link_optical_lane` |

The car's camera is published by
[`csi_camera_node.py`](../../src/cobraflex_rl/cobraflex_rl/csi_camera_node.py)
(started by layer 2) — unless the classical lane keeper opens it itself
([02](02_system_architecture.md)). The ground projection above is implemented
in [`camera_geometry.py`](../../src/cobraflex_rl/cobraflex_rl/camera_geometry.py),
with the intrinsics of the `Lane Cam` sensor and the mount height and pitch from
the URDF.

### Controller 1 — the histogram tracker on the car

[`lane_keeper_node.py`](../../src/cobraflex/cobraflex/lane_keeper_node.py),
launched by [`cobraflex_lane_keeper.launch.py`](../../src/cobraflex/launch/cobraflex_lane_keeper.launch.py),
works in image space at 20 Hz:

| Step | Default |
| --- | --- |
| Resize | 640×360 |
| Region of interest | from 58 % of the image height down (`roi_start_pct`) |
| Blur, threshold, morphology | kernel 5, threshold 145, kernel 5, inverted mask |
| Trapezoid mask | removes the image corners outside the road |
| Bands | three horizontal bands at 82 %, 68 % and 54 % of the ROI, weights 0.50, 0.30, 0.20 |
| Per band | column histogram → peaks → lane centre, using the last lane width (initially 150 px) |
| Output | weighted lane centre → pixel error → `angular.z`, capped at 0.8 rad/s |

It publishes overlay, mask and histogram images plus RViz markers, so every
stage can be watched.

### Controller 2 — CV estimator and pure pursuit in Gazebo

[`lane_keeper_gazebo_node.py`](../../src/cobraflex/cobraflex/lane_keeper_gazebo_node.py)
uses the shared
[`CVLaneController`](../../src/cobraflex_rl/cobraflex_rl/cv_lane_controller.py),
which reads the deterministic estimator
[`cv_lane_estimator.py`](../../src/cobraflex_rl/cobraflex_rl/cv_lane_estimator.py):

1. **White mask** — HSV threshold: low saturation, high value.
2. **Row scan** — rows between a near and a far look-ahead distance; white runs
   whose metric width fits a lane marking become candidate points, projected to
   the ground.
3. **Line clustering** — candidates grouped by lateral intercept, a
   least-squares line per group.
4. **Lane selection** — the pair of lines with a plausible separation whose
   centre is nearest the vehicle.
5. **State** — $e_y$ (+ = vehicle left of centre), $e_\psi$ (+ = yawed left),
   lane width, curvature (+ = left bend), confidence and feature count.

The controller then applies pure pursuit on the lane-centre polynomial:

| Parameter | Value |
| --- | --- |
| `linear_speed` | 0.20 m/s |
| `look_ahead_m` ($L$) | 0.40 m |
| `pursuit_gain` ($k$) | 1.0 |
| `max_angular_z` | 0.9 rad/s |
| `stop_on_no_lane` | `true` |
| `watchdog_timeout_sec` | 1.5 s |

On the nominal oval it tracks the lane to an RMSE of about 10 mm at 0.2 m/s
(requirement: below 50 mm). It replaced an earlier PD + curvature
feed-forward law that under-steered tight curves, because monocular curvature
over a short arc is too noisy to feed forward; pure pursuit needs no curvature
estimate. The node still declares the old `kp_ey`, `kd_epsi` and `kff_curv`
parameters for compatibility, but they no longer change anything.

The same estimator gives the safety cage its lane state ([08](08_safety_cage.md))
and drives the scored evaluation
([`eval_cv_controller.py`](../../src/cobraflex_rl/cobraflex_rl/eval_cv_controller.py)),
so deployment and evaluation run identical code. The ODD the estimator is tuned
for has a 0.245 m lane.

### Worlds that stress the perception

[`worlds/README`](../../src/cobraflex/worlds/README.md) lists the variants of
the oval track: paint worn by 25, 50 and 75 %, line gaps, mirrored layouts and
particles. The geometry stays the same, so any change in behaviour comes from
perception.

---

## Pitfalls

- **Two cameras, one sensor.** The classical lane keeper opens the CSI device
  itself; start layer 2 with `use_lane_camera:=false` or it fails to open.
- **Sign conventions.** $e_y$, $e_\psi$, curvature and `angular.z` are all
  positive to the left. A sign flip turns a lane keeper into a lane leaver.
- **Image messages.** `_build_image_msg` builds the payload with
  `array.array("B", …)`. Bytes or numpy fall through an rclpy check that walks
  every element in Python: 127 ms instead of 0.12 ms per 640×360 frame.
- **Lighting.** A fixed threshold (145) assumes the lighting it was tuned in;
  sunlight and glare change it.

---

## Try it

```bash
# Gazebo: CV estimator + pure pursuit on the oval (see Usage §3)
ros2 launch cobraflex lane_keeper_gazebo.launch.py

# Car: classical histogram tracker (camera opened by the node)
ros2 launch cobraflex cobraflex_sensors.launch.xml use_lane_camera:=false
ros2 launch cobraflex cobraflex_lane_keeper.launch.py
```

Exercises: compare `look_ahead_m` 0.25, 0.40 and 0.60 m on the oval; run the
worn-paint and gap worlds and find where the estimator loses the lane. The
controller reads its parameters once at start-up, so `ros2 param set` does not
change a running node: start `lane_keeper_gazebo_node` with
`--ros-args -p look_ahead_m:=0.25`, or add the parameter to the node in the
launch file.

---

## Lecture links

- **Computer Vision L02** — image filtering: the blur and morphology steps.
- **Computer Vision L03** — contrast, edges and gradients: thresholding and
  edge-based alternatives to the white mask.
- **Motion Planning** — path tracking: pure pursuit as a geometric tracking
  controller, next to the optimisation-based (MPC) approaches of part 2.

## Further reading

- R. C. Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01, 1992.
- R. Szeliski, *Computer Vision: Algorithms and Applications*, 2nd ed., 2022 — chapters on image processing and camera geometry.
