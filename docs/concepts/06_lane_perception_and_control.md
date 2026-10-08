# 06 · Lane perception and control

> **Summary:** a calibrated camera model converts white lane markings into
> metric lane geometry, and a pure-pursuit law converts that geometry into a
> yaw rate. Two classical controllers implement this: a histogram tracker on
> the robot and a ground-plane CV estimator with pure pursuit in Gazebo.

[← 05 Localisation and navigation](05_localization_and_navigation.md) · [Concepts](README.md) · Next: [07 Reinforcement learning →](07_reinforcement_learning.md)

---

## Theory

### Ground-plane projection

A pinhole camera maps a 3-D point to the pixel $(u, v)$ through the focal
lengths $f_x, f_y$ and the principal point $(c_x, c_y)$. For a camera at height
$h$, facing forward and pitched down by $\theta$ (no roll, no yaw), every image
row below the horizon corresponds to one ground distance. With
$a = (v - c_y)/f_y$ and $b = (u - c_x)/f_x$, a pixel projects onto the ground at

$$
X(v) = h\,\frac{\cos\theta - a\sin\theta}{\sin\theta + a\cos\theta},
\qquad
Y(u, v) = -\,\frac{h\, b}{\sin\theta + a\cos\theta}
$$

($X$ forward, $Y$ left, origin below the camera). $X$ depends only on the row,
and within a row $Y$ is linear in the column. This property allows a metric
lane estimate from a single camera.

### Classical lane detection

Standard processing chain:

1. **Region of interest**: the lower part of the image, which contains the
   road.
2. **Filtering**: blur for noise suppression; morphology to close gaps.
3. **Segmentation**: brightness threshold (or low saturation and high value in
   HSV) producing a binary mask of the white paint. Edge detectors (gradients,
   Canny) are an alternative.
4. **Line detection**: column histograms of horizontal bands, or line fits to
   candidate points.
5. **Lane state**: lateral offset $e_y$, heading error $e_\psi$, lane width,
   curvature.

### Pure pursuit

A target point is selected on the lane centre at a look-ahead distance $L$,
with lateral offset $y_L$. The circular arc tangent to the vehicle heading that
passes through this point has curvature

$$
\kappa = \frac{2\,y_L}{L^2 + y_L^2},
\qquad
\omega = k \cdot v \cdot \kappa
$$

A short $L$ responds quickly but oscillates; a long $L$ is smooth but cuts
corners.

---

## Implementation

### Lane camera

| | Robot | Gazebo |
| --- | --- | --- |
| Sensor | IMX219-160 on CSI | `Lane Cam` sensor in the URDF |
| Pipeline | `nvarguscamerasrc` 1280×720 at 60 fps, area-resized to 640×360 `bgr8` | 640×360 `R8G8B8`, Gaussian noise σ = 0.007 |
| Field of view | 90° horizontal | 1.5708 rad horizontal |
| Topic, rate | `camera/image_raw_lane`, 20 Hz | `camera/image_raw_lane`, 20 Hz, through the bridge |
| Frame | `camera_link_optical_lane` | `camera_link_optical_lane` |

On the robot the camera is published by
[`csi_camera_node.py`](../../src/cobraflex_rl/cobraflex_rl/csi_camera_node.py)
(started by layer 2), except when the classical lane keeper opens it directly
([02](02_system_architecture.md)). The ground projection above is implemented
in [`camera_geometry.py`](../../src/cobraflex_rl/cobraflex_rl/camera_geometry.py)
with the intrinsics of the `Lane Cam` sensor and the mount height and pitch of
the URDF.

### Controller 1: histogram tracker on the robot

[`lane_keeper_node.py`](../../src/cobraflex/cobraflex/lane_keeper_node.py),
started by [`cobraflex_lane_keeper.launch.py`](../../src/cobraflex/launch/cobraflex_lane_keeper.launch.py),
operates in image space at 20 Hz:

| Step | Default |
| --- | --- |
| Resize | 640×360 |
| Region of interest | From 58 % of the image height downwards (`roi_start_pct`) |
| Blur, threshold, morphology | Kernel 5, threshold 145, kernel 5, inverted mask |
| Trapezoid mask | Removes the image corners outside the road |
| Bands | Three horizontal bands at 82 %, 68 % and 54 % of the ROI, weights 0.50, 0.30, 0.20 |
| Per band | Column histogram → peaks → lane centre, using the last lane width (initially 150 px) |
| Output | Weighted lane centre → pixel error → `angular.z`, limited to 0.8 rad/s |

The node publishes overlay, mask and histogram images and RViz markers for
inspection of every stage.

### Controller 2: CV estimator and pure pursuit in Gazebo

[`lane_keeper_gazebo_node.py`](../../src/cobraflex/cobraflex/lane_keeper_gazebo_node.py)
uses the shared
[`CVLaneController`](../../src/cobraflex_rl/cobraflex_rl/cv_lane_controller.py),
which reads the deterministic estimator
[`cv_lane_estimator.py`](../../src/cobraflex_rl/cobraflex_rl/cv_lane_estimator.py):

1. **White mask**: HSV threshold, low saturation and high value.
2. **Row scan**: rows between a near and a far look-ahead distance; white runs
   whose metric width matches a lane marking become candidate points,
   projected onto the ground.
3. **Line clustering**: candidates grouped by lateral intercept, one
   least-squares line per group.
4. **Lane selection**: the pair of lines with a plausible separation whose
   centre is nearest to the vehicle.
5. **State**: $e_y$ (positive when the vehicle is left of the centre),
   $e_\psi$ (positive when yawed left), lane width, curvature (positive for a
   left bend), confidence and feature count.

The controller applies pure pursuit on the lane-centre polynomial:

| Parameter | Value |
| --- | --- |
| `linear_speed` | 0.20 m/s |
| `look_ahead_m` ($L$) | 0.40 m |
| `pursuit_gain` ($k$) | 1.0 |
| `max_angular_z` | 0.9 rad/s |
| `stop_on_no_lane` | `true` |
| `watchdog_timeout_sec` | 1.5 s |

On the nominal oval the controller tracks the lane with an RMSE of about 10 mm
at 0.2 m/s (requirement: below 50 mm). It replaced a PD law with curvature
feed-forward that under-steered in tight curves: monocular curvature over a
short arc is too noisy for feed-forward, and pure pursuit requires no curvature
estimate. The node still declares the former parameters `kp_ey`, `kd_epsi` and
`kff_curv` for compatibility; they have no effect.

The same estimator provides the lane state to the safety cage
([08](08_safety_cage.md)) and drives the scored evaluation
([`eval_cv_controller.py`](../../src/cobraflex_rl/cobraflex_rl/eval_cv_controller.py)),
so deployment and evaluation execute identical code. The ODD for which the
estimator is tuned has a lane width of 0.245 m.

### Perception test worlds

[`worlds/README`](../../src/cobraflex/worlds/README.md) lists the variants of
the oval track: worn markings at three levels, line gaps, mirrored layouts and
particles. The geometry is unchanged, so differences in behaviour originate in
perception.

---

## Common errors

- **Concurrent camera access.** The classical lane keeper opens the CSI device
  directly; layer 2 must then be started with `use_lane_camera:=false`,
  otherwise the device cannot be opened.
- **Sign conventions.** $e_y$, $e_\psi$, curvature and `angular.z` are all
  positive to the left. A sign error makes the controller steer away from the
  lane.
- **Image messages.** `_build_image_msg` builds the payload with
  `array.array("B", …)`. Bytes or numpy arrays pass through an rclpy check that
  iterates over every element in Python: 127 ms instead of 0.12 ms per
  640×360 frame.
- **Lighting.** The fixed threshold (145) is valid for the lighting in which it
  was tuned; sunlight and glare change the required value.

---

## Commands

```bash
# Gazebo: CV estimator + pure pursuit on the oval (see Usage §3)
ros2 launch cobraflex lane_keeper_gazebo.launch.py

# Robot: classical histogram tracker (camera opened by the node)
ros2 launch cobraflex cobraflex_sensors.launch.xml use_lane_camera:=false
ros2 launch cobraflex cobraflex_lane_keeper.launch.py
```

Exercises: compare `look_ahead_m` values of 0.25, 0.40 and 0.60 m on the oval;
run the worn-paint and gap worlds and identify where the estimator loses the
lane. The controller reads its parameters once at start-up, so
`ros2 param set` has no effect on a running node. Start
`lane_keeper_gazebo_node` with `--ros-args -p look_ahead_m:=0.25`, or add the
parameter to the node in the launch file.

---

## Lecture references

- **Computer Vision L02**: image filtering; blur and morphology steps.
- **Computer Vision L03**: contrast, edges and gradients; thresholding and
  edge-based alternatives to the white mask.
- **Motion Planning**: path tracking; pure pursuit as a geometric tracking
  controller, compared with the optimisation-based (MPC) approaches of part 2.

## Further reading

- R. C. Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01, 1992.
- R. Szeliski, *Computer Vision: Algorithms and Applications*, 2nd ed., 2022, chapters on image processing and camera geometry.
