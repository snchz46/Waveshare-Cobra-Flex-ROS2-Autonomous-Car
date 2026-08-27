# Cobra Flex 4WD Mathematical Model

## General Description

This is the mathematical model of the Waveshare Cobra Flex: a mobile platform
with **4 driven wheels in a skid-steer configuration**, treated throughout the
stack as a differential drive. The model covers kinematics, control, and the
physical parameters of the system.

> **Credit.** The structure of this documentation set, and the idea for the
> project as a whole, come from
> [MrDavidAlv/Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot).
> See [Credits](#credits) at the end of this file.

---

## Contents

1. **[Kinematics](./Kinematics.md)**
   - Differential-drive kinematic model, and where skid-steer departs from it
   - Forward and inverse kinematics
   - Odometry and TF ownership
   - Velocity, acceleration and wheel-speed limits

2. **[Control](./Control.md)**
   - Gazebo `DiffDrive` plugin
   - The `/cmd_vel` chain, in simulation and on hardware
   - Deadman timers

3. **[Parameters](./parameters.md)**
   - Geometry, and the unresolved firmware-constant disagreement (§1.4)
   - Mass budget and inertia tensors
   - Sensors (LiDAR, ZED Mini, lane camera)

---

## Mathematical Notation

### Coordinate Systems

| Symbol | Description | ROS 2 frame |
|--------|-------------|-------------|
| $\lbrace W \rbrace$ | World coordinate system | `map` / `odom` |
| $\lbrace R \rbrace$ | Robot coordinate system | `base_footprint` |

### State Variables

| Variable | Description | Unit |
|----------|-------------|------|
| $q = [x, y, \theta]^T$ | Robot pose in $\lbrace W \rbrace$ | m, m, rad |
| $\dot{q} = [v, \omega]^T$ | Robot velocity | m/s, rad/s |
| $v_L, v_R$ | Left/right side linear speed | m/s |
| $\omega_L, \omega_R$ | Left/right wheel angular speed | rad/s |
| $r$ | Wheel radius | m |
| $W$ | Wheel separation (track) | m |
| $L$ | Wheelbase (front-to-rear) | m |

---

## Fundamental Equations

### Forward Kinematics

$$
v = \frac{v_R + v_L}{2} = \frac{r(\omega_R + \omega_L)}{2}
$$

$$
\omega = \frac{v_R - v_L}{W} = \frac{r(\omega_R - \omega_L)}{W}
$$

### Inverse Kinematics

$$
\omega_L = \frac{v - \omega \cdot W/2}{r}
$$

$$
\omega_R = \frac{v + \omega \cdot W/2}{r}
$$

Where $r = 0.03725$ m and $W = 0.154$ m.

These are the **ideal differential-drive** equations. This robot is a
skid-steer: it has two axles 0.120 m apart and can only turn by dragging all
four wheels sideways, so the yaw channel carries a gain error that the model
above does not represent. See [Kinematics §3.3](./Kinematics.md).

---

## Cobra Flex Parameters

### Geometry

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel radius | $r$ | 0.03725 m | URDF `wheel_radius` |
| Wheel separation (track) | $W$ | 0.154 m | 2 × `wheel_off_y` |
| Wheelbase | $L$ | 0.120 m | 2 × `wheel_off_x` |
| Total mass | $m$ | 3.5 kg | measured |

### Operating Limits

The robot is bounded at two different levels. Nav2 plans inside the first set;
the serial driver clamps to the second before anything reaches the firmware.

| Parameter | Nav2 / DWB | Platform (driver) |
|-----------|-----------|-------------------|
| Maximum linear velocity | 0.35 m/s | 0.53 m/s |
| Minimum linear velocity | −0.15 m/s | −0.53 m/s |
| Maximum angular velocity | 2.0 rad/s | 6.0 rad/s |
| Maximum linear acceleration | 2.5 m/s² | — |
| Maximum angular acceleration | 3.2 rad/s² | — |

The ±2.5 m/s² linear acceleration limit is also enforced by the Gazebo
`DiffDrive` plugin. Neither the plugin nor the driver limits angular
acceleration; only Nav2 does.

---

## Model Structure

```text
4WD Skid-Steer System
│
├─ Forward kinematics: (ω_L, ω_R) → (v, ω)
│
├─ Inverse kinematics: (v, ω) → (ω_L, ω_R)
│
├─ Simulation: gz-sim-diff-drive-system
│  ├─ Input:  /cmd_vel (geometry_msgs/Twist)
│  ├─ Output: /odom (dead reckoning), TF on tf_diffdrive
│  └─ TF odom → base_footprint is owned by gz-sim-odometry-publisher-system
│
├─ Hardware: cobraflex_ros_driver
│  ├─ Input:  /cmd_vel, clamped and re-sent every 50 ms
│  ├─ Output: JSON frames over serial to the ESP32-S3
│  └─ TF odom → base_footprint is owned by the EKF (robot_localization)
│
└─ Navigation: Nav2
   ├─ AMCL (localisation)
   ├─ DWB local controller
   └─ NavFn global planner (Dijkstra)
```

---

## Implementation

- **Simulation plugin**: `gz-sim-diff-drive-system` (Gazebo Harmonic)
- **Odometry publish rate**: 50 Hz
- **Nav2 controller rate**: 20 Hz
- **Navigation stack**: Nav2 (AMCL + NavFn + DWB)

---

## Conventions

1. **Coordinate system**: right-handed — $x$ forward, $y$ left, $z$ up
2. **Positive angles**: counter-clockwise
3. **Velocities**: expressed in the robot frame $\lbrace R \rbrace$
4. **Configuration**: 4 driven wheels, no steering joint

---

## Credits

This documentation set — its split into kinematics, control and parameters, its
notation, and the derivations it carries — is adapted from
**[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)** by
[MrDavidAlv](https://github.com/MrDavidAlv), released under the BSD licence.

Axioma_robot is a ROS 2 Humble autonomous robot built on a 4WD skid-steer
chassis with SLAM Toolbox and Nav2 — the same class of platform and the same
software stack as this project — and it is where the idea for this project came
from.

The two robots are **not** the same chassis, and the numbers do not carry over:
Axioma uses a 0.0381 m wheel radius, a 0.1679 m effective track and a 0.26 m/s
top speed, against 0.03725 m, 0.154 m and 0.35/0.53 m/s here. Every figure in
these files has been re-derived from this repository's own URDFs, configuration
and bench measurements. Where a value is still unverified, it is marked as such
rather than inherited.

See [Kinematics §10](./Kinematics.md) for the detail.
