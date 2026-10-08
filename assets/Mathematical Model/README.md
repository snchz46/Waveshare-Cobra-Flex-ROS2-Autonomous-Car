# Cobra Flex 4WD mathematical model

## Overview

Mathematical model of the Waveshare Cobra Flex: a mobile platform with
**four driven wheels in skid-steer configuration**, modelled throughout the
stack as a differential drive. The model covers kinematics, control and the
physical parameters of the system.

> **Credit.** The structure of this documentation and the original idea of the
> project are based on
> [MrDavidAlv/Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot).
> See [Credits](#credits).

---

## Contents

1. **[Kinematics](./Kinematics.md)**
   - Differential-drive kinematic model and skid-steer deviations
   - Forward and inverse kinematics
   - Odometry and TF ownership
   - Velocity, acceleration and wheel-speed limits

2. **[Control](./Control.md)**
   - Gazebo `DiffDrive` plugin
   - `/cmd_vel` chain in simulation and on hardware
   - Deadman timers

3. **[Parameters](./parameters.md)**
   - Geometry and the open firmware-constant discrepancy (§1.4)
   - Mass budget and inertia tensors
   - Sensors (LiDAR, ZED Mini, lane camera)

---

## Notation

### Coordinate systems

| Symbol | Description | ROS 2 frame |
|--------|-------------|-------------|
| $\lbrace W \rbrace$ | World coordinate system | `map` / `odom` |
| $\lbrace R \rbrace$ | Robot coordinate system | `base_footprint` |

### State variables

| Variable | Description | Unit |
|----------|-------------|------|
| $q = [x, y, \theta]^T$ | Robot pose in $\lbrace W \rbrace$ | m, m, rad |
| $\dot{q} = [v, \omega]^T$ | Robot velocity | m/s, rad/s |
| $v_L, v_R$ | Left/right side linear speed | m/s |
| $\omega_L, \omega_R$ | Left/right wheel angular speed | rad/s |
| $r$ | Wheel radius | m |
| $W$ | Wheel separation (track) | m |
| $L$ | Wheelbase (front to rear) | m |

---

## Fundamental equations

### Forward kinematics

$$
v = \frac{v_R + v_L}{2} = \frac{r(\omega_R + \omega_L)}{2}
$$

$$
\omega = \frac{v_R - v_L}{W} = \frac{r(\omega_R - \omega_L)}{W}
$$

### Inverse kinematics

$$
\omega_L = \frac{v - \omega \cdot W/2}{r}
$$

$$
\omega_R = \frac{v + \omega \cdot W/2}{r}
$$

with $r = 0.03725$ m and $W = 0.154$ m.

These are the **ideal differential-drive** equations. The robot is a
skid-steer vehicle with two axles 0.120 m apart; it turns by dragging all four
wheels sideways, which introduces a yaw gain error not represented in the model
above. See [Kinematics §3.3](./Kinematics.md).

---

## Cobra Flex parameters

### Geometry

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel radius | $r$ | 0.03725 m | URDF `wheel_radius` |
| Wheel separation (track) | $W$ | 0.154 m | 2 × `wheel_off_y` |
| Wheelbase | $L$ | 0.120 m | 2 × `wheel_off_x` |
| Total mass | $m$ | 3.5 kg | Measured |

### Operating limits

Velocity is limited at two levels. Nav2 plans within the first set; the serial
driver clamps to the second before the firmware.

| Parameter | Nav2 / DWB | Platform (driver) |
|-----------|-----------|-------------------|
| Maximum linear velocity | 0.35 m/s | 0.53 m/s |
| Minimum linear velocity | −0.15 m/s | −0.53 m/s |
| Maximum angular velocity | 2.0 rad/s | 6.0 rad/s |
| Maximum linear acceleration | 2.5 m/s² | — |
| Maximum angular acceleration | 3.2 rad/s² | — |

The Gazebo `DiffDrive` plugin also enforces the ±2.5 m/s² linear acceleration
limit. Angular acceleration is limited by Nav2 only; neither the plugin nor the
driver limits it.

---

## Model structure

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
│  └─ TF odom → base_footprint published by gz-sim-odometry-publisher-system
│
├─ Hardware: cobraflex_ros_driver
│  ├─ Input:  /cmd_vel, clamped and re-sent every 50 ms
│  ├─ Output: JSON frames over serial to the ESP32-S3
│  └─ TF odom → base_footprint published by the EKF (robot_localization)
│
└─ Navigation: Nav2
   ├─ AMCL (localisation)
   ├─ DWB local controller
   └─ NavFn global planner (Dijkstra)
```

---

## Implementation

- **Simulation plugin**: `gz-sim-diff-drive-system` (Gazebo Harmonic)
- **Odometry publication rate**: 50 Hz
- **Nav2 controller rate**: 20 Hz
- **Navigation stack**: Nav2 (AMCL + NavFn + DWB)

---

## Conventions

1. **Coordinate system**: right-handed; $x$ forward, $y$ left, $z$ up
2. **Positive angles**: counter-clockwise
3. **Velocities**: expressed in the robot frame $\lbrace R \rbrace$
4. **Configuration**: four driven wheels, no steering joint

---

## Credits

The structure of this documentation (division into kinematics, control and
parameters), its notation and its derivations are adapted from
**[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)** by
[MrDavidAlv](https://github.com/MrDavidAlv), released under the BSD licence.

Axioma_robot is a ROS 2 Humble autonomous robot on a 4WD skid-steer chassis
with SLAM Toolbox and Nav2, the same platform class and software stack as this
project, and the origin of its idea.

The chassis differ and no values are shared. Axioma uses a 0.0381 m wheel
radius, a 0.1679 m effective track and a 0.26 m/s top speed; this robot uses
0.03725 m, 0.154 m and 0.35/0.53 m/s. All values in these files are derived
from the URDFs, configuration files and bench measurements of this repository.
Unverified values are marked as such.

Details: [Kinematics §10](./Kinematics.md).
