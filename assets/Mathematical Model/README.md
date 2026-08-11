# 📐 4WD Robot Mathematical Model

## General Description

This document presents the complete mathematical model of the robot, a mobile platform with **4 driven wheels in a skid-steer configuration** (4-wheel differential drive). The model covers kinematics, control, and physical system parameters.


---

## 📑 Contents

1. **[Kinematics](./Kinematics.md)**
   - Differential-drive kinematic model
   - Forward and inverse kinematics
   - Odometry

2. **[Control](./Control.md)**
   - Gazebo `diff_drive` plugin
   - Velocity and acceleration limits
   - Nav2 navigation system

3. **[Parameters](./parameters.md)**
   - Geometric parameters
   - Dynamic characteristics
   - Sensors (LiDAR)

---

## Mathematical Notation

### Coordinate Systems

| Symbol | Description | ROS2 Frame |
|--------|-------------|------------|
| $\\{W\\}$ | World coordinate system | `map` / `odom` |
| $\\{R\\}$ | Robot coordinate system | `base_footprint` |

### State Variables

| Variable | Description | Unit |
|----------|-------------|------|
| $q = [x, y, \\theta]^T$ | Robot pose in $\\{W\\}$ | $[m, m, rad]$ |
| $\\dot{q} = [v, \\omega]^T$ | Robot velocity | $[m/s, rad/s]$ |
| $v_L, v_R$ | Left/right wheel linear speed | $m/s$ |
| $\\omega_L, \\omega_R$ | Left/right wheel angular speed | $rad/s$ |

---

## Fundamental Equations

### Differential Kinematics

For a 4-wheel differential-drive robot:

$$
v = \\frac{v_R + v_L}{2} = \\frac{r(\\omega_R + \\omega_L)}{2}
$$

$$
\\omega = \\frac{v_R - v_L}{W} = \\frac{r(\\omega_R - \\omega_L)}{W}
$$

Where:
- $r = 0.03725$ m (wheel radius)
- $W = 0.154$ m (wheel separation)

### Inverse Kinematics

$$
\\omega_L = \\frac{v - \\omega \\cdot W/2}{r}
$$

$$
\\omega_R = \\frac{v + \\omega \\cdot W/2}{r}
$$

---

## Axioma Robot Parameters

### Geometry

| Parameter | Symbol | Real Value |
|-----------|--------|------------|
| Wheel radius | $r$ | 0.03725 m |
| Wheel separation | $W$ | 0.154 m |
| Wheelbase | $L$ | 0.1356 m |
| Total mass | $m$ | 3.5 kg |

### Operating Limits

| Parameter | Value |
|-----------|-------|
| Maximum linear speed | 0.53 m/s |
| Maximum angular speed | 6.0 rad/s |
| Maximum linear acceleration | 2.5 m/s² |
| Maximum angular acceleration | 3.2 rad/s² |
| Maximum torque per wheel | 20 N·m |

---

## Model Structure

```
4WD Skid-Steer System
│
├─ Forward Kinematics: (ω_L, ω_R) → (v, ω)
│
├─ Inverse Kinematics: (v, ω) → (ω_L, ω_R)
│
├─ Gazebo Control: Plugin libgazebo_ros_diff_drive.so
│  ├─ Input: /cmd_vel (Twist)
│  └─ Output: /odom (Odometry), TF (odom → base_footprint)
│
└─ Navigation: Nav2 Stack
   ├─ AMCL (Localization)
   ├─ DWB Local Planner
   └─ NavFn Global Planner
```

---

## Implementation

- **Control Plugin**: `gz-sim-diff-drive-system`
- **Update frequency**: 50 Hz
- **Odometry**: Published on `/odom` with TF
- **Navigation stack**: Nav2

---

## Conventions

1. **Coordinate system**: Right-handed (x forward, y left, z up)
2. **Positive angles**: Counterclockwise
3. **Velocities**: Expressed in the robot frame $\\{R\\}$
4. **Configuration**: 4 wheels, no independent steering

---
