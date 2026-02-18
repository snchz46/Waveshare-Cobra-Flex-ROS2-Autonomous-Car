# 📐 Axioma 4WD Robot Mathematical Model

## General Description

This document presents the complete mathematical model of the autonomous Axioma robot, a mobile platform with **4 driven wheels in a skid-steer configuration** (4-wheel differential drive). The model covers kinematics, control, and physical system parameters.

> **IMPORTANT NOTE**: All parameters in this documentation are REAL values extracted directly from the source code (`model.sdf`, `nav2_params.yaml`, Gazebo plugins).

---

## 📑 Contents

1. **[Kinematics](./cinematica.md)**
   - Differential-drive kinematic model
   - Forward and inverse kinematics
   - Odometry

2. **[Control](./control.md)**
   - Gazebo `diff_drive` plugin
   - Velocity and acceleration limits
   - Nav2 navigation system

3. **[Parameters](./parametros.md)**
   - Geometric parameters
   - Dynamic characteristics
   - Sensors (LiDAR)

4. **[Excalidraw Diagram](./modelo-axioma.excalidraw)**
   - Visual representation of the model
   - Fundamental equations
   - Key parameters

---

## Mathematical Notation

### Coordinate Systems

| Symbol | Description | ROS2 Frame |
|--------|-------------|------------|
| $\\{W\\}$ | World coordinate system | `map` / `odom` |
| $\\{R\\}$ | Robot coordinate system | `base_link` |

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
- $r = 0.0381$ m (wheel radius)
- $W = 0.1725$ m (wheel separation)

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

| Parameter | Symbol | Real Value | Source |
|-----------|--------|------------|--------|
| Wheel radius | $r$ | 0.0381 m | `model.sdf:72` |
| Wheel separation | $W$ | 0.1725 m | `model.sdf:441` |
| Wheelbase | $L$ | 0.1356 m | Calculated from positions |
| Total mass | $m$ | 5.525 kg | Sum of masses in SDF |

### Operating Limits

| Parameter | Value | Source |
|-----------|-------|--------|
| Maximum linear speed | 0.26 m/s | `nav2_params.yaml:120` |
| Maximum angular speed | 1.0 rad/s | `nav2_params.yaml:122` |
| Maximum linear acceleration | 2.5 m/s² | `nav2_params.yaml:129` |
| Maximum angular acceleration | 3.2 rad/s² | `nav2_params.yaml:130` |
| Maximum torque per wheel | 20 N·m | `model.sdf:446` |

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
│  └─ Output: /odom (Odometry), TF (odom → base_link)
│
└─ Navigation: Nav2 Stack
   ├─ AMCL (Localization)
   ├─ DWB Local Planner
   └─ NavFn Global Planner
```

---

## Implementation

- **Control Plugin**: `libgazebo_ros_diff_drive.so` (`model.sdf:427-454`)
- **Update frequency**: 50 Hz
- **Odometry**: Published on `/odom` with TF
- **Navigation stack**: Nav2 Humble

---

## Conventions

1. **Coordinate system**: Right-handed (x forward, y left, z up)
2. **Positive angles**: Counterclockwise
3. **Velocities**: Expressed in the robot frame $\\{R\\}$
4. **Configuration**: 4 wheels (2 pairs), no independent steering

---

**Author**: Samuel Sanchez
**Project**: Cobraflex - ADMIT14
**Date**: 2026
**Version**: 0.2.0
