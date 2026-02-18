# 📏 Axioma Robot Physical Parameters

> **IMPORTANT NOTE**: This document contains ONLY REAL values extracted directly from the source code. Parameters are marked with ✅ and their exact source (file and line).

---

## 1. Robot Geometry

### 1.1 Wheel Dimensions

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel radius | $r$ | 0.0381 m | ✅ `model.sdf:72,149,226,303` |
| Wheel diameter | $d$ | 0.0762 m | ✅ `model.sdf:443` |
| Wheel width | $w$ | 0.03 m | ✅ `model.sdf:73,150,227,304` |

### 1.2 Chassis Dimensions

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel separation | $W$ | 0.1725 m | ✅ `model.sdf:441` |
| Distance between axles | $L$ | 0.1356 m | ✅ Calculated: \|0.0644-(-0.0712)\| |
| Estimated length | - | ~0.14 m | Estimated from positions |
| Estimated width | - | ~0.14 m | Estimated from positions |

**NOTE**: The chassis uses custom meshes (`chasis.dae`), there are no explicit box dimensions.

### 1.3 Wheel Positions

**Coordinates in `base_link` frame** (✅ `model.sdf`):

| Wheel | Position $(x, y, z)$ [m] | Joint | Line |
|-------|---------------------------|-------|------|
| Wheel 1 (FL) | (0.06442, 0.07385, 0.04068) | `base_to_wheel1` | 38 |
| Wheel 2 (RL) | (-0.071224, 0.07385, 0.04068) | `base_to_wheel2` | 115 |
| Wheel 3 (RR) | (-0.071224, -0.0625, 0.04068) | `base_to_wheel3` | 192 |
| Wheel 4 (FR) | (0.064423, -0.0625, 0.04068) | `base_to_wheel4` | 269 |

---

## 2. Inertial Properties

### 2.1 Total Robot Mass

| Component | Quantity | Unit Mass | Total Mass |
|-----------|----------|-----------|------------|
| Base Link (chassis) | 1 | 5.0 kg | 5.0 kg |
| Wheels (`wheel_1...4`) | 4 | 0.1 kg | 0.4 kg |
| LiDAR (`base_scan`) | 1 | 0.125 kg | 0.125 kg |
| **TOTAL** | - | - | **5.525 kg** |

**Sources**: `model.sdf:8, 55, 132, 209, 286, 348`

### 2.2 Chassis Inertia Tensor

**Base Link** (✅ REAL values from `model.sdf:6-17`):

$$
\\mathbf{I}_{base} = \\begin{bmatrix}
0.1135 & 0 & 0 \\\\
0 & 0.426 & 0 \\\\
0 & 0 & 0.5205
\\end{bmatrix} \\text{ kg·m}^2
$$

```xml
<link name="base_link">
  <mass>5.0</mass>
  <inertia>
    <ixx>0.1135</ixx>
    <iyy>0.426</iyy>
    <izz>0.5205</izz>
    <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
  </inertia>
</link>
```

### 2.3 Wheel Inertia Tensor

**Each wheel** (✅ REAL values from `model.sdf:55-65, 132-142, 209-219, 286-296`):

$$
\\mathbf{I}_{wheel} = \\begin{bmatrix}
0.0029 & 0 & 0 \\\\
0 & 0.0029 & 0 \\\\
0 & 0 & 0.0056
\\end{bmatrix} \\text{ kg·m}^2, \\quad m_{wheel} = 0.1 \\text{ kg}
$$

```xml
<link name="wheel_X">
  <mass>0.1</mass>
  <inertia>
    <ixx>0.0029</ixx>
    <iyy>0.0029</iyy>
    <izz>0.0056</izz>
  </inertia>
</link>
```

### 2.4 LiDAR Inertia Tensor

**LiDAR** (✅ REAL values from `model.sdf:348-358`):

$$
\\mathbf{I}_{lidar} = \\begin{bmatrix}
0.001 & 0 & 0 \\\\
0 & 0.001 & 0 \\\\
0 & 0 & 0.001
\\end{bmatrix} \\text{ kg·m}^2, \\quad m_{lidar} = 0.125 \\text{ kg}
$$

---

## 3. Dynamic Parameters

### 3.1 Wheel-Ground Friction

**✅ REAL values from `model.sdf:77-96, 154-173, 231-250, 308-327`**:

```xml
<friction>
  <ode>
    <mu>1.0</mu>
    <mu2>1.0</mu2>
    <slip1>0.0</slip1>
    <slip2>0.0</slip2>
  </ode>
</friction>
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| $\\mu_1$ | 1.0 | Longitudinal Coulomb friction coefficient |
| $\\mu_2$ | 1.0 | Lateral Coulomb friction coefficient |
| slip1 | 0.0 | Longitudinal slip |
| slip2 | 0.0 | Lateral slip |

**Maximum friction force**:
$$
F_{friction,max} = \\mu \\cdot F_N = 1.0 \\times \\frac{mg}{4} = 1.0 \\times \\frac{5.525 \\times 9.81}{4} = 13.55 \\text{ N per wheel}
$$

**Maximum friction-limited acceleration**:
$$
a_{friction,max} = \\mu \\cdot g = 1.0 \\times 9.81 = 9.81 \\text{ m/s}^2
$$

### 3.2 Contact Parameters (ODE)

**✅ REAL values from `model.sdf:87-96`**:

```xml
<contact>
  <ode>
    <soft_cfm>0</soft_cfm>
    <soft_erp>0.2</soft_erp>
    <kp>1e+13</kp>
    <kd>1.0</kd>
    <max_vel>0.01</max_vel>
    <min_depth>0.01</min_depth>
  </ode>
</contact>
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `soft_cfm` | 0 | Constraint Force Mixing (contact stiffness) |
| `soft_erp` | 0.2 | Error Reduction Parameter (penetration correction) |
| `kp` | $10^{13}$ | Contact stiffness (spring constant) |
| `kd` | 1.0 | Contact damping |
| `max_vel` | 0.01 m/s | Maximum allowed penetration velocity |
| `min_depth` | 0.01 m | Minimum contact depth for activation |

### 3.3 Joint Damping

**NOTE**: There are no active `<damping>` or `<friction>` parameters in the wheel joints. The tags are commented out in the SDF.

---

## 4. Operational Limits

### 4.1 Kinematic Limits

**✅ REAL values from `nav2_params.yaml`**:

| Parameter | Value | Source |
|-----------|-------|--------|
| Maximum linear velocity | $v_{max} = 0.26$ m/s | `nav2_params.yaml:120` |
| Maximum angular velocity | $\\omega_{max} = 1.0$ rad/s | `nav2_params.yaml:122` |
| Maximum linear acceleration | $a_{max} = 2.5$ m/s² | `nav2_params.yaml:129` |
| Maximum angular acceleration | $\\alpha_{max} = 3.2$ rad/s² | `nav2_params.yaml:130` |
| Linear deceleration | $a_{min} = -2.5$ m/s² | `nav2_params.yaml:131` |
| Angular deceleration | $\\alpha_{min} = -3.2$ rad/s² | `nav2_params.yaml:132` |

### 4.2 Torque Limits

**✅ REAL values from `model.sdf:446-447`**:

```xml
<max_wheel_torque>20</max_wheel_torque>
<max_wheel_acceleration>1.0</max_wheel_acceleration>
```

| Parameter | Value |
|-----------|-------|
| Maximum torque per wheel | 20 N·m |
| Maximum wheel acceleration (plugin) | 1.0 m/s² |

### 4.3 Joint Limits

**✅ REAL values from `model.sdf:43-46, 120-123, 197-200, 274-277`**:

```xml
<limit>
  <lower>-1e+16</lower>
  <upper>1e+16</upper>
</limit>
```

The wheels can rotate infinitely (revolute type without limits).

---

## 5. LiDAR Sensor

### 5.1 Specifications

**✅ REAL values from `model.sdf:346-426`**:

| Parameter | Value | Line |
|-----------|-------|------|
| Position on robot | $(0, 0, 0.15)$ m from `base_link` | 347 |
| Type | 2D planar laser | 351 |
| Model | `hls_lfcd_lds` | 375 |
| Samples per scan | 360 | 383 |
| Angular resolution | 2.0° | 384 |
| Minimum angle | 0° (0 rad) | 385 |
| Maximum angle | 360° (6.28 rad) | 386 |
| Minimum range | 0.12 m | 24, 390 |
| Maximum range | 3.5 m | 391 |
| Frequency | 20 Hz | 379 |
| Noise (mean) | 0.0 | 395 |
| Noise (stddev) | 0.01 | 396 |

**Frame**: `base_scan` (line 413)
**Topic**: `/scan` (line 403)

### 5.2 LiDAR Transform

Transform from `base_link` → `base_scan`:

$$
\\mathbf{T}_{scan}^{base} = \\begin{bmatrix}
1 & 0 & 0 & 0 \\\\
0 & 1 & 0 & 0 \\\\
0 & 0 & 1 & 0.15 \\\\
0 & 0 & 0 & 1
\\end{bmatrix}
$$

The LiDAR is mounted **15 cm above** the `base_link` origin.

---

## 6. Navigation Parameters

### 6.1 Costmaps

**Local Costmap** (✅ `nav2_params.yaml:171-209`):

```yaml
update_frequency: 5.0 Hz
publish_frequency: 2.0 Hz
width: 3 m
height: 3 m
resolution: 0.05 m/cell
robot_radius: 0.15 m
inflation_radius: 0.55 m
cost_scaling_factor: 3.0
```

**Global Costmap** (✅ `nav2_params.yaml:211-244`):

```yaml
update_frequency: 1.0 Hz
publish_frequency: 1.0 Hz
resolution: 0.05 m/cell
robot_radius: 0.15 m
inflation_radius: 0.55 m
cost_scaling_factor: 3.0
```

### 6.2 Goal Tolerances

**✅ REAL values from `nav2_params.yaml:163-164`**:

```yaml
xy_goal_tolerance: 0.15 m
yaw_goal_tolerance: 0.25 rad (≈14.3°)
```

---

## 7. SLAM Parameters

### 7.1 SLAM Toolbox

**✅ REAL values from `slam_params.yaml`**:

```yaml
resolution: 0.05 m/cell
max_laser_range: 3.5 m
minimum_time_interval: 0.5 s
minimum_travel_distance: 0.5 m
minimum_travel_heading: 0.5 rad (≈28.6°)

Loop closure:
  do_loop_closing: true
  loop_search_maximum_distance: 3.0 m
  loop_match_minimum_chain_size: 10
```

### 7.2 Existing Map

**✅ REAL values from `maps/mapa.yaml`**:

```yaml
Image: mapa.pgm
Dimensions: 215 × 231 pixels
Resolution: 0.05 m/pixel
Origin: [-5.69, -5.14, 0] meters
occupied_thresh: 0.65
free_thresh: 0.25
```

**Map area**:
- Width: $215 \\times 0.05 = 10.75$ m
- Height: $231 \\times 0.05 = 11.55$ m
- Total area: $\\approx 124$ m²

---

## 8. Derived Calculations

### 8.1 Kinematic Performance

**Wheel angular speed at maximum linear speed**:
$$
\\omega_{wheel} = \\frac{v_{max}}{r} = \\frac{0.26}{0.0381} = 6.82 \\text{ rad/s} = 65.1 \\text{ RPM}
$$

**Wheel angular speed in pure rotation**:
$$
\\omega_{wheel,rot} = \\frac{\\omega_{max} \\cdot W}{2r} = \\frac{1.0 \\times 0.1725}{2 \\times 0.0381} = 2.26 \\text{ rad/s} = 21.6 \\text{ RPM}
$$

### 8.2 Braking Distance

From maximum speed with maximum deceleration:
$$
d_{brake} = \\frac{v_{max}^2}{2 \\cdot |a_{min}|} = \\frac{0.26^2}{2 \\times 2.5} = 0.0135 \\text{ m} = 1.35 \\text{ cm}
$$

### 8.3 Minimum Turning Radius

In pure rotation ($v = 0$):
$$
R_{min} = 0 \\text{ m (zero-point turn)}
$$

At maximum speed with maximum rotation:
$$
R = \\frac{v_{max}}{\\omega_{max}} = \\frac{0.26}{1.0} = 0.26 \\text{ m}
$$

---

## 9. Parameter Summary Table

### 9.1 Geometric Parameters

```python
PARAMS_GEOMETRY = {
    'wheel_radius': 0.0381,      # m (model.sdf:72)
    'wheel_separation': 0.1725,  # m (model.sdf:441)
    'wheel_base': 0.1356,        # m (calculated)
    'total_mass': 5.525          # kg (sum of masses)
}
```

### 9.2 Kinematic Parameters

```python
PARAMS_KINEMATICS = {
    'max_linear_velocity': 0.26,      # m/s (nav2_params:120)
    'max_angular_velocity': 1.0,      # rad/s (nav2_params:122)
    'max_linear_acceleration': 2.5,   # m/s² (nav2_params:129)
    'max_angular_acceleration': 3.2,  # rad/s² (nav2_params:130)
    'max_wheel_torque': 20.0          # N·m (model.sdf:446)
}
```

### 9.3 Control Parameters

```python
PARAMS_CONTROL = {
    'gazebo_update_rate': 50.0,       # Hz (model.sdf:439)
    'nav2_controller_freq': 20.0,     # Hz (nav2_params:114)
    'velocity_smoother_freq': 20.0,   # Hz (nav2_params:307)
    'costmap_update_freq': 5.0        # Hz (nav2_params:173)
}
```

---

## 10. Detected Inconsistencies

### 10.1 Base Link Mass

**SDF vs URDF**:
- `model.sdf:8`: mass = 5.0 kg ✅ (used in Gazebo)
- `axioma.urdf:22`: mass = 1.0 kg ⚠️ (inconsistent)

**Recommendation**: Unify to 5.0 kg

### 10.2 Wheel Separation

**Plugin vs Geometry**:
- `model.sdf:441`: wheel_separation = 0.1725 m ✅ (used by plugin)
- Real positions: $y_{left} - y_{right} = 0.07385 - (-0.0625) = 0.13635$ m ⚠️

**Difference**: 3.6 cm (21% error)

**Impact**: It may cause odometry drift during rotations.

---

## 11. Cross-References

- **Kinematics**: [cinematica.md](./cinematica.md) uses these geometric parameters
- **Control**: [control.md](./control.md) uses limits and frequencies
- **Source files**:
  - Main model: `src/axioma_description/models/axioma_v2/model.sdf`
  - Nav2 parameters: `src/axioma_navigation/config/nav2_params.yaml`
  - SLAM parameters: `src/axioma_navigation/config/slam_params.yaml`

---

**Author**: Samuel Sanchez
**Date**: 2026
**Version**: 0.2.0
