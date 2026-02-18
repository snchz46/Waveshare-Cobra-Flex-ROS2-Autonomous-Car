# 🔄 Axioma 4WD Robot Kinematics

> **NOTE**: This document presents the differential-drive kinematic model (4WD skid-steer). All parameters are REAL values extracted from `model.sdf` and Gazebo plugins.

## 1. Introduction

The Axioma robot uses a **4-wheel drive configuration with differential kinematics** (skid-steer). Although it has 4 wheels, it is controlled as a traditional differential drive:
- Left wheels (1 and 2) rotate at the same speed
- Right wheels (3 and 4) rotate at the same speed
- Differential control between sides enables rotation

---

## 2. Robot Geometry

### 2.1 Wheel Configuration

The robot has **4 wheels** arranged in a rectangular configuration:

**Positions in Frame {R}** (extracted from `model.sdf`):

| Wheel | Name | Position $(x, y, z)$ [m] | Joint |
|-------|------|---------------------------|-------|
| 1 | Front Left | (0.0644, 0.0738, 0.0407) | `base_to_wheel1` |
| 2 | Rear Left | (-0.0712, 0.0738, 0.0407) | `base_to_wheel2` |
| 3 | Rear Right | (-0.0712, -0.0625, 0.0407) | `base_to_wheel3` |
| 4 | Front Right | (0.0644, -0.0625, 0.0407) | `base_to_wheel4` |

**Source**: `model.sdf` lines 38, 115, 192, 269

### 2.2 Geometric Parameters

**✅ REAL values from `model.sdf`**:

```python
wheel_radius (r) = 0.0381 m        # lines 72, 149, 226, 303
wheel_diameter   = 0.0762 m        # line 443 (2 × radius)
wheel_separation = 0.1725 m        # line 441
wheel_base       = 0.135644 m      # Computed: |0.0644-(-0.0712)| m
wheel_width      = 0.03 m          # lines 73, 150, 227, 304
```

### 2.3 Coordinate System

```
        y (left)
        ↑
        |     W1 ●━━━━━● W4
        |        |     |
        |        | [R] |  (robot frame)
        |        |     |
        |     W2 ●━━━━━● W3
        |
        └─────────────────→ x (forward)

             θ (yaw, counterclockwise)
```

**Distances**:
- Wheelbase (L): Front-to-rear distance = 0.1356 m
- Track width (W): Left-to-right distance = 0.1725 m

---

## 3. Differential Drive Kinematic Model

### 3.1 Robot Velocity

The robot velocity vector in its own frame $\{R\}$ is:

$$
\mathbf{v}_R = \begin{bmatrix} v \\ \omega \end{bmatrix}
$$

Where:
- $v$: Linear velocity along the $x$ axis (forward/backward)
- $\omega$: Angular velocity around the $z$ axis (rotation)

**NOTE**: A differential drive has NO lateral mobility ($v_y = 0$), it can only move forward/backward and rotate.

### 3.2 Wheel Velocities

For a 4-wheel system grouped into 2 pairs (left/right):

**Left pair** (wheels 1 and 2):
$$
\omega_L = \omega_1 = \omega_2
$$

**Right pair** (wheels 3 and 4):
$$
\omega_R = \omega_3 = \omega_4
$$

Where $\omega_i$ is the angular velocity of wheel $i$ in $rad/s$.

---

## 4. Forward Kinematics

### 4.1 From Wheel Velocities to Robot Velocity

Given wheel angular velocities $\omega_L$ and $\omega_R$, robot velocity is computed as:

**Linear velocity**:
$$
v = \frac{r(\omega_R + \omega_L)}{2}
$$

**Angular velocity**:
$$
\omega = \frac{r(\omega_R - \omega_L)}{W}
$$

Where:
- $r = 0.0381$ m (wheel radius)
- $W = 0.1725$ m (wheel separation)

### 4.2 Derivation

The robot linear speed is the average of left and right wheel linear speeds:

$$
v = \frac{v_R + v_L}{2} = \frac{r\omega_R + r\omega_L}{2} = \frac{r(\omega_R + \omega_L)}{2}
$$

Angular speed comes from the speed difference between sides:

$$
\omega = \frac{v_R - v_L}{W} = \frac{r\omega_R - r\omega_L}{W} = \frac{r(\omega_R - \omega_L)}{W}
$$

### 4.3 Matrix Form

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} =
\begin{bmatrix}
\frac{r}{2} & \frac{r}{2} \\
\frac{r}{W} & -\frac{r}{W}
\end{bmatrix}
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}
$$

With real values:
$$
\begin{bmatrix} v \\ \omega \end{bmatrix} =
\begin{bmatrix}
0.01905 & 0.01905 \\
0.2209 & -0.2209
\end{bmatrix}
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}
$$

---

## 5. Inverse Kinematics

### 5.1 From Robot Velocity to Wheel Velocities

Given desired robot velocities $(v, \omega)$, wheel angular velocities are:

**Left wheels**:
$$
\omega_L = \frac{v - \omega \cdot \frac{W}{2}}{r}
$$

**Right wheels**:
$$
\omega_R = \frac{v + \omega \cdot \frac{W}{2}}{r}
$$

### 5.2 Derivation

For the robot to move at speed $v$ and rotate at angular speed $\omega$:

- Left wheel linear speed must be: $v_L = v - \omega \cdot \frac{W}{2}$
- Right wheel linear speed must be: $v_R = v + \omega \cdot \frac{W}{2}$

Converting linear speeds to angular speeds:

$$
\omega_L = \frac{v_L}{r} = \frac{v - \omega \cdot W/2}{r}
$$

$$
\omega_R = \frac{v_R}{r} = \frac{v + \omega \cdot W/2}{r}
$$

### 5.3 Matrix Form

$$
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} =
\begin{bmatrix}
\frac{1}{r} & -\frac{W}{2r} \\
\frac{1}{r} & \frac{W}{2r}
\end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix}
$$

With real values:
$$
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} =
\begin{bmatrix}
26.247 & -2.263 \\
26.247 & 2.263
\end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix}
$$

---

## 6. Odometry

### 6.1 Pose Integration

Robot pose in world frame $\{W\}$ evolves as:

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}_W =
\begin{bmatrix}
v \cos\theta \\
v \sin\theta \\
\omega
\end{bmatrix}
$$

### 6.2 Numerical Integration (Euler)

With sampling period $\Delta t = 0.02$ s (50 Hz):

$$
\begin{aligned}
x_{k+1} &= x_k + v \cos\theta_k \cdot \Delta t \\
y_{k+1} &= y_k + v \sin\theta_k \cdot \Delta t \\
\theta_{k+1} &= \theta_k + \omega \cdot \Delta t
\end{aligned}
$$

### 6.3 Gazebo Implementation

**Plugin**: `libgazebo_ros_diff_drive.so` (`model.sdf:427-454`)

**Configuration**:
```xml
<update_rate>50</update_rate>
<num_wheel_pairs>2</num_wheel_pairs>
<publish_odom>true</publish_odom>
<publish_odom_tf>true</publish_odom_tf>
<odometry_frame>odom</odometry_frame>
<robot_base_frame>base_link</robot_base_frame>
```

**Source**: `model.sdf` lines 439-451

---

## 7. Constraints and Limits

### 7.1 Kinematic Limits

**✅ REAL values from `nav2_params.yaml`**:

$$
\begin{aligned}
|v| &\leq v_{max} = 0.26 \text{ m/s} \quad \text{(line 120)} \\
|\omega| &\leq \omega_{max} = 1.0 \text{ rad/s} \quad \text{(line 122)} \\
|\dot{v}| &\leq a_{max} = 2.5 \text{ m/s}^2 \quad \text{(line 129)} \\
|\dot{\omega}| &\leq \alpha_{max} = 3.2 \text{ rad/s}^2 \quad \text{(line 130)}
\end{aligned}
$$

### 7.2 Wheel Limits

From inverse kinematics, maximum wheel angular speeds are:

**Straight motion** ($\omega = 0$):
$$
\omega_{wheel,max} = \frac{v_{max}}{r} = \frac{0.26}{0.0381} = 6.82 \text{ rad/s}
$$

**Pure rotation** ($v = 0$):
$$
\omega_{wheel,max} = \frac{\omega_{max} \cdot W}{2r} = \frac{1.0 \times 0.1725}{2 \times 0.0381} = 2.263 \text{ rad/s}
$$

**Maximum combined speed**:
$$
\omega_{wheel,max} = \frac{v_{max} + \omega_{max} \cdot W/2}{r} = \frac{0.26 + 0.08625}{0.0381} = 9.09 \text{ rad/s}
$$

### 7.3 Minimum Turning Radius

For in-place rotation ($v = 0, \omega \neq 0$):
$$
R_{min} = 0 \text{ m}
$$

The robot can turn about its own axis (zero-radius turn).

For motion at maximum linear speed with maximum angular speed:
$$
R = \frac{v_{max}}{\omega_{max}} = \frac{0.26}{1.0} = 0.26 \text{ m}
$$

---

## 8. Computational Implementation

### 8.1 Pseudocode: Inverse Kinematics

```python
def compute_wheel_velocities(v, omega):
    """
    Computes wheel angular velocities from robot velocity

    Args:
        v: Linear velocity [m/s]
        omega: Angular velocity [rad/s]

    Returns:
        omega_left, omega_right: Angular velocities [rad/s]
    """
    r = 0.0381  # wheel_radius
    W = 0.1725  # wheel_separation

    # Saturate commands
    v = clip(v, -0.26, 0.26)
    omega = clip(omega, -1.0, 1.0)

    # Inverse kinematics
    omega_left = (v - omega * W/2) / r
    omega_right = (v + omega * W/2) / r

    return omega_left, omega_right
```

### 8.2 Pseudocode: Forward Kinematics

```python
def compute_robot_velocity(omega_left, omega_right):
    """
    Computes robot velocity from wheel velocities

    Args:
        omega_left: Left wheel angular velocity [rad/s]
        omega_right: Right wheel angular velocity [rad/s]

    Returns:
        v, omega: Robot linear and angular velocity
    """
    r = 0.0381  # wheel_radius
    W = 0.1725  # wheel_separation

    # Forward kinematics
    v = r * (omega_right + omega_left) / 2
    omega = r * (omega_right - omega_left) / W

    return v, omega
```

---

## 9. References

**Implementation**:
- Gazebo plugin: `libgazebo_ros_diff_drive.so` (`model.sdf:427-454`)
- Geometric parameters: `model.sdf:441-444`
- Speed limits: `nav2_params.yaml:120-130`

**External documentation**:
- [Gazebo Diff Drive Plugin](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
- [Differential Drive Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)

---

**Author**: Samuel Sanchez
**Date**: 2026
**Version**: 0.2.0
