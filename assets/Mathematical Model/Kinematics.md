# 4WD skid-steer kinematics

## 1. Introduction

The Cobra Flex is a **four-wheel-drive skid-steer** platform. The four wheels
are fixed and the description contains no steering joint; the robot turns by
driving the left and right wheel pairs at different speeds, as a two-wheel
differential drive does.

For this reason the whole stack (Gazebo `DiffDrive` plugin, Nav2 DWB
controller, serial driver) models the robot as a differential drive, and
sections 4 and 5 derive the ideal differential-drive equations.

This model is an approximation. A two-wheel differential drive rotates about a
point on its wheel axle, and its wheels roll without sliding. A skid-steer
vehicle with a 0.120 m wheelbase has no such axle and turns by **dragging all
four wheels sideways**. Section 3.3 quantifies this effect and states where it
is accounted for.

> **Origin.** The structure and the derivations are adapted from
> [MrDavidAlv/Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot).
> See section 10.

---

## 2. Robot geometry

### 2.1 Wheel configuration

The four wheels are located at the corners of a rectangle. **Positions are the
wheel joint origins relative to `base_link`**, from `urdf/my_robot_gazebo.urdf`
(identical in `my_robot_basic.urdf`, `my_robot_mesh.urdf` and
`my_robot_gazebo_mesh.urdf`):

| Wheel | Name | Position $(x, y, z)$ [m] | Joint |
|-------|------|---------------------------|-------|
| W1 | Front left | (+0.060, +0.077, −0.020) | `front_left_wheel_joint` |
| W2 | Rear left | (−0.060, +0.077, −0.020) | `rear_left_wheel_joint` |
| W3 | Rear right | (−0.060, −0.077, −0.020) | `rear_right_wheel_joint` |
| W4 | Front right | (+0.060, −0.077, −0.020) | `front_right_wheel_joint` |

Both offsets are derived from xacro properties:

```xml
<xacro:property name="wheel_off_x" value="0.060" />
<xacro:property name="wheel_off_y" value="${(chassis_width/2) + (wheel_width/2) + 0.002}" />
```

With `chassis_width` = 0.130 m and `wheel_width` = 0.020 m, `wheel_off_y`
= 0.065 + 0.010 + 0.002 = **0.077 m**; the two sides are therefore 0.154 m
apart.

> **Vertical offset of −0.020 m.** `base_joint` raises `base_link` by one wheel
> radius (0.03725 m) above `base_footprint`, which would place the wheel axles
> at axle height. The wheel joints lower them by a further 0.020 m, so the axles
> are 0.01725 m above the `base_footprint` plane and the wheels extend 0.020 m
> below it. Planar kinematics uses only $x$ and $y$ and is not affected, but
> `base_footprint` is consequently not at ground level. This is a property of
> the robot description, not of the kinematics, and is left unchanged.

### 2.2 Geometric parameters

As declared in the URDFs and in the `DiffDrive` plugin (`urdf/robot.gazebo`):

```python
wheel_radius     (r) = 0.03725 m
wheel_diameter       = 0.0745  m
wheel_separation (W) = 0.154   m     # track,     2 * wheel_off_y
wheel_base       (L) = 0.120   m     # wheelbase, 2 * wheel_off_x
wheel_width          = 0.02    m
```

Only $r$ and $W$ appear in the kinematic equations. $L$ does not appear in an
ideal differential-drive model; it is relevant in section 3.3, as the
parameter that distinguishes this robot from a differential drive.

### 2.3 Coordinate system

Top view, ROS convention: $x$ forward, $y$ left, $z$ up, yaw $\theta$ positive
counter-clockwise.

```text
                            x (forward)
                                ^
                                |
        W1 (FL) o---------------+---------------o W4 (FR)   front axle, x = +0.060
                                |
       y (left) <---------------O---------------> -y (right)
                            base_link
                                |
        W2 (RL) o---------------+---------------o W3 (RR)   rear axle,  x = -0.060
                                |

                |<--------- W = 0.154 m --------->|   track     (left to right)
                          L = 0.120 m                 wheelbase (front to rear)
```

---

## 3. Kinematic model

### 3.1 Robot velocity

The robot velocity in its own frame $\lbrace R \rbrace$ is:

$$
\mathbf{v}_R = \begin{bmatrix} v \\ \omega \end{bmatrix}
$$

where:

- $v$: linear velocity along $x$, positive forward, in m/s
- $\omega$: angular velocity about $z$, positive counter-clockwise, in rad/s

**The model has no lateral term.** $v_y$ is not a commandable degree of
freedom: the plugin and the serial driver ignore `/cmd_vel.linear.y`, and Nav2
is configured with `max_vel_y: 0.0`. This is a *nonholonomic constraint* on the
commands; it does not imply that the wheels never slide sideways. On a
skid-steer vehicle they slide in every turn.

### 3.2 Wheel velocities

The four wheels are commanded in two pairs. The `DiffDrive` plugin declares two
`<left_joint>` and two `<right_joint>` entries, so each side receives a single
setpoint:

**Left pair** (W1 front left, W2 rear left):

$$
\omega_L = \omega_1 = \omega_2
$$

**Right pair** (W3 rear right, W4 front right):

$$
\omega_R = \omega_3 = \omega_4
$$

where $\omega_i$ is the angular velocity of wheel $i$ in rad/s.

### 3.3 Skid-steer and ideal differential drive

Sections 4 and 5 assume that every wheel rolls without slipping. On this
chassis the assumption is violated whenever $\omega \neq 0$.

An ideal differential drive has one axle; during a turn its instantaneous
centre of rotation (ICR) lies on that axle and both wheels roll without
slipping. This robot has two axles 0.120 m apart. Any single ICR lies off both
axles, so the front and rear wheels **scrub sideways** during every turn. The
rolling constraint is violated by design.

The consequence is a **yaw gain error**: the achieved yaw rate differs from the
commanded $\omega$. Part of the wheel-speed difference is lost to scrub and the
robot under-rotates. The usual first-order correction models a track wider than
the physical one:

$$
\omega = \frac{r(\omega_R - \omega_L)}{\chi \, W}, \qquad \chi \geq 1
$$

$\chi$ is the **effective-track (ICR) correction factor**. It cannot be derived
from geometry alone, since it depends on tyre, surface and load, and must be
measured.

Values in this repository:

| Source | Track constant | Implied $\chi$ |
|---|---|---|
| Measured on the robot (tape) | 0.153 m | — |
| URDF + Gazebo `DiffDrive` | 0.154 m | 1.00 (uncorrected) |
| Waveshare firmware `TRACK_WIDTH` | 0.159 m | ≈ 1.04 |
| **Measured yaw transfer** | **0.309 m effective** | **≈ 2.02** |

**Measured value of $\chi$: approximately 2.** In-place rotation on the
physical robot, 10 s per point, least squares through the origin:

| Commanded | Expected | Measured | Achieved | Gain |
|---|---|---|---|---|
| 0.20 rad/s | 114.6° | 55.6° | 0.097 rad/s | 0.485 |
| 0.40 rad/s | 229.2° | 114.5° | 0.200 rad/s | 0.500 |
| 0.53 rad/s | 303.7° | 150.4° | 0.263 rad/s | 0.495 |
| 0.80 rad/s | 458.4° | 226.9° | 0.396 rad/s | 0.495 |

The plant yaw gain is $k = 0.4954$ without offset: **the physical robot turns
at half the commanded rate.** Straight-line motion over the same 10 s achieves
a gain of about 0.99, so the deficit is purely rotational, consistent with the
wheel scrub described above. The implied effective track is $W/k = 0.309$ m,
about **2.02×** the physical 0.153 m.

The firmware constant must be interpreted accordingly. The 0.159 m
`TRACK_WIDTH` compensates 3.9 % of a deficit of **102 %**: a minor correction
compared with an error two orders of magnitude larger. The sim-to-real yaw gap
is the factor of two, not the 3.9 % constant mismatch.

**No correction is applied on either side.** Correcting the plugin would alter
the plant used for all frozen evaluation results. The full reasoning is given
in [parameters.md §1.4](./parameters.md); the measurement is recorded in the
handover specification of the RL repository (§2.3a).

> Source: the measurement is documented in the companion RL/thesis repository
> (`docs/14_isaacsim_handover_spec.md` §2.3a) and reproduced here as a property
> of this robot.

The following sections use the $\chi = 1$ model implemented in the code.

---

## 4. Forward kinematics

### 4.1 Wheel velocities to robot velocity

Given the wheel angular velocities $\omega_L$ and $\omega_R$:

**Linear velocity**:

$$
v = \frac{r(\omega_R + \omega_L)}{2}
$$

**Angular velocity**:

$$
\omega = \frac{r(\omega_R - \omega_L)}{W}
$$

with $r = 0.03725$ m and $W = 0.154$ m.

### 4.2 Derivation

The linear speed of the robot is the mean of the linear speeds of both sides:

$$
v = \frac{v_R + v_L}{2} = \frac{r\omega_R + r\omega_L}{2} = \frac{r(\omega_R + \omega_L)}{2}
$$

The yaw rate is the difference between the sides divided by their distance:

$$
\omega = \frac{v_R - v_L}{W} = \frac{r\omega_R - r\omega_L}{W} = \frac{r(\omega_R - \omega_L)}{W}
$$

### 4.3 Matrix form

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} =
\begin{bmatrix}
\dfrac{r}{2} & \dfrac{r}{2} \\[6pt]
-\dfrac{r}{W} & \dfrac{r}{W}
\end{bmatrix}
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}
$$

The second row is $-r/W$, $+r/W$: a positive (counter-clockwise) $\omega$
requires the **right** side to turn faster than the left.

With $r = 0.03725$ m and $W = 0.154$ m:

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} =
\begin{bmatrix}
0.018625 & 0.018625 \\[4pt]
-0.24188 & 0.24188
\end{bmatrix}
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}
$$

---

## 5. Inverse kinematics

### 5.1 Robot velocity to wheel velocities

Given a desired $(v, \omega)$:

**Left wheels**:

$$
\omega_L = \frac{v - \omega \cdot \frac{W}{2}}{r}
$$

**Right wheels**:

$$
\omega_R = \frac{v + \omega \cdot \frac{W}{2}}{r}
$$

### 5.2 Derivation

For the body to translate at $v$ while rotating at $\omega$, the contact point
of each side moves at the body velocity plus the rotational contribution of its
lever arm $W/2$:

- Left: $v_L = v - \omega \cdot \frac{W}{2}$
- Right: $v_R = v + \omega \cdot \frac{W}{2}$

Division by the wheel radius converts contact-point speed into wheel angular
speed:

$$
\omega_L = \frac{v_L}{r} = \frac{v - \omega W/2}{r}
\qquad
\omega_R = \frac{v_R}{r} = \frac{v + \omega W/2}{r}
$$

### 5.3 Matrix form

$$
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} =
\begin{bmatrix}
\dfrac{1}{r} & -\dfrac{W}{2r} \\[6pt]
\dfrac{1}{r} & \dfrac{W}{2r}
\end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix}
$$

With $r = 0.03725$ m and $W = 0.154$ m:

$$
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} =
\begin{bmatrix}
26.846 & -2.0671 \\[4pt]
26.846 & 2.0671
\end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix}
$$

This matrix is the inverse of the matrix in §4.3; the $2 \times 2$ forward map
is invertible for any $r > 0$, $W > 0$.

---

## 6. Odometry

### 6.1 Pose integration

The pose in the world frame $\lbrace W \rbrace$ evolves as:

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}_W =
\begin{bmatrix}
v \cos\theta \\
v \sin\theta \\
\omega
\end{bmatrix}
$$

### 6.2 Numerical integration (Euler)

The `OdometryPublisher` plugin publishes at 50 Hz
(`<odom_publish_frequency>50</odom_publish_frequency>`), so $\Delta t = 0.02$ s:

$$
\begin{aligned}
x_{k+1} &= x_k + v \cos\theta_k \cdot \Delta t \\
y_{k+1} &= y_k + v \sin\theta_k \cdot \Delta t \\
\theta_{k+1} &= \theta_k + \omega \cdot \Delta t
\end{aligned}
$$

### 6.3 Odometry publishers

Two plugins produce odometry in simulation; they are not interchangeable:

| Plugin | Topic | Content | Publishes `odom -> base_footprint` on `/tf` |
|---|---|---|---|
| `gz-sim-diff-drive-system` | `/odom` | Dead reckoning from the wheel model above | No; published on `tf_diffdrive` |
| `gz-sim-odometry-publisher-system` | `/odom_truth` | Ground-truth pose from the simulator | Yes; only publisher |

Exactly one node publishes `odom -> base_footprint`. In simulation this is the
ground-truth `OdometryPublisher`; therefore `ekf_gazebo.yaml` sets
`publish_tf: false` and the `DiffDrive` TF uses a separate topic. **On hardware
the EKF publishes the edge** (`ekf_hw.yaml`, `publish_tf: true`). More than one
publisher makes the robot pose jump in RViz.

Since simulation localises against ground truth, the SLAM and Nav2 runs in
simulation do not exercise odometric drift.

### 6.4 Gazebo configuration

Block from `urdf/robot.gazebo`:

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>front_left_wheel_joint</left_joint>
    <left_joint>rear_left_wheel_joint</left_joint>

    <right_joint>front_right_wheel_joint</right_joint>
    <right_joint>rear_right_wheel_joint</right_joint>

    <wheel_separation>0.154</wheel_separation>
    <wheel_radius>0.03725</wheel_radius>

    <max_linear_acceleration>2.5</max_linear_acceleration>
    <min_linear_acceleration>-2.5</min_linear_acceleration>

    <topic>cmd_vel</topic>

    <odom_topic>odom</odom_topic>
    <!-- Dead-reckoning TF on a separate topic: the ground-truth
         OdometryPublisher is the only publisher of odom -> base_footprint
         in simulation. The DiffDrive plugin, the OdometryPublisher and
         the EKF can all publish this transform; with more than one on
         ROS tf, the robot model jumps in RViz. This plugin drives the
         wheels and publishes the encoder odometry. -->
    <tf_topic>tf_diffdrive</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_footprint</child_frame_id>
</plugin>
```

`robot.gazebo` also contains a commented-out Gazebo Fortress (`ignition-*`)
block with `max_linear_acceleration` 0.53 and `min_linear_acceleration` −10.
These values are invalid: 0.53 is the maximum chassis *velocity* in m/s entered
in an acceleration field, and −10 makes braking twenty times stronger than
acceleration. The block is not a valid reference.

---

## 7. Constraints and limits

### 7.1 Limit sets

The robot is limited at two levels with different values.

**Nav2 / DWB planning limits**: `config/nav2_params.yaml`, keys `max_vel_x`,
`min_vel_x`, `max_vel_theta`, `acc_lim_x`, `acc_lim_theta`, `decel_lim_x`,
`decel_lim_theta`; the same values are repeated under `velocity_smoother`:

$$
\begin{aligned}
\lvert v \rvert &\le 0.35 \ \text{m/s} \quad (\text{reverse limited to } 0.15) \\
\lvert \omega \rvert &\le 2.0 \ \text{rad/s} \\
\lvert \dot{v} \rvert &\le 2.5 \ \text{m/s}^2 \\
\lvert \dot{\omega} \rvert &\le 3.2 \ \text{rad/s}^2
\end{aligned}
$$

**Platform saturation limits**: values to which `cobraflex_ros_driver` clamps
every `/cmd_vel` before the firmware (`max_linear`, `max_angular` parameters):

$$
\lvert v \rvert \le 0.53 \ \text{m/s}, \qquad
\lvert \omega \rvert \le 6.0 \ \text{rad/s}
$$

> **The 6.0 rad/s limit is not reachable.** It is the clamp constant of the
> driver, not a measured value. An ideal differential drive would reach
> $2 v_{\max}/W = 2 \times 0.53 / 0.153 = 6.93$ rad/s; with the measured scrub
> factor $k = 0.4954$ (§3.3) the real limit is about **3.4 rad/s**. The
> calibration campaign reached a maximum of 0.396 rad/s.

Nav2 does not approach the platform limit; the driver clamp protects against
any other publisher on `/cmd_vel`. In simulation only the ±2.5 m/s²
acceleration limits are configured in the `DiffDrive` plugin, without a
velocity limit. An unrestricted publisher is therefore unbounded in Gazebo,
unlike on the physical robot.

### 7.2 Wheel limits

Applying §5.1 to each limit set gives the required wheel speeds:

| Case | Formula | Nav2 (0.35, 2.0) | Platform (0.53, 6.0) |
|---|---|---|---|
| Straight, $\omega = 0$ | $v_{\max}/r$ | 9.396 rad/s (89.7 RPM) | 14.228 rad/s (135.9 RPM) |
| In-place rotation, $v = 0$ | $\omega_{\max} W / 2r$ | 4.134 rad/s | 12.403 rad/s |
| Combined | $(v_{\max} + \omega_{\max} W/2)\,/\,r$ | 13.530 rad/s | 26.631 rad/s |

Nav2 column:

$$
\frac{0.35}{0.03725} = 9.396 \ \text{rad/s}
\qquad
\frac{2.0 \times 0.154}{2 \times 0.03725} = 4.134 \ \text{rad/s}
\qquad
\frac{0.35 + 0.154}{0.03725} = 13.530 \ \text{rad/s}
$$

### 7.3 Turning radius

For in-place rotation ($v = 0$, $\omega \neq 0$) the radius is zero; the robot
rotates about its own centre:

$$
R = \frac{v}{\omega} \quad \Rightarrow \quad R_{\min} = 0 \ \text{m}
$$

At both Nav2 limits simultaneously:

$$
R = \frac{v_{\max}}{\omega_{\max}} = \frac{0.35}{2.0} = 0.175 \ \text{m}
$$

This is the tightest arc at full speed, not a lower bound; smaller radii are
reachable at lower speed. It is close to the circumscribed radius of the robot
(0.145 m), so a full-speed turn approximates a pivot.

---

## 8. Computational implementation

### 8.1 Pseudocode: inverse kinematics

```python
def compute_wheel_velocities(v, omega):
    """
    Compute wheel angular velocities from a robot velocity command.

    Args:
        v: Linear velocity [m/s]
        omega: Angular velocity [rad/s]

    Returns:
        omega_left, omega_right: Wheel angular velocities [rad/s]
    """
    r = 0.03725  # wheel_radius
    W = 0.154    # wheel_separation

    # Saturation to the Nav2 / DWB limits (section 7.1). The serial driver
    # applies a wider clamp at 0.53 m/s and 6.0 rad/s.
    v = clip(v, -0.15, 0.35)
    omega = clip(omega, -2.0, 2.0)

    omega_left = (v - omega * W / 2) / r
    omega_right = (v + omega * W / 2) / r

    return omega_left, omega_right
```

Independent saturation of $v$ and $\omega$, as above, corresponds to the
behaviour of the stack. It does not preserve the commanded path curvature:
clipping only one of the two changes $R = v/\omega$. A controller that must
preserve the arc scales both values together.

### 8.2 Pseudocode: forward kinematics

```python
def compute_robot_velocity(omega_left, omega_right):
    """
    Compute robot velocity from measured wheel angular velocities.

    Args:
        omega_left: Left wheel angular velocity [rad/s]
        omega_right: Right wheel angular velocity [rad/s]

    Returns:
        v, omega: Robot linear [m/s] and angular [rad/s] velocity
    """
    r = 0.03725  # wheel_radius
    W = 0.154    # wheel_separation

    v = r * (omega_right + omega_left) / 2
    omega = r * (omega_right - omega_left) / W

    return v, omega
```

### 8.3 Hardware implementation

The firmware runs the same inverse mapping on the ESP32, with its own
constants and a conversion to motor RPM (`Cobra_Driver/movtion_module.h`):

```c
setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);   // left wheel, m/s
setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);   // right wheel, m/s
setpointA = setpointA * 60 / (M_PI * WHEEL_D);   // -> RPM
setpointB = setpointB * 60 / (M_PI * WHEEL_D);
```

with `TRACK_WIDTH` = 0.159 and `WHEEL_D` = 0.0739, instead of the 0.154 and
0.0745 used in the rest of this repository. See §3.3 and
[parameters.md §1.4](./parameters.md).

---

## 9. Cross-references

- [README.md](./README.md): notation and model overview
- [Control.md](./Control.md): plugin configuration and the Nav2 chain
- [parameters.md](./parameters.md): geometry, mass, inertia, sensors and the
  open firmware-constant discrepancy (§1.4)

Source files:

- `src/cobraflex/urdf/my_robot_gazebo.urdf`: geometry
- `src/cobraflex/urdf/robot.gazebo`: `DiffDrive` and odometry plugins
- `src/cobraflex/config/nav2_params.yaml`: planning limits
- `src/cobraflex/cobraflex/cobraflex_ros_driver.py`: platform saturation

---

## 10. Credits and references

### Origin of the model

The structure of this mathematical model (division into kinematics, control and
parameters), its notation and the derivations of sections 4 and 5 are adapted
from **[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)** by
[MrDavidAlv](https://github.com/MrDavidAlv), released under the BSD licence.

Axioma_robot is a ROS 2 Humble autonomous robot on a 4WD skid-steer chassis
with SLAM Toolbox and Nav2, the same platform class and software stack, and the
origin of the idea for this project.

It describes a different chassis:

| Quantity | Axioma_robot | Cobra Flex (this repository) |
|---|---|---|
| Wheel radius $r$ | 0.0381 m | 0.03725 m |
| Effective track | 0.1679 m | 0.154 m |
| Maximum linear speed | 0.26 m/s | 0.35 m/s (Nav2), 0.53 m/s (platform) |

All numerical values in §4.3, §5.3, §7.1 and §7.2 are computed with the
parameters of §2.2.

### External documentation

- [Gazebo `DiffDrive` system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
- [Gazebo: moving a robot](https://gazebosim.org/docs/latest/moving_robot/)
- [Differential drive kinematics, ICC notes, Columbia](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
- Mandow et al., *Experimental kinematics for wheeled skid-steer mobile robots*,
  IROS 2007; standard reference for the ICR / effective-track correction $\chi$
  of §3.3.
