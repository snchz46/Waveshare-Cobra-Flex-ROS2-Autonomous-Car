# 4WD Skid-Steer Kinematics

## 1. Introduction

The Cobra Flex is a **4-wheel drive skid-steer** platform. The four wheels are
fixed: there is no steering joint anywhere in the description, so the robot
turns by driving the left pair and the right pair at different speeds, exactly
as a two-wheel differential drive does.

That is why the whole stack — the Gazebo `DiffDrive` plugin, Nav2's DWB
controller, the serial driver — treats it as a differential drive, and why
sections 4 and 5 derive the ideal differential-drive equations.

It is an approximation. A two-wheel differential drive pivots about a point on
the wheel axle and its wheels roll without sliding; a skid-steer with a 0.120 m
wheelbase has no such axle, and it can only turn by **dragging all four wheels
sideways across the ground**. Section 3.3 states what that costs and where the
correction is accounted for.

> **Origin of this document.** The structure and the derivations are adapted
> from [MrDavidAlv/Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot).
> See section 10.

---

## 2. Robot Geometry

### 2.1 Wheel Configuration

The four wheels sit at the corners of a rectangle. **Positions are the wheel
joint origins relative to `base_link`**, from `urdf/my_robot_gazebo.urdf`
(identical in `my_robot_basic.urdf`, `my_robot_mesh.urdf` and
`my_robot_gazebo_mesh.urdf`):

| Wheel | Name | Position $(x, y, z)$ [m] | Joint |
|-------|------|---------------------------|-------|
| W1 | front left | (+0.060, +0.077, −0.020) | `front_left_wheel_joint` |
| W2 | rear left | (−0.060, +0.077, −0.020) | `rear_left_wheel_joint` |
| W3 | rear right | (−0.060, −0.077, −0.020) | `rear_right_wheel_joint` |
| W4 | front right | (+0.060, −0.077, −0.020) | `front_right_wheel_joint` |

Neither offset is typed in directly:

```xml
<xacro:property name="wheel_off_x" value="0.060" />
<xacro:property name="wheel_off_y" value="${(chassis_width/2) + (wheel_width/2) + 0.002}" />
```

With `chassis_width` = 0.130 m and `wheel_width` = 0.020 m, `wheel_off_y`
= 0.065 + 0.010 + 0.002 = **0.077 m**, so the two sides are 0.154 m apart.

> **Note on the −0.020 m.** `base_joint` lifts `base_link` by exactly one wheel
> radius (0.03725 m) above `base_footprint`, which would put the wheel axles at
> axle height; the wheel joints then drop a further 0.020 m, so the axles sit
> 0.01725 m above the `base_footprint` plane and the wheels reach 0.020 m below
> it. Planar kinematics uses only $x$ and $y$, so nothing in this document
> depends on it — but it does mean `base_footprint` is not at ground level.
> Left as-is: that is a description question, not a kinematic one.

### 2.2 Geometric Parameters

As declared in the URDFs and in the `DiffDrive` plugin (`urdf/robot.gazebo`):

```python
wheel_radius     (r) = 0.03725 m
wheel_diameter       = 0.0745  m
wheel_separation (W) = 0.154   m     # track,     2 * wheel_off_y
wheel_base       (L) = 0.120   m     # wheelbase, 2 * wheel_off_x
wheel_width          = 0.02    m
```

Only $r$ and $W$ enter the kinematic equations. $L$ never appears in an ideal
differential-drive model — it shows up in section 3.3, because it is precisely
what makes this robot *not* one.

### 2.3 Coordinate System

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

                |<--------- W = 0.154 m --------->|   track     (left-to-right)
                          L = 0.120 m                 wheelbase (front-to-rear)
```

---

## 3. Kinematic Model

### 3.1 Robot Velocity

The robot's velocity in its own frame $\lbrace R \rbrace$ is:

$$
\mathbf{v}_R = \begin{bmatrix} v \\ \omega \end{bmatrix}
$$

Where:

- $v$ — linear velocity along $x$, forward positive, in m/s
- $\omega$ — angular velocity about $z$, counter-clockwise positive, in rad/s

**The model carries no lateral term.** $v_y$ is not a commandable degree of
freedom: `/cmd_vel.linear.y` is ignored by the plugin and by the serial driver,
and Nav2 is configured with `max_vel_y: 0.0`. That is a *nonholonomic
constraint* on what can be commanded — not a claim that the wheels never slide
sideways. On a skid-steer they do, every time it turns.

### 3.2 Wheel Velocities

The four wheels are commanded as two pairs. The `DiffDrive` plugin declares two
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

Where $\omega_i$ is the angular velocity of wheel $i$ in rad/s.

### 3.3 Skid-Steer vs. Ideal Differential Drive

Sections 4 and 5 assume every wheel rolls without slipping. On this chassis that
assumption breaks whenever $\omega \neq 0$.

An ideal differential drive has one axle, and during a turn its instantaneous
centre of rotation (ICR) lies on that axle, so both wheels roll cleanly. This
robot has two axles 0.120 m apart. Any single ICR is off-axle for both of them,
so the front and rear wheels must **scrub sideways** across the ground for the
turn to happen at all. The rolling constraint is violated by design.

The practical consequence is a **yaw gain error**: the commanded $\omega$ is not
the yaw rate you get. Because part of the wheel-speed difference is lost to
scrub, the real robot under-rotates, and the usual first-order fix is to pretend
the track is wider than it is:

$$
\omega = \frac{r(\omega_R - \omega_L)}{\chi \, W}, \qquad \chi \geq 1
$$

$\chi$ is the **effective-track (ICR) correction factor**. It is not derivable
from geometry alone — it depends on the tyre, the surface and the load — so it
has to be measured.

Where this stands in this repository:

| Source | Track constant | Implied $\chi$ |
|---|---|---|
| Measured on the car (tape) | 0.153 m | — |
| URDF + Gazebo `DiffDrive` | 0.154 m | 1.00 (uncorrected) |
| Waveshare firmware `TRACK_WIDTH` | 0.159 m | ≈ 1.04 |

Simulation therefore turns the *ideal* amount, while the hardware turns through
a constant 3.9 % wider than the measured track — a systematic sim-to-real gain
error in exactly the channel a lane-following policy controls. The reachable yaw
rate on the real robot is known to be roughly half the ideal, so the true $\chi$
is well above 1.04 and the firmware constant absorbs only part of it.

**Nothing has been changed on either side.** Resolving it needs a bench
measurement, not a guess: command $\omega = 1.0$ rad/s with $v = 0$ for 10 s and
measure the angle actually turned. The full argument — including why the URDF
and the plugin should probably *not* end up with the same number — is in
[parameters.md §1.4](./parameters.md).

Everything that follows is the $\chi = 1$ model, which is what the code
currently implements.

---

## 4. Forward Kinematics

### 4.1 From Wheel Velocities to Robot Velocity

Given wheel angular velocities $\omega_L$ and $\omega_R$:

**Linear velocity**:

$$
v = \frac{r(\omega_R + \omega_L)}{2}
$$

**Angular velocity**:

$$
\omega = \frac{r(\omega_R - \omega_L)}{W}
$$

Where $r = 0.03725$ m and $W = 0.154$ m.

### 4.2 Derivation

The robot's linear speed is the average of the two sides' linear speeds:

$$
v = \frac{v_R + v_L}{2} = \frac{r\omega_R + r\omega_L}{2} = \frac{r(\omega_R + \omega_L)}{2}
$$

The yaw rate follows from the difference between the sides, divided by the
distance between them:

$$
\omega = \frac{v_R - v_L}{W} = \frac{r\omega_R - r\omega_L}{W} = \frac{r(\omega_R - \omega_L)}{W}
$$

### 4.3 Matrix Form

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} =
\begin{bmatrix}
\dfrac{r}{2} & \dfrac{r}{2} \\[6pt]
-\dfrac{r}{W} & \dfrac{r}{W}
\end{bmatrix}
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}
$$

The second row is $-r/W$ then $+r/W$, in that order: a positive $\omega$
(counter-clockwise) requires the **right** side to turn faster than the left.

Substituting $r = 0.03725$ m and $W = 0.154$ m:

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} =
\begin{bmatrix}
0.018625 & 0.018625 \\[4pt]
-0.24188 & 0.24188
\end{bmatrix}
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}
$$

---

## 5. Inverse Kinematics

### 5.1 From Robot Velocity to Wheel Velocities

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

For the body to translate at $v$ while rotating at $\omega$, each side's contact
point must move at the body velocity plus the rotational contribution of its own
lever arm $W/2$:

- Left: $v_L = v - \omega \cdot \frac{W}{2}$
- Right: $v_R = v + \omega \cdot \frac{W}{2}$

Dividing by the wheel radius converts contact-point speed into wheel angular
speed:

$$
\omega_L = \frac{v_L}{r} = \frac{v - \omega W/2}{r}
\qquad
\omega_R = \frac{v_R}{r} = \frac{v + \omega W/2}{r}
$$

### 5.3 Matrix Form

$$
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} =
\begin{bmatrix}
\dfrac{1}{r} & -\dfrac{W}{2r} \\[6pt]
\dfrac{1}{r} & \dfrac{W}{2r}
\end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix}
$$

Substituting $r = 0.03725$ m and $W = 0.154$ m:

$$
\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} =
\begin{bmatrix}
26.846 & -2.0671 \\[4pt]
26.846 & 2.0671
\end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix}
$$

This matrix is the exact inverse of the one in §4.3, as it must be — the
$2 \times 2$ forward map is invertible for any $r > 0$, $W > 0$.

---

## 6. Odometry

### 6.1 Pose Integration

The pose in the world frame $\lbrace W \rbrace$ evolves as:

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}_W =
\begin{bmatrix}
v \cos\theta \\
v \sin\theta \\
\omega
\end{bmatrix}
$$

### 6.2 Numerical Integration (Euler)

The `OdometryPublisher` plugin publishes at 50 Hz
(`<odom_publish_frequency>50</odom_publish_frequency>`), so $\Delta t = 0.02$ s:

$$
\begin{aligned}
x_{k+1} &= x_k + v \cos\theta_k \cdot \Delta t \\
y_{k+1} &= y_k + v \sin\theta_k \cdot \Delta t \\
\theta_{k+1} &= \theta_k + \omega \cdot \Delta t
\end{aligned}
$$

### 6.3 Who Publishes What

Two plugins produce odometry in simulation, and they are **not**
interchangeable:

| Plugin | Topic | Content | Owns `odom -> base_footprint` on `/tf`? |
|---|---|---|---|
| `gz-sim-diff-drive-system` | `/odom` | dead reckoning from the wheel model above | **No** — diverted to `tf_diffdrive` |
| `gz-sim-odometry-publisher-system` | `/odom_truth` | ground-truth pose from the simulator | **Yes** — sole owner |

Exactly one node may own `odom -> base_footprint`. In simulation that is the
ground-truth `OdometryPublisher`, which is why `ekf_gazebo.yaml` sets
`publish_tf: false` and the `DiffDrive` TF is diverted. **On hardware the owner
is the EKF instead** (`ekf_hw.yaml`, `publish_tf: true`). Three publishers once
fought over that edge and RViz jumped every cycle.

One consequence is worth stating plainly: because simulation localises against
ground truth, the SLAM and Nav2 runs never exercise odometric drift at all.

### 6.4 Gazebo Configuration

Live block from `urdf/robot.gazebo`:

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
    <!-- Dead-reckoning TF kept off ROS /tf: the ground-truth
         OdometryPublisher is the sole owner of odom -> base_footprint. -->
    <tf_topic>tf_diffdrive</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_footprint</child_frame_id>
</plugin>
```

`robot.gazebo` also carries a commented-out Gazebo Fortress (`ignition-*`) block
with `max_linear_acceleration` 0.53 and `min_linear_acceleration` −10. Those
numbers are dead: 0.53 is the chassis's maximum *velocity* in m/s pasted into an
acceleration field, and −10 made braking twenty times more aggressive than
accelerating. Do not read values out of that block.

---

## 7. Constraints and Limits

### 7.1 Two Different Limit Sets

The robot is bounded twice, at different values, and the two are easy to
confuse.

**Nav2 / DWB planning limits** — `config/nav2_params.yaml`, keys `max_vel_x`,
`min_vel_x`, `max_vel_theta`, `acc_lim_x`, `acc_lim_theta`, `decel_lim_x`,
`decel_lim_theta`; the same figures repeat under `velocity_smoother`:

$$
\begin{aligned}
\lvert v \rvert &\le 0.35 \ \text{m/s} \quad (\text{reverse capped at } 0.15) \\
\lvert \omega \rvert &\le 2.0 \ \text{rad/s} \\
\lvert \dot{v} \rvert &\le 2.5 \ \text{m/s}^2 \\
\lvert \dot{\omega} \rvert &\le 3.2 \ \text{rad/s}^2
\end{aligned}
$$

**Platform saturation limits** — what `cobraflex_ros_driver` clamps every
`/cmd_vel` to before it reaches the firmware (`max_linear`, `max_angular`
parameters):

$$
\lvert v \rvert \le 0.53 \ \text{m/s}, \qquad
\lvert \omega \rvert \le 6.0 \ \text{rad/s}
$$

Nav2 never approaches the platform ceiling; the driver's clamp exists to catch
any *other* publisher on `/cmd_vel`. The simulated `DiffDrive` plugin enforces
only the ±2.5 m/s² acceleration limits — it has no velocity ceiling of its own.

### 7.2 Wheel Limits

Feeding §5.1 with each limit set gives the wheel speeds each one demands:

| Case | Formula | Nav2 (0.35, 2.0) | Platform (0.53, 6.0) |
|---|---|---|---|
| Straight, $\omega = 0$ | $v_{\max}/r$ | 9.396 rad/s (89.7 RPM) | 14.228 rad/s (135.9 RPM) |
| Spin in place, $v = 0$ | $\omega_{\max} W / 2r$ | 4.134 rad/s | 12.403 rad/s |
| Both at once | $(v_{\max} + \omega_{\max} W/2)\,/\,r$ | 13.530 rad/s | 26.631 rad/s |

Worked out, for the Nav2 column:

$$
\frac{0.35}{0.03725} = 9.396 \ \text{rad/s}
\qquad
\frac{2.0 \times 0.154}{2 \times 0.03725} = 4.134 \ \text{rad/s}
\qquad
\frac{0.35 + 0.154}{0.03725} = 13.530 \ \text{rad/s}
$$

### 7.3 Turning Radius

For in-place rotation ($v = 0$, $\omega \neq 0$) the radius is zero — the robot
pivots about its own centre, which is the point of a skid-steer:

$$
R = \frac{v}{\omega} \quad \Rightarrow \quad R_{\min} = 0 \ \text{m}
$$

At both Nav2 limits simultaneously:

$$
R = \frac{v_{\max}}{\omega_{\max}} = \frac{0.35}{2.0} = 0.175 \ \text{m}
$$

That is the *tightest arc while still at full speed*, not a lower bound —
smaller radii are reachable by slowing down. It is close to the robot's own
0.145 m circumscribed radius, so a full-speed turn is very nearly a pivot.

---

## 8. Computational Implementation

### 8.1 Pseudocode: Inverse Kinematics

```python
def compute_wheel_velocities(v, omega):
    """
    Computes wheel angular velocities from a robot velocity command.

    Args:
        v: Linear velocity [m/s]
        omega: Angular velocity [rad/s]

    Returns:
        omega_left, omega_right: Wheel angular velocities [rad/s]
    """
    r = 0.03725  # wheel_radius
    W = 0.154    # wheel_separation

    # Saturate to the Nav2 / DWB limits (section 7.1). The serial driver
    # applies its own, wider clamp at 0.53 m/s and 6.0 rad/s.
    v = clip(v, -0.15, 0.35)
    omega = clip(omega, -2.0, 2.0)

    omega_left = (v - omega * W / 2) / r
    omega_right = (v + omega * W / 2) / r

    return omega_left, omega_right
```

Saturating $v$ and $\omega$ independently, as above, is what the stack actually
does — but it does not preserve the commanded path curvature: clipping only one
of the two changes $R = v/\omega$. A controller that cares about the arc has to
scale both together instead.

### 8.2 Pseudocode: Forward Kinematics

```python
def compute_robot_velocity(omega_left, omega_right):
    """
    Computes robot velocity from measured wheel angular velocities.

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

### 8.3 On Hardware

The firmware runs this same inverse map on the ESP32, with its own constants and
a conversion to motor RPM (`Cobra_Driver/movtion_module.h`):

```c
setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);   // left wheel, m/s
setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);   // right wheel, m/s
setpointA = setpointA * 60 / (M_PI * WHEEL_D);   // -> RPM
setpointB = setpointB * 60 / (M_PI * WHEEL_D);
```

with `TRACK_WIDTH` = 0.159 and `WHEEL_D` = 0.0739 — **not** the 0.154 and 0.0745
used everywhere else in this repository. See §3.3 and
[parameters.md §1.4](./parameters.md).

---

## 9. Cross-References

- [README.md](./README.md) — notation and model overview
- [Control.md](./Control.md) — plugin configuration and the Nav2 chain
- [parameters.md](./parameters.md) — geometry, mass, inertia, sensors, and the
  unresolved firmware-constant disagreement (§1.4)

Source files:

- `src/cobraflex/urdf/my_robot_gazebo.urdf` — geometry
- `src/cobraflex/urdf/robot.gazebo` — `DiffDrive` and odometry plugins
- `src/cobraflex/config/nav2_params.yaml` — planning limits
- `src/cobraflex/cobraflex/cobraflex_ros_driver.py` — platform saturation

---

## 10. Credits and References

### Origin of this model

The structure of this mathematical model — the split into kinematics, control
and parameters, the notation, and the derivations in sections 4 and 5 — is
adapted from **[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)** by
[MrDavidAlv](https://github.com/MrDavidAlv), released under the BSD licence.

Axioma_robot is a ROS 2 Humble autonomous robot on a 4WD skid-steer chassis with
SLAM Toolbox and Nav2 — the same class of platform and the same software stack —
and it is where the idea for this project came from. Its documentation is worth
reading alongside this one.

It describes a **different** chassis, which is worth keeping in mind when reading
this file's history:

| Quantity | Axioma_robot | Cobra Flex (this repo) |
|---|---|---|
| Wheel radius $r$ | 0.0381 m | 0.03725 m |
| Effective track | 0.1679 m | 0.154 m |
| Maximum linear speed | 0.26 m/s | 0.35 m/s (Nav2), 0.53 m/s (platform) |

Earlier revisions of this document had updated section 2 to the Cobra Flex while
§4.3, §5.3, §7.1 and §7.2 still evaluated their formulas with Axioma's
$r = 0.0381$ m and a 0.1725 m track — so the symbolic equations were right and
every number computed from them was wrong. All of them have been recomputed
against the values in §2.2.

### External documentation

- [Gazebo `DiffDrive` system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
- [Gazebo: moving a robot](https://gazebosim.org/docs/latest/moving_robot/)
- [Differential drive kinematics — ICC notes, Columbia](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
- Mandow et al., *Experimental kinematics for wheeled skid-steer mobile robots*,
  IROS 2007 — the standard reference for the ICR / effective-track correction
  $\chi$ discussed in §3.3.
