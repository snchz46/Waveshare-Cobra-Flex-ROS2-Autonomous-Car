# Robot Physical Parameters

Geometry, mass, inertia and sensor configuration of the Waveshare Cobra Flex, as
declared in this repository. Where a figure is measured, assumed or unresolved,
it is labelled — see §1.4, §2.1 and §3.2 in particular.

> **Credit.** This documentation set is adapted from
> [MrDavidAlv/Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot); see
> section 8.

---

## 1. Robot Geometry

### 1.1 Wheel Dimensions

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel radius | $r$ | 0.03725 m | URDF `wheel_radius` |
| Wheel diameter | $d$ | 0.0745 m | 2 × $r$ |
| Wheel width | $w$ | 0.02 m | URDF `wheel_width` |

### 1.2 Chassis Dimensions

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel separation (track) | $W$ | 0.154 m | 2 × `wheel_off_y` |
| Wheelbase (front-to-rear) | $L$ | 0.120 m | 2 × `wheel_off_x` = 2 × 0.060 |
| Chassis length | - | 0.228 m | `chassis_length` |
| Chassis width | - | 0.130 m | `chassis_width` |
| Chassis height | - | 0.060 m | `chassis_height` |

$L$ does not appear in the kinematic equations — an ideal differential drive has
no wheelbase. It appears in [Kinematics.md §3.3](./Kinematics.md), because a gap
between the two axles is exactly what forces this robot to scrub sideways when
it turns.

> **The URDF wheelbase is 22 % short of the real one.** Three values have been
> in circulation: the URDF's `wheel_off_x = ±0.060` → **0.120 m**; an earlier
> revision of this document → **0.1356 m** "(calculated)", which had no source
> and has been removed; and the physical measurement (13.08.2026) →
> **0.154 m**. The table above reports the URDF value, because that is what the
> simulated robot has. Gazebo's `DiffDrive` is unaffected — it is kinematic and
> consumes only `wheel_separation` — so this changes no simulation result, but
> it does mean the modelled chassis is shorter than the real one, and any
> future dynamic model has to use 0.154 m.

**NOTE**: `my_robot_mesh.urdf` and `my_robot_gazebo_mesh.urdf` render the
chassis with custom meshes (`cobraflex_chasis.stl`), but the box dimensions
above are declared in all four URDFs and are what the collision and inertia
macros use.

### 1.3 Wheel Positions

**Coordinates in `base_link` frame**:

| Wheel | Position $(x, y, z)$ [m] | Joint |
|-------|---------------------------|-------|
| Wheel 1 (FL) | (+0.060, +0.077, −0.020) | `front_left_wheel_joint` |
| Wheel 2 (RL) | (−0.060, +0.077, −0.020) | `rear_left_wheel_joint` |
| Wheel 3 (RR) | (−0.060, −0.077, −0.020) | `rear_right_wheel_joint` |
| Wheel 4 (FR) | (+0.060, −0.077, −0.020) | `front_right_wheel_joint` |

The lateral offset is `wheel_off_y` in the URDFs, computed as
`chassis_width/2 + wheel_width/2 + 0.002` = 0.077 m, so the pair is 0.154 m
apart and agrees with §1.2. This table used to read 0.0745, which is the wheel
*diameter* (2 x 0.03725) pasted into the position column, and which implied a
0.149 m separation contradicting §1.2 five lines above.

The $z$ column used to read 0. It is **−0.020** in all four URDFs, and that is
worth a second look rather than a silent correction. `base_joint` already lifts
`base_link` by exactly one wheel radius (0.03725 m) above `base_footprint`,
which by itself would put the axles at axle height; the extra −0.020 m puts them
at 0.01725 m above the `base_footprint` plane, so the wheels reach 0.020 m
*below* it. Planar kinematics uses only $x$ and $y$, so no equation in these
documents depends on it — but `base_footprint` is then not the ground plane its
name promises. Left unchanged pending a check against the CAD.

### 1.4 Firmware kinematic constants — UNRESOLVED disagreement

Waveshare published the Cobra Flex ESP32-S3 firmware source after this model was
built. It declares its own geometry, and it does **not** match §1.1–§1.3:

```c
// Cobra_Driver/ugv_config.h, block labelled "mainType:02 Cobra_Flex"
double WHEEL_D          = 0.0739;   // wheel diameter
double TRACK_WIDTH      = 0.159;
int    ONE_CIRCLE_PLUSES = 32767;   // encoder counts per revolution
```

| Quantity | This model | Measured on the car | Firmware | Firmware vs measured |
|---|---|---|---|---|
| Wheel diameter | 0.0745 (r = 0.03725) | 0.0745 | **0.0739** | −0.8 % |
| Track | 0.154 | **0.153** | **0.159** | **+3.9 %** |

This is not a documentation detail: the firmware uses both constants to
interpret *every* twist we send it, in `rosCtrl` (`Cobra_Driver/movtion_module.h`):

```c
setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);   // left wheel, m/s
setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);   // right wheel, m/s
setpointA = setpointA * 60 / (M_PI * WHEEL_D);   // -> RPM
setpointB = setpointB * 60 / (M_PI * WHEEL_D);
```

So a commanded yaw rate is realised on hardware through 0.159 while Gazebo's
DiffDrive realises it through 0.154. If our figure is the true one the robot
turns ~3.2 % faster than commanded, i.e. a systematic sim-to-real gain error in
exactly the channel a lane-following policy controls. The linear channel is
~0.8 % the other way.

**Nothing has been changed, and the reason is that these are two different
quantities that happen to share a name.** Our 0.154 is geometric, derived from
the URDF (`chassis_width/2 + wheel_width/2 + 0.002`), and the tape says the real
track is 0.153 — so the geometry is right to 0.65 %.

> **Unsettled: where the 0.154 actually came from.** The companion RL/thesis
> repository records a different provenance in `src/cobraflex/urdf/robot.gazebo`
> — that 0.154 is the measured **wheelbase** (0.154 m) rather than the measured
> **track** (0.153 m), and the two agreeing to 0.65 % is a coincidence that
> hid the swap. Both accounts land on the same number and neither changes any
> result, so this is a provenance question, not a numerical one. Worth noting
> that the URDF derivation above yields 0.154 m *exactly* by construction from
> `chassis_width`, which is evidence for the geometric reading — but the two
> repositories should agree on one story. **Not resolved here.** The firmware's 0.159 is not
a geometric claim at all: it is the constant that converts a twist into wheel
RPM, and it sits **3.9 % above the measured track**, which is the direction and
rough magnitude of a scrub compensation. A skid-steer needs a larger wheel-speed
difference than ideal differential kinematics predict, because the wheels drag
sideways; inflating the track constant is the cheapest way to buy some of that
back.

That suggests the correct resolution is *not* to copy 0.159 into everything:

- The **URDF** describes the physical robot. It should carry the measured track
  (0.153, or the present 0.154), never a control constant with scrub baked in.
- The **DiffDrive plugin** in `robot.gazebo` plays the same role in simulation
  that `rosCtrl` plays on hardware — twist in, wheel speeds out. For sim-to-real
  parity *this* is the one that should match the firmware's 0.159.

Today both are 0.154, so Gazebo turns the ideal amount and the car turns through
a constant that is 3.9 % wider — a systematic yaw gain error, in exactly the
channel a lane-following policy controls.

**That measurement has since been made, and it settles the question — against
the firmware.** In-place rotation on the physical car, 10 s per point:

| Commanded | Expected | Measured | Achieved | Gain |
|---|---|---|---|---|
| 0.20 rad/s | 114.6° | 55.6° | 0.097 rad/s | 0.485 |
| 0.40 rad/s | 229.2° | 114.5° | 0.200 rad/s | 0.500 |
| 0.53 rad/s | 303.7° | 150.4° | 0.263 rad/s | 0.495 |
| 0.80 rad/s | 458.4° | 226.9° | 0.396 rad/s | 0.495 |

Least squares through the origin gives **k = 0.4954**, no offset: the car
delivers **half** the commanded yaw rate. Straight-line motion over the same
10 s tracks at ~0.99 (1.998/2.000, 3.964/4.000, 5.207/5.300), so the deficit is
**purely rotational** — the four fixed wheels scrub. The implied effective track
is `0.153 / 0.4954 = 0.309 m`, about **2.02×** the physical track.

So the 0.159 m firmware constant absorbs 3.9 % of a 102 % deficit. It is not the
scrub compensation it looked like — it is a rounding correction against an error
two orders of magnitude larger. Copying 0.159 into the URDF or the plugin would
have bought nothing.

**Still nothing has been changed**, and now for a stronger reason: the honest
correction is not a new track constant but a yaw gain of ~2 somewhere in the
chain, and applying it would perturb the plant that produced every frozen
evaluation result. The measurement is recorded in the companion RL/thesis
repository (`docs/14_isaacsim_handover_spec.md` §2.3a); it is reproduced here
because it is a property of this robot.

One consequence worth carrying forward: **the 6.0 rad/s driver ceiling is not
reachable.** Ideal differential drive gives `2 × 0.53 / 0.153 = 6.93 rad/s`;
with k = 0.4954 the real ceiling is ≈ **3.4 rad/s**, and the calibration
campaign only reached 0.396 rad/s. 6.0 is a clamp constant, not a capability.

### 1.5 Firmware odometry and feedback

From the same source, for whoever wires up wheel odometry later:

| Field | Meaning | Units |
|---|---|---|
| `odl`, `odr` | Cumulative distance per side, `(long int)(en_odom_l * 100)` | **integer centimetres**, monotonic |
| `v` | Battery, `(int)(loadVoltage_V * 100)` | **centivolts** |
| `M1`..`M4` | Per-motor feedback — but `ddsm_fb_*` is never assigned in the shipped build | always 0 |

Two consequences. The odometers are integrated on the ESP32 from encoder counts
(`delta / 32767 * pi * WHEEL_D`, with a 10-count deadband ≈ 0.07 mm), so they
already embed the firmware's `WHEEL_D` — reading them back with a different
wheel diameter double-counts the error in §1.4. And they are truncated to whole
centimetres before transmission: at the deployed 0.22 m/s with the frame rate
limited to 20 Hz (`feedbackFlowExtraDelay = 50`), the robot advances ~11 mm per
frame, so the quantisation is the same size as the signal. Usable for position,
not for speed without filtering.

The IMU fields that `json_cmd.h` documents in this frame, and the whole `T=1002`
frame, are commented out in the shipped build. The chassis carries an ICM-20948,
so it is a recompile away — but nothing arrives today.

---

## 2. Inertial Properties

### 2.1 Total Robot Mass

Measured total of the built robot: **3.5 kg** (bench measurement, 13.08.2026).

#### Bill of materials

Every row is tagged with how the figure was obtained. Only the first block is a
direct measurement of *this* robot.

| Component | Qty | Unit | Total | Provenance |
|---|---|---|---|---|
| PLA shell — bottom (carries the mono lane camera) | 1 | 91.3 g | 91.3 g | **weighed** |
| PLA shell — centre (carries the ZED + powerbank + cables) | 1 | 118.5 g | 118.5 g | **weighed** |
| PLA shell — top cover (carries the LiDAR) | 1 | 68.0 g | 68.0 g | **weighed** |
| *PLA subtotal* | 3 | — | *277.8 g* | **weighed** |
| Powerbank XTPower XT-27000DC | 1 | 550 g | 550 g | datasheet |
| LiDAR RPLIDAR A2 | 1 | 190 g | 190 g | datasheet |
| ZED Mini | 1 | 60 g | 60 g | datasheet |
| Jetson Orin Nano Developer Kit | 1 | 175 g | 175 g | datasheet |
| Mono lane camera (IMX219 CSI) | 1 | ~5 g | ~5 g | estimate |
| Wheels | 4 | 100 g | 400 g | assumption |
| Rolling chassis: frame, 4 motors, driver board, motor battery, wiring, fasteners | 1 | — | **1842.2 g** | **derived remainder** |
| **TOTAL** | | | **3500.0 g** | **measured** |

Accounting of confidence:

| | Mass | Share |
|---|---|---|
| Weighed or from a datasheet | 1252.8 g | 35.8 % |
| Estimated (lane camera) | 5.0 g | 0.1 % |
| Assumed (wheels) | 400.0 g | 11.4 % |
| Derived remainder (rolling chassis) | 1842.2 g | 52.6 % |



#### URDF mass distribution

What the URDFs declare, derived from the bill of materials above:

| Link | Mass | Contents |
|---|---|---|
| `base_link` (chassis) | **2.0172 kg** | frame, 4 motors, driver board, motor battery, Jetson Orin Nano DevKit, wiring |
| `body_link` (upper deck) | **0.8928 kg** | PLA shells 0.2778 + powerbank 0.550 + ZED Mini 0.060 + lane camera ~0.005 |
| `wheel_1…4` | **0.1 kg** ×4 = 0.4 kg | unchanged; not measured |
| `lidar_link` (RPLidar A2) | **0.190 kg** | manufacturer |
| **TOTAL** | **3.5000 kg** | |

The **Jetson rides on the chassis, not in the body** (confirmed by the platform
team, 17.08.2026). The ZED Mini carries no separate inertial — `zed_macro.urdf.xacro`
declares no `<inertial>` element at all — so its 60 g is folded into the body link
it is bolted to. `camera_link` is a frame only.

Three of the four links take their inertia from the `inertial_box` /
`inertial_cylinder` macros at these masses (chassis box 0.228 × 0.130 × 0.060;
wheel cylinder r = 0.03725, l = 0.02; lidar cylinder r = 0.0375, l = 0.04).
**`body_link` does not** — the values below are what the URDFs actually declare:

| Link | ixx | iyy | izz | Origin of the numbers |
|---|---|---|---|---|
| `base_link` | 0.00344605 | 0.00934367 | 0.01157940 | `inertial_box` macro |
| `body_link` | **0.00206253** | **0.00210198** | **0.00359719** | **hand-written, from CAD** |
| wheel (each) | 3.802240e-05 | 3.802240e-05 | 6.937813e-05 | `inertial_cylinder` macro |
| `lidar_link` | 9.213021e-05 | 9.213021e-05 | 1.335938e-04 | `inertial_cylinder` macro |

The `body_link` row previously carried 0.00315456 / 0.00461161 / 0.00627817 —
what `inertial_box` *would* produce for a 0.8928 kg 0.228 × 0.180 × 0.100 box.
That is not what the URDFs contain, and the difference is not small:

| | macro | declared | delta |
|---|---|---|---|
| ixx | 0.00315456 | 0.00206253 | −34.6 % |
| iyy | 0.00461161 | 0.00210198 | −54.4 % |
| izz | 0.00627817 | 0.00359719 | −42.7 % |

`body_link` is the one link whose box model was measurably wrong. An Inventor
assembly with every component at its weighed density (composite 0.908 g/cm³,
PLA shells at the 0.539 g/cm³ that reproduces their measured 277.8 g) puts the
tensor 35–54 % below the macro, because the 550 g powerbank sits compact and low
in the centre shell rather than smeared through the whole 0.228 × 0.180 × 0.100
box. Its inertial origin is `0 0 0.037415`, not `body_height/2`, for the same
reason — the CAD CoG is 12.6 mm below the box centre.

`base_link`, the wheels and the lidar were checked the same way and all land
within 5–6 % of their macro values, so they keep the macros. Do not "finish the
job" by hand-writing those too, and do not restore `inertial_box` on
`body_link`.

Still open: the CAD reports `ixz` = 1.26e−04 for `body_link` (6 % of `ixx`),
left at zero until the CAD's +X direction is confirmed. The macro cannot express
it anyway, and a sign error there couples pitch the wrong way, which is worse
than omitting the term.

#### Centre of gravity — reference frame unresolved

The supplied CoG is `(x, y, z) = (0.006, −0.004, 0.030) m`. Read in `base_link` it
is not reachable by this link layout: the powerbank (550 g) sits in the centre
shell and the LiDAR (190 g) on the top cover, so **740 g — 21 % of the vehicle —
is in the upper two layers**, and the itemised composite lands at **0.0566 m**
above `base_link` (0.0938 m above ground).

Read from the **chassis box centre** — which is itself 0.030 m above `base_link` —
the supplied figure becomes 0.060 m, **3.4 mm from the model**. That is the working
hypothesis for the reference frame, pending confirmation. Until it is confirmed, no
inertial origin has been moved.

### 2.2 Chassis and Body Inertia Tensors

**Box Inertia** :

$$
\mathbf{I}_{base} = \begin{bmatrix}
ixx & 0 & 0 \\
0 & iyy & 0 \\
0 & 0 & izz
\end{bmatrix} \text{ kg·m}^2
$$

```xml
  <xacro:macro name="inertial_box" params="mass x y z *origin">
      <inertial>
          <xacro:insert_block name="origin"/>
          <mass value="${mass}" />
          <inertia ixx="${(1/12) * mass * (y*y+z*z)}" ixy="0.0" ixz="0.0"
                  iyy="${(1/12) * mass * (x*x+z*z)}" iyz="0.0"
                  izz="${(1/12) * mass * (x*x+y*y)}" />
      </inertial>
  </xacro:macro>
```

### 2.3 Wheels and LiDAR Inertia Tensors

**Cylinder Inertia**:

$$
\mathbf{I}_{wheel} = \begin{bmatrix}
ixx & 0 & 0 \\
0 & iyy & 0 \\
0 & 0 & izz
\end{bmatrix} \text{ kg·m}^2
$$

```xml
  <xacro:macro name="inertial_cylinder" params="mass length radius *origin">
      <inertial>
          <xacro:insert_block name="origin"/>
          <mass value="${mass}" />
          <inertia ixx="${(1/12) * mass * (3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
                  iyy="${(1/12) * mass * (3*radius*radius + length*length)}" iyz="0.0"
                  izz="${(1/2) * mass * (radius*radius)}" />
      </inertial>
  </xacro:macro>
```

---

## 3. Operational Limits

### 3.1 Kinematic Limits

The robot is bounded at two levels, and the two used to be merged into one
table here, which is how 0.53 m/s ended up being quoted as if Nav2 could ever
command it.

| Parameter | Symbol | Nav2 / DWB | Platform (driver clamp) |
|-----------|--------|-----------|-------------------------|
| Maximum linear velocity | $v_{max}$ | 0.35 m/s | 0.53 m/s |
| Minimum linear velocity | $v_{min}$ | −0.15 m/s | −0.53 m/s |
| Maximum angular velocity | $\omega_{max}$ | 2.0 rad/s | 6.0 rad/s |
| Maximum linear acceleration | $a_{max}$ | 2.5 m/s² | — |
| Linear deceleration | $a_{min}$ | −2.5 m/s² | — |
| Maximum angular acceleration | $\alpha_{max}$ | 3.2 rad/s² | — |
| Angular deceleration | $\alpha_{min}$ | −3.2 rad/s² | — |

**Nav2 / DWB** comes from `config/nav2_params.yaml` — keys `max_vel_x`,
`min_vel_x`, `max_vel_theta`, `acc_lim_x`, `acc_lim_theta`, `decel_lim_x`,
`decel_lim_theta`, repeated under `velocity_smoother`. It is what the planner
plans inside.

**Platform** is what `cobraflex_ros_driver` clamps every `/cmd_vel` to before it
reaches the firmware (parameters `max_linear`, `max_angular`, symmetric). It is
a backstop against publishers that ignore the platform envelope, not an
operating point — Nav2 never approaches it.

Neither the driver nor the Gazebo plugin limits *angular* acceleration; only
Nav2 does. The resulting wheel speeds for each column are worked out in
[Kinematics.md §7.2](./Kinematics.md).

### 3.2 Acceleration Limits in the Plugin

```xml
<max_linear_acceleration>2.5</max_linear_acceleration>
<min_linear_acceleration>-2.5</min_linear_acceleration>
```

| Parameter | Value |
|-----------|-------|
| Plugin linear acceleration limit | ±2.5 m/s² |

These two used to read `0.53` and `-10` in `urdf/robot.gazebo`: `0.53` is this
chassis's maximum *velocity* in m/s, copied into an acceleration field, and the
`-10` braking limit was twenty times the acceleration limit. Both now match the
Nav2 column in §3.1.

> **Caveat on the replacement.** The 2.5 m/s² that replaced 0.53 has **no
> stated measurement provenance either.** The 13.08.2026 bench sheet
> independently reports "≈ 0.5–0.53 m/s²" for linear acceleration — which is the
> same copied number arriving from a second direction, not a confirmation.
> Treat **0.53 as refuted** and **2.5 as the platform spec**, not as a
> measurement. Deceleration is informed, not closed. This has no practical
> effect on any result recorded so far: at the 0.22 m/s speed cap used for
> lane-following work, commanded acceleration is bounded to 0.22 m/s², an order
> of magnitude under either limit.

**Torque is not configured anywhere.** This section previously also listed a
`<max_wheel_torque>20</max_wheel_torque>` tag and "maximum torque per wheel
20 N·m". That tag appears nowhere in this repository: it belongs to the Gazebo
Classic `libgazebo_ros_diff_drive.so` plugin, whereas this project runs
`gz-sim-diff-drive-system`, which commands wheel *velocity*. The 20 N·m figure
has no source anywhere in the repo, so it has been removed rather than
corrected — there is no measured value to put in its place, and any real limit
would come from the DDSM motor spec, which is not recorded here.

> **The 20 N·m still circulates downstream.** The companion RL/thesis
> repository quotes it — in `src/cobraflex/urdf/robot.gazebo` and in
> `docs/14` — citing *this section* as its source. That citation is now
> circular: it was never sourced here in the first place. It should be dropped
> there too, or replaced with a figure from the DDSM datasheet.


---

## 4. Sensors

### 4.1 LiDAR Specifications


| Parameter | Value |
|-----------|-------|
| Mount | `lidar_joint`, parent `body_link`, $(0, 0, 0.090)$ m, yaw $\pi$ |
| Type | 2D planar laser (`gpu_lidar`) |
| Model | RPLIDAR A2 |
| Samples per scan | 4000 |
| Angular resolution | 0.090° |
| Minimum angle | −180° (−3.14 rad) |
| Maximum angle | +180° (+3.14 rad) |
| Minimum range | 0.015 m |
| Maximum range | 8.0 m |
| Frequency | 10 Hz |
| Noise (mean) | 0.0 |
| Noise (stddev) | 0.01 |

Every row now matches the SDF below it. The table previously read 360 samples
at 1.0° over 0…360°, with a 0.15 m minimum range and a $(0, 0, 0.054)$ mount —
none of which is what the block declares. The mount offset is
`body_height/2 + 0.04` = 0.05 + 0.04 = 0.090 m above `body_link`, and the
$\pi$ yaw means the sensor's zero bearing points **backwards** along $-x$.

The simulated 4000 samples are a Gazebo figure, not a hardware one: a real
RPLIDAR A2 delivers on the order of 400 points per revolution at 10 Hz. A
consumer tuned against the simulated scan density will see roughly a tenth of it
on the real robot.

```xml
<gazebo reference="lidar_link">
    <sensor name="RPLiDAR" type="gpu_lidar">
        <pose relative_to="lidar_link">0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>10</update_rate>
        <topic>scan</topic>
        <gz_frame_id>lidar_link</gz_frame_id>
        <lidar>
            <scan>
                <horizontal>
                <samples>4000</samples>
                <resolution>1</resolution>
                <min_angle>-3.14</min_angle>
                <max_angle>3.14</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.015</min>
                <max>8.0</max>
                <resolution>0.01</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
            <frame_id>/lidar_link</frame_id>
        </lidar>
    </sensor>
</gazebo>
```

### 4.2 Cameras

There are **three** simulated cameras, not one. This section used to describe a
single `ZEDm Cam` on a link called `camera_link`, publishing `camera/image_raw`
— no such link and no such topic exist in `robot.gazebo`.

| | ZED Mini left | ZED Mini right | Lane camera |
|---|---|---|---|
| Sensor name | `ZEDm Left Cam` | `ZEDm Right Cam` | `Lane Cam` |
| Sensor type | `rgbd_camera` | `camera` | `camera` |
| Attached to | `zedm_left_camera_frame` | `zedm_right_camera_frame` | `camera_link_lane` |
| Topic | `camera/left` (prefix) | `camera/right/image_raw` | `camera/image_raw_lane` |
| Resolution | 480 × 270 | 640 × 480 | 640 × 360 |
| Horizontal FOV | 1.7802358 rad (102°) | 1.3962634 rad (80°) | 1.5707963 rad (90°) |
| Clip near / far | 0.1 / 15 m | 0.1 / 15 m | 0.1 / 15 m |
| Rate | 20 Hz | 20 Hz | 20 Hz |
| Noise stddev | 0.007 | 0.007 | 0.007 |

The left eye is an `rgbd_camera`, so its `<topic>` is a **prefix**: gz appends
`/image`, `/depth_image`, `/points` and `/camera_info` to it. It is the only one
of the three that yields depth, and it yields it the way the real ZED Mini does
— computed on the GPU and registered to the left eye — rather than by matching
two rendered images, which is what the SDK's output actually looks like to ROS.
Its FOV and 16:9 aspect are the real camera's; the vertical then falls out near
70° against the real 57°, because a pinhole cannot reproduce a 2.1 mm lens.
480 × 270 rather than a full WVGA 640 × 360 because the reprojection downstream,
not the render, is what costs real time factor.

`camera/left/points` is deliberately **not** bridged to ROS. gz-sensors emits
that cloud in body axes (x forward, y left, z up) while stamping it with the
optical frame, so bridging it lands it in RViz rotated 90°. The cloud is rebuilt
from the depth image and `camera_info` by `zed_depth_cloud.launch.py` instead,
which is also the projection the real ZED SDK performs. The full argument, with
the Gazebo source references, is in `src/cobraflex/config/gz_bridge.yaml`.

**Mounts** (both parented to `body_link`):

| Joint | Child | Origin from `body_link` [m] | Orientation |
|---|---|---|---|
| `zedm_mount_joint` | `zedm_camera_link` | (0.0665, 0, 0.00675) | none |
| `camera_joint_lane` | `camera_link_lane` | (0.124, 0, −0.030) | pitch +0.30 rad, nose down |

`zedm_mount_joint`'s offset is `body_length/2 - 0.0475` in $x$ and
`0.02 - 0.0265/2` in $z$. The stereo baseline is 0.063 m, from the `zedm` branch
of `zed_macro.urdf.xacro`, but the two frames are placed *asymmetrically* about
`zedm_camera_center` — left at $y = +0.0245$, right at $y = -0.0385$. The
separation is the correct 0.063 m; the pair's midpoint just sits 7 mm to the
right of the centre frame. That asymmetry comes from upstream Stereolabs
description and has not been touched.

The lane camera models the real IMX219-160 **as the controller consumes it**,
not as the sensor captures it: `lane_keeper_node` processes 640×360 frames at
20 Hz with an effective 90° horizontal FOV, while the physical capture is
1280×720 at 60 fps. Only the processed stream matters for parity, which is why
the simulated sensor is declared at the processed resolution.

Manufacturer figures for the real ZED Mini — up to 2K resolution, up to 100 fps,
0.1–15 m depth range — describe the hardware, not this simulation, which runs a
plain RGB `camera` sensor with no depth at all.

```xml
<gazebo reference="camera_link_lane">
    <!-- Mirrors the real IMX219-160 as consumed by lane_keeper_node.py
         on HW (proc frames 640x360, effective hfov 90 deg, timer 20 Hz);
         capture is 1280x720@60 but only the processed stream matters. -->
    <sensor name="Lane Cam" type="camera">
        <camera>
            <horizontal_fov>1.5707963</horizontal_fov>
            <image>
                <width>640</width>
                <height>360</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.1</near>
                <far>15</far>
            </clip>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
            <optical_frame_id>camera_link_optical_lane</optical_frame_id>
            <camera_info_topic>camera/camera_info</camera_info_topic>
        </camera>
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <visualize>true</visualize>
        <topic>camera/image_raw_lane</topic>
    </sensor>
</gazebo>
```

### 4.3 IMU

| Parameter | Value |
|-----------|-------|
| Sensor name | `ZEDm IMU` |
| Attached to | `imu_link` |
| Mount | `imu_joint`, parent `body_link`, (0.074, 0, 0.020) m |
| Topic | `imu` |
| Rate | 200 Hz |

The mount offset is `body_length/2 - 0.04` = 0.114 − 0.04 = 0.074 m in $x$.

On hardware there is no equivalent stream. The chassis carries an ICM-20948 and
`json_cmd.h` documents IMU fields in the feedback frame, but they — and the
whole `T=1002` frame — are commented out in the shipped firmware build. See
§1.5.

---

## 5. SLAM Parameters

### 5.1 SLAM Toolbox

There are **two** configurations, and they are not the same. This section used
to cite a single `slam_params.yaml`, which does not exist.

| Parameter | `slam_toolbox_mapping.yaml` (sim) | `slam_toolbox_mapping_hw.yaml` (hardware) |
|---|---|---|
| `mode` | mapping | mapping |
| `base_frame` | `base_footprint` | `base_footprint` |
| `scan_topic` | `/scan` | `/scan` |
| `resolution` | 0.01 m/cell | 0.01 m/cell |
| `max_laser_range` | 8.0 m | 20.0 m |
| `map_update_interval` | 1.0 s | 0.5 s |
| `minimum_time_interval` | 0.5 s | 0.5 s |
| `minimum_travel_distance` | 0.5 m | 0.1 m |
| `minimum_travel_heading` | 0.5 rad (≈28.6°) | 0.1 rad (≈5.7°) |
| `do_loop_closing` | **true** | **false** |
| `loop_search_maximum_distance` | 3.0 m | 3.0 m |
| `loop_match_minimum_chain_size` | 10 | 10 |

The hardware profile takes keyframes five times more often in both distance and
heading, and runs with loop closure **off**. That combination is deliberate for
the small indoor runs this robot does, but it means a hardware map has no
mechanism to correct accumulated drift — the sim profile does.

`max_laser_range: 20.0` on hardware exceeds the RPLIDAR A2's 8 m rated range
(§4.1), so it is not a claim about the sensor; it only affects rastering.

`base_frame` is `base_footprint` in both, matching `ekf_*.yaml`
`base_link_frame` and `nav2_params.yaml` `robot_base_frame`. Naming a different
frame in any one of them gives some link two parents.

---


## 6. Parameter Summary Table

### 6.1 Geometric Parameters

```python
PARAMS_GEOMETRY = {
    'wheel_radius': 0.03725,     # m   urdf: wheel_radius
    'wheel_separation': 0.154,   # m   urdf: 2 * wheel_off_y
    'wheel_base': 0.120,         # m   urdf: 2 * wheel_off_x
    'wheel_width': 0.02,         # m   urdf: wheel_width
    'total_mass': 3.5,           # kg  measured; see section 2.1
}
```

### 6.2 Kinematic Parameters

Two sets, per §3.1 — the Nav2 planning envelope and the driver's clamp:

```python
PARAMS_KINEMATICS_NAV2 = {
    'max_linear_velocity': 0.35,      # m/s     nav2_params: max_vel_x
    'min_linear_velocity': -0.15,     # m/s     nav2_params: min_vel_x
    'max_angular_velocity': 2.0,      # rad/s   nav2_params: max_vel_theta
    'max_linear_acceleration': 2.5,   # m/s^2   nav2_params: acc_lim_x
    'max_angular_acceleration': 3.2,  # rad/s^2 nav2_params: acc_lim_theta
}

PARAMS_KINEMATICS_PLATFORM = {
    'max_linear_velocity': 0.53,      # m/s     driver: max_linear
    'max_angular_velocity': 6.0,      # rad/s   driver: max_angular
}
```

Wheel torque is deliberately absent — see §3.2.

### 6.3 Control Parameters

```python
PARAMS_CONTROL = {
    'odom_publish_rate': 50.0,        # Hz  robot.gazebo: odom_publish_frequency
    'nav2_controller_freq': 20.0,     # Hz  nav2_params: controller_frequency
    'velocity_smoother_freq': 20.0,   # Hz  nav2_params: smoothing_frequency
    'local_costmap_freq': 5.0,        # Hz  nav2_params: local update_frequency
    'global_costmap_freq': 1.0,       # Hz  nav2_params: global update_frequency
    'driver_keepalive_rate': 20.0,    # Hz  driver: 50 ms cmd timer
    'driver_cmd_timeout': 0.5,        # s   driver: cmd_timeout (deadman)
}
```

These blocks previously cited `model.sdf:72`, `model.sdf:441` and
`model.sdf:439`. There is no `model.sdf` in this repository — the plugins live
in `urdf/robot.gazebo`. Line-number citations into `nav2_params.yaml` have been
replaced with key names for the same reason: the line numbers had already
drifted.

---

## 7. Cross-References

- **Kinematics**: [Kinematics.md](./Kinematics.md) uses these geometric
  parameters; §3.3 there picks up the skid-steer scrub question raised in §1.4
- **Control**: [Control.md](./Control.md) uses the limits and frequencies
- **Overview**: [README.md](./README.md)

Source files:

| What | Where |
|---|---|
| Robot descriptions | `src/cobraflex/urdf/my_robot_{basic,mesh,gazebo,gazebo_mesh}.urdf` |
| Inertia macros | `src/cobraflex/urdf/inertial_macros.xacro` |
| Plugins and sensors | `src/cobraflex/urdf/robot.gazebo` |
| ZED description | `src/cobraflex/urdf/zed_macro.urdf.xacro` |
| Nav2 parameters | `src/cobraflex/config/nav2_params.yaml` |
| SLAM parameters | `src/cobraflex/config/slam_toolbox_mapping.yaml`, `..._hw.yaml` |
| EKF parameters | `src/cobraflex/config/ekf_gazebo.yaml`, `ekf_hw.yaml` |
| Serial driver | `src/cobraflex/cobraflex/cobraflex_ros_driver.py` |

---

## 8. Credits

This documentation set is adapted from
**[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)** by
[MrDavidAlv](https://github.com/MrDavidAlv), released under the BSD licence — a
ROS 2 Humble autonomous robot on a 4WD skid-steer chassis with SLAM Toolbox and
Nav2. It is the origin of the idea for this project, and of the way this model
is organised into kinematics, control and parameters.

The two robots are different chassis, so none of the numbers transfer. Every
value in this file has been re-derived from this repository's own URDFs,
configuration files, firmware source and bench measurements; where a figure is
still unverified, it says so rather than being inherited. See
[README.md § Credits](./README.md) and
[Kinematics.md §10](./Kinematics.md).

