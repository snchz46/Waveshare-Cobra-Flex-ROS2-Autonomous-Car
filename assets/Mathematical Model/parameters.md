# Robot physical parameters

Geometry, mass, inertia and sensor configuration of the Waveshare Cobra Flex as
declared in this repository. Values are marked as measured, assumed or open
where applicable; see in particular §1.4, §2.1 and §3.2.

> **Credit.** This documentation is adapted from
> [MrDavidAlv/Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot); see
> section 8.

---

## 1. Robot geometry

### 1.1 Wheel dimensions

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel radius | $r$ | 0.03725 m | URDF `wheel_radius` |
| Wheel diameter | $d$ | 0.0745 m | 2 × $r$ |
| Wheel width | $w$ | 0.02 m | URDF `wheel_width` |

### 1.2 Chassis dimensions

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel separation (track) | $W$ | 0.154 m | 2 × `wheel_off_y` |
| Wheelbase (front to rear) | $L$ | 0.120 m | 2 × `wheel_off_x` = 2 × 0.060 |
| Chassis length | - | 0.228 m | `chassis_length` |
| Chassis width | - | 0.130 m | `chassis_width` |
| Chassis height | - | 0.060 m | `chassis_height` |

$L$ does not appear in the kinematic equations, since an ideal differential
drive has no wheelbase. It is relevant in [Kinematics.md §3.3](./Kinematics.md):
the distance between the two axles causes the sideways wheel scrub in turns.

> **URDF wheelbase and physical wheelbase.** The URDF value `wheel_off_x =
> ±0.060` gives **0.120 m**; the physical measurement (13.08.2026) gives
> **0.154 m**, 22 % more. The table reports the URDF value, which is the
> wheelbase of the simulated robot. The Gazebo `DiffDrive` plugin is kinematic
> and uses only `wheel_separation`, so no simulation result is affected. The
> modelled chassis is nevertheless shorter than the physical one, and any
> future dynamic model must use 0.154 m.

`my_robot_mesh.urdf` and `my_robot_gazebo_mesh.urdf` render the chassis with
custom meshes (`cobraflex_chasis.stl`). The box dimensions above are declared
in all four URDFs and are used by the collision and inertia macros.

### 1.3 Wheel positions

**Coordinates in the `base_link` frame**:

| Wheel | Position $(x, y, z)$ [m] | Joint |
|-------|---------------------------|-------|
| Wheel 1 (FL) | (+0.060, +0.077, −0.020) | `front_left_wheel_joint` |
| Wheel 2 (RL) | (−0.060, +0.077, −0.020) | `rear_left_wheel_joint` |
| Wheel 3 (RR) | (−0.060, −0.077, −0.020) | `rear_right_wheel_joint` |
| Wheel 4 (FR) | (+0.060, −0.077, −0.020) | `front_right_wheel_joint` |

The lateral offset `wheel_off_y` is computed in the URDFs as
`chassis_width/2 + wheel_width/2 + 0.002` = 0.077 m; the separation of 0.154 m
is consistent with §1.2.

The vertical offset is **−0.020 m** in all four URDFs. `base_joint` raises
`base_link` by one wheel radius (0.03725 m) above `base_footprint`, which alone
would place the axles at axle height; the additional −0.020 m places them
0.01725 m above the `base_footprint` plane, so the wheels extend 0.020 m below
it. Planar kinematics uses only $x$ and $y$, and no equation in these documents
depends on this offset; `base_footprint` is however not at ground level. The
value is left unchanged pending a check against the CAD.

### 1.4 Firmware kinematic constants (open discrepancy)

The ESP32-S3 firmware source of the Cobra Flex, published by Waveshare,
declares its own geometry, which differs from §1.1–§1.3:

```c
// Cobra_Driver/ugv_config.h, block labelled "mainType:02 Cobra_Flex"
double WHEEL_D          = 0.0739;   // wheel diameter
double TRACK_WIDTH      = 0.159;
int    ONE_CIRCLE_PLUSES = 32767;   // encoder counts per revolution
```

| Quantity | This model | Measured on the robot | Firmware | Firmware vs measured |
|---|---|---|---|---|
| Wheel diameter | 0.0745 (r = 0.03725) | 0.0745 | **0.0739** | −0.8 % |
| Track | 0.154 | **0.153** | **0.159** | **+3.9 %** |

The firmware uses both constants to convert every received twist in `rosCtrl`
(`Cobra_Driver/movtion_module.h`):

```c
setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);   // left wheel, m/s
setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);   // right wheel, m/s
setpointA = setpointA * 60 / (M_PI * WHEEL_D);   // -> RPM
setpointB = setpointB * 60 / (M_PI * WHEEL_D);
```

A commanded yaw rate is therefore realised on hardware with 0.159 m, whereas
the Gazebo `DiffDrive` plugin uses 0.154 m. If 0.154 m is the correct value,
the robot turns about 3.2 % faster than commanded: a systematic sim-to-real
gain error in the channel controlled by a lane-following policy. The linear
channel deviates by about 0.8 % in the opposite direction.

**Interpretation.** The two values are different quantities with the same
name. The value 0.154 m is geometric, derived from the URDF
(`chassis_width/2 + wheel_width/2 + 0.002`); the tape measurement of the track
is 0.153 m, a deviation of 0.65 %.

> **Open item: provenance of 0.154 m.** The companion RL/thesis repository
> records a different provenance in `src/cobraflex/urdf/robot.gazebo`: 0.154 m
> as the measured **wheelbase**, not the measured **track** (0.153 m). Both
> accounts lead to the same value and neither changes any result; the question
> concerns provenance only. The URDF derivation yields exactly 0.154 m from
> `chassis_width`, which supports the geometric interpretation. The two
> repositories have not yet been aligned on this point.

The firmware value of 0.159 m is not a geometric value. It is the constant that
converts a twist into wheel RPM, and it lies **3.9 % above the measured
track**, which corresponds in direction and approximate magnitude to a scrub
compensation. A skid-steer vehicle requires a larger wheel-speed difference
than ideal differential kinematics predicts, because the wheels slide sideways;
an increased track constant compensates part of this difference.

Consequently, 0.159 m is not to be copied into every file:

- The **URDF** describes the physical robot and carries the measured track
  (0.153 m, or the current 0.154 m), not a control constant with scrub
  compensation.
- The **DiffDrive plugin** in `robot.gazebo` has the same role in simulation as
  `rosCtrl` on hardware (twist in, wheel speeds out). For sim-to-real parity,
  this is the component that would match the firmware value of 0.159 m.

Both currently use 0.154 m: Gazebo turns by the ideal amount, and the physical
robot turns with a constant 3.9 % wider.

**Measurement result.** In-place rotation on the physical robot, 10 s per
point:

| Commanded | Expected | Measured | Achieved | Gain |
|---|---|---|---|---|
| 0.20 rad/s | 114.6° | 55.6° | 0.097 rad/s | 0.485 |
| 0.40 rad/s | 229.2° | 114.5° | 0.200 rad/s | 0.500 |
| 0.53 rad/s | 303.7° | 150.4° | 0.263 rad/s | 0.495 |
| 0.80 rad/s | 458.4° | 226.9° | 0.396 rad/s | 0.495 |

Least squares through the origin gives **k = 0.4954** without offset: the
robot achieves **half** the commanded yaw rate. Straight-line motion over the
same 10 s achieves a gain of about 0.99 (1.998/2.000, 3.964/4.000,
5.207/5.300), so the deficit is **purely rotational** and caused by wheel
scrub. The implied effective track is `0.153 / 0.4954 = 0.309 m`, about
**2.02×** the physical track.

The firmware constant of 0.159 m therefore compensates 3.9 % of a deficit of
102 %; it is a minor correction compared with an error two orders of magnitude
larger. Copying 0.159 m into the URDF or the plugin would have no relevant
effect.

**No change is applied.** The appropriate correction is a yaw gain of about 2
in the chain rather than a new track constant, and applying it would alter the
plant used for all frozen evaluation results. The measurement is recorded in
the companion RL/thesis repository (`docs/14_isaacsim_handover_spec.md`
§2.3a) and reproduced here as a property of this robot.

**Consequence: the 6.0 rad/s driver limit is not reachable.** An ideal
differential drive gives `2 × 0.53 / 0.153 = 6.93 rad/s`; with k = 0.4954 the
real limit is about **3.4 rad/s**, and the calibration campaign reached
0.396 rad/s. 6.0 rad/s is a clamp constant, not a capability.

### 1.5 Firmware odometry and feedback

From the same firmware source:

| Field | Meaning | Units |
|---|---|---|
| `odl`, `odr` | Cumulative distance per side, `(long int)(en_odom_l * 100)` | **Integer centimetres**, monotonic |
| `v` | Battery voltage, `(int)(loadVoltage_V * 100)` | **Centivolts** |
| `M1`..`M4` | Per-motor feedback; `ddsm_fb_*` is never assigned in the published build | Always 0 |

Implications:

- The odometers are integrated on the ESP32 from encoder counts
  (`delta / 32767 * pi * WHEEL_D`, with a 10-count deadband ≈ 0.07 mm) and
  therefore already contain the firmware `WHEEL_D`. Reading them with a
  different wheel diameter compounds the error described in §1.4.
- The values are truncated to whole centimetres before transmission. At the
  deployed 0.22 m/s and a frame rate limited to 20 Hz
  (`feedbackFlowExtraDelay = 50`), the robot advances about 11 mm per frame,
  so the quantisation is of the same order as the signal. The odometers are
  usable for position, but not for speed without filtering.

The IMU fields documented in `json_cmd.h` for this frame, and the complete
`T=1002` frame, are commented out in the published build. The chassis carries
an ICM-20948; IMU data is available after recompilation of the firmware, but
not in the current build.

---

## 2. Inertial properties

### 2.1 Total robot mass

Measured total mass of the robot: **3.5 kg** (bench measurement, 13.08.2026).

#### Bill of materials

Each row states the origin of its value. Only the first block is a direct
measurement of this robot.

| Component | Qty | Unit | Total | Origin |
|---|---|---|---|---|
| PLA shell, bottom (carries the mono lane camera) | 1 | 91.3 g | 91.3 g | **Weighed** |
| PLA shell, centre (carries the ZED, powerbank and cables) | 1 | 118.5 g | 118.5 g | **Weighed** |
| PLA shell, top cover (carries the LiDAR) | 1 | 68.0 g | 68.0 g | **Weighed** |
| *PLA subtotal* | 3 | — | *277.8 g* | **Weighed** |
| Powerbank XTPower XT-27000DC | 1 | 550 g | 550 g | Datasheet |
| LiDAR RPLIDAR A2 | 1 | 190 g | 190 g | Datasheet |
| ZED Mini | 1 | 60 g | 60 g | Datasheet |
| Jetson Orin Nano Developer Kit | 1 | 175 g | 175 g | Datasheet |
| Mono lane camera (IMX219 CSI) | 1 | ~5 g | ~5 g | Estimate |
| Wheels | 4 | 100 g | 400 g | Assumption |
| Rolling chassis: frame, 4 motors, driver board, motor battery, wiring, fasteners | 1 | — | **1842.2 g** | **Derived remainder** |
| **TOTAL** | | | **3500.0 g** | **Measured** |

Confidence breakdown:

| | Mass | Share |
|---|---|---|
| Weighed or from a datasheet | 1252.8 g | 35.8 % |
| Estimated (lane camera) | 5.0 g | 0.1 % |
| Assumed (wheels) | 400.0 g | 11.4 % |
| Derived remainder (rolling chassis) | 1842.2 g | 52.6 % |

#### URDF mass distribution

Values declared in the URDFs, derived from the bill of materials:

| Link | Mass | Contents |
|---|---|---|
| `base_link` (chassis) | **2.0172 kg** | Frame, 4 motors, driver board, motor battery, Jetson Orin Nano Developer Kit, wiring |
| `body_link` (upper deck) | **0.8928 kg** | PLA shells 0.2778 + powerbank 0.550 + ZED Mini 0.060 + lane camera ~0.005 |
| `wheel_1…4` | **0.1 kg** ×4 = 0.4 kg | Not measured |
| `lidar_link` (RPLIDAR A2) | **0.190 kg** | Manufacturer |
| **TOTAL** | **3.5000 kg** | |

The **Jetson is mounted on the chassis, not in the body** (confirmed by the
platform team, 17.08.2026). The ZED Mini has no separate inertial element
(`zed_macro.urdf.xacro` declares no `<inertial>`); its 60 g are included in the
body link to which it is mounted. `camera_link` is a frame only.

Three of the four link types take their inertia from the `inertial_box` /
`inertial_cylinder` macros at these masses (chassis box 0.228 × 0.130 × 0.060;
wheel cylinder r = 0.03725, l = 0.02; LiDAR cylinder r = 0.0375, l = 0.04).
**`body_link` is the exception**; the following values are those declared in
the URDFs:

| Link | ixx | iyy | izz | Origin |
|---|---|---|---|---|
| `base_link` | 0.00344605 | 0.00934367 | 0.01157940 | `inertial_box` macro |
| `body_link` | **0.00206253** | **0.00210198** | **0.00359719** | **Hand-written, from CAD** |
| Wheel (each) | 3.802240e-05 | 3.802240e-05 | 6.937813e-05 | `inertial_cylinder` macro |
| `lidar_link` | 9.213021e-05 | 9.213021e-05 | 1.335938e-04 | `inertial_cylinder` macro |

Comparison of the `body_link` tensor with the `inertial_box` values for a
0.8928 kg box of 0.228 × 0.180 × 0.100 m:

| | Macro | Declared | Difference |
|---|---|---|---|
| ixx | 0.00315456 | 0.00206253 | −34.6 % |
| iyy | 0.00461161 | 0.00210198 | −54.4 % |
| izz | 0.00627817 | 0.00359719 | −42.7 % |

`body_link` is the only link whose box model deviates significantly. An
Inventor assembly with every component at its weighed density (composite
0.908 g/cm³; PLA shells at 0.539 g/cm³, which reproduces their measured
277.8 g) places the tensor 35–54 % below the macro, because the 550 g powerbank
is compact and located low in the centre shell instead of being distributed
over the 0.228 × 0.180 × 0.100 m box. For the same reason the inertial origin
is `0 0 0.037415` instead of `body_height/2`: the CAD centre of gravity is
12.6 mm below the box centre.

`base_link`, the wheels and the LiDAR were checked with the same method and are
within 5–6 % of their macro values; they therefore keep the macros. The
hand-written tensor applies to `body_link` only, and `inertial_box` is not to
be restored on `body_link`.

Open item: the CAD reports `ixz` = 1.26e−04 for `body_link` (6 % of `ixx`). It
is set to zero until the +X direction of the CAD is confirmed. The macro cannot
represent the term, and a sign error would couple pitch in the wrong direction,
which is worse than omitting it.

#### Centre of gravity (reference frame open)

The supplied centre of gravity is `(x, y, z) = (0.006, −0.004, 0.030) m`.
Interpreted in `base_link`, this value is not consistent with the link layout:
the powerbank (550 g) is in the centre shell and the LiDAR (190 g) on the top
cover, so **740 g, 21 % of the vehicle, are in the two upper layers**, and the
itemised composite lies **0.0566 m** above `base_link` (0.0938 m above ground).

Interpreted relative to the **chassis box centre**, which is 0.030 m above
`base_link`, the supplied value becomes 0.060 m, **3.4 mm from the model**.
This is the working hypothesis for the reference frame, pending confirmation.
No inertial origin is moved until it is confirmed.

### 2.2 Chassis and body inertia tensors

**Box inertia**:

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

### 2.3 Wheel and LiDAR inertia tensors

**Cylinder inertia**:

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

## 3. Operational limits

### 3.1 Kinematic limits

The robot is limited at two levels:

| Parameter | Symbol | Nav2 / DWB | Platform (driver clamp) |
|-----------|--------|-----------|-------------------------|
| Maximum linear velocity | $v_{max}$ | 0.35 m/s | 0.53 m/s |
| Minimum linear velocity | $v_{min}$ | −0.15 m/s | −0.53 m/s |
| Maximum angular velocity | $\omega_{max}$ | 2.0 rad/s | 6.0 rad/s |
| Maximum linear acceleration | $a_{max}$ | 2.5 m/s² | — |
| Linear deceleration | $a_{min}$ | −2.5 m/s² | — |
| Maximum angular acceleration | $\alpha_{max}$ | 3.2 rad/s² | — |
| Angular deceleration | $\alpha_{min}$ | −3.2 rad/s² | — |

**Nav2 / DWB** values are taken from `config/nav2_params.yaml` (keys
`max_vel_x`, `min_vel_x`, `max_vel_theta`, `acc_lim_x`, `acc_lim_theta`,
`decel_lim_x`, `decel_lim_theta`, repeated under `velocity_smoother`). They
define the planning envelope.

**Platform** values are those to which `cobraflex_ros_driver` clamps every
`/cmd_vel` before the firmware (parameters `max_linear`, `max_angular`,
symmetric). They protect against publishers that ignore the platform envelope
and are not an operating point; Nav2 does not approach them.

Neither the driver nor the Gazebo plugin limits *angular* acceleration; only
Nav2 does. The resulting wheel speeds per column are derived in
[Kinematics.md §7.2](./Kinematics.md).

### 3.2 Acceleration limits in the plugin

```xml
<max_linear_acceleration>2.5</max_linear_acceleration>
<min_linear_acceleration>-2.5</min_linear_acceleration>
```

| Parameter | Value |
|-----------|-------|
| Plugin linear acceleration limit | ±2.5 m/s² |

Both values match the Nav2 column in §3.1.

> **Provenance of 2.5 m/s².** The value has no documented measurement. The
> bench sheet of 13.08.2026 reports "≈ 0.5–0.53 m/s²" for linear
> acceleration, which corresponds to the maximum velocity of 0.53 m/s entered
> as an acceleration and is not an independent confirmation. 0.53 m/s² is
> therefore considered refuted, and 2.5 m/s² is treated as the platform
> specification, not as a measurement. Deceleration is not yet verified. This
> has no practical effect on any recorded result: at the 0.22 m/s speed limit
> of the lane-following work, the commanded acceleration is bounded to
> 0.22 m/s², an order of magnitude below both values.

**Wheel torque is not configured.** The `gz-sim-diff-drive-system` plugin used
in this project commands wheel *velocity*; a `<max_wheel_torque>` tag exists
only in the Gazebo Classic plugin `libgazebo_ros_diff_drive.so`. No torque
value is documented in this repository; a real limit would come from the DDSM
motor specification, which is not recorded here.

> **Open item in the companion repository.** The RL/thesis repository quotes a
> wheel torque of 20 N·m in `src/cobraflex/urdf/robot.gazebo` and in
> `docs/14`, citing this section as its source. This section contains no such
> value; the reference is to be removed there or replaced with a value from
> the DDSM datasheet.

---

## 4. Sensors

### 4.1 LiDAR

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

The mount offset is `body_height/2 + 0.04` = 0.05 + 0.04 = 0.090 m above
`body_link`. The yaw of $\pi$ means that the zero bearing of the sensor points
**backwards** along $-x$.

The 4000 simulated samples are a Gazebo setting, not a hardware value: a
physical RPLIDAR A2 delivers about 400 points per revolution at 10 Hz. A
consumer tuned to the simulated scan density receives about one tenth of it on
the physical robot.

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

Three cameras are simulated:

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

The left camera is an `rgbd_camera`; its `<topic>` is therefore a **prefix**,
to which gz appends `/image`, `/depth_image`, `/points` and `/camera_info`. It
is the only one of the three that provides depth, computed on the GPU and
registered to the left image, as on the physical ZED Mini, rather than by
stereo matching of two rendered images. Its FOV and 16:9 aspect ratio are those
of the physical camera; the resulting vertical FOV is about 70° instead of the
real 57°, because a pinhole model cannot reproduce a 2.1 mm lens. The
resolution is 480 × 270 instead of WVGA 640 × 360 because the downstream
reprojection, not the rendering, determines the real-time factor.

`camera/left/points` is **not** bridged to ROS. gz-sensors emits this cloud in
body axes (x forward, y left, z up) but stamps it with the optical frame; a
bridged cloud appears rotated by 90° in RViz. The cloud is reconstructed from
the depth image and `camera_info` by `zed_depth_cloud.launch.py`, the same
projection performed by the ZED SDK on the physical robot. The detailed
analysis, with references to the Gazebo sources, is in
`src/cobraflex/config/gz_bridge.yaml`.

**Mounts** (both with parent `body_link`):

| Joint | Child | Origin from `body_link` [m] | Orientation |
|---|---|---|---|
| `zedm_mount_joint` | `zedm_camera_link` | (0.0665, 0, 0.00675) | None |
| `camera_joint_lane` | `camera_link_lane` | (0.124, 0, −0.030) | Pitch +0.30 rad, nose down |

The offset of `zedm_mount_joint` is `body_length/2 - 0.0475` in $x$ and
`0.02 - 0.0265/2` in $z$. The stereo baseline is 0.063 m, from the `zedm`
branch of `zed_macro.urdf.xacro`. The two frames are placed asymmetrically
about `zedm_camera_center` (left at $y = +0.0245$, right at $y = -0.0385$):
the separation is the correct 0.063 m, but the midpoint lies 7 mm to the right
of the centre frame. This asymmetry originates in the Stereolabs description
and is left unchanged.

The lane camera models the IMX219-160 **as consumed by the controller**, not as
captured by the sensor: `lane_keeper_node` processes 640×360 frames at 20 Hz
with an effective horizontal FOV of 90°, while the physical capture is
1280×720 at 60 fps. Only the processed stream is relevant for parity; the
simulated sensor is therefore declared at the processed resolution.

The manufacturer data of the ZED Mini (up to 2K resolution, up to 100 fps,
0.1–15 m depth range) describe the hardware; the simulated sensors are
configured as listed in the table above.

```xml
<gazebo reference="camera_link_lane">
    <!-- Models the IMX219-160 as consumed by lane_keeper_node.py on hardware
         (processed frames 640x360, effective hfov 90 deg, timer 20 Hz); the
         1280x720@60 capture is not relevant for parity. -->
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

The hardware provides no equivalent stream. The chassis carries an ICM-20948
and `json_cmd.h` documents IMU fields in the feedback frame, but these fields
and the complete `T=1002` frame are commented out in the published firmware
build. See §1.5.

---

## 5. SLAM parameters

### 5.1 SLAM Toolbox

Two configurations exist:

| Parameter | `slam_toolbox_mapping.yaml` (simulation) | `slam_toolbox_mapping_hw.yaml` (hardware) |
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

The hardware profile adds keyframes five times more often in distance and
heading and disables loop closure. This combination targets the small indoor
runs of this robot; a hardware map therefore has no mechanism to correct
accumulated drift, unlike the simulation profile.

`max_laser_range: 20.0` on hardware exceeds the 8 m rated range of the RPLIDAR
A2 (§4.1); it affects only the rasterisation.

`base_frame` is `base_footprint` in both profiles, consistent with
`ekf_*.yaml` `base_link_frame` and `nav2_params.yaml` `robot_base_frame`. A
different frame in any of these files gives one link two parents.

---

## 6. Parameter summary

### 6.1 Geometric parameters

```python
PARAMS_GEOMETRY = {
    'wheel_radius': 0.03725,     # m   urdf: wheel_radius
    'wheel_separation': 0.154,   # m   urdf: 2 * wheel_off_y
    'wheel_base': 0.120,         # m   urdf: 2 * wheel_off_x
    'wheel_width': 0.02,         # m   urdf: wheel_width
    'total_mass': 3.5,           # kg  measured; see section 2.1
}
```

### 6.2 Kinematic parameters

Two sets according to §3.1, the Nav2 planning envelope and the driver clamp:

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

Wheel torque is not listed; see §3.2.

### 6.3 Control parameters

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

Values reference configuration keys rather than line numbers. The Gazebo
plugins are defined in `urdf/robot.gazebo`.

---

## 7. Cross-references

- **Kinematics**: [Kinematics.md](./Kinematics.md) uses these geometric
  parameters; its §3.3 treats the skid-steer scrub raised in §1.4
- **Control**: [Control.md](./Control.md) uses the limits and frequencies
- **Overview**: [README.md](./README.md)

Source files:

| Content | Location |
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

This documentation is adapted from
**[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)** by
[MrDavidAlv](https://github.com/MrDavidAlv), released under the BSD licence: a
ROS 2 Humble autonomous robot on a 4WD skid-steer chassis with SLAM Toolbox and
Nav2. It is the origin of the idea for this project and of the organisation of
this model into kinematics, control and parameters.

The chassis differ and no values are shared. All values in this file are
derived from the URDFs, configuration files, firmware source and bench
measurements of this repository; unverified values are marked as such. See
[README.md § Credits](./README.md) and [Kinematics.md §10](./Kinematics.md).
