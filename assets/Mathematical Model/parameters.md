# 📏 Robot Physical Parameters

---

## 1. Robot Geometry

### 1.1 Wheel Dimensions

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Wheel radius | $r$ | 0.03725 m |
| Wheel diameter | $d$ | 0.0745 m |
| Wheel width | $w$ | 0.02 m |

### 1.2 Chassis Dimensions

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Wheel separation | $W$ | 0.154 m |
| Estimated length | - | ~0.228 m |
| Estimated width | - | ~0.130 m |

**NOTE**: The chassis uses custom meshes (`cobraflex_chasis.stl`), there are no explicit box dimensions.

### 1.3 Wheel Positions

**Coordinates in `base_link` frame**:

| Wheel | Position $(x, y, z)$ [m] | Joint |
|-------|---------------------------|-------|
| Wheel 1 (FL) | (0.06, 0.0745, 0) | `front_left_wheel_joint` |
| Wheel 2 (RL) | (-0.06, 0.0745, 0) | `rear_left_wheel_joint` |
| Wheel 3 (RR) | (-0.06, -0.0745, 0) | `rear_right_wheel_joint` |
| Wheel 4 (FR) | (0.06, -0.0745, 0) | `front_right_wheel_joint` |

---

## 2. Inertial Properties

### 2.1 Total Robot Mass

Measured total of the built robot: **3.5 kg**. The breakdown below is what the
URDFs actually declare — `chassis_mass` and `body_mass` in
`src/cobraflex/urdf/my_robot_*.urdf`, the wheel macro, and the LiDAR link.

| Component | Quantity | Unit Mass | Total Mass |
|-----------|----------|-----------|------------|
| Base Link (chassis, motors, drivetrain) | 1 | 2.20 kg | 2.20 kg |
| Body Link (upper deck, Jetson, LiPo, wiring) | 1 | 0.71 kg | 0.71 kg |
| Wheels (`wheel_1...4`) | 4 | 0.10 kg | 0.40 kg |
| LiDAR (`lidar_link`, RPLidar A2) | 1 | 0.19 kg | 0.19 kg |
| **TOTAL** | - | - | **3.50 kg** |

The ZED Mini (~62 g) carries no separate inertial in the URDF; its mass is
folded into the body link it is bolted to. `camera_link` is a frame only.


### 2.2 Chassis, Body and Camera Inertia Tensors

**Box Inertia** :

$$
\\mathbf{I}_{base} = \\begin{bmatrix}
ixx & 0 & 0 \\\\
0 & iyy & 0 \\\\
0 & 0 & izz
\\end{bmatrix} \\text{ kg·m}^2
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
\\mathbf{I}_{wheel} = \\begin{bmatrix}
ixx & 0 & 0 \\\\
0 & iyy & 0 \\\\
0 & 0 & izz
\\end{bmatrix} \\text{ kg·m}^2
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


| Parameter | Value |
|-----------|-------|
| Maximum linear velocity | $v_{max} = 0.53$ m/s |
| Maximum angular velocity | $\\omega_{max} = 6.0$ rad/s |
| Maximum linear acceleration | $a_{max} = 2.5$ m/s² |
| Maximum angular acceleration | $\\alpha_{max} = 3.2$ rad/s² |
| Linear deceleration | $a_{min} = -2.5$ m/s² |
| Angular deceleration | $\\alpha_{min} = -3.2$ rad/s² |

### 3.2 Torque Limits

```xml
<max_wheel_torque>20</max_wheel_torque>
<max_linear_acceleration>2.5</max_linear_acceleration>
<min_linear_acceleration>-2.5</min_linear_acceleration>
```

| Parameter | Value |
|-----------|-------|
| Maximum torque per wheel | 20 N·m |
| Plugin linear acceleration limit | ±2.5 m/s² |

These last two used to read `0.53` and `-10` in `urdf/robot.gazebo`: `0.53` is
this chassis's maximum *velocity* in m/s, copied into an acceleration field,
and the `-10` braking limit was twenty times the acceleration limit. Both now
match the kinematic limits in §3.1.


---

## 4. Sensors

### 4.1 LiDAR Specifications


| Parameter | Value |
|-----------|-------|
| Position on robot | $(0, 0, 0.054)$ m from `body_link` |
| Type | 2D planar laser |
| Model | `rplidar_a2m8` |
| Samples per scan | 360 |
| Angular resolution | 1.0° |
| Minimum angle | 0° (0 rad) |
| Maximum angle | 360° (6.28 rad) |
| Minimum range | 0.15 m |
| Maximum range | 8.0 m |
| Frequency | 10 Hz |
| Noise (mean) | 0.0 |
| Noise (stddev) | 0.01 |

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

### 4.2 Camera Specifications


| Parameter | Value |
|-----------|-------|
| Position on robot | $(0, 0, 0.054)$ m from `body_link` |
| Type | Stereo Depth Camera |
| Model | `zed mini` |
| Video Output Resolution | up to 2K |
| Video Output FPS | up to 100 FPS |
| Minimum Depth Range | 0.1 m |
| Maximum Depth Range | 15 m |
| Frequency | 20 Hz |
| Noise (mean) | 0.0 |
| Noise (stddev) | 0.007 |

```xml
<gazebo reference="camera_link">
    <sensor name="ZEDm Cam" type="camera">
        <camera>
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
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
            <optical_frame_id>camera_link_optical</optical_frame_id>
            <camera_info_topic>camera/camera_info</camera_info_topic>
        </camera>
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <visualize>true</visualize>
        <topic>camera/image_raw</topic>            
    </sensor>
</gazebo>
```

---

## 5. SLAM Parameters

### 5.1 SLAM Toolbox

**✅ REAL values from `slam_params.yaml`**:

```yaml
resolution: 0.01 m/cell
max_laser_range: 8 m
minimum_time_interval: 0.5 s
minimum_travel_distance: 0.5 m
minimum_travel_heading: 0.5 rad (≈28.6°)

Loop closure:
  do_loop_closing: true
  loop_search_maximum_distance: 3.0 m
  loop_match_minimum_chain_size: 10
```
---


## 6. Parameter Summary Table

### 6.1 Geometric Parameters

```python
PARAMS_GEOMETRY = {
    'wheel_radius': 0.03725,      # m (model.sdf:72)
    'wheel_separation': 0.154,  # m (model.sdf:441)
    'wheel_base': 0.1356,        # m (calculated)
    'total_mass': 3.5            # kg (measured; see section 2.1)
}
```

### 6.2 Kinematic Parameters

```python
PARAMS_KINEMATICS = {
    'max_linear_velocity': 0.53,      # m/s 
    'max_angular_velocity': 6.0,      # rad/s 
    'max_linear_acceleration': 2.5,   # m/s² 
    'max_angular_acceleration': 3.2,  # rad/s²
    'max_wheel_torque': 20.0          # N·m 
```

### 6.3 Control Parameters

```python
PARAMS_CONTROL = {
    'gazebo_update_rate': 50.0,       # Hz (model.sdf:439)
    'nav2_controller_freq': 20.0,     # Hz (nav2_params:114)
    'velocity_smoother_freq': 20.0,   # Hz (nav2_params:307)
    'costmap_update_freq': 5.0        # Hz (nav2_params:173)
}
```

---

## 7. Cross-References

- **Kinematics**: [Kinematics.md](./Kinematics.md) uses these geometric parameters
- **Control**: [Control.md](./Control.md) uses limits and frequencies
- **Source files**:
  - Main model: `src/cobraflex/urdf/robot.urdf`
  - Nav2 parameters: `src/cobraflex/config/nav2_params.yaml`
  - SLAM parameters: `src/cobraflex/config/slam_params.yaml`

