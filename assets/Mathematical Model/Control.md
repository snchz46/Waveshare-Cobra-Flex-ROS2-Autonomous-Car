# ⚙️ Axioma Robot Control System

> **NOTE**: This document describes the REAL control system implemented in the Axioma robot. There are NO custom PID controllers in the code; the standard Gazebo plugin is used with direct kinematic control.

## 1. Control Architecture

The Axioma robot control system implements a simple Gazebo plugin-based architecture:

```
Navigation Layer (Nav2)
    ↓
    /cmd_vel (Twist)
    ↓
Gazebo Diff Drive Plugin
    ↓
Direct application of wheel velocities
    ↓
Physics simulation (Gazebo)
    ↓
Odometry (/odom) + TF
```

**IMPORTANT**: There is no intermediate PID control layer. The Gazebo plugin applies velocities directly to the wheels based on differential-drive kinematics.

---

## 2. Control Plugin: Gazebo Diff Drive

### 2.1 Configuration

**File**: `/home/axioma/ros2/axioma_humble_ws/src/axioma_description/models/axioma_v2/model.sdf`
**Lines**: 427-454

```xml
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <update_rate>50</update_rate>
  <num_wheel_pairs>2</num_wheel_pairs>

  <!-- Left pair -->
  <left_joint>base_to_wheel1</left_joint>
  <left_joint>base_to_wheel2</left_joint>

  <!-- Right pair -->
  <right_joint>base_to_wheel4</right_joint>
  <right_joint>base_to_wheel3</right_joint>

  <!-- Geometric parameters -->
  <wheel_separation>0.1725</wheel_separation>
  <wheel_diameter>0.0762</wheel_diameter>

  <!-- Limits -->
  <max_wheel_torque>20</max_wheel_torque>
  <max_wheel_acceleration>1.0</max_wheel_acceleration>

  <!-- Odometry -->
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

### 2.2 Plugin Operation

**Input**: `/cmd_vel` topic (type `geometry_msgs/Twist`)
```
linear.x: desired linear velocity [m/s]
angular.z: desired angular velocity [rad/s]
```

**Processing**:
1. Read velocity command from `/cmd_vel`
2. Apply inverse kinematics (see `cinematica.md`):
   - $\\omega_L = \\frac{v - \\omega \\cdot W/2}{r}$
   - $\\omega_R = \\frac{v + \\omega \\cdot W/2}{r}$
3. Apply angular velocities to all 4 wheels (synchronized pairs)
4. Compute odometry from virtual encoders
5. Publish `/odom` and TF `odom → base_link`

**Output**:
- `/odom` topic (type `nav_msgs/Odometry`)
- TF transform: `odom → base_link`
- Joint states (via `joint_state_publisher`)

---

## 3. Control Parameters

### 3.1 Update Frequency

**✅ REAL value**:
```
update_rate: 50 Hz
```
**Source**: `model.sdf:439`

Control period: $\\Delta t = \\frac{1}{50} = 0.02$ s

### 3.2 Torque and Acceleration Limits

**✅ REAL values from `model.sdf:446-447`**:

```
max_wheel_torque: 20 N·m
max_wheel_acceleration: 1.0 m/s²
```

**Maximum torque per wheel**: 20 N·m

**Maximum tractive force** (per wheel):
$$
F_{max} = \\frac{\\tau_{max}}{r} = \\frac{20}{0.0381} = 524.9 \\text{ N}
$$

**Total tractive force** (4 wheels):
$$
F_{total} = 4 \\times 524.9 = 2099.6 \\text{ N}
$$

**Theoretical maximum acceleration**:
$$
a_{max,theory} = \\frac{F_{total}}{m} = \\frac{2099.6}{5.525} = 380.0 \\text{ m/s}^2
$$

**NOTE**: In practice, acceleration is limited by:
1. Plugin: `max_wheel_acceleration = 1.0 m/s²`
2. Nav2: `acc_lim_x = 2.5 m/s²` (`nav2_params.yaml:129`)
3. Friction: $a_{friction} = \\mu \\cdot g = 1.0 \\times 9.81 = 9.81 \\text{ m/s}^2$

The effective limit is **1.0 m/s²** (more conservative).

---

## 4. Nav2 Navigation System

### 4.1 Local Controller: DWB

**File**: `nav2_params.yaml` lines 108-170
**Plugin**: `dwb_core::DWBLocalPlanner`

#### Velocity Limits

**✅ REAL values**:
```yaml
max_vel_x: 0.26 m/s              # line 120
min_vel_x: 0.0 m/s               # line 121
max_vel_theta: 1.0 rad/s         # line 122
min_speed_xy: 0.0 m/s            # line 124
max_speed_xy: 0.26 m/s           # line 125
```

**Command sent to `/cmd_vel`**:
$$
\\begin{aligned}
v &\\in [0, 0.26] \\text{ m/s} \\\\
\\omega &\\in [-1.0, 1.0] \\text{ rad/s}
\\end{aligned}
$$

#### Acceleration Limits

**✅ REAL values**:
```yaml
acc_lim_x: 2.5 m/s²              # line 129
acc_lim_theta: 3.2 rad/s²        # line 130
decel_lim_x: -2.5 m/s²           # line 131
decel_lim_theta: -3.2 rad/s²     # line 132
```

#### Sampling Parameters

**✅ REAL values**:
```yaml
vx_samples: 20                   # line 133
vy_samples: 5                    # line 134 (unused in diff drive)
vtheta_samples: 20               # line 135
sim_time: 1.7 s                  # line 136
```

DWB generates a dynamic window of trajectories:
- 20 linear velocities between $[v_{min}, v_{max}]$
- 20 angular velocities between $[-\\omega_{max}, \\omega_{max}]$
- Simulates each trajectory 1.7 seconds forward
- Selects the best trajectory according to cost functions

#### Cost Functions (Critics)

**✅ REAL configuration** (`nav2_params.yaml:137-154`):

```yaml
critics: ["RotateToGoal", "Oscillation", "BaseObstacle",
          "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

Weights:
  BaseObstacle.scale: 0.02       # Obstacle avoidance
  PathAlign.scale: 32.0          # Align with global path
  GoalAlign.scale: 24.0          # Align with goal
  PathDist.scale: 32.0           # Path proximity
  GoalDist.scale: 24.0           # Goal proximity
  RotateToGoal.scale: 32.0       # Rotate toward goal
```

Total cost function:
$$
J_{total} = \\sum_{i} w_i \\cdot J_i
$$

---

## 5. Velocity Smoother

**File**: `nav2_params.yaml` lines 305-318
**Plugin**: `nav2_velocity_smoother::VelocitySmoother`

### 5.1 Configuration

**✅ REAL values**:
```yaml
smoothing_frequency: 20.0 Hz
feedback: "OPEN_LOOP"

max_velocity: [0.26, 0.0, 1.0]     # [vx, vy, ω]
min_velocity: [-0.26, 0.0, -1.0]

max_accel: [2.5, 0.0, 3.2]         # [ax, ay, α]
max_decel: [-2.5, 0.0, -3.2]
```

### 5.2 Smoothing Filter

The Velocity Smoother applies acceleration constraints to velocity commands:

For each axis (x, θ):
$$
v_{cmd,k} = \\text{clip}\\left(v_{des}, v_{k-1} - a_{max}\\Delta t, v_{k-1} + a_{max}\\Delta t\\right)
$$

Where:
- $v_{des}$: desired velocity from the controller
- $v_{k-1}$: previous velocity
- $a_{max}$: maximum acceleration
- $\\Delta t = 0.05$ s (20 Hz)

---

## 6. Localization: AMCL

**File**: `nav2_params.yaml` lines 6-50
**Plugin**: `nav2_amcl::AmclNode`

### 6.1 Motion Model

**✅ REAL configuration**:
```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"

Model parameters:
  alpha1: 0.05    # Rotation from rotation
  alpha2: 0.05    # Rotation from translation
  alpha3: 0.05    # Translation from translation
  alpha4: 0.05    # Translation from rotation
  alpha5: 0.05    # Additional noise
```

### 6.2 Odometry Noise Model

The differential model assumes odometry error is proportional to motion:

**Rotation error**:
$$
\\sigma_{\\theta_1}^2 = \\alpha_1 \\cdot |\\Delta\\theta| + \\alpha_2 \\cdot \\sqrt{\\Delta x^2 + \\Delta y^2}
$$

**Translation error**:
$$
\\sigma_{trans}^2 = \\alpha_3 \\cdot \\sqrt{\\Delta x^2 + \\Delta y^2} + \\alpha_4 \\cdot |\\Delta\\theta|
$$

**Final rotation error**:
$$
\\sigma_{\\theta_2}^2 = \\alpha_1 \\cdot |\\Delta\\theta| + \\alpha_2 \\cdot \\sqrt{\\Delta x^2 + \\Delta y^2}
$$

---

## 7. Control Flow Diagram

### 7.1 Autonomous Navigation

```
Goal (2D Nav Goal in RViz)
    ↓
Nav2 BT Navigator
    ↓
Global Planner (NavFn) → Global path
    ↓
Local Planner (DWB) → Optimal local trajectory
    ↓
Velocity Smoother → Acceleration smoothing
    ↓
/cmd_vel (Twist)
    ↓
Gazebo Diff Drive Plugin → Inverse kinematics
    ↓
4 Wheels (2 synchronized pairs)
    ↓
Gazebo Physics → Dynamic simulation
    ↓
/odom + TF → Odometry
    ↓
AMCL → Localization with particle filter
    ↓
/amcl_pose → Estimated pose in map
```

### 7.2 Manual Teleoperation

```
Joystick (Xbox controller) / Keyboard
    ↓
joy_node / teleop_twist_keyboard
    ↓
teleop_twist_joy
  - scale_linear.x: 0.5 m/s
  - scale_angular.yaw: 2.0 rad/s
    ↓
/cmd_vel (Twist)
    ↓
[Same flow as navigation]
```

**Source**: `slam_bringup.launch.py:99-107`

---

## 8. Performance Analysis

### 8.1 Response Time

**System frequencies**:
- diff_drive plugin: 50 Hz (20 ms)
- Nav2 DWB controller: 20 Hz (50 ms)
- Velocity smoother: 20 Hz (50 ms)
- AMCL: Variable (triggered by odometry updates)

**Estimated total latency**: ~70-100 ms from command to execution

### 8.2 Odometry Accuracy

Odometry in simulation is **perfect** (noise-free). On the real robot, errors would come from:

1. **Wheel slip** (inherent to skid-steering)
2. **Encoder resolution**: 1000 PPR (mentioned in README, not implemented in simulation)
3. **Wheel diameter variation**

**Expected drift** (based on AMCL parameters):
- Translational: ~5% per meter traveled
- Rotational: ~5% per radian turned

---

## 9. System Limitations

### 9.1 No Real PID Control

The system does NOT implement PID velocity tracking control. The Gazebo plugin applies velocities directly (perfect kinematic control).

**Implication**: On the real robot, you would need:
- PID controller for each motor
- Encoder-based velocity measurement
- Slip compensation

### 9.2 Velocity Saturation

The system has multiple saturation layers:

1. **DWB Controller**: Limits to 0.26 m/s and 1.0 rad/s
2. **Velocity Smoother**: Applies acceleration constraints
3. **Gazebo Plugin**: Limits torque (20 N·m) and acceleration (1.0 m/s²)
4. **Gazebo Physics**: Friction and slip

---

## 10. References

**Implementation**:
- Gazebo plugin: `model.sdf:427-454`
- Nav2 controller: `nav2_params.yaml:108-170`
- Velocity smoother: `nav2_params.yaml:305-318`
- AMCL: `nav2_params.yaml:6-50`

**External documentation**:
- [Gazebo Diff Drive](http://gazebosim.org/tutorials?tut=ros2_diff_drive)
- [Nav2 DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

---

**Author**: Samuel Sanchez
**Date**: 2026
**Version**: 0.2.0
