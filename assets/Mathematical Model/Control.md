# ⚙️ Robot Control System

## 1. Control Architecture

The robot control system implements a simple Gazebo plugin-based architecture:

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

**File**: `/home/ros2_ws/src/cobraflex/urdf/robot.urdf`

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
        <!-- Dead-reckoning TF is kept off the ROS /tf topic: the ground-truth
             OdometryPublisher is the sole owner of odom -> base_footprint in
             simulation. See urdf/robot.gazebo. -->
        <tf_topic>tf_diffdrive</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_footprint</child_frame_id>
    </plugin>
    
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_frame>odom</odom_frame>
        <robot_base_frame>base_footprint</robot_base_frame>
        <odom_publish_frequency>50</odom_publish_frequency>
        <odom_topic>/odom</odom_topic>
        <tf_topic>/tf</tf_topic>
        <dimensions>2</dimensions>
    </plugin>
  
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
        <topic>joint_states</topic>
        <joint_name>front_left_wheel_joint</joint_name>
        <joint_name>front_right_wheel_joint</joint_name>
        <joint_name>rear_left_wheel_joint</joint_name>
        <joint_name>rear_right_wheel_joint</joint_name>
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
- TF transform: `odom → base_footprint`
- Joint states (via `joint_state_publisher`)

---


## 3. References

**External documentation**:
- [Gazebo Diff Drive](http://gazebosim.org/tutorials?tut=ros2_diff_drive)

