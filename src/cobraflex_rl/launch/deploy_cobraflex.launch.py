"""
deploy_cobraflex.launch.py — Phase-5 physical bring-up of the track-'E' RL
camera policy behind the safety cage, on the real CobraFlex 1:14.

**STATUS: deployment scaffolding — NOT yet run on hardware.** Wires the same
distributed node chain as the F2 physical demo, now camera-driven and 2-D:

      csi_camera_node       (Jetson CSI → camera/image_raw_lane) [NEW, Phase 5]
      rl_policy_node        (image → CNN → /raw_action)           [NEW, Phase 5]
      cv_lane_estimator_node(image → /state_obs + /perception_invalid, D-43)
      cage_ros_node         (/raw_action + /state_obs → /safe_action + /cage_status)
      vehicle_control_node  (/safe_action → /cmd_vel)
      cobraflex_ros_driver  (/cmd_vel → JSON {"T":13,"X":vx,"Z":wz} over serial)
      cage_logger_node      (/cage_status → CSV evidence)

This is a **Layer-3 controller launch**, matching the platform's own layering:

    Layer 1  cobraflex_bringup.launch.xml   description (robot_state_publisher +
             joint_state_publisher + rviz) + cobraflex_ros_driver (ROS→JSON serial)
    Layer 2  cobraflex_sensors.launch.xml   SLLIDAR A2M8 + ZED Mini + the Jetson
             CSI LANE CAMERA + the ekf (→ /odometry/filtered)  [REQUIRED, see below]
    Layer 3  a controller: cobraflex_lane_keeper.launch.py (classical CV),
             cobraflex_automatic.launch.xml (lidar avoidance), or THIS launch (RL)

Like `cobraflex_lane_keeper.launch.py`, it attaches to a bring-up that is already
running, so the RL policy actuates through exactly the same ROS→JSON serial
interface as every other CobraFlex controller. ``bringup:=true`` includes Layer 1
for a one-shot standalone start; do NOT use it if the bring-up is already up
(Linux does not lock /dev/ttyACM*, so a second driver silently interleaves writes
on the same port).

**Layer 2 is a dependency of this launch**, unlike the classical lane keeper:

  * it publishes the **lane camera** (``csi_camera_node``, 640x360 / 90 deg /
    20 Hz — the Gazebo ``Lane Cam`` mirror). ``camera`` therefore defaults to
    **false** here: nvarguscamerasrc does not share a sensor, so opening it twice
    just fails. Set ``camera:=true`` only when running without Layer 2.
  * it publishes ``/odometry/filtered`` (ekf over the ZED visual odometry), the
    **only source of speed for the cage** on this platform — the chassis driver
    emits ``/cobraflex/wheel_speeds``, not ``nav_msgs/Odometry``, and nothing
    publishes ``/odom``. With no speed the cage still runs, but C-03's TTLC is
    always infinite, C-04 never finds an excess and C-05's high-energy trigger
    can never arm. ``cv_lane_estimator_node`` logs an error if this happens.

The ZED image itself is NOT in the loop — only its odometry, via the ekf. To watch
exactly what the policy sees, put rviz on ``camera_topic``.

``lane_keeper_node`` (the classical CV controller) must NOT run alongside this
chain: it opens the same CSI device and also commands ``/cmd_vel``.

PREREQUISITES (see docs/17_physical_deployment.md):
  * Layer-1 bring-up and Layer-2 sensors running (or ``bringup:=true camera:=true``
    for a standalone start — then the cage runs without speed, see above).
  * Camera extrinsics matched to the trained IPM (pitch 0.30 rad, height 0.077 m)
    OR the D-57 ``perception_heading_bias_rad`` re-calibrated for the real mount.
  * A hardware e-stop wired to /external_stop (Bool) — MANDATORY before first run.

THE DEPLOYED CONTRACT, and why it is spelled out in this file. The defaults below
reproduce the 2-D PPO cap-0.22 550k campaign (D-66/D-69) end to end. Three of them
were WRONG until 2026-08-05 and each was silent — the chain would have run and
driven badly rather than failed:

  1. ``throttle_map`` — ``/raw_action.linear.x`` carries the cage's normalised
     throttle u ∈ [0, 1], not the policy's raw action a ∈ [-1, 1]. The sim applies
     ``cage_bridge.policy_throttle_to_cage`` before the cage; the node now does too.
  2. ``speed_map:=linear_2d`` — the 2-D contract's actuation map (max_speed·u,
     full stop below the deadband, no lower clamp). The node's default
     ``cruise_scaling`` is the frozen 1-D map: it saturates at u = 0.5 and floors
     the speed at 0.35·max, so the cage could never bring the car below 0.077 m/s.
  3. ``control_rate_hz:=10`` — the trained ``control_dt`` is 0.1 s. Inference used
     to run per camera frame (20 Hz), which doubles how often C-06's per-cycle
     ``delta_max_steering_per_cycle`` budget is granted and halves the k=4 frame
     stack's horizon.

And the perception contract: the 550k trunk was trained and scored with the
posterior D-43 estimator (``joint_pair_quadratic``, gain 1.6, T3 temporal window
4), not the frozen GE4 ``near_secant``/1.0 that ``cv_lane_estimator_node``
defaults to. The values come from the ``cage:`` block of
``config/train_ppo_camera_2d_cap022_1M.yaml``. The four ``heading_temporal_*``
tunables in that config are already the ``CvLaneEstimatorConfig`` defaults
(0.08 / 0.03 / 0.3 / 0.32), so they are deliberately NOT restated here — one
source of truth.

KNOWN ITEMS TO VERIFY ON HARDWARE:
  1. The ``steering_to_yaw_rate_gain`` (0.8). It was calibrated against the
     Gazebo DiffDrive plugin's reading of /cmd_vel.angular.z; the firmware's
     ``Z`` in ``{"T":13,...}`` is a nominally equivalent yaw rate on an Ackermann
     chassis, but the two have never been compared on the real car. Re-calibrate
     before trusting the cage's C-02/C-03 margins on hardware.
  2. The driver has NO /cmd_vel watchdog: ``_resend_last_cmd`` re-sends the last
     command every 50 ms as a firmware keep-alive. ``vehicle_control_node``
     covers the cage dying (``safe_action_timeout_s`` → zero Twist), but if
     *vehicle_control_node itself* dies the car keeps driving on the last
     command. The hardware e-stop is the only mitigation — hence MANDATORY.
  3. The 90 deg effective HFOV and the camera height. ``camera_geometry`` assumes
     0.07725 m (the ``my_robot_gazebo_mesh.urdf`` mount); the hardware description
     Layer 1 actually loads, ``my_robot_basic.urdf``, places it 1 cm higher. The
     IPM reads the constant, not the TF, so measure the real mount and reconcile.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_evidence_dir() -> str:
    """``<repo>/experiments/physical/runs`` — the cage CSV's home for a hardware run.

    ``cage_logger_node`` otherwise defaults to ``experiments/sim/runs`` *relative to
    the launching shell's cwd*, which on the car means physical evidence lands in a
    sim path, or wherever the terminal happened to be. Resolved by the same walk-up
    the cage nodes use (``_resolve_cage_yaml``): ``Path(__file__).resolve()`` follows
    the ``--symlink-install`` link back into the repo checkout, so the marker
    ``cage/cage.yaml`` identifies the repo root. Falls back to "" (the node's own
    default) if the launch was copied out of the repo — better a wrong-but-working
    path than a launch that refuses to start.
    """
    for parent in Path(__file__).resolve().parents:
        if (parent / "cage" / "cage.yaml").is_file():
            return str(parent / "experiments" / "physical" / "runs")
    return ""


def generate_launch_description() -> LaunchDescription:
    # Only override the node's own resolution if a cage.yaml is actually installed
    # in this package's share/config. It normally is NOT (cage.yaml lives at the
    # repo root, cage/cage.yaml, and is not a cobraflex_rl data_file), and an empty
    # default is what lets cage_ros_node._resolve_cage_yaml walk up to it. Testing
    # isdir(config) instead of isfile(cage.yaml) would hand the node a path that
    # does not exist — non-empty, so the walk-up never runs and SafetyCageNode
    # raises on load.
    _cage_yaml_installed = os.path.join(
        get_package_share_directory("cobraflex_rl"), "config", "cage.yaml"
    )
    cage_yaml_default = _cage_yaml_installed if os.path.isfile(_cage_yaml_installed) else ""

    args = [
        DeclareLaunchArgument("checkpoint", description="SB3 .zip to deploy (required)."),
        # ppo, not sac: the deployed trunk is the 2-D PPO cap-0.22 550k checkpoint
        # (D-66/D-67) — the SAC 2-D arms are findings, not the verdict of record.
        # A wrong algorithm here is not a soft failure: SB3 refuses to load the zip.
        DeclareLaunchArgument("algorithm", default_value="ppo", description="ppo|sac."),
        DeclareLaunchArgument("cage_yaml", default_value=cage_yaml_default,
                              description="cage.yaml (empty → node default)."),
        DeclareLaunchArgument("mode", default_value="enforcement",
                              description="enforcement (deploy) | monitoring (shadow)."),
        DeclareLaunchArgument("camera_topic", default_value="camera/image_raw_lane"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered",
                              description="speed source for the cage state. The "
                                          "platform's ekf (Layer 2); NOT /odom, which "
                                          "only exists in Gazebo."),
        DeclareLaunchArgument("max_speed_mps", default_value="0.22",
                              description="action.max_speed_mps of the deployed contract."),
        DeclareLaunchArgument("throttle_deadband", default_value="0.05",
                              description="action.throttle_deadband; below this the "
                                          "2-D map commands a full stop."),
        DeclareLaunchArgument("control_rate_hz", default_value="10.0",
                              description="policy + cage cycle rate = 1/control_dt of "
                                          "the training config. NOT the camera's 20 Hz."),
        DeclareLaunchArgument("steering_to_yaw_rate_gain", default_value="0.8",
                              description="normalised steering -> /cmd_vel.angular.z "
                                          "[VERIFY on hardware]."),
        # D-43 perception contract of the deployed trunk (training config `cage:`).
        DeclareLaunchArgument("heading_fit_mode", default_value="joint_pair_quadratic",
                              description="near_secant = frozen GE4; "
                                          "joint_pair_quadratic = the 2-D trunk."),
        DeclareLaunchArgument("heading_gain", default_value="1.6"),
        DeclareLaunchArgument("heading_temporal_window", default_value="4",
                              description="T3 temporal heading gate (0 = disabled)."),
        DeclareLaunchArgument("camera", default_value="false",
                              description="launch csi_camera_node here. Default false: "
                                          "Layer 2 (cobraflex_sensors.launch.xml) owns "
                                          "the CSI camera. true only when running "
                                          "without Layer 2 — never both, the sensor "
                                          "cannot be opened twice."),
        DeclareLaunchArgument("evidence_dir", default_value=_default_evidence_dir(),
                              description="cage_logger_node output_dir; empty = the "
                                          "node's own (sim) default."),
        DeclareLaunchArgument("run_id", default_value="",
                              description="evidence run id; empty = ros_run_<UTC>."),
        DeclareLaunchArgument("sensor_id", default_value="0",
                              description="CSI sensor index."),
        DeclareLaunchArgument("flip_method", default_value="0",
                              description="nvvidconv flip-method for the CSI capture."),
        DeclareLaunchArgument("bringup", default_value="false",
                              description="also launch cobraflex_bringup.launch.xml "
                                          "(description + ROS→JSON serial driver). Leave "
                                          "false when the platform bring-up is already "
                                          "running — two drivers on one serial port is "
                                          "silent corruption, not an error."),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM1",
                              description="chassis serial device (used only with bringup:=true)."),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="rviz in the included bring-up (default off for "
                                          "a deploy run; the platform default is true)."),
    ]
    checkpoint = LaunchConfiguration("checkpoint")
    algorithm = LaunchConfiguration("algorithm")
    cage_yaml = LaunchConfiguration("cage_yaml")
    mode = LaunchConfiguration("mode")
    camera_topic = LaunchConfiguration("camera_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    max_speed = LaunchConfiguration("max_speed_mps")

    # The image source both consumers subscribe to. Frame geometry is NOT
    # parameterised here on purpose: csi_camera_node defaults out_width/out_height
    # from camera_geometry, so the trained 640x360 / 90 deg contract cannot be
    # overridden from a launch file by accident.
    csi_camera = Node(
        package="cobraflex_rl", executable="csi_camera_node", output="screen",
        condition=IfCondition(LaunchConfiguration("camera")),
        parameters=[{
            "image_topic": camera_topic,
            "sensor_id": LaunchConfiguration("sensor_id"),
            "flip_method": LaunchConfiguration("flip_method"),
        }],
    )
    rl_policy = Node(
        package="cobraflex_rl", executable="rl_policy_node", output="screen",
        parameters=[{
            "checkpoint": checkpoint, "algorithm": algorithm,
            "image_topic": camera_topic, "raw_action_topic": "/raw_action",
            # Contract items 1 and 3 of the module docstring.
            "control_rate_hz": LaunchConfiguration("control_rate_hz"),
            "throttle_map": "policy_2d",
        }],
    )
    cv_estimator = Node(
        package="cobraflex_rl", executable="cv_lane_estimator_node", output="screen",
        parameters=[{
            "image_topic": camera_topic, "state_obs_topic": "/state_obs",
            "perception_invalid_topic": "/perception_invalid",
            "odom_topic": LaunchConfiguration("odom_topic"),
            # The supervisor cycles with the cage, not with the camera.
            "publish_rate_hz": LaunchConfiguration("control_rate_hz"),
            "heading_fit_mode": LaunchConfiguration("heading_fit_mode"),
            "heading_gain": LaunchConfiguration("heading_gain"),
            "heading_temporal_window": LaunchConfiguration("heading_temporal_window"),
        }],
    )
    cage = Node(
        package="safety_cage", executable="cage_ros_node", output="screen",
        parameters=[{
            "cage_yaml": cage_yaml, "mode": mode,
            "raw_action_topic": "/raw_action", "state_obs_topic": "/state_obs",
            "safe_action_topic": "/safe_action", "cage_status_topic": "/cage_status",
            "perception_invalid_topic": "/perception_invalid",
            "external_stop_topic": "/external_stop",
        }],
    )
    vehicle_control = Node(
        package="cobraflex_rl", executable="vehicle_control_node", output="screen",
        parameters=[{
            "safe_action_topic": "/safe_action", "cmd_vel_topic": cmd_vel_topic,
            # Contract item 2: the 2-D map, with fixed_speed_mps == max_speed_mps.
            "use_safe_throttle": True, "fixed_speed_mps": max_speed,
            "speed_map": "linear_2d",
            "throttle_deadband": LaunchConfiguration("throttle_deadband"),
            "steering_to_yaw_rate_gain": LaunchConfiguration(
                "steering_to_yaw_rate_gain"),
        }],
    )
    # cage_mode is passed explicitly: the logger stamps it into every CSV row and
    # its own default is "enforcement", so a monitoring run would otherwise write
    # evidence labelled as enforcement.
    cage_logger = Node(
        package="cobraflex_rl", executable="cage_logger_node", output="screen",
        parameters=[{
            "cage_status_topic": "/cage_status", "state_obs_topic": "/state_obs",
            "cage_mode": mode,
            "output_dir": LaunchConfiguration("evidence_dir"),
            "run_id": LaunchConfiguration("run_id"),
        }],
    )
    # Layer 1 of the platform's own bring-up (robot_state_publisher +
    # joint_state_publisher + rviz, then the ROS→JSON serial driver), included
    # verbatim so the RL policy actuates through exactly the same interface as
    # the PD/CV controllers. OFF by default: the platform convention is that a
    # controller launch attaches to a bring-up that is already running (cf.
    # cobraflex_lane_keeper.launch.py). Enabling it while cobraflex_bringup is
    # already up would start a SECOND cobraflex_ros_driver on the same serial
    # device — Linux does not lock /dev/ttyACM*, so both would interleave JSON
    # writes and keep-alives undetected.
    # The path is a *substitution*, not get_package_share_directory(): the latter
    # resolves while the launch description is being built, i.e. even under
    # bringup:=false, which would make the whole Layer-3 RL launch unusable on a
    # workspace where `cobraflex` is not built. FindPackageShare is evaluated only
    # when the (conditional) include is actually visited.
    platform_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("cobraflex"), "launch", "cobraflex_bringup.launch.xml",
        ])),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("bringup")),
    )

    return LaunchDescription(args + [
        platform_bringup, csi_camera, rl_policy, cv_estimator, cage,
        vehicle_control, cage_logger,
    ])
