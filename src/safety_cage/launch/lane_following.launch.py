"""
lane_following.launch.py — F2 end-to-end demo launch.

Pipeline:
    Gazebo (lane_following_oval.world + cobraflex robot)
      |  /odom
      v
    lane_perception_node  --(state_obs)-->  pd_baseline_node
                                              |
                                              v  /raw_action (Twist)
                                          cage_ros_node
                                              |
                                              +-- /safe_action (Twist) ---> vehicle_control_node -> /cmd_vel
                                              +-- /cage_status (CageStatus) -> cage_logger_node -> CSV
                                              +-- /emergency (Bool)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cobraflex_share = get_package_share_directory("cobraflex")
    cobraflex_rl_share = get_package_share_directory("cobraflex_rl")

    default_world = os.path.join(
        cobraflex_share, "worlds", "lane_following_oval.world"
    )
    # Right-lane centerline: matches the spawn at y = -0.1225 (centred in
    # the right lane of the two-lane oval). Pass centerline_yaml:= to
    # override, e.g. to track the road centerline (oval_centerline.yaml).
    default_centerline = os.path.join(
        cobraflex_rl_share, "config", "oval_right_lane_centerline.yaml"
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo .world to load (default: oval lane-following).",
    )
    centerline_arg = DeclareLaunchArgument(
        "centerline_yaml",
        default_value=default_centerline,
        description="Centerline YAML consumed by lane_perception and the cage.",
    )
    cage_yaml_arg = DeclareLaunchArgument(
        "cage_yaml",
        default_value="",
        description="Path to cage.yaml. Empty triggers the walk-up search.",
    )
    cage_mode_arg = DeclareLaunchArgument(
        "cage_mode",
        default_value="enforcement",
        description="enforcement | monitoring",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="experiments/sim/runs",
        description="Directory under which the cage logger writes CSV runs.",
    )
    run_id_arg = DeclareLaunchArgument(
        "run_id",
        default_value="",
        description="Override the auto-generated run id.",
    )
    fixed_speed_arg = DeclareLaunchArgument(
        "fixed_speed_mps",
        default_value="0.2",
        description="Straight cruise speed before safe throttle scaling.",
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="lane_following_oval",
        description=(
            "SDF <world name> attribute. Must match the world file's "
            "<world name='...'> tag, NOT the filename (gz service paths "
            "are built from the SDF name). Override when swapping worlds."
        ),
    )

    # Start Gazebo paused so the sim clock is frozen at t≈0 while all
    # ROS2 nodes (EKF, lane_perception, cage) initialise. The unpause
    # is gated on the gz service actually becoming responsive (poll loop
    # below) instead of a fixed grace period, so a slow Gazebo load does
    # not leave the sim paused forever — the previous 4 s hardcoded
    # TimerAction failed silently if Gazebo took longer, producing runs
    # with no /state_obs and a header-only cage_status.csv.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cobraflex_share, "launch", "gazebo_mesh.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "rviz": "false",
            "use_sim_time": "true",
            "start_paused": "true",
        }.items(),
    )

    # Polling unpause: wait 2 s for nodes to wire, then retry the gz
    # service every 0.5 s for up to 30 s. The service appears once
    # Gazebo finishes loading the SDF; querying it sooner returns
    # "Service call failed" without retrying. Loop emits a clear
    # FAILED message on timeout so the operator can diagnose.
    unpause_cmd = (
        "for i in $(seq 1 60); do "
        "  if gz service -s /world/$WORLD_NAME/control "
        "       --reqtype gz.msgs.WorldControl "
        "       --reptype gz.msgs.Boolean "
        "       --timeout 1000 "
        '       --req "pause: false" 2>/dev/null | grep -q "data: true"; then '
        '    echo "[unpause] Gazebo unpaused after $i attempt(s)."; '
        "    exit 0; "
        "  fi; "
        "  sleep 0.5; "
        "done; "
        '  echo "[unpause] FAILED: gz service /world/$WORLD_NAME/control '
        'never responded in 30 s. Check the SDF <world name> matches '
        "WORLD_NAME and that Gazebo started.\"; "
        "exit 1"
    )
    unpause = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", unpause_cmd],
                additional_env={"WORLD_NAME": LaunchConfiguration("world_name")},
                output="screen",
            )
        ],
    )

    perception = Node(
        package="cobraflex_rl",
        executable="lane_perception_node",
        name="lane_perception",
        output="screen",
        parameters=[{
            "centerline_yaml": LaunchConfiguration("centerline_yaml"),
            # Use the EKF-fused estimate instead of raw Gazebo encoder odom.
            # Raw skid-steer odom oscillates ~20 cm between consecutive 50 ms
            # ticks at the curve (~0.07 m/s), producing alternating ey ≈ -0.14
            # and +0.06, which drives C-05 Trigger 7. The EKF fuses IMU +
            # velocities and is substantially smoother.
            "odom_topic": "/odometry/filtered",
            "use_sim_time": True,
        }],
    )

    pd = Node(
        package="cobraflex_rl",
        executable="pd_baseline_node",
        name="pd_baseline",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    cage = Node(
        package="safety_cage",
        executable="cage_ros_node",
        name="safety_cage",
        output="screen",
        parameters=[{
            "cage_yaml": LaunchConfiguration("cage_yaml"),
            "mode": LaunchConfiguration("cage_mode"),
            "use_sim_time": True,
        }],
    )

    vehicle = Node(
        package="cobraflex_rl",
        executable="vehicle_control_node",
        name="vehicle_control",
        output="screen",
        parameters=[{
            "fixed_speed_mps": LaunchConfiguration("fixed_speed_mps"),
            "use_safe_throttle": True,
            "throttle_nominal": 0.5,
            "min_speed_scale": 0.35,
            "use_sim_time": True,
        }],
    )

    logger = Node(
        package="cobraflex_rl",
        executable="cage_logger_node",
        name="cage_logger",
        output="screen",
        parameters=[{
            "output_dir": LaunchConfiguration("output_dir"),
            "run_id": LaunchConfiguration("run_id"),
            "cage_mode": LaunchConfiguration("cage_mode"),
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([
        world_arg,
        centerline_arg,
        cage_yaml_arg,
        cage_mode_arg,
        output_dir_arg,
        run_id_arg,
        fixed_speed_arg,
        world_name_arg,
        gazebo,
        unpause,
        perception,
        pd,
        cage,
        vehicle,
        logger,
    ])
