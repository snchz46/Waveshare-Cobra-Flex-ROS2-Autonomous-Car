"""Launch Gazebo simulation for CobraFlex."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _resolve_world(context):
    """Resolve `world:=` from a short map name or a full path.

    A bare token such as ``oval_zebra`` resolves to
    ``<cobraflex_share>/worlds/lane_following_oval_zebra.world``; a token that
    already starts with ``lane_following_`` is used unchanged under worlds/.
    A value that contains a path separator or ends in .world/.sdf is passed
    through unchanged, which keeps existing callers and the campaign
    orchestrator compatible. Maps are selected with, for example,
    ``world:=oval_no_lines`` without editing the launch file.
    """
    raw = LaunchConfiguration("world").perform(context)
    if os.sep in raw or raw.endswith((".world", ".sdf")):
        return []  # already a path; used unchanged
    share = get_package_share_directory("cobraflex")
    stem = raw if raw.startswith("lane_following_") else f"lane_following_{raw}"
    return [SetLaunchConfiguration("world", os.path.join(share, "worlds", f"{stem}.world"))]


def generate_launch_description():
    """Build the Gazebo simulation launch description."""
    package_name = "cobraflex"
    package_share = get_package_share_directory(package_name)

    world = LaunchConfiguration("world")
    rviz = LaunchConfiguration("rviz")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_paused = LaunchConfiguration("start_paused")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    depth_cloud = LaunchConfiguration("depth_cloud")

    world_path = os.path.join(package_share, "worlds", "lane_following_oval_complex.world")
    urdf_path = os.path.join(package_share, "urdf", "my_robot_gazebo_mesh.urdf")
    bridge_params = os.path.join(package_share, "config", "gz_bridge.yaml")
    rviz_config = os.path.join(package_share, "rviz", "bot.rviz")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "urdf": urdf_path,
        }.items(),
    )

    # start_paused=false → -r flag (simulation runs immediately)
    # start_paused=true  → no -r flag (paused; the caller resumes it)
    gazebo_server_running = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r -s -v1 ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(start_paused),
    )

    gazebo_server_paused = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-s -v1 ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(start_paused),
    )

    gazebo_client = GroupAction(
        condition=IfCondition(gui),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    )
                ),
                launch_arguments={"gz_args": "-g "}.items(),
            )
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "cobraflex_robot",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # /camera/left/points, reprojected from the ZED depth image. Disabled by
    # default here, unlike in gazebo.launch.py: this launch file serves the lane
    # keeper and the RL runs, and the projection uses about a fifth of a CPU
    # core, which reduces the real-time factor of these runs. The depth image is
    # bridged in both cases and remains available in RViz; only the cloud is
    # optional. zed_depth_cloud.launch.py documents why the cloud is rebuilt
    # instead of bridged.
    depth_cloud_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "zed_depth_cloud.launch.py")
        ),
        condition=IfCondition(depth_cloud),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rviz_node = GroupAction(
        condition=IfCondition(rviz),
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            )
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(package_share, "config", "ekf_gazebo.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=world_path,
                description="World to load: full path, or a short map name "
                            "(e.g. world:=oval_zebra -> worlds/lane_following_oval_zebra.world).",
            ),
            OpaqueFunction(function=_resolve_world),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz alongside Gazebo.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Open the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.0",
                description="Robot spawn X position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-1.625",
                description="Robot spawn Y position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "start_paused",
                default_value="false",
                description="Start Gazebo paused; caller is responsible for unpausing.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                # wheel_radius=0.03725 m; a small clearance places the robot
                # just above the ground at spawn, so that the physics settle
                # without a bounce that displaces the robot.
                default_value="0.05",
                description="Robot spawn Z position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
                description="Robot spawn yaw in radians.",
            ),
            DeclareLaunchArgument(
                "depth_cloud",
                default_value="false",
                description="Reproject the ZED depth image into "
                            "/camera/left/points. Off by default here: it "
                            "costs real time factor the lane keeping and RL "
                            "runs need. Turn it on to debug depth in RViz.",
            ),
            rsp,
            gazebo_server_running,
            gazebo_server_paused,
            gazebo_client,
            ros_gz_bridge,
            spawn_robot,
            ekf_node,
            depth_cloud_node,
            rviz_node,
        ]
    )
