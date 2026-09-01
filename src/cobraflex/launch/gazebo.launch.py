"""Launch Gazebo simulation for CobraFlex."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the Gazebo simulation launch description."""
    package_name = "cobraflex"
    package_share = get_package_share_directory(package_name)

    world = LaunchConfiguration("world")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    depth_cloud = LaunchConfiguration("depth_cloud")

    world_path = os.path.join(package_share, "worlds", "obstacles.world")
    urdf_path = os.path.join(package_share, "urdf", "my_robot_gazebo.urdf")
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

    gazebo_server = IncludeLaunchDescription(
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
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-g "}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "cobraflex_robot",
            "-z",
            "0.2",
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

    # /camera/left/points, reprojected from the depth image rather than
    # bridged straight out of Gazebo. The why is a frame-convention trap and
    # it lives in zed_depth_cloud.launch.py, next to the node it explains.
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
                description="Full path to the world file to load.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz alongside Gazebo.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "depth_cloud",
                default_value="true",
                description="Reproject the ZED depth image into "
                            "/camera/left/points for RViz. Costs roughly a "
                            "fifth of a core, which this stack can afford and "
                            "gazebo_mesh.launch.py cannot.",
            ),
            rsp,
            gazebo_server,
            gazebo_client,
            ros_gz_bridge,
            spawn_robot,
            ekf_node,
            depth_cloud_node,
            rviz_node,
        ]
    )
