"""Launch Navigation2 with RViz for CobraFlex."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Build the Navigation2 launch description."""
    pkg_share = get_package_share_directory("cobraflex")

    rviz_launch_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Open RViz.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="navigation.rviz",
        description="RViz config file.",
    )
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true.",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        # Was hardcoded to another machine's home directory, so this launch
        # file could not work on any fresh clone. Maps saved into the package's
        # own maps/ directory are installed to the share and resolve here; see
        # maps/README.md for the map_saver_cli invocation.
        default_value=os.path.join(pkg_share, "maps", "cobraflex_map.yaml"),
        description="Map YAML for Nav2. Save one with map_saver_cli first.",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        # Without this, nav2_bringup falls back on its own defaults, which are
        # tuned for a TurtleBot3: robot_radius 0.22 m against this robot's real
        # 0.145 m, base_link instead of base_footprint, and unrelated velocity
        # limits. See config/nav2_params.yaml.
        default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
        description="Nav2 parameter file.",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [pkg_share, "rviz", LaunchConfiguration("rviz_config")]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    nav2_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    return LaunchDescription(
        [
            rviz_launch_arg,
            rviz_config_arg,
            map_arg,
            params_arg,
            sim_time_arg,
            rviz_node,
            nav2_toolbox_launch,
        ]
    )
