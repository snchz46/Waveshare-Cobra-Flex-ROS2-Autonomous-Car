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
        default_value="/home/admit14/my_map_save.yaml",
        description="Map file path for Nav2.",
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
        }.items(),
    )

    return LaunchDescription(
        [
            rviz_launch_arg,
            rviz_config_arg,
            map_arg,
            sim_time_arg,
            rviz_node,
            nav2_toolbox_launch,
        ]
    )
