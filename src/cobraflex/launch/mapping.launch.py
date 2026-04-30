"""Launch SLAM mapping with RViz for simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Build the mapping launch description."""
    pkg_share = get_package_share_directory("cobraflex")

    rviz_launch_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Open RViz.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="mapping.rviz",
        description="RViz config file.",
    )
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true.",
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

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "slam_params_file": os.path.join(
                pkg_share,
                "config",
                "slam_toolbox_mapping.yaml",
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            rviz_launch_arg,
            rviz_config_arg,
            sim_time_arg,
            rviz_node,
            slam_toolbox_launch,
        ]
    )
