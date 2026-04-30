"""Launch hardware SLAM mapping for CobraFlex."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Build the hardware mapping launch description."""
    pkg_share = get_package_share_directory("cobraflex")


    slam = IncludeLaunchDescription(
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
                "slam_toolbox_mapping_hw.yaml",
            ),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [pkg_share, "rviz", LaunchConfiguration("rviz_config")]
            ),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="mapping_hw.rviz",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            slam,
            rviz,
        ]
    )
