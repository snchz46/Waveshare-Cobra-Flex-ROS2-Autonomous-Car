"""Launch the CobraFlex lane keeper node with optional RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the lane keeper launch description."""
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    show_debug_windows = LaunchConfiguration("show_debug_windows")
    show_control_window = LaunchConfiguration("show_control_window")

    lane_keeper_node = Node(
        package="cobraflex",
        executable="lane_keeper",
        name="lane_keeper",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"lane_side": 1},
            {"linear_speed": 0.12},
            {"angular_gain": 1.30},
            {
                "show_debug_windows": ParameterValue(
                    show_debug_windows,
                    value_type=bool,
                )
            },
            {
                "show_control_window": ParameterValue(
                    show_control_window,
                    value_type=bool,
                )
            },
            {"publish_raw_image": True},
            {"publish_overlay_image": True},
            {"publish_mask_image": True},
            {"publish_histogram_image": True},
            {"publish_camera_info": True},
            {"publish_markers": True},
            {"camera_frame_id": "camera_link_optical"},
            {"marker_frame_id": "base_footprint"},
            {"flip_method": 0},
            {"invert": True},
            {"roi_start_pct": 58},
            {"threshold_val": 145},
            {"trap_top_y_pct": 18},
            {"trap_top_w_pct": 26},
            {"trap_bottom_w_pct": 82},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_lane_keeper",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("cobraflex"), "rviz", rviz_config]
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with a lane-keeper focused layout.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="lane_keeper.rviz",
                description="RViz config file inside the cobraflex/rviz folder.",
            ),
            DeclareLaunchArgument(
                "show_debug_windows",
                default_value="false",
                description="Show OpenCV debug windows alongside RViz.",
            ),
            DeclareLaunchArgument(
                "show_control_window",
                default_value="false",
                description="Show the OpenCV tuning panel with trackbars.",
            ),
            lane_keeper_node,
            rviz_node,
        ]
    )
