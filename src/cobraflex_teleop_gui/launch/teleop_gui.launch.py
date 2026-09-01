"""Launch the teleoperation window."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the teleop GUI launch description."""
    teleop_gui = Node(
        package="cobraflex_teleop_gui",
        executable="teleop_gui",
        name="cobraflex_teleop_gui",
        output="screen",
        parameters=[{
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "max_linear": LaunchConfiguration("max_linear"),
            "max_angular": LaunchConfiguration("max_angular"),
            "ui_watchdog": LaunchConfiguration("ui_watchdog"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="cmd_vel",
            description="Where to publish. SI units: this is not the cage's "
                        "/raw_action, which carries normalised throttle and "
                        "steering instead.",
        ),
        DeclareLaunchArgument(
            "max_linear",
            default_value="0.35",
            description="Linear limit in m/s. Matches nav2_params.yaml's "
                        "max_vel_x; the driver clamps harder, at 0.53.",
        ),
        DeclareLaunchArgument(
            "max_angular",
            default_value="2.0",
            description="Angular limit in rad/s. Matches nav2_params.yaml's "
                        "max_vel_theta; the driver clamps at 6.0.",
        ),
        DeclareLaunchArgument(
            "ui_watchdog",
            default_value="0.5",
            description="Seconds without a heartbeat from the window before "
                        "the node commands zero. 0 disables it.",
        ),
        teleop_gui,
    ])
