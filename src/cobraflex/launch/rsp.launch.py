"""Launch the robot_state_publisher for CobraFlex."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the robot_state_publisher launch description."""
    package_share = FindPackageShare("cobraflex")
    default_urdf = PathJoinSubstitution(
        [package_share, "urdf", "my_robot_gazebo_mesh.urdf"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf = LaunchConfiguration("urdf")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                # value_type=str: robot_description is the xacro-expanded URDF
                # XML; without this, launch tries to YAML-parse it and fails
                # (e.g. on ':' inside comments).
                "robot_description": ParameterValue(
                    Command(["xacro", " ", urdf]), value_type=str
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "urdf",
                default_value=default_urdf,
                description="Path to the robot description file.",
            ),
            robot_state_publisher,
        ]
    )
