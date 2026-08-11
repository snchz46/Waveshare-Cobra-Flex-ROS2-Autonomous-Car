"""Minimal launch for lane keeping in Gazebo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_sim = LaunchConfiguration("launch_sim")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    show_debug_windows = LaunchConfiguration("show_debug_windows")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    world = LaunchConfiguration("world")
    image_topic = LaunchConfiguration("image_topic")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("cobraflex"), "launch", "gazebo_mesh.launch.py"]
            )
        ),
        condition=IfCondition(launch_sim),
        launch_arguments={
            "world": world,
            "rviz": "false",
            "gui": gui,
            "use_sim_time": use_sim_time,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    lane_keeper_node = Node(
        package="cobraflex",
        executable="lane_keeper_gazebo_node",
        name="lane_keeper_gazebo_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "show_debug_windows": ParameterValue(
                    show_debug_windows,
                    value_type=bool,
                ),
                "publish_debug_image": ParameterValue(
                    publish_debug_image,
                    value_type=bool,
                ),
            },
            {"image_topic": image_topic},
            # CV-estimator + PD/feedforward controller (see cv_lane_controller).
            {"linear_speed": 0.20},
            {"kp_ey": 6.0},
            {"kd_epsi": 1.6},
            {"kff_curv": 1.0},
            {"max_angular_z": 0.90},
            {"stop_on_no_lane": True},
            {"watchdog_timeout_sec": 1.5},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_lane_keeper_gazebo",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("cobraflex"), "rviz", "lane_keeper.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_sim",
                default_value="true",
                description="Launch Gazebo together with the lane keeper node.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Open the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Open RViz.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "show_debug_windows",
                default_value="false",
                description="Show native OpenCV debug windows (off by default; "
                "use the /lane/* image topics in RViz instead).",
            ),
            DeclareLaunchArgument(
                "publish_debug_image",
                default_value="true",
                description="Publish debug images: /lane/image_overlay "
                "(detections+errors) and /lane/mask (filter).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("cobraflex"), "worlds", "lane_following_oval_complex.world"]
                ),
                description="World file to load in Gazebo.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="camera/image_raw_lane",
                description="Bridged Gazebo image topic.",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="-2.841", #oval: 0.0 a: -3.111 b: -2.8
                description="Robot spawn X position.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-1.623", #oval: -0.1225 a: 1.105 b: -1.5
                description="Robot spawn Y position.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.05",
                description="Robot spawn Z position.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="-0.321", #oval: 0.0 a: 0.755 b: -0.321
                description="Robot spawn yaw in radians.",
            ),
            gazebo_launch,
            lane_keeper_node,
            rviz_node,
        ]
    )
