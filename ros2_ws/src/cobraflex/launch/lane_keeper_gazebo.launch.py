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
                [FindPackageShare("cobraflex"), "launch", "gazebo.launch.py"]
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
            {"proc_width": 480},
            {"proc_height": 360},
            {"roi_start_pct": 55},
            {"white_sat_max": 70},
            {"white_val_min": 150},
            {"morph_k": 3},
            {"peak_threshold": 6.0},
            {"min_lane_width_px": 60.0},
            {"lane_width_px": 110.0},
            {"linear_speed": 0.10},
            {"angular_gain": 1.25},
            {"max_angular_z": 0.90},
            {"min_linear_scale": 0.35},
            {"single_side_scale": 0.65},
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
                default_value="true",
                description="Show an OpenCV debug window.",
            ),
            DeclareLaunchArgument(
                "publish_debug_image",
                default_value="true",
                description="Publish one debug image on /lane/image_overlay.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("cobraflex"), "worlds", "road_carpet.world"]
                ),
                description="World file to load in Gazebo.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="camera/image_raw",
                description="Bridged Gazebo image topic.",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0",
                description="Robot spawn X position.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="0",
                description="Robot spawn Y position.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.2",
                description="Robot spawn Z position.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0",
                description="Robot spawn yaw in radians.",
            ),
            gazebo_launch,
            lane_keeper_node,
            rviz_node,
        ]
    )
