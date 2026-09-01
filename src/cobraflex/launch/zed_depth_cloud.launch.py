"""Reproject the simulated ZED Mini depth image into a point cloud.

Shared by gazebo.launch.py and gazebo_mesh.launch.py, and deliberately not
inlined in either. The reason this node exists at all is a frame-convention
trap, and a rationale duplicated across two launch files is a rationale that
drifts.

gz-sensors gives the rgbd_camera's four outputs one single frame_id - whatever
<optical_frame_id> in urdf/robot.gazebo says - but they do not share one axis
convention. image, depth_image and camera_info are optical (z forward, x
right, y down), and have to be, because camera_info carries a pinhole K and
the depth measures along the optical axis. The sensor's own /points is not:
the depth shader ends on

    point = vec3(-vsPos.z, -vsPos.x, vsPos.y)

under the comment "convert to z up" (gz-rendering8, depth_camera_fs.glsl),
which is x forward, y left, z up. Nothing swaps it back - PointCloudUtil::
FillMsg copies x, y, z verbatim - and then RgbdCameraSensor.cc:397 stamps the
cloud with OpticalFrameId() anyway. Bridging it lays the cloud on its side in
RViz, rotated 90 degrees, because the optical frame makes RViz apply the
URDF's -90/0/-90 a second time.

So config/gz_bridge.yaml does not bridge that cloud, and this file rebuilds
one from the depth image and camera_info's intrinsics instead. That lands it
in the optical frame every ROS consumer assumes, and it is the same projection
the ZED SDK performs on the real robot.

Checked against the Gazebo sources rather than assumed: the behaviour is the
same in Fortress (gz-sensors6, RgbdCameraSensor.cc:469) and in Harmonic, and
the shader math is untouched between ign-rendering6 and gz-rendering8 - the
only changes there are the Vulkan/Ogre GLSL syntax migration.

PointCloudXyzrgbNode ships as a component only, with no standalone executable
registered by image_pipeline, hence the container. Its own defaults are what
this needs: approximate sync, because RGB and depth arrive as two messages.

It publishes /points with rclcpp::SensorDataQoS(), i.e. BEST_EFFORT. Any
subscriber asking for RELIABLE never matches it and sits there showing
nothing, which is why the PointCloud2 display in rviz/bot.rviz carries
Reliability Policy: Best Effort. Keep it that way.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Build the depth-image-to-point-cloud launch description."""
    use_sim_time = LaunchConfiguration("use_sim_time")

    container = ComposableNodeContainer(
        name="zedm_depth_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="zedm_points",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("rgb/image_rect_color", "/camera/left/image"),
                    ("rgb/camera_info", "/camera/left/camera_info"),
                    ("depth_registered/image_rect", "/camera/left/depth_image"),
                    ("points", "/camera/left/points"),
                ],
            )
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time if true.",
            ),
            container,
        ]
    )
