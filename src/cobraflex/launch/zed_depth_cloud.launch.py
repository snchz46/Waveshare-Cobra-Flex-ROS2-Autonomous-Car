"""Reproject the simulated ZED Mini depth image into a point cloud.

Shared by gazebo.launch.py and gazebo_mesh.launch.py and kept in a separate
file, so that the rationale below exists in one place only.

gz-sensors assigns a single frame_id to the four outputs of the rgbd_camera
(the <optical_frame_id> in urdf/robot.gazebo), but the outputs do not share
one axis convention. image, depth_image and camera_info use the optical
convention (z forward, x right, y down), as required by the pinhole K in
camera_info and by depth measured along the optical axis. The /points output
of the sensor does not: the depth shader ends with

    point = vec3(-vsPos.z, -vsPos.x, vsPos.y)

under the comment "convert to z up" (gz-rendering8, depth_camera_fs.glsl),
i.e. x forward, y left, z up. No later step converts it back
(PointCloudUtil::FillMsg copies x, y, z unchanged), and
RgbdCameraSensor.cc:397 stamps the cloud with OpticalFrameId(). A bridged
cloud therefore appears rotated by 90 degrees in RViz, because the optical
frame makes RViz apply the URDF rotation -90/0/-90 a second time.

config/gz_bridge.yaml therefore does not bridge this cloud; this file
reconstructs it from the depth image and the intrinsics in camera_info. The
result is in the optical frame expected by all ROS consumers and corresponds
to the projection performed by the ZED SDK on the physical robot.

Verified against the Gazebo sources: the behaviour is identical in Fortress
(gz-sensors6, RgbdCameraSensor.cc:469) and in Harmonic, and the shader
mathematics is unchanged between ign-rendering6 and gz-rendering8; the only
changes concern the Vulkan/Ogre GLSL syntax migration.

PointCloudXyzrgbNode is available only as a component (image_pipeline
registers no standalone executable), hence the container. Its defaults match
the requirements: approximate synchronisation, since RGB and depth arrive as
two messages.

The node publishes /points with rclcpp::SensorDataQoS() (BEST_EFFORT). A
subscriber requesting RELIABLE does not match and receives nothing; the
PointCloud2 display in rviz/bot.rviz therefore uses Reliability Policy: Best
Effort.
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
