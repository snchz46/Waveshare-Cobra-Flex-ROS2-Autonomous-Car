from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    zed_xacro = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf', 'zed_descr.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', zed_xacro])
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.08", "0", "0.13", "0", "0", "0", "base_link", "zed_camera_link"]
        ),
    ])
