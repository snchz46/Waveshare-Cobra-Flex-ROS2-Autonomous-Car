"""
Lab session 1 - start the two Session 1 nodes with their parameter file.

    ros2 launch cobraflex_lab session1.launch.py                       # Gazebo
    ros2 launch cobraflex_lab session1.launch.py use_sim_time:=false   # car

The two gaps, K and L, are completed as described in Part 5 of the
workbook. The file is copied into the launch directory of the package
cobraflex_lab (Part 2). The launch file and the parameter file are read
from the install space (share/cobraflex_lab), where setup.py copies them
during the build.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare one argument and two nodes."""
    # Installed copy of config/session1.yaml
    params = os.path.join(
        get_package_share_directory(___K___),  # gap K: package (string)
        'config', 'session1.yaml')

    # Value of the argument use_sim_time, known when the launch runs
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true in Gazebo, false on the car'),

        Node(
            package='cobraflex_lab',
            executable='obstacle_monitor',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='cobraflex_lab',
            executable=___L___,  # gap L: executable of the circle driver
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen'),
    ])
