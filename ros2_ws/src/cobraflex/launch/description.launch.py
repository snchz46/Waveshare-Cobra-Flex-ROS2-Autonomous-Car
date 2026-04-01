import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'cobraflex'
    pkg_share = get_package_share_directory(pkg_name)

    # ROBOT STATE PUBLISHER (RSP) 
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # JOINT STATE PUBLISHER GUI

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'bot.rviz')],
        output='screen'
    )

    return LaunchDescription([
        rsp_launch,
        joint_state_publisher_node,
        rviz_node
    ])