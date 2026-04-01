import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_cobraflex = get_package_share_directory('cobraflex')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    rviz_cfg_arg = DeclareLaunchArgument('rviz_config', default_value='mapping_hw.rviz')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': os.path.join(pkg_cobraflex, 'config', 'slam_toolbox_mapping_hw.yaml'),
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution([pkg_cobraflex, 'rviz', LaunchConfiguration('rviz_config')])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        rviz_arg,
        rviz_cfg_arg,
        sim_time_arg,
        slam,
        rviz
    ])