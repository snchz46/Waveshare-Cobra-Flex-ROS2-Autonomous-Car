import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_cobraflex_robot = get_package_share_directory('cobraflex')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_cobraflex_robot)

    # Ensure that the GZ_SIM_RESOURCE_PATH is initialized before adding to it
    if "GZ_SIM_RESOURCE_PATH" not in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )
    
    nav2_map = DeclareLaunchArgument(
        'map', default_value='/home/admit14/my_map_save.yaml',
        description='Map file path for Nav2'
    )

    # Path to the Nav2 Toolbox launch file
    nav2_toolbox_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )


    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_cobraflex_robot, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )


    nav2_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map': LaunchConfiguration('map'),
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(nav2_map)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(nav2_toolbox_launch)

    return launchDescriptionObject
