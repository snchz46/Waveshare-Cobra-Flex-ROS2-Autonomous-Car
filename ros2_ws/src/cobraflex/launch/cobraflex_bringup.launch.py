from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz')

    cobraflex_pkg = get_package_share_directory('cobraflex')
    zed_pkg = get_package_share_directory('zed_wrapper')

    # Paths to launch files
    desc_robot = os.path.join(cobraflex_pkg, 'launch', 'cobraflex_description.launch.xml')
    desc_zed   = os.path.join(cobraflex_pkg, 'launch', 'cobraflex_zed_description.launch.py')
    zed_wrapper_launch = os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')
    driver_launch = os.path.join(cobraflex_pkg, 'launch', 'cobraflex_driver.launch.xml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz automatically'
        ),

        # 1. Load robot description (URDF)
        TimerAction(
            period=0.0,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(desc_robot),
                    launch_arguments={'use_rviz': use_rviz}.items()
                )
            ]
        ),

        # 2. Load ZED description (URDF + static TF)
        TimerAction(
            period=0.5,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(desc_zed)
                )
            ]
        ),

        # 3. Load ZED wrapper
        TimerAction(
            period=1.5,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(zed_wrapper_launch),
                    launch_arguments={'camera_model': 'zedm'}.items()
                )
            ]
        ),

        # 4. Load CobraFlex driver
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(driver_launch)
                )
            ]
        ),
    ])
