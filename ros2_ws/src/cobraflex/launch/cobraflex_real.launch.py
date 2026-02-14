import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'cobraflex_robot'
    pkg_share = FindPackageShare(package_name)
    zed_pkg_share = FindPackageShare('zed_wrapper') # Asegúrate de tener instalado el wrapper

    # ================= ARGUMENTOS =================
    # Puerto del Lidar (A veces cambia entre USB0 y USB1 si conectas la ZED)
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0') 
    
    # Argumentos ZED
    camera_model = LaunchConfiguration('camera_model', default='zedm') # zedm = ZED Mini

    # ================= 1. ROBOT STATE PUBLISHER (URDF) =================
    # Carga tu URDF actualizado (con camera_link, lidar_link, etc)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'rsp.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # ================= 2. DRIVER DEL ROBOT (Python Script) =================
    # Tu nuevo driver que publica /cobraflex/vel_raw y /imu/data
    driver_node = Node(
        package=package_name,
        executable='cobraflex_odom.py',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM1', # ¡Revisa que este sea el puerto del microcontrolador!
            'baud': 115200,
            'wheel_base': 0.154
        }]
    )

    # ================= 3. EKF (Odometría Robusta) =================
    # Fusiona Ruedas + IMU para publicar /odom -> /base_link
    ekf_config = PathJoinSubstitution([pkg_share, 'config', 'ekf_cobraflex.yaml'])
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': False}]
    )

    # ================= 4. RPLIDAR A2M8 =================
    # Lanzamos el nodo directamente (más limpio que incluir launchs antiguos)
    rplidar_node = Node(
        package='rplidar_ros', # O 'sllidar_ros2' si usas el driver nuevo
        executable='rplidar_composition', # O 'sllidar_node'
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 115200, # A2M8 usa 115200
            'frame_id': 'lidar_link',  # Debe coincidir con tu URDF
            'inverted': False,
            'angle_compensate': True
        }]
    )

    # ================= 5. CÁMARA ZED MINI =================
    # Incluimos el launch oficial del wrapper
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([zed_pkg_share, 'launch', 'zed_camera.launch.py'])
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'publish_tf': 'false',      # IMPORTANTE: Que ZED NO publique odom->base_link (lo hace el EKF)
            'publish_map_tf': 'false'   # IMPORTANTE: Que ZED NO publique map->odom (lo hace SLAM)
        }.items()
    )

    # ================= 6. UNIÓN TF (URDF <-> ZED) =================
    # Tu URDF tiene 'camera_link'. La ZED espera 'zed_camera_center' o similar.
    # Creamos un puente estático para que la cámara se "pegue" al modelo del robot.
    # Ajusta los 0 0 0 si la cámara real no está exactamente en el centro del link definido en URDF.
    
    tf_zed_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'camera_link', 'zed_camera_link']
    )

    return LaunchDescription([
        rsp,
        driver_node,
        ekf_node,
        rplidar_node,
        zed_launch,
        tf_zed_bridge
    ])