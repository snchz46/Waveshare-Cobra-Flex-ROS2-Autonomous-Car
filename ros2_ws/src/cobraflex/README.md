# Cobraflex ROS 2

Integración ROS 2 para el chasis Cobra Flex 4WD. Incluye driver sobre serial, nodos de evitación basados en LIDAR, y descripciones URDF con RPLIDAR A2M8 y cámara ZED (ZED Mini/ZED2).

## Instalación rápida

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Requisitos: `pyserial` para el driver serial.

```bash
sudo apt install python3-serial
# o
python3 -m pip install pyserial
```

## Nodos principales

- **`cobraflex_cmdvel_driver`** (`cobraflex_cmdvel_driver.py`): Convierte `/cmd_vel` en comandos JSON para el microcontrolador del chasis. Parámetros clave: `port`, `baud`, `max_speed_value`, `max_linear`, `max_angular`.
- **`lidar_avoidance_pid`** (`lidar_avoidance_pid_node.py`): Escucha `/scan` y publica `/cmd_vel` aplicando correcciones PID para mantener distancia de seguridad (`front_angle_deg`, `safe_distance`, `forward_speed`).
- **`cobraflex_ros_driver`**: Complemento para publicar estados y facilitar la integración del driver serial con el stack ROS 2.

### Ejecutar

```bash
ros2 run cobraflex cobraflex_cmdvel_driver
ros2 run cobraflex lidar_avoidance_pid
```

## Launch files destacados

- `cobraflex_sensors.launch.xml`: Sólo sensores (RPLIDAR + ZED) para calibración y pruebas en RViz.
- `cobraflex_driver.launch.xml`: Driver de chasis para teleoperación.
- `cobraflex_bringup.launch.xml` / `.py`: Stack completo (sensores + driver + evasión).
- `cobraflex_manual.launch.xml`: Perfil de teleoperación manual.
- `cobraflex_automatic.launch.xml`: Perfil de navegación autónoma con evasión.
- `cobraflex_description.launch.xml` / `cobraflex_zed_description.launch.py`: Publica el URDF y el árbol TF.
- `mav1_gazebo.launch.xml`: Simulación en Gazebo del MAV1.

## Descripción del paquete

- Código Python en [`cobraflex/`](cobraflex/)
- Parámetros y bridges en [`config/`](config/)
- URDF/Xacro en [`urdf/`](urdf/)
- Configuración de RViz en [`rviz/`](rviz/)
- Pruebas ament en [`test/`](test/)
- Archivos de lanzamiento en [`launch/`](launch/)

Mantén estos directorios actualizados al agregar nodos, sensores o cambios de frames para que el resto del equipo pueda usarlos desde los launch files existentes.
