# Cobraflex ROS2

Integración ROS2 para el chasis Cobraflex 4WD con:

- Driver JSON sobre serial
- Nodo de evitación de obstáculos basado en LIDAR (PID + ramping)
- RPLIDAR A2M8
- Cámara ZED (ZED2 / ZED Mini)

## Instalación

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Guide

Asegúrate de tener instalado pyserial:

```bash
sudo apt install python3-serial
# o
python3 -m pip install pyserial
```

Nodos
cobraflex_cmdvel_driver

Convierte /cmd_vel en comandos JSON para el chasis Cobraflex.

```bash
ros2 run cobraflex cobraflex_cmdvel_driver
```

Parámetros:

- port (string): puerto serial, ej. /dev/ttyACM2
- baud (int): baudrate, por defecto 115200
- max_speed_value (int): escala de potencia para L/R (ej. 100)
- max_linear (float): velocidad lineal máxima esperada [m/s]
- max_angular (float): velocidad angular máxima esperada [rad/s]

lidar_avoidance_pid

Nodo que escucha /scan y publica /cmd_vel con evitación de obstáculos.

```bash
ros2 run cobraflex lidar_avoidance_pid
```

Parámetros principales:

- front_angle_deg (float): ángulo frontal ± en grados
- safe_distance (float): distancia mínima segura [m]
- forward_speed (float): velocidad lineal máxima [m/s]

Launch files
Sensores solamente

```bash
ros2 launch cobraflex cobraflex_sensors.launch.xml
```

Chasis + evitación

```bash
ros2 launch cobraflex cobraflex_driver.launch.xml
```

Bringup completo (sensores + driver + evitación)

```bash
ros2 launch cobraflex cobraflex_bringup.launch.xml
```
