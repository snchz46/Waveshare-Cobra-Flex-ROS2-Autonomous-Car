# Cobraflex ROS2

Integracion ROS 2 para el chasis CobraFlex 4WD con:

- Driver JSON por puerto serie
- Evasion de obstaculos basada en LiDAR
- Seguimiento de carril con camara CSI
- Bringup de sensores, modelo robotico y simulacion

## Instalacion

```bash
cd ~/ros2_ws
colcon build --packages-select cobraflex
source install/setup.bash
```

Dependencias de Python recomendadas:

```bash
sudo apt install python3-serial python3-numpy python3-opencv
```

## Ejecutables

### Driver del chasis

Convierte `/cmd_vel` en comandos JSON para el chasis CobraFlex.

```bash
ros2 run cobraflex cobraflex_cmdvel_driver
```

Alias equivalente:

```bash
ros2 run cobraflex cobraflex_ros_driver
```

Parametros principales:

- `port`: puerto serie, por ejemplo `/dev/ttyACM1`
- `baud`: baudrate, por defecto `115200`
- `max_linear`: velocidad lineal maxima esperada en m/s
- `max_angular`: velocidad angular maxima esperada en rad/s
- `turn_threshold`: umbral de giro para luces

### Evasion con LiDAR

Escucha `/scan` y publica `/cmd_vel` con evitacion de obstaculos.

```bash
ros2 run cobraflex lidar_avoidance_pid
```

Alias equivalente:

```bash
ros2 run cobraflex lidar_avoidance_pid_node
```

Parametros principales:

- `front_angle_deg`: angulo frontal en grados
- `safe_distance`: distancia minima segura en metros
- `hard_stop_distance`: distancia de parada
- `forward_speed`: velocidad lineal de avance

### Lane keeper

```bash
ros2 run cobraflex lane_keeper
```

O con launch y RViz:

```bash
ros2 launch cobraflex cobraflex_lane_keeper.launch.py
```

## Launch files

Solo sensores:

```bash
ros2 launch cobraflex cobraflex_sensors.launch.xml
```

Modelo del robot + sensores + driver:

```bash
ros2 launch cobraflex cobraflex_bringup.launch.xml
```

Modo automatico con avoidance:

```bash
ros2 launch cobraflex cobraflex_automatic.launch.xml
```

Simulacion en Gazebo:

```bash
ros2 launch cobraflex gazebo.launch.py
```

Mapeado en simulacion:

```bash
ros2 launch cobraflex mapping.launch.py
```

Mapeado en hardware real:

```bash
ros2 launch cobraflex cobraflex_mapping.launch.py
```

Si ya tienes `cobraflex_bringup.launch.xml` corriendo, puedes evitar duplicar la
descripcion del robot asi:

```bash
ros2 launch cobraflex cobraflex_mapping.launch.py start_description:=false
```
