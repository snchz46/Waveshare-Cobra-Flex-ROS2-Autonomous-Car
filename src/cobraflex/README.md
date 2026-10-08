# cobraflex

Integracion ROS 2 del chasis CobraFlex 4WD:

- Driver JSON por puerto serie
- Evasion de obstaculos con LiDAR
- Seguimiento de carril con camara CSI
- Bringup de sensores, descripcion del robot y simulacion

## Instalacion

```bash
cd ~/ros2_ws
colcon build --packages-select cobraflex
source install/setup.bash
```

Dependencias de Python:

```bash
sudo apt install python3-serial python3-numpy python3-opencv
```

## Nodos

### Driver del chasis

Convierte `/cmd_vel` en comandos JSON para el chasis CobraFlex.

```bash
ros2 run cobraflex cobraflex_ros_driver
```

Parametros:

| Parametro | Descripcion |
| --- | --- |
| `port` | Puerto serie, por ejemplo `/dev/ttyACM1` |
| `baud` | Velocidad en baudios; por defecto `115200` |
| `max_linear` | Velocidad lineal maxima en m/s |
| `max_angular` | Velocidad angular maxima en rad/s |
| `turn_threshold` | Umbral de giro para las luces |
| `cmd_timeout` | Temporizador de seguridad (deadman): segundos sin `/cmd_vel` antes de detener el robot; por defecto `0.5` |

El nodo reenvia el ultimo comando cada 50 ms, lo que anula el timeout del
firmware. `cmd_timeout` es por tanto el unico mecanismo que detiene el robot si
falla el nodo que publica. El valor `0.0` lo desactiva y solo se admite en
banco de pruebas.

### Evasion con LiDAR

Se suscribe a `/scan` y publica `/cmd_vel` con evasion de obstaculos.

```bash
ros2 run cobraflex lidar_avoidance_node
```

Parametros:

| Parametro | Descripcion |
| --- | --- |
| `front_angle_deg` | Semiangulo del sector frontal en grados |
| `side_sample_deg` | Semiangulo de los sectores laterales |
| `front_offset_deg` | Orientacion del eje de avance en el frame del scan; por defecto `180.0`, ya que el LiDAR esta montado girado media vuelta (`lidar_joint` con yaw = pi) |
| `safe_distance` | Distancia minima de seguridad en metros |
| `hard_stop_distance` | Distancia de parada |
| `forward_speed` | Velocidad lineal de avance |
| `scan_timeout` | Temporizador de seguridad: segundos sin `/scan` antes de detener el robot; por defecto `0.5` |

El nodo requiere un LiDAR de 360 grados: los sectores se indexan con envoltura
modular sobre el anillo de rayos. Si el scan cubre menos de ~360 grados, el
nodo emite un aviso y mantiene el robot detenido.

### Seguimiento de carril

Camara CSI del Jetson, controlador clasico por histograma:

```bash
ros2 run cobraflex lane_keeper_node
```

Con launch y RViz:

```bash
ros2 launch cobraflex cobraflex_lane_keeper.launch.py
```

Version para Gazebo (estimador CV calibrado y pure pursuit de `cobraflex_rl`):

```bash
ros2 launch cobraflex lane_keeper_gazebo.launch.py
```

## Launch files

| Funcion | Comando |
| --- | --- |
| Sensores | `ros2 launch cobraflex cobraflex_sensors.launch.xml` |
| Descripcion del robot y driver | `ros2 launch cobraflex cobraflex_bringup.launch.xml` |
| Modo automatico con evasion | `ros2 launch cobraflex cobraflex_automatic.launch.xml` |
| Simulacion en Gazebo | `ros2 launch cobraflex gazebo.launch.py` |
| Mapeado en simulacion | `ros2 launch cobraflex mapping.launch.py` |
| Mapeado en hardware | `ros2 launch cobraflex cobraflex_mapping.launch.py` |
| Navegacion autonoma en simulacion | `ros2 launch cobraflex navigation.launch.py` |

La navegacion requiere un mapa guardado ([maps/README.md](maps/README.md)) y
usa `config/nav2_params.yaml`, configurado con la geometria del robot
(footprint 0.228 x 0.180 m, radio circunscrito 0.145 m, `base_footprint`).
Mapa o parametros alternativos:

```bash
ros2 launch cobraflex navigation.launch.py map:=/ruta/mapa.yaml params_file:=/ruta/params.yaml
```

## Ejecutables

Ejecutables registrados en `setup.py`; son los unicos validos para
`ros2 run cobraflex ...`:

| Ejecutable | Modulo |
| --- | --- |
| `cobraflex_ros_driver` | `cobraflex.cobraflex_ros_driver` |
| `lidar_avoidance_node` | `cobraflex.lidar_avoidance_node` |
| `lane_keeper_node` | `cobraflex.lane_keeper_node` |
| `lane_keeper_gazebo_node` | `cobraflex.lane_keeper_gazebo_node` |
