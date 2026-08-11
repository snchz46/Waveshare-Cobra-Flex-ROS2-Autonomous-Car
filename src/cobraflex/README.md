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
ros2 run cobraflex cobraflex_ros_driver
```

Parametros principales:

- `port`: puerto serie, por ejemplo `/dev/ttyACM1`
- `baud`: baudrate, por defecto `115200`
- `max_linear`: velocidad lineal maxima esperada en m/s
- `max_angular`: velocidad angular maxima esperada en rad/s
- `turn_threshold`: umbral de giro para luces
- `cmd_timeout`: **deadman**, segundos sin `/cmd_vel` antes de parar el robot
  (por defecto `0.5`). El nodo reenvia el ultimo comando cada 50 ms para
  esquivar el timeout del firmware, asi que sin esto el robot sigue rodando
  indefinidamente si muere quien publica. `0.0` lo desactiva: solo banco.

### Evasion con LiDAR

Escucha `/scan` y publica `/cmd_vel` con evitacion de obstaculos.

```bash
ros2 run cobraflex lidar_avoidance_node
```

Parametros principales:

- `front_angle_deg`: semiangulo del sector frontal en grados
- `side_sample_deg`: semiangulo de los sectores laterales
- `front_offset_deg`: orientacion del eje de avance dentro del frame del scan.
  Por defecto `180.0`, porque el lidar va montado girado media vuelta
  (`lidar_joint` lleva yaw = pi)
- `safe_distance`: distancia minima segura en metros
- `hard_stop_distance`: distancia de parada
- `forward_speed`: velocidad lineal de avance
- `scan_timeout`: deadman, segundos sin `/scan` antes de parar (por defecto `0.5`)

Requiere un lidar de 360 grados: los sectores se indexan con envoltura modular
sobre el anillo de rayos. Si el scan cubre menos de ~360 grados el nodo avisa y
se queda parado en vez de conducir con lecturas mal proyectadas.

### Lane keeper

Camara CSI del Jetson, controlador clasico por histograma:

```bash
ros2 run cobraflex lane_keeper_node
```

O con launch y RViz:

```bash
ros2 launch cobraflex cobraflex_lane_keeper.launch.py
```

Version para Gazebo (estimador CV calibrado + pure-pursuit de `cobraflex_rl`):

```bash
ros2 launch cobraflex lane_keeper_gazebo.launch.py
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

Navegacion autonoma en simulacion (necesita un mapa guardado, ver
[maps/README.md](maps/README.md)):

```bash
ros2 launch cobraflex navigation.launch.py
```

Usa `config/nav2_params.yaml`, con la geometria real del robot (footprint
0.228 x 0.180 m, radio circunscrito 0.145 m, `base_footprint`). Para otro mapa
o parametros:

```bash
ros2 launch cobraflex navigation.launch.py map:=/ruta/mapa.yaml params_file:=/ruta/params.yaml
```

## Ejecutables disponibles

Los cuatro nombres registrados en `setup.py`, que son los unicos validos para
`ros2 run cobraflex ...`:

| Ejecutable | Modulo |
| --- | --- |
| `cobraflex_ros_driver` | `cobraflex.cobraflex_ros_driver` |
| `lidar_avoidance_node` | `cobraflex.lidar_avoidance_node` |
| `lane_keeper_node` | `cobraflex.lane_keeper_node` |
| `lane_keeper_gazebo_node` | `cobraflex.lane_keeper_gazebo_node` |
