# Python Nodes

ROS 2 Python nodes for the Cobra Flex chassis live here. Esta carpeta es el paquete Python instalado por `setup.py`.

Archivos principales:

- `cobraflex_cmdvel_driver.py`: Traduce mensajes de `/cmd_vel` a comandos JSON para el microcontrolador vía serial, con parámetros de puerto, baudrate y escalas.
- `cobraflex_ros_driver.py`: Puentea feedback del chasis y publica tópicos de estado junto con la interfaz de comando.
- `lidar_avoidance_pid_node.py`: Nodo de evitación de obstáculos; escucha `/scan`, aplica corrección tipo PID y publica velocidades seguras.
- `__init__.py`: Define el paquete Python y facilita imports internos.

Para añadir nuevos nodos (telemetría, diagnósticos, etc.), crea el archivo aquí y registra el entry point en `setup.py` para que `ros2 run cobraflex <node>` lo encuentre.
