# Launch Files

Launch descriptions for the Cobra Flex stack. Common entry points include:

- `cobraflex_sensors.launch.xml`: Starts sensing only (RPLIDAR + ZED) for calibration and RViz alignment.
- `cobraflex_driver.launch.xml`: Brings up the chassis driver to bridge `/cmd_vel` to the microcontroller.
- `cobraflex_bringup.launch.xml` / `cobraflex_bringup.launch.py`: Full stack (sensors + driver + avoidance) for field tests.
- `cobraflex_manual.launch.xml`: Manual control profile for teleop sessions.
- `cobraflex_automatic.launch.xml`: Autonomous profile that wires up obstacle avoidance.
- `cobraflex_description.launch.xml` / `cobraflex_zed_description.launch.py`: Publish the URDF and TF tree for visualization.
- `mav1_gazebo.launch.xml`: Spawn the MAV1 URDF into Gazebo for simulation.

Además:

- Estos launch files suelen consumir parámetros de `../config/`.
- Las descripciones URDF viven en `../urdf/`.
- Los perfiles de visualización RViz están en `../rviz/`.

Actualiza estas descripciones cuando añadas nodos o parámetros, y mantén la documentación sincronizada con el README principal para que el equipo pueda elegir el entry point correcto rápidamente.
