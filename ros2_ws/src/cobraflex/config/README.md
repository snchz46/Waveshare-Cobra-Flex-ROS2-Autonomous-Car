# Configuration Files

Parameter and bridge configuration lives in this directory.

Archivos destacados:

- `gazebo_bridge.yaml`: Archivo de parámetros para el bridge ROS 2 ↔ Gazebo. Ajusta aquí los mapeos cuando simules el Cobra Flex o conectes tópicos entre namespaces.

Puedes añadir nuevos YAML para:

- Ajustes PID y límites de seguridad.
- Calibración de sensores (LiDAR, cámara, IMU).
- Valores por defecto para launch files.

Referéncialos desde los launch files en `../launch/` para mantener la configuración centralizada.
