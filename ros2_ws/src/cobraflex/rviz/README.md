# RViz Configurations

Visualization profiles for the Cobra Flex description:

- `mav1_description.rviz`: RViz layout that loads the MAV1 URDF, shows TF frames, and enables sensors for quick sanity checks.

Duplica y adapta este perfil cuando agregues sensores o cambies nombres de frames. Mantén una configuración base para que las vistas comunes sigan a un comando:

```bash
rviz2 -d mav1_description.rviz
```
