# RViz Configurations

Visualization profiles for the Cobra Flex description:

- `mav1_description.rviz`: RViz layout that loads the MAV1 URDF, shows TF frames, and enables sensors for quick sanity checks.

Duplicate and adapt this profile when you add sensors or change frame names. Keep a baseline configuration so common views can follow one command:

```bash
rviz2 -d mav1_description.rviz
```
