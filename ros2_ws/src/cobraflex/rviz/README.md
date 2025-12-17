# RViz Configurations

Visualization profiles for the Cobra Flex description:

- `mav1_description.rviz`: RViz layout that loads the MAV1 URDF, shows TF frames, and enables sensors for quick sanity checks.

Duplicate and adapt this config when adding new sensors or changing frame names so common views remain one command away:

```bash
rviz2 -d mav1_description.rviz
```
