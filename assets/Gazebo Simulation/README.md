# Gazebo simulation fundamentals

Overview of the elements of the Cobra Flex simulation in Gazebo Harmonic and of
the files that implement them.

| Element | Description | Implementation |
| --- | --- | --- |
| Robot description | Links, joints, inertial, collision and visual properties | [`urdf/`](../../src/cobraflex/urdf/) |
| Gazebo plugins | DiffDrive, OdometryPublisher, LiDAR, cameras, IMU | [`urdf/robot.gazebo`](../../src/cobraflex/urdf/robot.gazebo) |
| Worlds | SDF worlds with lane textures and obstacles | [`worlds/`](../../src/cobraflex/worlds/README.md) |
| ROS bridge | Gazebo ↔ ROS 2 topic mapping | [`config/gz_bridge.yaml`](../../src/cobraflex/config/gz_bridge.yaml) |
| Spawning | Robot spawn and bridge start-up | [`launch/gazebo.launch.py`](../../src/cobraflex/launch/gazebo.launch.py), [`launch/gazebo_mesh.launch.py`](../../src/cobraflex/launch/gazebo_mesh.launch.py) |

## Procedure for model changes

1. Build or modify the URDF with correct frames and inertial properties.
2. Test the model in a minimal world (`empty.world`).
3. Add one sensor plugin at a time.
4. Verify topics, frame IDs and update rates.
5. Integrate the spawn into the launch files.

## References

- [Building a robot](https://gazebosim.org/docs/latest/building_robot/)
- [SDF worlds](https://gazebosim.org/docs/latest/sdf_worlds/)
- [Sensors](https://gazebosim.org/docs/latest/sensors/)
- [Spawn URDF](https://gazebosim.org/docs/latest/spawn_urdf/)
