# cobraflex/meshes

Visual meshes (STL) referenced by the URDF / SDF of the CobraFlex 1:14
platform. All files are tracked in git and installed into the package share by
`setup.py`; a fresh clone renders the complete robot without additional
downloads.

| File | Size | Source |
| ---- | ---- | ------ |
| `cobraflex_body.stl` | 0.6 MB | Author CAD design, V4 printed body |
| `cobraflex_chassis.stl` | 12 MB | Author CAD design |
| `cobraflex_wheel.stl` | 4.5 MB | Author CAD design |
| `rplidar-a2m4-r1.stl` | 176 KB | Slamtec RPLIDAR A2 visual mesh |
| `rplidar-a2m4-r1_hull.stl` | 55 KB | Convex hull of the A2 mesh: `lidar_link` visual of the mesh descriptions, cannot occlude the simulated scan |
| `zedmini_camera.stl` | 78 KB | Stereolabs ZED Mini visual reference |

The meshes are **visual only**. Collision geometry in the URDFs consists of
primitives (boxes and cylinders), and the inertias are defined in the URDFs
(macros from `inertial_macros.xacro`, hand-written tensor for `body_link`).
Replacing a mesh changes the visualisation only, not the physics.

`cobraflex_body.stl` contains the printed parts of the V4 body (bot_V3
with the tapered cable chamber, side fairings and rear port, the port cover,
the top_V3 powerbank cabin, its lid, LiDAR pedestal, ZED cradle and cap, and
cam_plate_V4, the wheels plate with the lane-camera pod),
exported in the Plate_bot frame of the CAD: x forward, z = 0 on top of
Full Plate_wheels. The URDFs place this origin at (0.005, 0, 0.10725) in
`base_footprint`.

## Referencing

The package-resolved form works in RViz, robot_state_publisher and Gazebo:

```xml
<mesh filename="file://$(find cobraflex)/meshes/cobraflex_chassis.stl"
      scale="0.001 0.001 0.001"/>
```

The scale factor is required: the STLs are exported in millimetres, and URDF
uses metres.

## Duplication with `assets/3d-models/`

The three author meshes (about 17 MB together) are also stored in
`assets/3d-models/`: `assets/` is the CAD archive, `meshes/` is the installed
copy. A CAD change is applied to both locations.
