# cobraflex/meshes

Visualisation meshes (STL) referenced by the URDF / SDF of the
CobraFlex 1:14 platform. All of them are tracked in git and installed to the
package share by `setup.py`, so a fresh clone renders the full robot with no
extra download step.

| File | Size | Source |
| ---- | ---- | ------ |
| `cobraflex_body.stl` | 6.4 MB | Author CAD design |
| `cobraflex_chassis.stl` | 12 MB | Author CAD design |
| `cobraflex_wheel.stl` | 4.5 MB | Author CAD design |
| `rplidar-a2m4-r1.stl` | 176 KB | Slamtec RPLidar A2 visualisation mesh |
| `zedmini_camera.stl` | 78 KB | Stereolabs ZED Mini visual reference |

These are **visual** meshes only. Collision geometry in the URDFs is primitive
(boxes and cylinders) and the inertias come from the `inertial_macros.xacro`
box/cylinder formulas, so replacing a mesh changes what you see and nothing
about the physics.

## Referencing them

Use the package-resolved form, which works in RViz, robot_state_publisher and
Gazebo alike:

```xml
<mesh filename="file://$(find cobraflex)/meshes/cobraflex_chassis.stl"
      scale="0.001 0.001 0.001"/>
```

The scale factor is not optional: the STLs are exported in millimetres and
URDF works in metres.

## Note on repository size

The three author meshes are ~23 MB together and are duplicated under
`assets/3d-models/`. That duplication is deliberate — `assets/` is the CAD
archive, `meshes/` is what the package installs — but it does mean a change to
the CAD has to be copied to both places.
