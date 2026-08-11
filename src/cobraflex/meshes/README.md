# cobraflex/meshes

Visualisation meshes (STL) referenced by the URDF / SDF of the
CobraFlex 1:14 platform.

## Tracked in git

| File | Size | Source |
| ---- | ---- | ------ |
| `cobraflex_body.stl` | 6.4 MB | Author CAD design |
| `cobraflex_chassis.stl` | 12 MB | Author CAD design |
| `cobraflex_wheel.stl` | 4.5 MB | Author CAD design |
| `zedmini_camera.stl` | 78 KB | Stereolabs ZED Mini visual reference |

## Not tracked in git (obtain externally)

| File | Size | Source | How to obtain |
| ---- | ---- | ------ | ------------- |
| `rplidar-a2m4-r1.stl` | 87 MB | Slamtec RPLidar A2 visualisation mesh | Run `scripts/download_meshes.sh` from the repo root, or download manually from the Slamtec CAD library and place it in this directory. |

The RPLidar STL is excluded from git via `.gitignore` because it
exceeds the 50 MB soft-limit that GitHub warns about and because the
file is upstream-distributed and does not change with the thesis
work. Without it, the URDF will still load but the simulator's rviz
visualisation of the LIDAR sensor will be a placeholder primitive
(or missing) — perception and the cage are unaffected because they
operate on the sensor's data, not on its visual model.

If your local clone does not have the file and you need the visual
representation for a demo or a figure, run:

    ./scripts/download_meshes.sh

The script is idempotent and will skip files that already exist.
