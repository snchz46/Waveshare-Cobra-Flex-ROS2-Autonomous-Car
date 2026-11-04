# 3D models

CAD files of the robot and its components. The three meshes `cobraflex_*.stl`
are duplicated in
[`src/cobraflex/meshes/`](../../src/cobraflex/meshes/README.md), where the URDFs
reference them; a CAD change is applied to both locations.

| File | Content |
| --- | --- |
| `cobraflex_chassis.stl` | Chassis |
| `cobraflex_body.stl` | Body shell: V4.1 printed body, assembled (Plate_bot frame) |
| `cobraflex_wheel.stl` | Wheel |
| [`body_v4/`](body_v4/README.md) | V4.1 printed body, part by part: STL oriented for printing and STEP, with print settings, fasteners and assembly |
| `Cobra Flex Full Plate_top.stl`, `Cobra Flex Full Plate_bot.stl`, `Cobra Flex Full Plate_wheels.stl` | Previous printed body (V3): top cover, centre shell and wheels plate. Replaced by `body_v4/`; `cam_plate_V4` is derived from Full Plate_wheels |
| `Intel Realsense RPLIDAR A2.stl` | RPLIDAR A2 |
| `zedmini_camera.stl` | ZED Mini |
| `Arducam IMX219 Autofocus v3.step` | IMX219 lane camera |
| `Orin_Nano_Dev_Kit.STEP` | Jetson Orin Nano Developer Kit |
| `Battery 12V 1200mAh Li-ion Rechargeable v2.step` | Motor battery pack |
| `Antenna SMA Male 100 mm 868 MHz.STEP` | Antenna |
