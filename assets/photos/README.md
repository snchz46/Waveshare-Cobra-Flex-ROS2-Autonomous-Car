# Photo Gallery

High-resolution build photos and documentation shots live here. Embed them directly into Markdown files using relative links so the visuals travel with the repository.

| Snapshot | Caption |
| --- | --- |
| ![Initial prototype front view](Initial%20prototype%20front.jpg) | First bench-top wiring pass with the Jetson, ZED Mini, and RPLIDAR mounted to the Cobra Flex chassis. |
| ![Initial prototype side profile](Initial%20prototype%20side.jpg) | Side angle of the early stack showing sensor spacing and temporary cable management. |
| ![Mockup V1 front view](Mockup%20V1%20front.jpg) | Version 1 mockup with refined plate geometry and a centered ZED Mini bracket. |
| ![Mockup V1 back view](Mockup%20V1%20back.jpg) | Rear of the V1 layout highlighting power distribution and lidar positioning. |
| ![Mockup V1 side view](Mockup%20V1%20side.jpg) | Side profile of the V1 iteration showing lowered center of gravity and cleaner wiring routes. |
| ![Cobra Flex bench build with Jetson Orin Nano](Testing%20build.jpg) | Jetson Orin Nano developer kit mounted on the Waveshare Cobra Flex chassis during wiring. |
| ![Alternate angle highlighting ZED and RPLIDAR alignment](Testing%20build%202.jpg) | Early alignment check for the ZED Mini stereo camera and RPLIDAR A2M8. |
| ![Hardware iteration comparison](Physical%20comparison.jpg) | Side-by-side comparison of mounting plate revisions. |
| ![Point cloud fusion overlay](pointcloud%20fusion.png) | RViz capture illustrating fused LiDAR and ZED data. |
| ![Depth agreement plot between LiDAR and ZED](Lidar_ZED_Distance.png) | Analysis plot comparing range measurements from both sensors. |
| ![Current build on the lab circuit](cobraflex_v3.jpg) | Current (V3) build on the lab circuit: RPLIDAR, ZED Mini, CSI lane camera on the front bumper. |
| ![Lab circuit](lab_circuit.jpg) | The lab circuit, a still from the physical emergency-stop clip. |
| ![Gazebo twin](gazebo_twin.png) | The robot model in Gazebo Harmonic, cropped from `digital_twin.png`. |
| ![Isaac Sim twin](isaac_sim_twin.png) | The robot's USD asset rendered in NVIDIA Isaac Sim. |
| ![Gazebo lane following](gazebo_lane_following.png) | `complex_b` circuit in Gazebo, with RViz drawing the lane boundaries the safety cage watches. |
| ![RViz lane camera](rviz_lane_camera.png) | Lane camera in RViz: CV lane estimate overlay and the grayscale frame the CNN policy sees. |
| ![Safety cage node chain](safety_cage_node_chain.png) | Perception → policy → safety cage → vehicle control, with the topics between them. |

When adding new media:
1. Place the file in this directory (or a dated subfolder).
2. Use web-friendly names (lowercase, spaces allowed) so GitHub renders them cleanly.
3. Reference the image with Markdown syntax: `![alt text](relative/path/to/image.png)`.
4. Update any relevant documentation (`README.md`, `docs/`, or experiment logs) so readers can see the new visuals in context.

For large photo dumps, create a subfolder (for example, `2024-06-trackday/`) with its own `README.md` summarizing the highlights.
