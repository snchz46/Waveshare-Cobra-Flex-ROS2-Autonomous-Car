# Photo Gallery

High-resolution build photos and documentation shots live here. Embed them directly into Markdown files using relative links so the visuals travel with the repository.

| Snapshot | Caption |
| --- | --- |
| ![Cobra Flex bench build with Jetson Orin Nano](Testing%20build.jpg) | Jetson Orin Nano developer kit mounted on the Waveshare Cobra Flex chassis during wiring. |
| ![Alternate angle highlighting ZED and RPLIDAR alignment](Testing%20build%202.jpg) | Early alignment check for the ZED Mini stereo camera and RPLIDAR A2M8. |
| ![Hardware iteration comparison](Physical%20comparison.jpg) | Side-by-side comparison of mounting plate revisions. |
| ![Point cloud fusion overlay](pointcloud%20fusion.png) | RViz capture illustrating fused LiDAR and ZED data. |
| ![Depth agreement plot between LiDAR and ZED](Lidar_ZED_Distance.png) | Analysis plot comparing range measurements from both sensors. |

When adding new media:
1. Place the file in this directory (or a dated subfolder).
2. Use web-friendly names (lowercase, spaces allowed) so GitHub renders them cleanly.
3. Reference the image with Markdown syntax: `![alt text](relative/path/to/image.png)`.
4. Update any relevant documentation (`README.md`, `docs/`, or experiment logs) so readers can see the new visuals in context.

For large photo dumps, create a subfolder (for example, `2024-06-trackday/`) with its own `README.md` summarizing the highlights.
