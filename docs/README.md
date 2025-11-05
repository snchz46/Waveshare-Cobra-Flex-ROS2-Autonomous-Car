# Documentation Hub

Use this directory to collect detailed notes, diagrams, experiments, and media logs for the Jetson Orin Nano autonomous car project. The Markdown files in this folder support inline photo embedding, so you can illustrate each step without leaving GitHub.

## 1. Hardware Assembly & Bill of Materials
- Mechanical layout of the Waveshare Cobra Flex chassis
- Reference the [Cobra Flex hardware resource pack](https://www.waveshare.com/wiki/Cobra_Flex#Resources) for exploded diagrams, the TB6612FNG motor driver pinout, and recommended servo linkage geometry.
- Mounting instructions for the ZED camera and RPLIDAR. Pair written instructions with photos such as `![Cobra Flex top plate](../assets/photos/Testing%20build.jpg)` to show sensor placement.
- Wiring diagrams and power distribution notes
- Maintain a table of confirmed components (model numbers, vendor links, firmware revisions) that mirrors the BOM in the root `README.md`

## 2. Software & ROS 2 Setup
- JetPack version and OS image
- ROS 2 distribution, DDS configuration, and workspace layout
- ZED SDK, RPLIDAR drivers, and other dependencies
- Capture steps used to clone and build the [Stereolabs ROS 2 examples](https://github.com/stereolabs/zed-ros2-examples) alongside the [Slamtec ROS 2 driver](https://github.com/Slamtec/rplidar_ros/tree/ros2) so future rebuilds are reproducible.
- Embed screenshots or terminal captures by exporting them as PNGs into `../assets/photos/` and referencing them with relative paths.

## 3. Calibration & Sensor Fusion
- Camera–LiDAR extrinsic calibration steps
- Time synchronization strategies
- Validation experiments and metrics. Include calibration charts or RViz overlays inline using Markdown images for quick visual checks.

## 4. Experimentation & Media
- Record each test session in [`experiments/`](experiments/) using the provided template.
- Cross-link supporting photos (`../assets/photos/`) and videos (`../assets/videos/`) so future readers can watch the setup in action.
- Summarize significant demos in `media-log.md` with links to hosted clips when available. Consider adding a thumbnail image beside each major milestone.

## 5. Autonomy Stack
- Planned navigation or control architecture
- Launch files, parameter sets, and ROS graphs
- Test scenarios and performance results, ideally with plots or photos demonstrating the outcome.

## 6. Future Work & Lessons Learned
- Enhancements to pursue next
- Challenges encountered and how they were addressed
- Links to demo videos or write-ups, plus supporting images when helpful

### Embedding Local Images
Use this pattern to add images that live inside the repository:
```markdown
![Short caption describing the scene](../assets/photos/pointcloud%20fusion.png)
```
Keeping the media under version control ensures that exported PDFs, GitHub previews, and static site generators all display the same visuals without external hosting.

Feel free to add more Markdown files in this folder (e.g., `calibration.md`, `ros-graph.md`, `experiments/2024-05-01.md`) as the project progresses.
