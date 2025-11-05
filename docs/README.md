# Documentation Hub

Use this directory to collect detailed notes, diagrams, and experiments for the Jetson Orin Nano autonomous car project. Suggested subtopics:

## 1. Hardware Assembly
- Mechanical layout of the Waveshare Cobra Flex chassis
- Reference the [Cobra Flex hardware resource pack](https://www.waveshare.com/wiki/Cobra_Flex#Resources) for exploded diagrams, the TB6612FNG motor driver pinout, and recommended servo linkage geometry.
- Mounting instructions for the ZED camera and RPLIDAR
- Wiring diagrams and power distribution notes

## 2. Software & ROS 2 Setup
- JetPack version and OS image
- ROS 2 distribution, DDS configuration, and workspace layout
- ZED SDK, RPLIDAR drivers, and other dependencies
- Capture steps used to clone and build the [Stereolabs ROS 2 examples](https://github.com/stereolabs/zed-ros2-examples) alongside the [Slamtec ROS 2 driver](https://github.com/Slamtec/rplidar_ros/tree/ros2) so future rebuilds are reproducible.

## 3. Calibration & Sensor Fusion
- Camera–LiDAR extrinsic calibration steps
- Time synchronization strategies
- Validation experiments and metrics

## 4. Autonomy Stack
- Planned navigation or control architecture
- Launch files, parameter sets, and ROS graphs
- Test scenarios and performance results

## 5. Future Work & Lessons Learned
- Enhancements to pursue next
- Challenges encountered and how they were addressed
- Links to demo videos or write-ups

Feel free to add more Markdown files in this folder (e.g., `calibration.md`, `ros-graph.md`, `experiments/2024-05-01.md`) as the project progresses.
