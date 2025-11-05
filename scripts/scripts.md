**Scripts overview**

- `lidar_to_zed_pointcloud.py`: ROS 2 node that consumes Slamtec `/scan` messages from the [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros/tree/ros2) driver and republishes them as `sensor_msgs/PointCloud2` in the ZED left camera optical frame by applying the configured static transform.
- `lidar_to_zed_projection_debug.py`: Debugging utility that projects the transformed LiDAR points onto rectified images from the Stereolabs [`zed-ros2-wrapper`](https://github.com/stereolabs/zed-ros2-examples) topics so you can visually inspect alignment.
- `lidar_zed_distance_comparison.py`: Subscribes to the ZED depth map, samples pixels along the LiDAR azimuth, and reports per-beam distance differences for calibration logging.
- `windows_yolov8_cam_sub_ZED.py`: Prototype subscriber used when testing the ZED Windows streaming example; ingests the `zed_video_sub` sample topics from the Stereolabs ROS 2 examples to feed a YOLOv8 pipeline.
- `Pointcloud_Fusion.rviz`: RViz workspace preconfigured with ZED colorized point clouds and LiDAR overlays to validate transforms.
- `Lidar_ZED_Distance.png` & `pointcloud fusion.png`: Captured outputs that illustrate expected comparison plots and fused RViz views when the setup is configured correctly.
