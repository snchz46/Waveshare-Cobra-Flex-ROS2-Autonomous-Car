**Scripts overview**

- `lidar_to_zed_pointcloud.py`: ROS 2 node that consumes Slamtec `/scan` messages from the [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros/tree/ros2) driver and republishes them as `sensor_msgs/PointCloud2` in the ZED left camera optical frame by applying the configured static transform.

![Lidar to pointcloud](https://github.com/user-attachments/assets/f64cecb1-fdad-4167-b95c-de5bda47d781)

- `lidar_to_zed_projection_debug.py`: Debugging utility that projects the transformed LiDAR points onto rectified images from the Stereolabs [`zed-ros2-wrapper`](https://github.com/stereolabs/zed-ros2-examples) topics so you can visually inspect alignment.

![Lidar to projection](https://github.com/user-attachments/assets/436d0942-7625-4674-b75d-2ee134e7ed57)
<img width="500" alt="pointcloud fusion" src="https://github.com/user-attachments/assets/094aa773-c915-4dae-91e6-4327a00ebec8" />



- `lidar_zed_distance_comparison.py`: Subscribes to the ZED depth map, samples pixels along the LiDAR azimuth, and reports per-beam distance differences for calibration logging.

<img width="600"  alt="Lidar_ZED_Distance-1" src="https://github.com/user-attachments/assets/33e6c66d-20b4-4d6d-ab65-ea58ab8815ff" />

<img width="400" src="https://github.com/user-attachments/assets/5b9fd95d-693c-4f69-80ea-a42db716e69b" />

- `windows_yolov8_cam_sub_ZED.py`: Prototype subscriber used when testing the ZED Windows streaming example; ingests the `zed_video_sub` sample topics from the Stereolabs ROS 2 examples to feed a YOLOv8 pipeline.

<img width="600" src="https://github.com/user-attachments/assets/cd150dc4-41c0-40fc-a27b-c6985adce918" />


- `Pointcloud_Fusion.rviz`: RViz workspace preconfigured with ZED colorized point clouds and LiDAR overlays to validate transforms.
