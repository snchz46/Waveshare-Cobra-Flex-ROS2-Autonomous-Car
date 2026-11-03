#!/usr/bin/env bash
# Reference installation of ROS 2 Humble + Gazebo Harmonic on Ubuntu 22.04.
#
# Follows the official Gazebo instructions for the Humble/Harmonic pairing:
# Gazebo Harmonic from packages.osrfoundation.org and the ros_gz bridge from
# ros-humble-ros-gzharmonic, which replaces (and conflicts with) the
# ros-humble-ros-gz* packages. rosdep therefore skips the ros_gz keys, and the
# hardware-only packages that are built from source on the robot.
#   https://gazebosim.org/docs/harmonic/install_ubuntu/
#   https://gazebosim.org/docs/harmonic/ros_installation/
set -eo pipefail

sudo apt-get update
sudo apt-get install locales curl lsb-release gnupg
sudo locale-gen en_US en_US.UTF-8

# ROS 2 Humble
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Gazebo Harmonic
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install ros-humble-desktop gz-harmonic ros-humble-ros-gzharmonic

# Project dependencies
sudo apt-get install \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-rviz2 ros-humble-xacro \
  ros-humble-teleop-twist-keyboard ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-tf2-tools

sudo rosdep init
rosdep update

# Workspace
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "ros_gz_sim ros_gz_bridge sllidar_ros2 zed_wrapper"
colcon build --symlink-install
