#!/usr/bin/env bash
# Reference installation of ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04, the
# setup of the lab PC.
#
# Follows the official instructions, independently of docs/1_INSTALLATION.md:
# the ROS repository through the ros2-apt-source package, and Gazebo Harmonic
# with the ros_gz bridge from ros-jazzy-ros-gz, whose vendor packages come from
# the ROS repository (no OSRF repository). rosdep skips the hardware-only
# packages that are built from source on the robot.
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#   https://gazebosim.org/docs/harmonic/ros_installation/
set -eo pipefail

sudo apt-get update
sudo apt-get install locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository -y universe

# ROS 2 Jazzy
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-jazzy-desktop ros-jazzy-ros-gz

# Project dependencies
sudo apt-get install \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-rviz2 ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-joy \
  ros-jazzy-robot-state-publisher ros-jazzy-tf2-tools

sudo rosdep init
rosdep update

# Workspace
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "sllidar_ros2 zed_wrapper"
colcon build --symlink-install
