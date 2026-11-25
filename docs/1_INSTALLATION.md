# Installation

Setup of the two machines of the Cobra Flex, from a clean operating system to a
built and sourced ROS 2 workspace. The lab PC runs the simulation and the
operator tools; the Jetson Orin Nano on the robot runs the driver, the sensors
and the on-board applications. Both machines build this repository, on
different Ubuntu and ROS 2 releases.

Short version for the lab PC: [Quick start](../README.md#quick-start) in the root README.

**Machines**

| | Lab PC | Robot |
| --- | --- | --- |
| Hardware | x86-64 (amd64) | NVIDIA Jetson Orin Nano Developer Kit (arm64) |
| Operating system | Ubuntu 24.04 LTS | Ubuntu 22.04 LTS, base of JetPack 6 (L4T R36) |
| ROS 2 | Jazzy Jalisco | Humble Hawksbill |
| Gazebo | Harmonic (gz-sim 8), from the ROS repository | Harmonic (gz-sim 8), from the OSRF repository; bridge `ros_gz` built from source |
| Python | 3.12 | 3.10 |
| Applications | Gazebo, RViz, teleoperation GUI, launch manager, lab session programs | Driver, sensors and EKF; LiDAR avoidance, SLAM, Nav2 and lane keeping on the physical robot; Gazebo with a monitor attached to the Jetson |
| Procedure | [Lab PC](#lab-pc-ubuntu-2404-and-ros-2-jazzy) | [Robot](#robot-jetson-orin-nano-and-ros-2-humble) |

Each machine runs the ROS 2 release of its Ubuntu version. JetPack 6, the
software stack of the Jetson, is based on Ubuntu 22.04, the platform of ROS 2
Humble. The lab PCs run Ubuntu 24.04, the platform of ROS 2 Jazzy. Jazzy is
also the release paired with Gazebo Harmonic in the ROS repository; Humble is
paired with Gazebo Fortress. On amd64, Humble reaches Harmonic through the
unofficial `ros-humble-ros-gzharmonic` packages; these are not available for
arm64, so the Jetson builds the bridge from source
([Gazebo Harmonic on the robot](#gazebo-harmonic-on-the-robot)).
Conditions for the communication between the two releases:
[Interoperability of Humble and Jazzy](#interoperability-of-humble-and-jazzy).

> **The repository is the workspace root**, on both machines. It contains
> `src/` with all six packages and is cloned as `~/ros2_ws`, not into
> `~/ros2_ws/src` or another subdirectory of an existing workspace
> ([Repository](#repository-1)).

---

## Lab PC: Ubuntu 24.04 and ROS 2 Jazzy

Starting point: Ubuntu 24.04 LTS on an x86-64 PC with internet access.

Installed Ubuntu release:

```bash
lsb_release -cs   # "noble" for Ubuntu 24.04
```

On Ubuntu 26.04 (`resolute`), the ROS repository provides ROS 2 Lyrical
instead of Jazzy, and `ros-jazzy-desktop` cannot be found. The lab PCs use
Ubuntu 24.04.

### ROS 2 Jazzy

```bash
sudo apt update && sudo apt install locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade
sudo apt install ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

The package `ros2-apt-source` installs the ROS repository entry together with
its signing key, and apt updates both when the key changes
([ROS 2 installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)).
The last line loads the ROS 2 environment in every new terminal.

### Gazebo Harmonic

Under ROS 2 Jazzy, Gazebo Harmonic is distributed in the ROS repository as
vendor packages (`gz_*_vendor`). `ros-jazzy-ros-gz` installs it together with
the ROS–Gazebo bridge; the OSRF repository (`packages.osrfoundation.org`) is not
required ([Gazebo documentation](https://gazebosim.org/docs/harmonic/ros_installation/)).

```bash
sudo apt install ros-jazzy-ros-gz
```

### Project dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-rviz2 ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-joy \
  ros-jazzy-robot-state-publisher ros-jazzy-tf2-tools

sudo rosdep init
rosdep update
```

| Package | Role |
| --- | --- |
| `ros-jazzy-ros-gz` | Gazebo Harmonic and the Gazebo ↔ ROS 2 bridge ([Gazebo Harmonic](#gazebo-harmonic)) |
| `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` | Nav2 stack and launch files |
| `ros-jazzy-slam-toolbox` | Graph SLAM, occupancy grid |
| `ros-jazzy-robot-localization` | EKF; in simulation without TF output (`ekf_gazebo.yaml`) |
| `ros-jazzy-rviz2` | Visualisation |
| `ros-jazzy-teleop-twist-keyboard` | Keyboard driving during mapping |
| `ros-jazzy-robot-state-publisher`, `ros-jazzy-xacro` | URDF expansion and TF publication |
| `ros-jazzy-tf2-tools` | `view_frames`, `tf2_echo` for TF diagnostics |

### Repository

```bash
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git ~/ros2_ws
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "sllidar_ros2 zed_wrapper"
```

rosdep requires the ROS 2 environment (`ROS_DISTRO`); without it, it installs
nothing and still exits without an error. The skipped keys are the hardware
packages, which are built from source on the robot only
([Hardware dependencies](#hardware-dependencies)). Under Jazzy, the keys
`ros_gz_sim` and `ros_gz_bridge` resolve to the packages of
[Gazebo Harmonic](#gazebo-harmonic) and are not skipped.

An existing `~/ros2_ws`, for example a workspace created by hand, makes
`git clone` fail; it is moved aside first (`mv ~/ros2_ws ~/ros2_ws_old`).
Procedure and failure mode: [Repository](#repository-1) in the robot section.
Check: `git -C ~/ros2_ws remote -v` shows this repository, and
`ls ~/ros2_ws/src` lists `cobraflex`, `cobraflex_launch_gui`, `cobraflex_rl`,
`cobraflex_safety_msgs`, `cobraflex_teleop_gui` and `safety_cage`.

### Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

With `--symlink-install`, changes to Python nodes take effect without a
rebuild.

Single package:

```bash
colcon build --packages-select cobraflex --symlink-install
```

Automatic sourcing in every new terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Robot: Jetson Orin Nano and ROS 2 Humble

Starting point: a Jetson Orin Nano Developer Kit flashed with JetPack 6
according to the
[NVIDIA getting-started guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit),
with internet access. The robot is operated without a display; the commands
run in an SSH terminal
([3_ROBOT_OPERATION.md §5](3_ROBOT_OPERATION.md#5-switching-on-and-connecting)).
Gazebo is the exception: its GUI needs a monitor attached to the Jetson.

Installed JetPack release:

```bash
cat /etc/nv_tegra_release   # "# R36 (release), REVISION: 4.x" = L4T 36.4, JetPack 6
```

The order of the sections is the installation order. On a Jetson that already
has ROS 2 Humble (a prepared image or an earlier installation), the procedure
starts with the repository check in [ROS 2 Humble](#ros-2-humble) and then
continues with [Project dependencies](#project-dependencies-1).

### ROS 2 Humble

New installation:

```bash
sudo apt update && sudo apt install locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade
sudo apt install ros-humble-ros-base
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

`ros-humble-ros-base` contains no graphical tools; RViz runs on the lab PC.
rosdep installs the remaining dependencies of the workspace in the build step.

**ROS 2 already installed.** Installations made with the former procedure
(key file `/usr/share/keyrings/ros-archive-keyring.gpg` and
`/etc/apt/sources.list.d/ros2.list`) carry the signing key
`F42ED6FBAB17C654`, which has expired. apt then rejects the ROS repository and
keeps its old package index; `apt install` and `rosdep install` request
package versions that no longer exist on the server and fail with
`404 Not Found`. Check:

```bash
sudo apt update 2>&1 | grep -i "packages.ros.org"
# "EXPKEYSIG F42ED6FBAB17C654" or "signatures were invalid" -> migration below
```

Migration to `ros2-apt-source`:

```bash
grep -rl packages.ros.org /etc/apt/sources.list /etc/apt/sources.list.d/   # old entry, usually ros2.list
sudo rm /etc/apt/sources.list.d/ros2.list                                    # each file listed above except ros2.sources
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
echo $ROS_APT_SOURCE_VERSION        # must not be empty
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update                                  # no error for packages.ros.org
sudo apt install --only-upgrade 'ros-humble-*'   # align the installed ROS packages
```

- The old entry is removed before the new one is installed; two entries for
  the same repository with different `Signed-By` keys make apt stop with a
  configuration error.
- `/etc/apt/sources.list.d/ros2.sources` is the entry of `ros2-apt-source`
  and is never removed. After an earlier installation attempt, `grep` lists it
  next to the old entry.
- `--only-upgrade 'ros-humble-*'` brings the installed ROS packages to the
  versions of the current index, so that new packages and installed ones
  match. A full `sudo apt upgrade` also updates the NVIDIA packages of L4T.

**Only one ROS 2 installation.** The workspace is built against
`/opt/ros/humble`. A second installation, for example a source build in
`~/ros2_humble`, is removed from `~/.bashrc`. A workspace built while that
installation was sourced keeps loading it through its `install/setup.bash`,
also after the change to `~/.bashrc`; such a workspace is rebuilt from scratch
([Build](#build-1)):

```bash
grep -n "setup.bash" ~/.bashrc   # only /opt/ros/humble and ~/ros2_ws/install remain
rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log
```

### Project dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-tf2-tools

sudo rosdep init
rosdep update
```

With the physical robot, SLAM Toolbox and Nav2 run on the Jetson;
`robot_localization` provides the EKF that publishes `odom -> base_footprint`
(`ekf_hw.yaml`). On a Jetson with ROS 2 already installed, `sudo rosdep init`
reports that the sources list already exists; `rosdep update` is still
required.

### Repository

```bash
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git ~/ros2_ws
```

The repository itself is the workspace `~/ros2_ws`. If `~/ros2_ws` already
exists, for example a workspace created by hand with the driver sources,
`git clone` stops with `destination path '…/ros2_ws' already exists and is not
an empty directory`. The existing directory is moved aside, and the driver
sources are moved back into `src/` after the clone:

```bash
mv ~/ros2_ws ~/ros2_ws_old
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git ~/ros2_ws
mv ~/ros2_ws_old/src/sllidar_ros2 ~/ros2_ws/src/       # if present
mv ~/ros2_ws_old/src/zed-ros2-wrapper ~/ros2_ws/src/   # if present
```

Driver sources moved back are not cloned again in
[Hardware dependencies](#hardware-dependencies).

A repository cloned into the existing workspace instead, for example as
`~/ros2_ws/Waveshare-Cobra-Flex-ROS2-Autonomous-Car`, builds but does not run:
`colcon` finds packages in every subdirectory, whereas
`rosdep install --from-paths src` reads only `src/`. The dependencies of the
project packages are then never installed, and the launch files stop with
`package 'joint_state_publisher' not found` or a similar error.

Check:

```bash
git -C ~/ros2_ws remote -v   # origin …/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git
ls ~/ros2_ws/src
```

`ls ~/ros2_ws/src` lists `cobraflex`, `cobraflex_launch_gui`, `cobraflex_rl`,
`cobraflex_safety_msgs`, `cobraflex_teleop_gui` and `safety_cage`; after
[Hardware dependencies](#hardware-dependencies) and
[Gazebo Harmonic on the robot](#gazebo-harmonic-on-the-robot), also
`sllidar_ros2`, `zed-ros2-wrapper` and `ros_gz`.

### Hardware dependencies

The robot requires two packages that are not available in the apt
repositories. They are built from source in the same workspace:

| Package | Device | Source |
| --- | --- | --- |
| `sllidar_ros2` | RPLIDAR A2M8 or A2M12 | [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) |
| `zed_wrapper` | ZED Mini | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) |

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
```

**ZED SDK.** `zed_wrapper` builds only with the ZED SDK installed, in the
release for the exact L4T version of the Jetson; the installer aborts on any
other L4T version ([ZED SDK downloads](https://www.stereolabs.com/developers/release),
[installation on Jetson](https://www.stereolabs.com/docs/installation/jetson)).
The default branch of the wrapper targets the latest ZED SDK; versions for
older SDK releases are listed on its
[releases page](https://github.com/stereolabs/zed-ros2-wrapper/releases).

```bash
cat /etc/nv_tegra_release                          # L4T version, e.g. 36.4
sudo apt install -y zstd
chmod +x ZED_SDK_Tegra_L4T36.4_vX.Y.Z.zstd.run     # file downloaded for that L4T version
./ZED_SDK_Tegra_L4T36.4_vX.Y.Z.zstd.run -- silent  # space between "--" and "silent"
```

**Dependencies of the wrapper.** `zed_wrapper` depends on packages from apt
that rosdep installs in the [build step](#build-1): `zed_msgs` and
`zed_description` (maintained in separate Stereolabs repositories and released
as `ros-humble-zed-msgs` and `ros-humble-zed-description`), `nmea_msgs`,
`geographic_msgs`, `diagnostic_updater`, `robot_localization`, `backward_ros`,
`image_transport_plugins` and `nlohmann-json3-dev`. When rosdep has not
installed them, `zed_components` stops in CMake with
`Could not find a package configuration file provided by "nlohmann_json"`,
`"nmea_msgs"` or `"zed_msgs"`.

**Serial links.** Access to the serial links of the ESP32-S3 and of the LiDAR
requires membership in the `dialout` group:

```bash
sudo usermod -aG dialout $USER   # effective after the next login
```

### LiDAR setup

The fleet uses two LiDAR models with the same housing and driver:

| | RPLIDAR A2M8 | RPLIDAR A2M12 |
| --- | --- | --- |
| Baud rate | 115200 | 256000 |
| `COBRAFLEX_LIDAR_MODEL` | `a2m8` (default, variable unset) | `a2m12` |

**Cable.** The USB adapter board of the LiDAR is connected to the Jetson with
a micro-USB **data** cable. The 5 V USB cable supplied with the A2M12 only
powers the adapter; no serial device appears. A charge-only micro-USB cable
behaves the same way.

```bash
lsusb | grep 10c4:ea60   # Silicon Labs CP210x: the USB adapter of the LiDAR
ls /dev/ttyUSB*          # /dev/ttyUSB0
```

**Device name.** The udev rule of `sllidar_ros2` gives the adapter the link
`/dev/rplidar` and the permissions `0777`:

```bash
sudo cp ~/ros2_ws/src/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/rplidar       # -> ttyUSB0
```

- The rule matches every CP210x adapter (`10c4:ea60`). The driver board of
  the chassis appears as `/dev/ttyACM*`, so on the robot the LiDAR is the only
  match.
- The rule and the link keep the former name `rplidar`; the package was
  renamed to `sllidar_ros2`, the udev files were not.
- The upstream script `scripts/create_udev_rules.sh` performs the same copy,
  but calls `colcon_cd`, which is not defined in a plain shell; the error it
  prints is harmless when the script is run from `src/sllidar_ros2`.

**Model per car.** `cobraflex_sensors.launch.xml` takes the LiDAR model from
the environment variable `COBRAFLEX_LIDAR_MODEL` and derives the baud rate from
it. Each car with an A2M12 declares its model once:

```bash
echo 'export COBRAFLEX_LIDAR_MODEL=a2m12' >> ~/.bashrc
```

The launch arguments `lidar_model:=`, `lidar_baudrate:=` and
`lidar_serial_port:=` override the defaults for one launch. A wrong baud rate
produces no explicit error: `sllidar_node` reports
`SL_RESULT_OPERATION_TIMEOUT` and exits.

**Upstream launch files.** Slamtec ships the A2M12 launch file as
`sllidar_a2m12_launch .py`, with a space before `.py`; it cannot be included
or started by name. `cobraflex_sensors.launch.xml` therefore starts
`sllidar_node` directly with the parameters of the Slamtec launch files.
`view_sllidar_a2m12_launch.py` is named correctly; it also starts RViz.

**Motor of the A2M12.** The adapter board of the A2M12 drives the motor by
PWM on command of the driver. The motor stays still until a driver starts it;
a still LiDAR before the launch is not a fault.

**Check without ROS.** `tools/lidar_check.py` talks to the LiDAR over the
serial port: device information and health, motor start as in the SDK, and a
4 s scan with samples per second and rotations per second (about 10). It
separates hardware faults (power, cable, adapter) from launch and parameter
faults. The port must be free; no LiDAR node may be running.

```bash
python3 ~/ros2_ws/tools/lidar_check.py /dev/rplidar 256000   # A2M12
python3 ~/ros2_ws/tools/lidar_check.py /dev/rplidar 115200   # A2M8
```

### Gazebo Harmonic on the robot

The worlds and models of the repository are written for Gazebo Harmonic
(system plugins `gz-sim-*`, messages `gz.msgs.*`). Under Humble, the ROS
repository provides Gazebo Fortress, whose names are `ignition-*`. Harmonic is
installed from the OSRF repository; its ROS bridge for Humble,
`ros-humble-ros-gzharmonic`, is not available for arm64. The bridge is
therefore built from source, branch `humble` of `ros_gz`, which supports
Harmonic ([ros_gz](https://github.com/gazebosim/ros_gz/tree/humble)).

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install gz-harmonic
gz sim --version                       # Gazebo Sim, version 8.x

echo "export GZ_VERSION=harmonic" >> ~/.bashrc
export GZ_VERSION=harmonic

cd ~/ros2_ws/src
git clone -b humble https://github.com/gazebosim/ros_gz.git
cd ros_gz
for p in ros_gz ros_gz_image ros_gz_point_cloud ros_gz_sim_demos test_ros_gz_bridge ros_ign*; do
  touch "$p/COLCON_IGNORE"
done
```

- `GZ_VERSION=harmonic` selects the Harmonic libraries in the `package.xml`
  files of `ros_gz`. Without it, rosdep resolves the Fortress dependencies
  (`ignition-gazebo6`) and installs a second Gazebo. The variable is set
  before rosdep and before the build.
- The workspace uses `ros_gz_sim` and `ros_gz_bridge`, together with their
  dependency `ros_gz_interfaces`. `COLCON_IGNORE` excludes the other packages
  of the repository from rosdep and from the build.
- `gazebo.launch.py` starts the Gazebo server and the GUI. After the launch is
  stopped, the server (`gz sim -r -s …`) can remain running and occupy the
  CPU: `pgrep -af "gz sim"`, `pkill -f "gz sim"`.

### Build

**Memory.** The Orin Nano shares 8 GB between CPU and GPU. `ros_gz_bridge` and
`zed_components` need much memory per compiler process; with all cores in
parallel, the kernel kills the compiler and the build stops without a compiler
error. A swap file on the disk (the zram of JetPack is not sufficient) and a
limited number of parallel jobs prevent this:

```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
echo "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab   # keep after reboot
```

**Dependencies and build:**

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
rosdep update
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

- rosdep also installs the dependencies of the hardware packages and of
  `ros_gz`, which are now part of `src/`. Its output ends without
  `ERROR: the following rosdeps failed to install`; with `-r` it continues
  after errors, which then appear only later as CMake errors. `404 Not Found`
  in its output indicates the expired key
  ([ROS 2 Humble](#ros-2-humble)).
- `rosdep update` refreshes the rosdep index; with an old index, keys such as
  `zed_description` cannot be resolved.
- No keys are skipped: `ros_gz_sim` and `ros_gz_bridge` are in `src/` and
  `--ignore-src` excludes them. Without `ros_gz` in `src/`, rosdep would
  install the Fortress bridge from the ROS repository.
- `--parallel-workers 1` and `MAKEFLAGS="-j2"` limit the build to two compiler
  processes; the full build takes considerably longer than on the lab PC.
  With `MAKEFLAGS="-j1"`, the memory use is lowest.
- `CMAKE_BUILD_TYPE=Release` follows the build instructions of the ZED
  wrapper.
- A single package is rebuilt with `--packages-select <package>`; the
  complete compiler output is shown with `--event-handlers console_direct+`.

---

## Interoperability of Humble and Jazzy

### Communication between the lab PC and the robot

The lab PC and the robot form one ROS 2 graph across the two releases
([09 · Multi-machine networking](concepts/09_multi_machine_networking.md)).
Nodes of Humble and Jazzy exchange messages when both machines use the same
middleware and the interface has the same definition in both releases.

| Condition | Setting |
| --- | --- |
| Same RMW implementation | Fast DDS (`rmw_fastrtps_cpp`), the default of Humble and Jazzy: `RMW_IMPLEMENTATION` unset on both machines, or set to the same value |
| Same domain | Identical `ROS_DOMAIN_ID` on the robot and its PC ([3_ROBOT_OPERATION.md §4.1](3_ROBOT_OPERATION.md#41-environment-of-each-terminal)) |
| Same repository version | The same commit built on both machines; each machine generates the messages of `cobraflex_safety_msgs` from its own copy ([3_ROBOT_OPERATION.md §4.2](3_ROBOT_OPERATION.md#42-software-update)) |

The interfaces between the PC and the robot are topics with standard message
types whose definitions are the same in both releases, for example `/cmd_vel`
(`geometry_msgs/Twist`), `/scan`, the camera images, `/tf`, `/map`,
`/initialpose` and `/goal_pose`.

Several interfaces of Nav2 (`nav2_msgs`) differ between the releases; the
result of the action `NavigateToPose`, for example, is empty in Humble and
carries an error code and message in Jazzy. Programs that use the Nav2 actions,
such as `nav2_simple_commander` scripts, therefore run on the same machine as
Nav2. From the PC, goals are set with the RViz tools *2D Pose Estimate* and
*2D Goal Pose*, which publish `/initialpose` and `/goal_pose`.

### Release-dependent files and variables

| Item | Lab PC, Jazzy | Robot, Humble |
| --- | --- | --- |
| `ROS_DISTRO` | `jazzy` | `humble` |
| Nav2 parameters | `nav2_params_jazzy.yaml` | `nav2_params.yaml` |
| Gazebo–ROS bridge | `ros-jazzy-ros-gz` | `ros_gz`, branch `humble`, built from source against Gazebo Harmonic |
| `GZ_VERSION` | Not required | `harmonic` |

- `navigation.launch.py` selects the Nav2 parameter file from `ROS_DISTRO`.
  The Jazzy file uses the plugin names of Jazzy, has no `plugin_lib_names` and
  adds the sections `collision_monitor` and `docking_server`. A change to one
  file is applied to the other.
- Simulation terminals on the PC set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
  ([3_ROBOT_OPERATION.md §8](3_ROBOT_OPERATION.md#8-simulation)).
  `ROS_LOCALHOST_ONLY=1`, the Humble form, has the same effect under Jazzy and
  logs a deprecation warning.

### Platform outlook

The robots remain on JetPack 6 and the lab PCs on Ubuntu 24.04:

| Machine | Platform | Support of the ROS 2 release |
| --- | --- | --- |
| Lab PC | Ubuntu 24.04, ROS 2 Jazzy | Until May 2029 |
| Robot | JetPack 6 (Ubuntu 22.04), ROS 2 Humble | Until May 2027 |

JetPack 7.2 (Jetson Linux 39.2) brings the Orin family to Ubuntu 24.04. With
JetPack 7.2 and ROS 2 Jazzy on the Jetsons, both machines run one ROS 2
release: the release differences of this section (`nav2_msgs`,
[release-dependent files](#release-dependent-files-and-variables)) no longer
apply, and the Gazebo bridge is installed from apt instead of built from
source ([Gazebo Harmonic on the robot](#gazebo-harmonic-on-the-robot)).

The migration is not done yet. It requires:

- reflashing every Jetson: JetPack 7.2 is installed from an ISO image via USB;
  the Orin Nano Developer Kit has no SD card image for it;
- a ZED SDK release for L4T 39.x;
- a retest of the CSI lane camera.

Ubuntu 26.04 is not an option for the robot: no JetPack release is based on it.

---

## Verification

### Lab PC

```bash
source ~/ros2_ws/install/setup.bash

# 1. Package list
ros2 pkg list | grep -E 'cobraflex|safety_cage'
#    cobraflex
#    cobraflex_launch_gui
#    cobraflex_rl
#    cobraflex_safety_msgs
#    cobraflex_teleop_gui
#    safety_cage

# 2. Robot description and world
ros2 launch cobraflex gazebo.launch.py

# 3. Second terminal: TF tree
ros2 run tf2_tools view_frames
```

A correct tree has exactly one publisher of `odom -> base_footprint`. A robot
model that jumps in RViz indicates a second publisher of that edge; see
[Mathematical Model § Odometry](../assets/Mathematical%20Model/Kinematics.md).

### Robot

```bash
source ~/ros2_ws/install/setup.bash

# 1. Package list
ros2 pkg list | grep -E 'cobraflex|safety_cage|sllidar_ros2|zed_wrapper|ros_gz'
#    cobraflex
#    cobraflex_launch_gui
#    cobraflex_rl
#    cobraflex_safety_msgs
#    cobraflex_teleop_gui
#    ros_gz_bridge
#    ros_gz_interfaces
#    ros_gz_sim
#    safety_cage
#    sllidar_ros2
#    zed_wrapper

# 2. Gazebo
gz sim --version                 # Gazebo Sim, version 8.x
echo $GZ_VERSION                 # harmonic

# 3. LiDAR without ROS (no LiDAR node running)
python3 ~/ros2_ws/tools/lidar_check.py /dev/rplidar
#    ... LiDAR answers at 256000 baud: hardware OK.   (115200 for the A2M8)

# 4. LiDAR with ROS (baud rate of the model of this car)
ros2 run sllidar_ros2 sllidar_node --ros-args -p channel_type:=serial \
  -p serial_port:=/dev/rplidar -p serial_baudrate:=256000 -p frame_id:=lidar_link
ros2 topic hz /scan              # second terminal: about 10 Hz
```

Gazebo on the robot, with a monitor attached to the Jetson:
`ros2 launch cobraflex gazebo.launch.py`.

Bring-up of the platform and the sensors, and the connection to the lab PC:
[3_ROBOT_OPERATION.md §4 to §6](3_ROBOT_OPERATION.md#4-preparation).

---

## Troubleshooting

### General

| Symptom | Cause | Solution |
| --- | --- | --- |
| `Unable to locate package ros-jazzy-…` or `ros-humble-…` | Release for another Ubuntu version: Jazzy packages exist for Ubuntu 24.04, Humble packages for 22.04 | Ubuntu 24.04 (`lsb_release -cs`: `noble`) on the lab PC, 22.04 (`jammy`) on the Jetson |
| apt output shows `resolute`; `Unable to locate package ros-jazzy-desktop` | Ubuntu 26.04, whose ROS 2 release is Lyrical | Ubuntu 24.04 on the lab PC ([Lab PC](#lab-pc-ubuntu-2404-and-ros-2-jazzy)) |
| `NO_PUBKEY F42ED6FBAB17C654` together with `curl: command not found` | Download of the signing key failed because `curl` was missing; the broken repository entry makes `apt update` fail, which stops every `&&` chain | `sudo rm /etc/apt/sources.list.d/ros2.list`, `sudo apt update`, `sudo apt install curl`; then the `ros2-apt-source` commands one by one ([ROS 2 Jazzy](#ros-2-jazzy), [ROS 2 Humble](#ros-2-humble)) |
| `git clone`: `destination path '…/ros2_ws' already exists and is not an empty directory` | `~/ros2_ws` exists, for example a workspace created by hand | Move it aside, clone, move the driver sources back ([Repository](#repository-1)) |
| Python changes have no effect | Built without `--symlink-install` | Rebuild with the flag |
| `zed_wrapper` does not build | ZED SDK missing or not the release for the L4T version; on JetPack 6, JetPack development packages missing | ZED SDK as in [Hardware dependencies](#hardware-dependencies); `sudo apt install nvidia-jetpack nvidia-jetpack-dev` |
| Serial permission denied | User not in `dialout` | `sudo usermod -aG dialout $USER`, then log in again |
| Robot model jumps in RViz | More than one publisher of `odom -> base_footprint` | Check the DiffDrive plugin, the OdometryPublisher and the EKF |
| Nav2 does not start | No saved map | Run the mapping first; see [Usage](2_USAGE.md#1-slam-and-mapping) |
| On the lab PC, `bt_navigator` stops with `ID [ComputePathToPose] already registered`, or `planner_server` does not configure | Humble parameter file under Jazzy, passed with `params_file:=` | Default `params_file` of `navigation.launch.py`, or `nav2_params_jazzy.yaml` |
| Nav2 action called from the PC fails against Nav2 on the robot | `nav2_msgs` differs between Jazzy and Humble | Program on the robot; from the PC, *2D Goal Pose* in RViz |
| Warning `ROS_LOCALHOST_ONLY is deprecated` on the lab PC | Humble variable under Jazzy; still effective | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` instead ([3_ROBOT_OPERATION.md §8](3_ROBOT_OPERATION.md#8-simulation)) |
| New asset directory missing in `share/` | Directory not listed in `setup.py` `data_files` | Add it there and rebuild |

### Robot

| Symptom | Cause | Solution |
| --- | --- | --- |
| `apt update`: `EXPKEYSIG F42ED6FBAB17C654` for `packages.ros.org`; apt or rosdep: `404 Not Found` for `ros-humble-…` | Expired ROS signing key; apt keeps the old package index | Migration to `ros2-apt-source` ([ROS 2 Humble](#ros-2-humble)) |
| rosdep: `Cannot locate rosdep definition for [zed_description]` | Old rosdep index | `rosdep update` |
| `zed_components`: `Could not find a package configuration file provided by "nlohmann_json"`, `"nmea_msgs"` or `"zed_msgs"` | rosdep not run, or failed on apt errors | `rosdep install` as in [Build](#build-1); its output without `failed to install` |
| `package 'joint_state_publisher' not found`, or another `exec_depend` of `cobraflex` | `~/ros2_ws` is not the repository; rosdep did not see the project packages | [Repository](#repository-1) |
| `Package 'sllidar_ros2' not found` after a build | Build stopped by another package, or workspace not sourced | Fix the failing package or `colcon build --packages-up-to sllidar_ros2`; `source install/setup.bash` |
| No `/dev/ttyUSB0`; `sllidar_node`: `Error, unexpected error, code: 80008004` | LiDAR connected with the 5 V power cable or a charge-only cable | Micro-USB data cable; `lsusb \| grep 10c4:ea60` ([LiDAR setup](#lidar-setup)) |
| `sllidar_node`: `SL_RESULT_OPERATION_TIMEOUT` | Baud rate of the other model; or LiDAR not answering (power, flat cable to the adapter) | `COBRAFLEX_LIDAR_MODEL`; `tools/lidar_check.py` to separate hardware from configuration |
| `No such file or directory: …/sllidar_a2m12_launch.py` | Upstream file name contains a space (`sllidar_a2m12_launch .py`) | `cobraflex_sensors.launch.xml`, which starts the node directly |
| A2M12 motor still after power-on | Normal: the motor is started by the driver | None; `tools/lidar_check.py` starts it for a test |
| `tools/lidar_check.py`: no answer while the launch works | Port still used by a running LiDAR node | Stop the launch; the script lists the processes that hold the port |
| `Unable to locate package ros-humble-ros-gzharmonic` | Not available for arm64 | `ros_gz` from source ([Gazebo Harmonic on the robot](#gazebo-harmonic-on-the-robot)) |
| `ros_gz_sim`: `Could not find a package configuration file provided by "gz-math7"` | Gazebo Harmonic not installed, usually because the OSRF repository was not added | [Gazebo Harmonic on the robot](#gazebo-harmonic-on-the-robot); `dpkg -l libgz-math7-dev` shows `ii` |
| `package 'ros_gz_sim' not found` or `package 'ros_gz_bridge' not found` | `ros_gz` missing in `src/`, or its build stopped | [Gazebo Harmonic on the robot](#gazebo-harmonic-on-the-robot); `colcon build --packages-select ros_gz_bridge` |
| Build stops without a compiler error; `sudo dmesg \| grep -i oom` shows a killed `cc1plus` | Out of memory | Swap file and `MAKEFLAGS="-j1"` ([Build](#build-1)) |
| colcon: `The path '…/ros2_ws/src/install' in the environment variable COLCON_PREFIX_PATH doesn't exist` | A build was once run inside `src/`, and its install space was sourced | `rm -rf ~/ros2_ws/src/{build,install,log}`; remove the line that sources `src/install/setup.bash` from `~/.bashrc`; new terminal |
| rosdep installs `ignition-*` packages (Gazebo Fortress) | `GZ_VERSION` not set during rosdep | `export GZ_VERSION=harmonic`; remove the Fortress packages (`apt list --installed \| grep ignition`) |
| `gz sim` keeps using the CPU after Gazebo was closed | Server process `gz sim -s` left running | `pkill -f "gz sim"` |

---

## Further documentation

- [2_USAGE.md](2_USAGE.md): SLAM, navigation and lane-keeping controllers
- [3_ROBOT_OPERATION.md](3_ROBOT_OPERATION.md): operation of the physical robot
- [09 · Multi-machine networking](concepts/09_multi_machine_networking.md): DDS discovery, domain IDs, Wi-Fi
- [Mathematical Model](../assets/Mathematical%20Model/README.md): kinematics, control and parameters
