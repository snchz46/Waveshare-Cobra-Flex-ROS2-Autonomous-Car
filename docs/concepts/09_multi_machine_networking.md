# 09 · Multi-machine networking

> **Summary:** the robot and the lab PC form one ROS 2 graph over Wi-Fi when
> they share a domain, discovery traffic is delivered, the clocks are
> synchronised and raw images are not transmitted over the air.

[← 08 Safety cage](08_safety_cage.md) · [Concepts](README.md)

---

## Theory

### Discovery and domains

DDS discovers participants without a central master. Each participant
announces itself by **multicast** (SPDP); the participants then exchange their
publishers and subscribers (SEDP) and connect by unicast. The **domain ID**
selects the UDP ports: discovery uses port $7400 + 250 \cdot \text{ROS\_DOMAIN\_ID}$,
so different domains remain isolated on the same network. On Linux the safe
range is 0–101. Domain 0 is the default for any machine without an explicit
setting.

For work restricted to one machine (Gazebo exercises), `ROS_LOCALHOST_ONLY=1`
(Humble) keeps all traffic local. Without it, a simulated `/cmd_vel` can reach
a physical robot in the same domain.

### Wi-Fi characteristics

- **Shared medium.** All clients of an access point share its channel; every
  byte transmitted by one robot consumes airtime of the others.
- **Multicast.** Wi-Fi transmits multicast at the lowest basic rate without
  retransmission, so discovery packets are lost more often than data. When
  nodes appear and disappear, the multicast settings of the access point
  (IGMP snooping, multicast-to-unicast) are the first item to check.
- **Reliable QoS on a lossy link.** Large messages are retransmitted fragment
  by fragment, and one lost fragment delays the whole message. Sensor topics
  therefore use best effort.

### Bandwidth

The raw image rate is width × height × bytes per pixel × frame rate.
Compressed transport (`image_transport` JPEG) reduces a 640×360 frame by one to
two orders of magnitude.

### Time synchronisation

TF lookups compare time stamps from different machines. If the robot clock
deviates by more than the TF buffer tolerance, RViz on the PC reports
"extrapolation into the future". The clocks must agree with each other;
absolute accuracy is secondary.

---

## Implementation

### Raw bandwidth per sensor

Rates from [`zed_common_stereo.yaml`](../../src/cobraflex/config/zed_common_stereo.yaml)
(`pub_resolution: CUSTOM`, `pub_downscale_factor: 2.0`, `pub_frame_rate: 15.0`,
`point_cloud_freq: 10.0`) and the CSI camera node (640×360 at 20 Hz). The ZED
values assume the HD720 grab resolution set in the camera file of the wrapper;
at HD1080 they increase by a factor of 2.25.

| Stream | Format | Raw |
| --- | --- | --- |
| ZED RGB, 640×360 at 15 Hz | `bgra8` | ≈ 111 Mbit/s |
| ZED depth, 640×360 at 15 Hz | `32FC1` | ≈ 111 Mbit/s |
| ZED point cloud, 640×360 at 10 Hz | 16 bytes per point | ≈ 295 Mbit/s |
| Lane camera, 640×360 at 20 Hz | `bgr8` | ≈ 111 Mbit/s |
| LiDAR scan at 10 Hz | ~1000 points | < 1 Mbit/s |
| TF, odometry, `/cmd_vel` | Small | < 1 Mbit/s |

Displaying all raw streams in RViz on the PC requires about 630 Mbit/s per
robot, which exceeds the throughput of a Wi-Fi 5 client, especially with
several robots on one channel. Compressed RGB and lane images require about
10 Mbit/s per robot.

### Lab network rules

1. **One domain per setup.** `ROS_DOMAIN_ID` (1–N, never 0) is set in
   `~/.bashrc` on the robot and on its PC.
2. **Wired stationary machines.** PCs on Ethernet; only the robots on Wi-Fi
   (5 GHz, fixed non-DFS channel 36–48).
3. **Processing on the robot.** Raw images and point clouds remain on the
   Jetson; only compressed images and throttled or downsampled clouds are
   transmitted. Rosbags are recorded on the robot and copied by cable.
4. **Uniform RMW and distribution.** One DDS implementation and one shared DDS
   configuration on all machines.
5. **DDS bound to the lab interface** on PCs connected to another network as
   well (CycloneDDS `NetworkInterface`, Fast DDS interface allowlist); that
   interface has no default gateway.
6. **Clock synchronisation** with chrony against one machine on the lab
   network.

### Kernel settings for lossy links

From the [ROS 2 DDS tuning guide](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html):

```bash
sudo sysctl net.ipv4.ipfrag_time=3
sudo sysctl net.ipv4.ipfrag_high_thresh=134217728
# CycloneDDS, large messages:
sudo sysctl -w net.core.rmem_max=2147483647
```

---

## Common errors

- **Nodes visible, no data.** QoS mismatch (reliable subscriber on a
  best-effort publisher) or a firewall blocking UDP on the lab interface.
- **Nodes disappear after a few minutes.** Multicast filtered by the network
  (IGMP snooping without a querier). Use unicast discovery instead (CycloneDDS
  peer list or Fast DDS discovery server).
- **Gazebo session driving the physical robot.** Same domain without
  `ROS_LOCALHOST_ONLY`.
- **TF extrapolation errors on the PC only.** Clock offset between robot and
  PC.

---

## Commands

```bash
# Both machines
echo $ROS_DOMAIN_ID

# PC, with layers 1 and 2 running on the robot
ros2 node list
ros2 topic hz /scan
ros2 topic bw /zed/zed_node/rgb/image_rect_color             # raw
ros2 topic bw /zed/zed_node/rgb/image_rect_color/compressed  # compressed

# Clock offset between the machines
chronyc tracking
```

Exercise: measure `ros2 topic bw` for the raw and the compressed RGB stream
over Wi-Fi and over the bench cable, and explain the difference.

---

## Lecture references

No direct lecture reference; this page covers lab infrastructure. It explains
why the system architecture of [02](02_system_architecture.md) keeps the
computationally intensive processing on the robot.

## Further reading

- ROS 2 domain ID: <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html>
- ROS 2 DDS tuning: <https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html>
- `image_transport`: <https://github.com/ros-perception/image_common>
