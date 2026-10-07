# 09 · Multi-machine networking

> **In one sentence:** the car and the lab PC form one ROS 2 graph over Wi-Fi as
> long as they share a domain, discovery traffic gets through, the clocks agree
> and nobody streams raw images over the air.

[← 08 Safety cage](08_safety_cage.md) · [Concepts](README.md)

---

## Concept

### Discovery and domains

DDS finds participants without a central master. Each participant announces
itself by **multicast** (SPDP), then the participants exchange their
publishers and subscribers (SEDP) and connect by unicast. The **domain ID**
selects the UDP ports: discovery uses port $7400 + 250 \cdot \text{ROS\_DOMAIN\_ID}$,
so different domains never see each other even on the same network. On Linux
the safe range is 0–101. Domain 0 is the default: a machine with nothing set
joins it.

For work that must stay on one machine (Gazebo exercises),
`ROS_LOCALHOST_ONLY=1` (Humble) keeps all traffic local — otherwise a
simulated `/cmd_vel` can reach a real car on the same domain.

### Wi-Fi is not Ethernet

- **Shared air.** All clients of one access point share its channel; every
  byte one car sends costs airtime for the others.
- **Multicast is slow and unacknowledged.** Wi-Fi sends multicast at the lowest
  basic rate without retries, so discovery packets get lost more easily than
  data. If nodes appear and vanish, look at the access point's multicast
  settings (IGMP snooping, multicast-to-unicast) first.
- **Reliable QoS over a lossy link** resends large messages fragment by
  fragment; one lost fragment delays the whole message. Sensor topics use best
  effort.

### Bandwidth

Raw images are large: width × height × bytes per pixel × rate. Compressed
transport (`image_transport` JPEG) cuts a 640×360 frame by one to two orders of
magnitude.

### Time

TF lookups compare time stamps from different machines. If the car's clock is
off by more than the TF buffer tolerates, RViz on the PC reports
"extrapolation into the future". The clocks must agree with each other; being
correct matters less.

---

## In this repository

### What each sensor would cost raw

Rates from [`zed_common_stereo.yaml`](../../src/cobraflex/config/zed_common_stereo.yaml)
(`pub_resolution: CUSTOM`, `pub_downscale_factor: 2.0`, `pub_frame_rate: 15.0`,
`point_cloud_freq: 10.0`) and the CSI camera node (640×360 at 20 Hz). The ZED
numbers assume the HD720 grab resolution set in the wrapper's camera file; at
HD1080 they grow by 2.25×.

| Stream | Format | Raw |
| --- | --- | --- |
| ZED RGB, 640×360 at 15 Hz | `bgra8` | ≈ 111 Mbit/s |
| ZED depth, 640×360 at 15 Hz | `32FC1` | ≈ 111 Mbit/s |
| ZED point cloud, 640×360 at 10 Hz | 16 bytes per point | ≈ 295 Mbit/s |
| Lane camera, 640×360 at 20 Hz | `bgr8` | ≈ 111 Mbit/s |
| LiDAR scan at 10 Hz | ~1000 points | < 1 Mbit/s |
| TF, odometry, `/cmd_vel` | small | < 1 Mbit/s |

Opening everything raw in RViz on the PC asks for about 630 Mbit/s per car —
more than a Wi-Fi 5 client delivers, with several cars sharing one channel.
Compressed RGB and lane images need roughly 10 Mbit/s per car.

### Rules for the lab network

1. **One domain per setup.** Set `ROS_DOMAIN_ID` (1–N, never 0) in `~/.bashrc`
   on the car and on its PC.
2. **Wire what does not move.** PCs on Ethernet; only the cars on Wi-Fi (5 GHz,
   fixed non-DFS channel 36–48).
3. **Process on the car.** Raw images and point clouds stay on the Jetson;
   send compressed images, throttled or downsampled clouds; record rosbags on
   the car and copy them by cable.
4. **Same RMW and distro everywhere.** One DDS implementation and one shared
   DDS configuration for all machines.
5. **Bind DDS to the lab interface** on PCs that are also on another network
   (CycloneDDS `NetworkInterface`, Fast DDS interface allowlist), and give that
   interface no default gateway.
6. **Synchronise clocks** with chrony against one machine on the lab network.

### Kernel settings for lossy links

From the [ROS 2 DDS tuning guide](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html):

```bash
sudo sysctl net.ipv4.ipfrag_time=3
sudo sysctl net.ipv4.ipfrag_high_thresh=134217728
# CycloneDDS, large messages:
sudo sysctl -w net.core.rmem_max=2147483647
```

---

## Pitfalls

- **Nodes visible but no data.** QoS mismatch (a reliable subscriber on a
  best-effort publisher) or a firewall blocking UDP on the lab interface.
- **Nodes appear and vanish after a few minutes.** Multicast filtered by the
  network (IGMP snooping without a querier). Fall back to unicast discovery
  (CycloneDDS peer list or a Fast DDS discovery server).
- **A Gazebo session drives the real car.** Same domain, no
  `ROS_LOCALHOST_ONLY`.
- **TF extrapolation errors on the PC only.** Clock offset between car and PC.

---

## Try it

```bash
# On both machines
echo $ROS_DOMAIN_ID

# From the PC, with the car's layers 1 and 2 running
ros2 node list
ros2 topic hz /scan
ros2 topic bw /zed/zed_node/rgb/image_rect_color             # raw
ros2 topic bw /zed/zed_node/rgb/image_rect_color/compressed  # compressed

# Clock offset between the machines
chronyc tracking
```

Exercise: measure `ros2 topic bw` for the raw and the compressed RGB stream,
over Wi-Fi and over the bench cable, and explain the difference.

---

## Lecture links

None directly: this is lab infrastructure. It explains why the system
architecture of [02](02_system_architecture.md) keeps the heavy processing on
the car.

## Further reading

- ROS 2 domain ID: <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html>
- ROS 2 DDS tuning: <https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html>
- `image_transport`: <https://github.com/ros-perception/image_common>
