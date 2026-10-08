# 04 · SLAM and mapping

> **Summary:** SLAM Toolbox builds a pose graph of laser scans, corrects it by
> scan matching (and, in Gazebo, by loop closure), and rasterises it into the
> 1 cm occupancy grid used by Nav2.

[← 03 State estimation](03_state_estimation.md) · [Concepts](README.md) · Next: [05 Localisation and navigation →](05_localization_and_navigation.md)

---

## Theory

### Problem statement

SLAM estimates the robot trajectory and the map simultaneously:

$$
p(x_{1:t},\, m \mid z_{1:t},\, u_{1:t})
$$

Neither quantity is known in advance: localisation requires the map, and
mapping requires the pose.

### Occupancy grids

The map is a grid of cells, each holding the probability of being occupied.
For numerical stability each cell stores **log-odds** and accumulates the
evidence of every scan:

$$
l_{t,i} = l_{t-1,i} + \log\frac{p(m_i \mid z_t, x_t)}{1 - p(m_i \mid z_t, x_t)} - l_0
$$

The inverse sensor model marks the cells along a laser beam as free and the
cell at its end as occupied.

### SLAM families

| Approach | Principle | Cost |
| --- | --- | --- |
| EKF-SLAM | One Gaussian over the pose and all landmarks | Quadratic in the number of landmarks |
| FastSLAM | Particle filter over trajectories, one small map per particle (Rao-Blackwellisation) | Linear in particles × landmarks |
| **Graph SLAM** (used here) | Poses are nodes, measurements are edges; the solution is the set of poses that best satisfies all edges | Sparse least squares |

### Graph SLAM

When the robot has moved a minimum distance, a node with its pose and scan is
added to the graph. **Scan matching** links it to previous nodes: a search over
a small window of $(x, y, \theta)$ finds the pose at which the new scan best
overlaps the existing map. When a scan also matches a region visited earlier, a
**loop closure** edge is added. The optimiser then adjusts all poses to
minimise the weighted squared error of every edge,

$$
x^* = \arg\min_x \sum_{(i,j)} e_{ij}(x_i, x_j)^\top\, \Omega_{ij}\, e_{ij}(x_i, x_j)
$$

which distributes the accumulated drift over the whole loop. The grid is then
redrawn from the corrected poses.

---

## Implementation

SLAM Toolbox runs in asynchronous mapping mode, started by
[`mapping.launch.py`](../../src/cobraflex/launch/mapping.launch.py) in Gazebo
and [`cobraflex_mapping.launch.py`](../../src/cobraflex/launch/cobraflex_mapping.launch.py)
on the robot. Frames: `map_frame: map`, `odom_frame: odom`,
`base_frame: base_footprint`; scans from `/scan`. It publishes `map → odom`
every 0.02 s.

| Parameter | Gazebo ([`slam_toolbox_mapping.yaml`](../../src/cobraflex/config/slam_toolbox_mapping.yaml)) | Robot ([`slam_toolbox_mapping_hw.yaml`](../../src/cobraflex/config/slam_toolbox_mapping_hw.yaml)) | Function |
| --- | --- | --- | --- |
| `resolution` | 0.01 m | 0.01 m | Grid cell size |
| `max_laser_range` | 8.0 m | 20.0 m | Range used to rasterise the map |
| `map_update_interval` | 1.0 s | 0.5 s | `/map` publication period |
| `minimum_travel_distance` | 0.5 m | 0.1 m | Translation before a new graph node |
| `minimum_travel_heading` | 0.5 rad | 0.1 rad | Rotation before a new graph node |
| `do_loop_closing` | `true` | `false` | Loop closure |
| `loop_search_maximum_distance` | 3.0 m | 3.0 m | Search radius for loop candidates |
| `solver_plugin` | Ceres, `SPARSE_NORMAL_CHOLESKY` | Ceres, `SPARSE_NORMAL_CHOLESKY` | Pose-graph optimiser |

The hardware configuration adds nodes five times more often and disables loop
closure. The file records no rationale for these values; they are settings to
be evaluated. Further SLAM parameters are listed in
[parameters.md §5.1](../../assets/Mathematical%20Model/parameters.md).

**Gazebo and robot.** In Gazebo the ground-truth odometry publishes
`odom → base_footprint`, so SLAM corrects an odometry without drift. On the
robot it corrects the EKF visual odometry ([03](03_state_estimation.md)); the
difference between both maps reflects the drift.

**Map storage.** The 1 cm grid is finer than the 0.025 m Nav2 costmaps; the
static layer resamples it. Maps are site-specific and not tracked in git.
Saving procedure and expected location:
[maps/README](../../src/cobraflex/maps/README.md).

---

## Common errors

- **Fast in-place rotation.** Scans arrive at about 10 Hz; a fast in-place
  rotation leaves little overlap between consecutive scans and scan matching
  fails. Mapping requires low speed.
- **Surfaces invisible to the LiDAR.** Glass, mirrors and matte black surfaces
  return few or no points; walls lower than the scan plane are not detected.
- **Map size.** A 1 cm map of a large area grows quickly. For areas larger
  than a room, `resolution` is set to 0.02–0.05 m.
- **LiDAR mounting.** The LiDAR is rotated half a turn (`lidar_joint`
  yaw = π). The URDF accounts for this rotation; scan angles must not be
  corrected elsewhere.

---

## Commands

```bash
# Gazebo (see Usage §1)
ros2 launch cobraflex gazebo.launch.py
ros2 launch cobraflex mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Robot: layers 1 and 2 running
ros2 launch cobraflex cobraflex_mapping.launch.py

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/cobraflex/maps/cobraflex_map
```

Exercise: map the arena once with `do_loop_closing: false` and once with
`true`, driving the same loop in both runs, and compare the alignment of the
start and end of the loop.

---

## Lecture references

- **SLAM 11**: occupancy grid maps and the log-odds update.
- **SLAM 06** (EKF-SLAM) and **SLAM 10** (FastSLAM): comparison with the graph
  SLAM used here.
- **SLAM 04**: sensor models; the beam model behind the inverse sensor model.

## Further reading

- SLAM Toolbox: <https://github.com/SteveMacenski/slam_toolbox>
- S. Macenski, I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world", *Journal of Open Source Software*, 2021.
- G. Grisetti et al., "A tutorial on graph-based SLAM", *IEEE ITS Magazine*, 2010.
