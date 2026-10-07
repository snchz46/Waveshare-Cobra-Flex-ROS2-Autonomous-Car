# 05 · Localisation and navigation

> **In one sentence:** on a saved map, AMCL localises the robot with a particle
> filter, NavFn plans a global path on the costmap, and DWB samples velocity
> commands that follow it without hitting anything — all orchestrated by Nav2's
> behaviour tree.

[← 04 SLAM and mapping](04_slam_and_mapping.md) · [Concepts](README.md) · Next: [06 Lane perception and control →](06_lane_perception_and_control.md)

---

## Concept

### Monte Carlo localisation (AMCL)

The belief over the pose is a set of weighted samples (particles). Each cycle:

1. **Predict** — move every particle with the odometry increment plus noise
   from the motion model.
2. **Weight** — score every particle by how well the current scan fits the map
   from that pose. The *likelihood field* model looks up, for each beam
   endpoint, the distance to the nearest obstacle in the map.
3. **Resample** — draw a new set in proportion to the weights.

*Adaptive* MCL changes the number of particles with the spread of the belief
(KLD sampling): many while the pose is uncertain, few once it has converged.

### Nav2 in one picture

```mermaid
flowchart LR
    BT["BT Navigator<br/>behaviour tree"] --> PL["Planner server<br/>NavFn"]
    PL --> SM["Smoother"]
    SM --> CT["Controller server<br/>DWB"]
    CT -- "/cmd_vel" --> R["Robot"]
    BT --> BH["Behaviour server<br/>spin, back up, wait"]
    GC["Global costmap<br/>static + obstacle + inflation"] --> PL
    LC["Local costmap<br/>obstacle + inflation, rolling"] --> CT
```

### Global planning: Dijkstra and A*

The global costmap is a graph: cells are nodes, neighbours are edges, the cell
cost is the edge weight. **Dijkstra** expands nodes in order of the cost $g$
from the start. **A*** expands in order of $f = g + h$, where $h$ estimates the
remaining cost to the goal. With an admissible heuristic (one that never
overestimates, such as the straight-line distance) A* returns the same optimal
path while expanding fewer nodes. NavFn computes a navigation function from the
goal with Dijkstra by default; `use_astar` switches it to A*.

### Costmaps and inflation

A costmap stacks layers: the **static** layer (the saved map), the
**obstacle** layer (live scans mark obstacles and clear free space by ray
tracing) and the **inflation** layer, which spreads cost around obstacles so
the planner keeps a margin. Within the inscribed radius of the robot the cost
is 253 (certain collision); beyond it, up to `inflation_radius`, it decays as

$$
\text{cost}(d) = 252 \cdot e^{-k\,(d - r_{\text{inscribed}})}
$$

with $k$ = `cost_scaling_factor`. A larger $k$ lets paths hug obstacles; a
smaller one pushes them towards the middle of free space.

### Local control: the dynamic window

DWB samples pairs $(v_x, \omega_z)$ that are reachable within the acceleration
limits during the next control period (the *dynamic window*), simulates each
pair forward for `sim_time` seconds, scores every resulting trajectory with a
weighted sum of **critics** (distance to the path, alignment with the goal,
obstacle cost…) and sends the best one. It repeats this at the controller
frequency.

### Behaviour trees

Nav2 encodes "plan, follow, recover when stuck" as a behaviour tree: a tree of
conditions and actions ticked from the root. It plays the role that state
machines and state charts play in behaviour planning, and is easier to extend
with recoveries such as spinning or backing up.

---

## In this repository

Everything lives in [`nav2_params.yaml`](../../src/cobraflex/config/nav2_params.yaml),
launched by [`navigation.launch.py`](../../src/cobraflex/launch/navigation.launch.py)
together with the map server, AMCL and RViz.

**AMCL**

| Parameter | Value | Meaning |
| --- | --- | --- |
| `robot_model_type` | `DifferentialMotionModel` | Skid-steer treated as differential drive |
| `alpha1`…`alpha5` | 0.2 each | Motion-model noise |
| `laser_model_type` | `likelihood_field` | Sensor model |
| `min_particles` / `max_particles` | 500 / 2000 | KLD-sampling bounds |
| `update_min_d` / `update_min_a` | 0.05 m / 0.2 rad | Movement before a filter update |
| `laser_max_range` | 8.0 m | Beams beyond this are ignored |

**Planner and smoother**

| Parameter | Value |
| --- | --- |
| Planner plugin | `nav2_navfn_planner/NavfnPlanner` (`GridBased`) |
| `use_astar` | `false` — Dijkstra |
| `tolerance` | 0.25 m |
| `allow_unknown` | `true` |
| Smoother | `nav2_smoother::SimpleSmoother` |

**Controller (DWB)** — `controller_frequency` 20 Hz

| Parameter | Value |
| --- | --- |
| `min_vel_x` / `max_vel_x` | −0.15 / 0.35 m/s |
| `max_vel_theta` | 2.0 rad/s |
| `acc_lim_x` / `acc_lim_theta` | 2.5 m/s² / 3.2 rad/s² |
| `vx_samples` × `vtheta_samples` | 20 × 20 |
| `sim_time` | 1.5 s |
| Critics (scale) | RotateToGoal, Oscillation, BaseObstacle (0.02), PathAlign (32), PathDist (32), GoalAlign (24), GoalDist (24) |
| Goal tolerance | 0.05 m, 0.10 rad |

**Costmaps** — resolution 0.025 m, footprint 0.228 × 0.180 m

| | Local | Global |
| --- | --- | --- |
| Layers | obstacle, inflation | static, obstacle, inflation |
| Size | rolling window 2 × 2 m | the map |
| Update / publish | 5 Hz / 2 Hz | 1 Hz / 1 Hz |
| `inflation_radius` | 0.30 m | 0.30 m |
| `cost_scaling_factor` | 3.0 | 3.0 |
| Obstacle range / ray trace | 6.0 m / 8.0 m | 6.0 m / 8.0 m |

The footprint gives an inscribed radius of 0.090 m and a circumscribed radius
of 0.145 m, so the 0.30 m inflation keeps about 0.2 m of margin beyond the
robot's side.

**Behaviours:** spin, back up, drive on heading, assisted teleop, wait.

---

## Pitfalls

- **Sim time on the car.** `navigation.launch.py` defaults to
  `use_sim_time:=true`. On the car pass `use_sim_time:=false`, and check the
  launch on hardware before a class depends on it.
- **No initial pose, no plan.** AMCL starts unlocalised; set the pose with
  *2D Pose Estimate* in RViz first.
- **No map.** The launch expects `cobraflex_map.yaml` in
  `src/cobraflex/maps` unless you pass `map:=`.
- **Tight goal tolerance.** 0.05 m is small compared with the car's
  odometry drift; near the goal the robot may hunt back and forth.
- **Two speed limits.** Nav2 plans inside 0.35 m/s and 2.0 rad/s; the driver
  clamps at 0.53 m/s and 6.0 rad/s ([02](02_system_architecture.md)).

---

## Try it

```bash
# Gazebo, with a saved map (see Usage §2)
ros2 launch cobraflex gazebo.launch.py
ros2 launch cobraflex navigation.launch.py
# RViz: 2D Pose Estimate, then Nav2 Goal
```

Exercises:

1. Set `use_astar: true` in `nav2_params.yaml`, relaunch (rebuild first unless
   you built with `--symlink-install`) and compare the path and the planning
   time with Dijkstra for the same start and goal.
2. Halve `inflation_radius` to 0.15 m and watch where the paths run.
3. Drive into a dead end and watch the behaviour tree call its recoveries.

---

## Lecture links

- **SLAM 09** — particle filters and Monte Carlo localisation: AMCL is MCL
  with adaptive sample size.
- **Motion Planning part 1** — graph search and A*: NavFn on the costmap.
- **Motion Planning part 2** — sampling-based and optimisation-based planning
  (PRM, RRT, MPC): contrast them with DWB's sampled velocities.
- **Motion Planning part 3** — behaviour planning with state machines: compare
  with Nav2's behaviour tree.

## Further reading

- Nav2 documentation: <https://docs.nav2.org>
- Nav2 costmap and inflation tuning guide: <https://docs.nav2.org/tuning/index.html>
- D. Fox, W. Burgard, S. Thrun, "The dynamic window approach to collision avoidance", *IEEE Robotics & Automation Magazine*, 1997.
