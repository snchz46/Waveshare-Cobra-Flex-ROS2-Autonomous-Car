# 07 · Reinforcement learning

> **In one sentence:** a PPO agent with a convolutional policy learns to keep the
> lane from four stacked 84×84 grayscale camera frames in Gazebo, with the safety
> cage already in the loop during training and the same preprocessing and cage
> on the way to the car.

[← 06 Lane perception and control](06_lane_perception_and_control.md) · [Concepts](README.md) · Next: [08 Safety cage →](08_safety_cage.md)

---

## Concept

### Lane keeping as a Markov decision process

| Element | Here |
| --- | --- |
| State $s_t$ | What the agent observes: camera frames (or, on the state track, $e_y$, $e_\psi$, speed, curvature…) |
| Action $a_t$ | Steering in $[-1, 1]$, optionally throttle as a second dimension |
| Transition $P(s_{t+1} \mid s_t, a_t)$ | The simulated car and track, unknown to the agent |
| Reward $r_t$ | Progress along the lane minus penalties for error, jerk and leaving the lane |
| Discount $\gamma$ | 0.99 |

The agent looks for a policy $\pi_\theta(a \mid s)$ that maximises the expected
discounted return $\mathbb{E}\left[\sum_t \gamma^t r_t\right]$.

### Policy gradients and PPO

Policy-gradient methods adjust $\theta$ in the direction that makes actions with
a positive **advantage** $\hat A_t$ (better than the critic expected) more
likely. **Proximal Policy Optimisation** limits how far one update can move the
policy by clipping the probability ratio
$r_t(\theta) = \pi_\theta(a_t \mid s_t) / \pi_{\theta_\text{old}}(a_t \mid s_t)$:

$$
L^{\text{CLIP}}(\theta) = \mathbb{E}_t\left[\min\left(r_t(\theta)\,\hat A_t,\;
\operatorname{clip}\left(r_t(\theta),\, 1 - \varepsilon,\, 1 + \varepsilon\right)\hat A_t\right)\right]
$$

The advantage comes from **generalised advantage estimation** (GAE),

$$
\hat A_t = \sum_{l \ge 0} (\gamma\lambda)^l\, \delta_{t+l},
\qquad
\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)
$$

where $V$ is the critic. Actor and critic share a convolutional feature
extractor: Stable-Baselines3's `CnnPolicy` uses the "Nature CNN" (three
convolution layers, then a 512-unit dense layer). Stacking four frames gives
the network motion information that a single image lacks.

### Sim-to-real

A policy trained in simulation meets a different camera, lighting, floor and
vehicle response on the car. **Domain randomisation** varies those factors
during training so the policy learns what stays constant — the lane — instead
of the details of one simulator.

---

## In this repository

### Environment

[`GazeboLaneEnv`](../../src/cobraflex_rl/cobraflex_rl/gazebo_lane_env.py) is a
Gymnasium environment around the Gazebo lane-following worlds:

| Setting | Camera track (`train_ppo_camera.yaml`) |
| --- | --- |
| Observation | 84×84 grayscale, 4 stacked frames, from `/camera/image_raw_lane` |
| Action | Steering in $[-1, 1]$ at a fixed 0.20 m/s; `train_ppo_camera_2d.yaml` adds throttle (`steer_throttle`) |
| Control period | 0.10 s |
| Episode length | up to 1024 steps |
| Safety cage | in the loop, in-process, in `enforcement` mode |

The ground-truth pose is used only for the reward, termination and metrics; the
cage reads the CV lane estimator, never the ground truth
([06](06_lane_perception_and_control.md), [08](08_safety_cage.md)). An Isaac Sim
backend ([`isaac_interface.py`](../../src/cobraflex_rl/cobraflex_rl/isaac_interface.py))
exposes the same interface, so the environment runs unchanged on either
simulator.

### Reward (v1.2)

[`rewards.py`](../../src/cobraflex_rl/cobraflex_rl/rewards.py):

$$
r = w_\text{fwd}\max(\text{progress}, 0) - w_{e_y}|e_y| - w_{e_\psi}|e_\psi|
    - w_{\Delta s}|\Delta \text{steer}| - w_\text{term}\,[\text{done}]
$$

(the 2-D action adds throttle-change and stall terms, zero by default).

| Weight | Value in `train_ppo_camera.yaml` |
| --- | --- |
| `forward_progress` | 1.0 |
| `lateral_error` | 2.5 |
| `heading_error` | 0.75 |
| `steer_delta` | 0.20 |
| `termination` | 25.0 |

Two choices are deliberate. Progress is the normalised advance along the lane,
not speed, so every step on track is net positive and ending early is never
attractive. The steering-change penalty uses the **raw** policy output, not the
cage's smoothed one; otherwise the cage's rate limiter would absorb the jerk and
the penalty would cost nothing.

### PPO hyperparameters

[`train_ppo_camera.yaml`](../../src/cobraflex_rl/config/train_ppo_camera.yaml),
trained by [`train_ppo.py`](../../src/cobraflex_rl/cobraflex_rl/train_ppo.py)
with Stable-Baselines3:

| Parameter | Value |
| --- | --- |
| `total_timesteps` | 1 000 000 (about 34 h at about 8 steps/s) |
| `learning_rate` | 3·10⁻⁴, linear decay |
| `n_steps` / `batch_size` / `n_epochs` | 1024 / 64 / 10 |
| `gamma` / `gae_lambda` | 0.99 / 0.95 |
| `clip_range` ($\varepsilon$) / `clip_range_vf` | 0.2 / 0.2 |
| `ent_coef` / `vf_coef` | 0.005 / 0.5 |
| `max_grad_norm` / `target_kl` | 0.5 / 0.5 |
| Reward normalisation | `VecNormalize`, reward clipped at 10 |
| `seed` | 2024 |

SAC configurations (`train_sac_*.yaml`) share everything but the update rule,
so the two algorithms compare like for like. Each run writes
`metadata.json` (git commit, config, cage and policy hashes, seed), learning
curves and checkpoints under `experiments/sim/training/<run_id>/`. No trained
checkpoint ships with this repository.

### Domain randomisation

| Kind | Where | Settings |
| --- | --- | --- |
| Visual degradation (glare, low light, motion blur) | Gazebo and Isaac | per episode with probability 0.5, level 0.2–0.8 |
| Dynamics | Isaac only | friction 0.04–0.07, mass ×0.85–1.15, yaw gain ×0.8–1.2, actuation delay 0–2 steps |
| Scene | Isaac only | light intensity and tint, asphalt, line and grass colour jitter |

### From simulation to the car

[`deploy_cobraflex.launch.py`](../../src/cobraflex_rl/launch/deploy_cobraflex.launch.py)
runs the chain drawn in [02](02_system_architecture.md):
`rl_policy_node` → `cage_ros_node` → `vehicle_control_node` → driver.
[`rl_policy_node.py`](../../src/cobraflex_rl/cobraflex_rl/rl_policy_node.py)
reuses the exact training preprocessing (decode, 84×84 grayscale, 4-frame
stack) and calls `model.predict(obs, deterministic=True)`.

**Status: the deploy launch is scaffolding and has not run on hardware yet.**

---

## Pitfalls

- **Throttle domain.** The policy outputs $a \in [-1, 1]$; the cage expects
  $u = (a + 1)/2 \in [0, 1]$. Publishing $a$ on `/raw_action.linear.x` feeds
  the cage and the vehicle controller a number in the wrong range.
- **Identical observations.** Any change to resizing, grayscale conversion or
  frame stacking between training and deployment shows the CNN images it has
  never seen.
- **Camera ownership.** The deploy launch attaches to `camera/image_raw_lane`
  (`camera:=false`); layer 2 must be running.
- **Training time.** Camera rendering is bound to real time; plan days, not
  hours, for a full run.

---

## Try it

```bash
# Evaluate the classical baseline with the cage in the loop
ros2 launch cobraflex_rl eval_cv_controller.launch.py mode:=enforcement

# Train the camera policy (long; for a pilot, copy the YAML and lower total_timesteps).
# Without train_config the launch uses train_ppo.yaml, the state-vector track.
CFG=$(ros2 pkg prefix cobraflex_rl)/share/cobraflex_rl/config/train_ppo_camera.yaml
ros2 launch cobraflex_rl train_lane.launch.py train_config:=$CFG

# Evaluate a trained policy
ros2 launch cobraflex_rl eval_lane.launch.py model_path:=/path/to/checkpoint.zip
```

Exercise: run a pilot with `steer_delta` set to 0 and compare the steering
traces with the default 0.20.

---

## Lecture links

- **Motion Planning part 3** — Markov decision processes, Q-learning and
  imitation learning: PPO is the policy-gradient counterpart of the value-based
  methods in the lecture.
- **Computer Vision L04–L08** — machine learning, optimisation, neural
  networks, ConvNets and architectures: the policy's feature extractor.

## Further reading

- J. Schulman et al., "Proximal Policy Optimization Algorithms", arXiv:1707.06347, 2017.
- J. Schulman et al., "High-Dimensional Continuous Control Using Generalized Advantage Estimation", arXiv:1506.02438, 2015.
- Stable-Baselines3 PPO: <https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html>
- J. Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World", IROS 2017.
