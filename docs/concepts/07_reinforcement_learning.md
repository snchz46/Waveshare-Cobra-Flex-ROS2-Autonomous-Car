# 07 · Reinforcement learning

> **Summary:** a PPO agent with a convolutional policy learns lane keeping from
> four stacked 84×84 grayscale camera frames in Gazebo. The safety cage is part
> of the loop during training, and deployment on the robot uses the same
> preprocessing and the same cage.

[← 06 Lane perception and control](06_lane_perception_and_control.md) · [Concepts](README.md) · Next: [08 Safety cage →](08_safety_cage.md)

---

## Theory

### Lane keeping as a Markov decision process

| Element | Definition in this stack |
| --- | --- |
| State $s_t$ | Agent observation: camera frames (on the state track: $e_y$, $e_\psi$, speed, curvature, among others) |
| Action $a_t$ | Steering in $[-1, 1]$; optionally throttle as a second dimension |
| Transition $P(s_{t+1} \mid s_t, a_t)$ | Simulated vehicle and track, unknown to the agent |
| Reward $r_t$ | Progress along the lane minus penalties for error, jerk and lane departure |
| Discount $\gamma$ | 0.99 |

The agent seeks a policy $\pi_\theta(a \mid s)$ that maximises the expected
discounted return $\mathbb{E}\left[\sum_t \gamma^t r_t\right]$.

### Policy gradients and PPO

Policy-gradient methods adjust $\theta$ so that actions with a positive
**advantage** $\hat A_t$ (better than the critic estimate) become more likely.
**Proximal Policy Optimisation** limits the change of the policy per update by
clipping the probability ratio
$r_t(\theta) = \pi_\theta(a_t \mid s_t) / \pi_{\theta_\text{old}}(a_t \mid s_t)$:

$$
L^{\text{CLIP}}(\theta) = \mathbb{E}_t\left[\min\left(r_t(\theta)\,\hat A_t,\;
\operatorname{clip}\left(r_t(\theta),\, 1 - \varepsilon,\, 1 + \varepsilon\right)\hat A_t\right)\right]
$$

The advantage is obtained by **generalised advantage estimation** (GAE),

$$
\hat A_t = \sum_{l \ge 0} (\gamma\lambda)^l\, \delta_{t+l},
\qquad
\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)
$$

where $V$ is the critic. Actor and critic share a convolutional feature
extractor: the Stable-Baselines3 `CnnPolicy` uses the "Nature CNN" (three
convolutional layers followed by a 512-unit dense layer). Stacking four frames
provides motion information that a single image does not contain.

### Sim-to-real transfer

A policy trained in simulation encounters a different camera, lighting, floor
and vehicle response on the robot. **Domain randomisation** varies these
factors during training so that the policy relies on the invariant feature,
the lane, rather than on simulator-specific details.

---

## Implementation

### Environment

[`GazeboLaneEnv`](../../src/cobraflex_rl/cobraflex_rl/gazebo_lane_env.py) is a
Gymnasium environment built on the Gazebo lane-following worlds:

| Setting | Camera track (`train_ppo_camera.yaml`) |
| --- | --- |
| Observation | 84×84 grayscale, 4 stacked frames, from `/camera/image_raw_lane` |
| Action | Steering in $[-1, 1]$ at a fixed 0.20 m/s; `train_ppo_camera_2d.yaml` adds throttle (`steer_throttle`) |
| Control period | 0.10 s |
| Episode length | Up to 1024 steps |
| Safety cage | In the loop, in-process, `enforcement` mode |

The ground-truth pose is used only for reward, termination and metrics; the
cage reads the CV lane estimator and never the ground truth
([06](06_lane_perception_and_control.md), [08](08_safety_cage.md)). An Isaac
Sim backend ([`isaac_interface.py`](../../src/cobraflex_rl/cobraflex_rl/isaac_interface.py))
provides the same interface, so the environment runs unchanged on both
simulators.

### Reward (v1.2)

[`rewards.py`](../../src/cobraflex_rl/cobraflex_rl/rewards.py):

$$
r = w_\text{fwd}\max(\text{progress}, 0) - w_{e_y}|e_y| - w_{e_\psi}|e_\psi|
    - w_{\Delta s}|\Delta \text{steer}| - w_\text{term}\,[\text{done}]
$$

The 2-D action adds throttle-change and stall terms, zero by default.

| Weight | Value in `train_ppo_camera.yaml` |
| --- | --- |
| `forward_progress` | 1.0 |
| `lateral_error` | 2.5 |
| `heading_error` | 0.75 |
| `steer_delta` | 0.20 |
| `termination` | 25.0 |

Design decisions:

- Progress is the normalised advance along the lane, not the speed. Every step
  on the track therefore has a positive net reward, and early termination is
  never advantageous.
- The steering-change penalty uses the **raw** policy output and not the
  smoothed output of the cage. With the smoothed output, the cage rate limiter
  would absorb the jerk and the penalty would have no effect.

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

The SAC configurations (`train_sac_*.yaml`) differ only in the update rule, so
both algorithms are compared under identical conditions. Each run writes
`metadata.json` (git commit, configuration, cage and policy hashes, seed),
learning curves and checkpoints to `experiments/sim/training/<run_id>/`. No
trained checkpoint is included in this repository.

### Domain randomisation

| Type | Simulator | Settings |
| --- | --- | --- |
| Visual degradation (glare, low light, motion blur) | Gazebo and Isaac | Per episode with probability 0.5, level 0.2–0.8 |
| Dynamics | Isaac only | Friction 0.04–0.07, mass ×0.85–1.15, yaw gain ×0.8–1.2, actuation delay 0–2 steps |
| Scene | Isaac only | Light intensity and tint, asphalt, line and grass colour variation |

### Deployment on the robot

[`deploy_cobraflex.launch.py`](../../src/cobraflex_rl/launch/deploy_cobraflex.launch.py)
runs the chain shown in [02](02_system_architecture.md):
`rl_policy_node` → `cage_ros_node` → `vehicle_control_node` → driver.
[`rl_policy_node.py`](../../src/cobraflex_rl/cobraflex_rl/rl_policy_node.py)
reuses the training preprocessing (decoding, 84×84 grayscale, 4-frame stack)
and calls `model.predict(obs, deterministic=True)`.

**Status: the deployment launch file is a scaffold and has not yet been
executed on hardware.**

---

## Common errors

- **Throttle range.** The policy outputs $a \in [-1, 1]$; the cage expects
  $u = (a + 1)/2 \in [0, 1]$. Publishing $a$ on `/raw_action.linear.x` passes a
  value outside the expected range to the cage and the vehicle controller.
- **Observation mismatch.** Any difference in resizing, grayscale conversion or
  frame stacking between training and deployment presents the CNN with inputs
  outside its training distribution.
- **Camera ownership.** The deployment launch file subscribes to
  `camera/image_raw_lane` (`camera:=false`); layer 2 must be running.
- **Training time.** Camera rendering is bound to real time; a full run takes
  days.

---

## Commands

```bash
# Evaluate the classical baseline with the cage in the loop
ros2 launch cobraflex_rl eval_cv_controller.launch.py mode:=enforcement

# Train the camera policy (long run; for a pilot, copy the YAML and reduce total_timesteps).
# Without train_config the launch file uses train_ppo.yaml, the state-vector track.
CFG=$(ros2 pkg prefix cobraflex_rl)/share/cobraflex_rl/config/train_ppo_camera.yaml
ros2 launch cobraflex_rl train_lane.launch.py train_config:=$CFG

# Evaluate a trained policy
ros2 launch cobraflex_rl eval_lane.launch.py model_path:=/path/to/checkpoint.zip
```

Exercise: run a pilot with `steer_delta` set to 0 and compare the steering
traces with the default value of 0.20.

---

## Lecture references

- **Motion Planning part 3**: Markov decision processes, Q-learning and
  imitation learning. PPO is the policy-gradient counterpart of the
  value-based methods of the lecture.
- **Computer Vision L04–L08**: machine learning, optimisation, neural
  networks, ConvNets and architectures; the feature extractor of the policy.

## Further reading

- J. Schulman et al., "Proximal Policy Optimization Algorithms", arXiv:1707.06347, 2017.
- J. Schulman et al., "High-Dimensional Continuous Control Using Generalized Advantage Estimation", arXiv:1506.02438, 2015.
- Stable-Baselines3 PPO: <https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html>
- J. Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World", IROS 2017.
