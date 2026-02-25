# IMU Plugin Basics

Use this folder for inertial sensor simulation notes and snippets.

## Core Configuration Items

- Linear acceleration and angular velocity outputs
- Gravity inclusion and frame alignment
- Noise model and bias settings
- Update rate (Hz)
- Topic name and frame ID

## Validation Checks

- Stationary robot reports near-zero angular velocity.
- Gravity axis is consistent with frame convention.
- Noise level is realistic for intended use.
- IMU topic timestamps are monotonic and stable.
