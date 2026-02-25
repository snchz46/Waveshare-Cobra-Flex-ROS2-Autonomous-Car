# URDF File Creation

This section tracks the minimum structure needed to create and validate a robot URDF before adding advanced simulation features.

## Checklist

- Define `base_link` and wheel/sensor links.
- Add fixed and revolute joints with consistent parent/child chains.
- Include inertial values (`mass`, inertia tensor) for dynamic stability.
- Add visual and collision meshes with realistic scaling.
- Validate with `check_urdf` and inspect frame tree.

## Recommended Steps

1. Start with a compact URDF containing only chassis and wheels.
2. Add sensors as separate links attached to `base_link`.
3. Confirm frame naming matches ROS 2 conventions (`base_link`, `odom`, `map`, sensor frames).
4. Export to Gazebo and verify pose / collision behavior.

## Output

Keep finalized URDF/Xacro assets in the project URDF package (for this repo: `ros2_ws/src/cobraflex/urdf/`).
