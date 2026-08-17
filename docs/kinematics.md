# Kinematics and Motion Planning

Joint positions are represented in radians. The reference model contains twelve revolute joints with configurable `a`, `alpha`, `d`, and `theta_offset` values. For each joint, the implementation applies the standard revolute DH transform and composes it from `base_link` to `tool0`. The returned pose contains a 3-by-3 rotation matrix and a Cartesian translation in metres.

The geometric Jacobian has six rows: three linear velocity rows followed by three angular velocity rows. For revolute joint `i`, the implementation uses `Jv_i = z_i × (p_tool − p_i)` and `Jw_i = z_i`. It also computes a fixed-size symmetric eigendecomposition of `J Jᵀ` to expose singular values, manipulability, and a condition number. A rank-deficient configuration is reported with an infinite condition number, while damped IK remains usable there.

Inverse kinematics accepts a full pose or a position-only target and a seed configuration. The solver forms the damped least-squares pseudoinverse

```text
J⁺ = Jᵀ (J Jᵀ + λ² I)⁻¹
```

and adds a projected null-space bias toward the midpoint of each joint range. Each update is step-limited and clamped to joint limits. Structured result codes distinguish invalid input, numerical instability, joint-limit stalling, maximum iterations, and convergence.

Joint trajectories use a quintic time law with zero velocity and acceleration at both endpoints. Cartesian trajectories linearly interpolate translation and use quaternion SLERP for orientation, solving every waypoint through the same IK implementation. A production ROS 2 wrapper should additionally retime waypoints against measured velocity, acceleration, and jerk limits before execution.
