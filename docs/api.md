# API Reference

## `RobotModel`

`RobotModel` owns a `RobotConfig` containing twelve `JointSpec` records and an end-effector transform.

| Method | Behavior |
|---|---|
| `validateJointVector(q, error)` | Checks finite values and configured joint position limits. |
| `clampToLimits(q)` | Returns a copy clamped to each configured position range. |
| `forwardKinematics(q)` | Returns tool pose and all intermediate frames. |
| `jacobian(q)` | Returns linear/angular 6x12 Jacobian and metrics. |

## `Kinematics`

`forwardKinematics` and `jacobian` delegate to the model. `inverseKinematics` accepts a `PoseTarget`, a seed joint vector, and optional `IKOptions`. A target may be full six-dimensional pose or position-only.

`IKResult` includes the joint solution, convergence flag, iteration count, position error, orientation error, error code, and human-readable message. Consumers must inspect `converged` and `code` before using the returned joints.

## `TrajectoryPlanner`

`planJointSpace(start, goal, duration, samples)` returns `JointTrajectory`, which contains time-stamped position, velocity, and acceleration samples. `planCartesian(start, goal, seed, duration, samples, options)` returns Cartesian waypoints with solved joint configurations.

## `CollisionChecker`

Use `addObstacle` with `SphereObstacle`, `BoxObstacle`, `CylinderObstacle`, or `PlaneObstacle`. The returned integer ID is used by `removeObstacle`. `check(joints)` returns collision status, minimum signed distance, obstacle ID, and reason. Invalid robot states are treated as collision/failure conditions.

## `SafetyManager`

The state machine begins at `INIT`. Use `transition(SafetyEvent::ConfigurationValid)`, `Enable`, `BeginMotion`, `Pause`, `Resume`, `FaultDetected`, `EmergencyStop`, `Reset`, and `Shutdown` according to the allowed state transitions. `latchEmergencyStop` is always available and prevents motion until manual reset.

`validateCommand(position, velocity, age_seconds, error)` rejects commands when motion is disallowed, the command is stale, positions exceed limits, or velocities are non-finite or too large.

## `RobotHardwareInterface`

Implement `connect`, `disconnect`, `enable`, `disable`, `readJointStates`, `sendJointCommands`, `stop`, and `emergencyStop`. The interface uses `MotorState` and `MotorCommand` with position, velocity, torque, thermal, electrical, enabled, and fault fields. Physical implementations must preserve the safety and watchdog semantics described in [hardware.md](hardware.md).

## Future ROS and web APIs

A ROS 2 wrapper should expose joint states, robot status, safety status, trajectory status, and vision objects as topics; enable/disable/reset/obstacle operations as services; and long-running motion/pick-and-place operations as actions. A web backend may mirror those validated operations through authenticated HTTP and WebSocket endpoints, but must never expose a direct motor-control endpoint.
