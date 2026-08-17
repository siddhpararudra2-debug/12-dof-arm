# Motion Planning

## Joint-space planning

`TrajectoryPlanner::planJointSpace` accepts start joints, goal joints, duration, and sample count. It uses a quintic time law with zero velocity and acceleration at both endpoints. The planner validates finite values, position limits, velocity limits, and acceleration limits before returning a trajectory.

```cpp
TrajectoryPlanner planner(model);
JointTrajectory trajectory = planner.planJointSpace(start, goal, 2.0, 101);
if (!trajectory.valid) {
  // Reject trajectory.error and keep the robot in its safe state.
}
```

A production controller should additionally enforce jerk limits, controller interpolation behavior, command freshness, and measured tracking error. A generated trajectory must be validated again immediately before execution because robot state and obstacles may have changed.

## Cartesian planning

`planCartesian` linearly interpolates position and uses quaternion SLERP for orientation. Each waypoint is solved through redundant IK using the previous waypoint as the seed, which improves continuity. If a waypoint cannot be solved, the entire path is rejected.

Cartesian waypoint planning is not a substitute for continuous collision checking or full time parameterization. A ROS 2/MoveIt integration should validate the full swept path, retime it against joint limits, and use an execution controller with feedback and cancellation.

## Execution flow

The intended execution sequence is:

```text
receive target
validate input
solve FK/IK or plan target
check joint and dynamic limits
check collision scene
admit through SafetyManager
request explicit authorization if configured
send to controller
monitor feedback
stop on fault, stale state, collision, or E-stop
```

The frontend, AI layer, and task planner must not skip validation steps or send direct motor commands.

## MoveIt integration

A future MoveIt package should use the same validated URDF/Xacro and joint limits, configure a twelve-joint planning group, synchronize the planning scene with `CollisionChecker` obstacles, and route execution through the safety manager and hardware abstraction. MoveIt success alone must not override a lower-level safety fault.
