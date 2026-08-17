# AETHER-12 Architecture

The repository is organized as a reusable C++ core rather than a UI-driven motor script. `RobotModel` owns configurable kinematic data and is consumed by `Kinematics`, `TrajectoryPlanner`, and `CollisionChecker`. `SafetyManager` is independent of the user interface and determines whether motion commands may be admitted. `RobotHardwareInterface` is the only boundary through which actuator commands can be sent.

```text
application / ROS 2 wrapper / dashboard
                 |
           task validation
                 |
           motion planning
                 |
       Kinematics + TrajectoryPlanner
                 |
          CollisionChecker
                 |
           SafetyManager
                 |
      RobotHardwareInterface
          /               \
 SimulatedHardware       RealHardware adapter
```

The reference model uses standard Denavit–Hartenberg parameters. `RobotModel::forwardKinematics` returns the end-effector transform plus every intermediate link frame. `RobotModel::jacobian` uses each revolute joint axis and the end-effector offset to produce a 6-by-12 geometric Jacobian. `Kinematics::inverseKinematics` uses a damped normal equation and a null-space centering term so the redundant arm can satisfy a six-dimensional pose target without abandoning configured position limits.

The core has no dependency on ROS 2 so it can be tested deterministically in a small build environment. A ROS 2 integration should wrap the same classes in nodes, actions, and controller adapters. It must not duplicate FK, IK, safety, or command-validation logic in the UI or transport layer.

## Safety hierarchy

The required ordering is physical E-stop, motor/controller safety, independent safety manager, motion validation, task validation, then user interface. The software state machine latches E-stop and requires an explicit reset transition; it never resumes motion automatically. Hardware-specific code must preserve this ordering and must fail closed when configuration, communication, encoder state, or controller health is unknown.
