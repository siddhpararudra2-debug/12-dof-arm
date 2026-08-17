# Simulation

## Verified standalone simulation

The current verified simulation is the deterministic `SimulatedHardware` implementation. It models actuator position response with a first-order time constant and supports connect, enable, position commands, update, stop, and emergency stop.

Run it with:

```bash
make sim
```

The demonstration executes forward kinematics, Jacobian analysis, joint-space trajectory generation, safety transitions, and a simulated motor update. It does not require a camera, ROS 2, Gazebo, or physical hardware.

## Gazebo integration path

A full simulator deployment should add a URDF/Xacro generated from the validated `RobotConfig`, inertial and collision geometry for every link, `ros2_control` transmissions, a Gazebo Harmonic control plugin, controller YAML, robot-state publishers, and a launch file. The simulated controller must publish the same joint-state and trajectory interfaces used by hardware mode.

The expected launch sequence is:

```text
load and validate configuration
build the ROS workspace
start Gazebo
spawn the 12-DOF robot
start joint-state and trajectory controllers
start the safety node
start the motion planner
start any backend and dashboard
```

The exact launch commands belong to the ROS integration package and are not fabricated by this standalone C++ repository. The core should be linked into the nodes so FK, IK, collision, trajectory, and safety decisions remain consistent between simulation and hardware modes.

## Simulation test cases

A simulator integration should test joint moves, pose moves, unreachable targets, invalid trajectories, obstacle insertion, collision rejection, E-stop, fault recovery, stale commands, and controller disconnect. A successful simulator test must not be labeled hardware verification.
