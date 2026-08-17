# AETHER-12 C++ Core

AETHER-12 is a modular, vendor-neutral C++17 core for a configurable 12-revolute-joint robotic arm. This repository implements the safety-critical mathematical and control abstractions that can be compiled without ROS 2, Gazebo, MoveIt, vendor motor SDKs, or physical hardware. The design keeps the high-level application independent from motors through `RobotHardwareInterface`.

> **Safety warning:** The reference dimensions are simulation values and do not describe a manufactured robot. Passing software tests does not establish that any physical robot is safe for human interaction. A real deployment requires reviewed mechanical limits, independent hardware safety, validated controllers, physical E-stop circuitry, commissioning procedures, and a qualified robotics engineer.

## Implemented modules

| Module | Contents |
|---|---|
| `robot_model` | Configurable DH model, joint limits, FK, link frames, Jacobian, singular-value metrics |
| `kinematics` | 6D pose error, damped-least-squares redundant IK, null-space centering, structured outcomes |
| `trajectory` | Quintic joint-space interpolation and Cartesian position/SLERP waypoints solved through IK |
| `collision` | Sphere, box, cylinder, and plane obstacles with conservative link-frame sample checking |
| `safety` | Explicit INIT/IDLE/READY/MOVING/PAUSED/FAULT/ESTOP/SHUTDOWN state machine, stale-command and velocity checks |
| `hardware` | Abstract motor interface, simulated first-order hardware, fail-safe physical adapter boundary |
| `config/robot_config.yaml` | All 12 joints, DH values, limits, IDs, safety defaults, and end-effector configuration |

The source is intentionally free of direct motor-control calls. Commands must be validated by the safety layer before a hardware adapter receives them. `RealHardware` refuses to connect or dispatch commands until an explicit transport adapter is supplied.

## Repository layout

```text
include/aether12/       Public C++ headers
src/                    Core implementations
examples/               Runnable command-line demonstration
tests/                  C++ unit/integration-style tests
config/                 Reference YAML configuration
CMakeLists.txt          CMake build definition
Makefile                Convenience commands
```

## Requirements

The verified build uses Ubuntu 24.04, CMake 3.28, and GCC 13 with C++17. No third-party C++ library is required for the core build. The larger product requirements mention ROS 2, MoveIt, Gazebo, FastAPI, React, and computer vision; those are intentionally not silently emulated here because they require a separate deployment integration and hardware/simulator environment.

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Equivalent convenience targets are available:

```bash
make build
make test
make sim
make clean
```

The demonstration prints a forward-kinematics pose, Jacobian metrics, a valid quintic trajectory result, current safety state, and simulated actuator motion:

```bash
./build/aether12_demo
```

## C++ usage

```cpp
#include "aether12/kinematics.hpp"
#include "aether12/trajectory.hpp"

using namespace aether12;

RobotModel model;
Kinematics kinematics(model);
std::array<double, kDof> q{};
FKResult pose = kinematics.forwardKinematics(q);
JacobianResult jacobian = kinematics.jacobian(q);

PoseTarget target{pose.pose, false};
IKResult solution = kinematics.inverseKinematics(target, q);
TrajectoryPlanner planner(model);
JointTrajectory trajectory = planner.planJointSpace(q, solution.joints, 2.0);
```

All public operations report validity or structured error state instead of silently accepting invalid values. Joint vectors are checked for finite values and configured position limits. IK uses damped least squares near singular configurations and applies a null-space bias toward the center of each configured joint range.

## Simulation and hardware status

The simulated path is runnable and tested through `SimulatedHardware`. It supports connection, explicit enable, position commands, state updates, stop, and a latched E-stop. The physical path is **hardware-ready at the interface boundary but not hardware-verified**. To integrate a real arm, implement a transport adapter behind `RobotHardwareInterface`, map encoder units and motor directions using `robot_config.yaml`, enforce independent controller limits, and validate the complete chain on a mechanically constrained test fixture before enabling torque.

The repository does not claim that ROS 2 nodes, Gazebo, MoveIt, camera drivers, a web dashboard, vendor motor transports, or physical pick-and-place have been verified. Those systems should consume this core through reviewed adapters rather than bypassing it.

## Known limitations and extension path

The collision implementation is conservative and checks configured robot-frame sample points against obstacles; it is not a replacement for a full continuous-time geometry engine or MoveIt collision scene. Jacobian metrics use a fixed-size internal symmetric eigensolver and report rank deficiency as an infinite condition number. Cartesian planning validates each waypoint through IK but does not yet perform time-parameterized velocity and acceleration retiming between waypoints. YAML loading is represented by the committed configuration file; production startup should add a schema-validated loader and fail-safe configuration admission.

The next integration layer can add ROS 2 message/action wrappers, URDF/Xacro generated from the same model, Gazebo controllers, MoveIt planning scene synchronization, camera/vision nodes, a backend, and a dashboard. Each addition should preserve the safety hierarchy: physical E-stop, controller safety, independent safety manager, motion validation, task validation, then user interface.
