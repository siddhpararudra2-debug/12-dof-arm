# AETHER-12

AETHER-12 is a **C++17 software core for a configurable 12-axis robotic arm**. It provides the mathematics, trajectory generation, conservative collision checking, independent safety state machine, and hardware abstraction required to build a larger ROS 2, simulator, or physical-arm system without coupling application code directly to motors.

> **Important safety notice:** This repository is not a safety-rated robot controller. The reference dimensions are simulation values and are not a manufacturing specification. A passing software test does not prove that a physical robot is safe. Physical deployment requires independent emergency-stop circuitry, drive/controller safety functions, mechanical guarding, validated limits, encoder commissioning, and a qualified robotics review.

## Project status

The repository contains a **buildable and tested C++ core**. The simulated hardware path is runnable. The physical hardware path is deliberately fail-safe and requires an explicit transport adapter. ROS 2, MoveIt 2, Gazebo, cameras, computer vision, backend services, and the dashboard are documented integration layers rather than silently mocked inside this standalone repository.

| Capability | Status | Notes |
|---|---|---|
| 12-DOF configurable robot model | Implemented | DH parameters and limits are held in `config/robot_config.yaml` and represented by `RobotConfig`. |
| Forward kinematics | Implemented and tested | Returns tool pose and all intermediate frames. |
| 6x12 Jacobian | Implemented and tested | Includes linear/angular blocks and singular-value metrics. |
| Redundant inverse kinematics | Implemented and tested | Damped least squares with null-space centering and joint-limit clamping. |
| Joint-space trajectories | Implemented and tested | Quintic position, velocity, and acceleration interpolation. |
| Cartesian trajectories | Implemented | Translation interpolation, quaternion SLERP, and waypoint IK. |
| Obstacle checking | Implemented and tested | Sphere, box, cylinder, and plane primitives with conservative link samples. |
| Independent safety state machine | Implemented and tested | Explicit transitions, stale command rejection, velocity checks, latched E-stop. |
| Simulated hardware | Implemented and tested | First-order position-response model with enable, stop, and E-stop. |
| Physical hardware | Adapter boundary only | `RealHardware` refuses implicit access until a reviewed transport is supplied. |
| ROS 2 / MoveIt / Gazebo | Integration target | Not claimed as verified in this C++-only build. |
| Vision, calibration, web API, dashboard | Integration target | Interfaces and deployment guidance are documented below. |

## Repository structure

```text
.
├── CMakeLists.txt
├── Makefile
├── README.md
├── LICENSE
├── config/
│   └── robot_config.yaml
├── docs/
│   ├── api.md
│   ├── architecture.md
│   ├── calibration.md
│   ├── development.md
│   ├── hardware.md
│   ├── installation.md
│   ├── kinematics.md
│   ├── motion_planning.md
│   ├── simulation.md
│   ├── safety.md
│   ├── troubleshooting.md
│   └── vision.md
├── include/aether12/
│   ├── collision.hpp
│   ├── hardware.hpp
│   ├── kinematics.hpp
│   ├── math.hpp
│   ├── robot_model.hpp
│   ├── safety.hpp
│   └── trajectory.hpp
├── src/
│   ├── collision.cpp
│   ├── hardware.cpp
│   ├── kinematics.cpp
│   ├── robot_model.cpp
│   ├── safety.cpp
│   └── trajectory.cpp
├── examples/
│   └── aether12_demo.cpp
└── tests/
    └── test_core.cpp
```

## Quick start

The verified local toolchain is Ubuntu 24.04 with CMake and GCC. The core has no third-party C++ dependency.

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git

cd 12-dof-arm
make build
make test
make sim
```

The expected test result is:

```text
100% tests passed, 0 tests failed out of 1
```

`make sim` runs `build/aether12_demo`, which prints the forward-kinematics pose, Jacobian metrics, trajectory status, safety state, and simulated joint response. It does not start Gazebo or energize physical hardware.

## CMake workflow

The equivalent direct commands are:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/aether12_demo
```

Useful Make targets are listed below.

| Command | Purpose |
|---|---|
| `make build` | Configure and compile the core, demo, and tests. |
| `make test` | Build and run the CTest suite. |
| `make sim` | Run the deterministic simulated-hardware demonstration. |
| `make hardware` | Explain the intentionally disabled physical path. |
| `make hybrid` | Explain the planned real-state/simulated-planning integration boundary. |
| `make lint` | Rebuild from a clean state with warnings enabled by the CMake targets. |
| `make format` | Run `clang-format` when it is installed. |
| `make clean` | Remove the generated build directory. |

## Architecture

The core follows this control flow:

```text
Application / ROS 2 wrapper / dashboard
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
           /                  \
SimulatedHardware       RealHardware adapter
```

`RobotModel` owns the configurable geometry and limits. `Kinematics` consumes that model for FK, Jacobian, and IK. `TrajectoryPlanner` generates joint or Cartesian paths. `CollisionChecker` rejects configurations intersecting configured obstacles. `SafetyManager` controls whether commands may proceed. Only `RobotHardwareInterface` sends commands to an actuator implementation.

The layers are deliberately independent. A future ROS 2 node should wrap these classes rather than duplicate their mathematics. A web dashboard or AI task layer must never call a motor adapter directly.

## C++ usage

```cpp
#include "aether12/kinematics.hpp"
#include "aether12/trajectory.hpp"

using namespace aether12;

RobotModel model;
Kinematics kinematics(model);
std::array<double, kDof> q{};

const FKResult fk = kinematics.forwardKinematics(q);
const JacobianResult jacobian = kinematics.jacobian(q);

PoseTarget target{fk.pose, false};
const IKResult ik = kinematics.inverseKinematics(target, q);

TrajectoryPlanner planner(model);
const JointTrajectory trajectory = planner.planJointSpace(q, ik.joints, 2.0);
```

All operations validate finite values and configured joint limits. IK reports structured outcomes through `IKErrorCode`. Safety commands must be checked by `SafetyManager` before they are handed to a hardware adapter.

## Configuration

`config/robot_config.yaml` records the reference configuration, including the 12 joints, DH dimensions, position/velocity/acceleration/torque limits, motor IDs, controller IDs, direction, offsets, end-effector transform, collision radius, and safety thresholds. The current standalone C++ core uses `RobotConfig::defaultConfig()` so it remains dependency-free. A production ROS 2 or service layer should load this YAML with schema validation and fail safe when the file is invalid.

The dimensions in the file are **configurable simulation values**. They must be replaced and validated against the actual mechanical design before any physical use.

## Testing

The current tests cover the core behaviors:

| Test area | Coverage |
|---|---|
| FK and transforms | Valid zero configuration, rotation normalization, finite output. |
| Jacobian | Valid 6x12 output and finite entries. |
| IK | Convergence to a pose generated by the same model and bounded Cartesian error. |
| Trajectories | Quintic endpoints, zero endpoint velocity, sample count, and limits. |
| Collision | Obstacle registration, detection, and removal. |
| Safety | Explicit transitions, command admission, E-stop latching, and manual reset. |
| Hardware | Simulation connection, enable, command response, and E-stop rejection. |

Run the suite with:

```bash
make test
```

## Hardware integration boundary

The physical path is intentionally not enabled by default. Implement a transport-specific adapter behind `RobotHardwareInterface` for CAN, CAN-FD, EtherCAT, RS-485, serial, or Ethernet. The adapter must translate `MotorCommand` and `MotorState` without changing the kinematics or safety modules.

Before physical commissioning, validate encoder polarity, zero offsets, joint limits, drive-side velocity and torque limits, communication watchdog behavior, fault handling, physical E-stop behavior, and collision geometry. Start with low energy and mechanical restraints. The software must be classified as **hardware-ready**, not **hardware-verified**, until those tests have been completed on the real assembly.

## Planned integration layers

The original AETHER-12 product requirements also describe ROS 2 interfaces, URDF/Xacro, ros2_control, MoveIt 2, Gazebo Harmonic, RGB-D vision, hand-eye calibration, pick-and-place tasks, FastAPI, WebSockets, and a React/Three.js dashboard. Those components should be added as separate packages around this core. The separation is intentional: real-time command admission and safety must not depend exclusively on a browser, camera, AI model, or network service.

## Known limitations

The obstacle checker samples robot frames and the tool frame; it is conservative but is not a continuous swept-volume collision engine. Jacobian metrics use a fixed-size internal symmetric eigensolver. Cartesian waypoints are solved through IK but are not yet retimed against measured velocity, acceleration, and jerk limits. The standalone core does not load YAML at runtime, publish ROS messages, start Gazebo, expose a web API, or verify any physical motor transport.

## License

This project is provided under the MIT License. See [LICENSE](LICENSE).

## References

[1]: https://cmake.org/cmake/help/latest/ "CMake Documentation"
[2]: https://docs.ros.org/en/jazzy/ "ROS 2 Jazzy Documentation"
[3]: https://moveit.picknik.ai/main/index.html "MoveIt Documentation"
[4]: https://gazebosim.org/docs/harmonic/ "Gazebo Harmonic Documentation"
[5]: https://docs.opencv.org/ "OpenCV Documentation"
