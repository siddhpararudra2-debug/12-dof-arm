# Installation

## Standalone C++ core

The standalone core is intended for Ubuntu 24.04 or another modern Linux distribution with a C++17 compiler. Install the build tools and clone the repository:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git

git clone https://github.com/siddhpararudra2-debug/12-dof-arm.git
cd 12-dof-arm
```

Configure and build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

Run tests and the demonstration:

```bash
ctest --test-dir build --output-on-failure
./build/aether12_demo
```

The Makefile provides the shorter equivalent commands `make build`, `make test`, and `make sim`.

## ROS 2 integration layer

The standalone repository intentionally does not vendor ROS 2, MoveIt 2, Gazebo, camera drivers, or a browser dashboard. For a full robotics deployment, install a supported ROS 2 distribution for the target Ubuntu release, then add ROS packages around the C++ core. The integration should provide URDF/Xacro, `ros2_control`, controller configuration, TF2, MoveIt planning, Gazebo simulation, and ROS actions/topics without duplicating the core mathematics.

A typical integration workspace should build the core first, source the ROS environment, then build the ROS packages with `colcon`. The exact package list and controller configuration depend on the mechanical design, simulator version, camera model, and motor transport; those values must not be guessed from the reference configuration.

## Configuration installation

Copy and review `config/robot_config.yaml`. Replace all reference dimensions, inertia values, offsets, motor IDs, controller IDs, and limits with values validated against the intended robot. A production launcher must parse and schema-validate the file before enabling motion. Invalid configuration must leave the robot disabled.

## Hardware prerequisites

Physical deployment additionally requires independently powered safety systems, drive/controller configuration, encoder feedback, a communication transport, mechanical limits, guarding, and a commissioning procedure. Installing the software alone does not authorize connecting motors or applying torque.
