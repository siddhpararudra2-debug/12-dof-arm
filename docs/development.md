# Development Workflow

Configure the project with CMake and build the core library, demo, and test executable:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

The same flow is available through `make build`, `make test`, and `make sim`. Add mathematical behavior to the corresponding focused module rather than extending the demo. Add a regression test in `tests/test_core.cpp` for every safety or numerical behavior change.

The current build is intentionally dependency-light. ROS 2, MoveIt, Gazebo, camera drivers, backend services, and a dashboard should be integrated as separate packages that link against or wrap the core. A future integration should add a schema-validated configuration loader, generated URDF/Xacro, ROS 2 actions for long-running motion, controller state feedback, a full continuous collision scene, and end-to-end simulation tests.

Do not represent a simulation pass as hardware verification. Keep hardware adapters transport-agnostic, test them against recorded or simulated actuator traces first, and require a reviewed commissioning procedure before enabling physical drives.
