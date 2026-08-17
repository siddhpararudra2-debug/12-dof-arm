# Troubleshooting

## CMake or compiler errors

Confirm that `cmake`, `g++`, and a C++17-capable standard library are installed. Remove stale generated files and configure again:

```bash
make clean
make build
```

If a compiler reports a missing header, check that the command includes the repository `include` directory through the CMake target rather than compiling a source file manually without `-Iinclude`.

## Test failures

Run the failing binary directly after a debug build to preserve diagnostics:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
./build/aether12_tests
```

A failed IK test usually indicates a changed model, an invalid target, an overly strict tolerance, or a numerically difficult seed. Inspect the returned `IKResult::code`, message, position error, orientation error, and iteration count before changing solver parameters.

## IK does not converge

Verify that the target was generated in the same base/tool frame as the model, that the seed is within all joint limits, and that the target is reachable. Near singular configurations, use damped least squares and a smaller step size. A solver failure must reject the motion; it must not fall back to sending an unverified joint vector.

## Collision result is unexpected

Confirm obstacle units are metres, obstacle frame conventions are explicit, and the configured collision radius matches the conservative geometry approximation. The standalone checker samples link frames and the tool frame; a full deployment should replace or supplement it with a continuous geometry engine and planning scene.

## Safety state will not enable

The normal sequence is `INIT -> IDLE -> READY -> MOVING`. Configuration must be accepted before enable, and E-stop or fault states require an explicit reset. Inspect `SafetyStatus::reason`. Do not bypass the state machine to force a command through.

## Hardware commands are rejected

This is expected from `RealHardware` until a reviewed transport adapter is implemented. Check connection state, controller enable state, command age, velocity limits, encoder validity, drive faults, and watchdog state. Never resolve an intentional fail-safe rejection by removing validation.

## Configuration changes

Treat `config/robot_config.yaml` as a controlled engineering artifact. Validate dimensions, joint limits, directions, offsets, IDs, and safety thresholds against the actual system. Restart in a disabled state after configuration changes and repeat commissioning checks.
