# Hardware Integration

## Boundary

`RobotHardwareInterface` is the only actuator boundary in the C++ core. It exposes connection, enable, disable, state readback, command dispatch, stop, and emergency stop. `SimulatedHardware` is a tested implementation. `RealHardware` is deliberately fail-safe: it refuses implicit connection and command dispatch until a reviewed physical transport adapter is installed.

The adapter may use CAN, CAN-FD, EtherCAT, RS-485, serial, or Ethernet. The core must not depend on a motor manufacturer. Map physical actuator units to the SI units used by the core: radians, radians per second, radians per second squared, and Newton-metres.

## Required adapter behavior

A production adapter must reject malformed commands, enforce controller-side limits, verify sequence numbers or timestamps, detect communication loss, report motor faults and temperatures, and guarantee that `stop()` and `emergencyStop()` result in a safe drive state. It must not bypass `SafetyManager` or accept commands directly from a dashboard.

The adapter should keep transport I/O separate from real-time command admission. A watchdog must disable or safely stop the drives when state feedback becomes stale. Startup must leave motors disabled until configuration, encoder state, controller state, and safety state have all passed validation.

## Configuration mapping

Use `config/robot_config.yaml` as a review checklist, not as proof of a physical design. Validate each joint's direction, zero offset, motor ID, controller ID, position limit, velocity limit, acceleration limit, torque limit, encoder resolution, gear ratio, and mechanical range. Confirm the sign convention by moving one mechanically restrained joint at low energy.

## Commissioning sequence

Commissioning should proceed from de-energized inspection to encoder verification, one-joint low-energy motion, multi-joint low-energy motion, limit validation, communication-loss validation, controller fault validation, physical E-stop validation, and only then normal operation. Keep the emergency-stop path independent of the web server and ROS graph.

The repository status terminology is:

| Term | Meaning |
|---|---|
| Simulation-tested | Behavior was exercised through the standalone core and simulated hardware. |
| Hardware-ready | An explicit adapter boundary exists, but physical tests are incomplete. |
| Hardware-verified | A specific assembled robot, transport, controller, and safety procedure passed documented commissioning tests. |

This repository currently reaches the first two categories only.
