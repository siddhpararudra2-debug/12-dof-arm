# Safety and Commissioning

The reference implementation is a simulation-oriented software core. It is not a safety-rated controller and it cannot replace a physical emergency-stop circuit, motor drive protections, current limiting, watchdogs, guarding, or risk assessment. A physical arm must remain mechanically restrained and torque-disabled until the entire command path has been reviewed.

`SafetyManager` begins in `INIT`. A valid configuration transitions to `IDLE`, explicit enable transitions to `READY`, and only an explicit begin-motion transition enters `MOVING`. Faults disable motion. E-stop transitions to a latched `ESTOP` state and the only recovery path is a deliberate reset to `IDLE`; there is no automatic restart.

Before a physical commissioning session, verify encoder polarity and zero offsets at low energy, validate each joint limit independently, confirm controller-side velocity and torque limits, test loss-of-communications behavior, verify the physical E-stop removes drive energy, check obstacle and self-collision models, and use a second person for the first enabled motion. The `RealHardware` class intentionally refuses all operations until a reviewed transport adapter is provided. This is a fail-safe boundary, not a claim of hardware support.

Any ROS 2, dashboard, vision, or AI layer added later must submit commands to task validation, motion planning, collision checking, and the safety manager. No web endpoint or natural-language interface may call a motor transport directly.
