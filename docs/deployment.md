# Deployment

## Operating modes

AETHER-12 is designed around three deployment modes.

| Mode | State source | Planning | Execution |
|---|---|---|---|
| Simulation | Simulated hardware and simulator feedback | Simulated model | Simulated controller only. |
| Hardware | Real encoder/controller feedback | Validated robot model and planning scene | Physical controller after explicit authorization. |
| Hybrid | Real state with simulated or shadow planning | No automatic physical execution | Requires explicit authorization before any physical command. |

The standalone repository directly verifies the simulated core. A complete deployment should implement the three modes in a ROS 2 launch/orchestration package while keeping the same kinematics and safety interfaces.

## Startup sequence

A safe launcher should load and validate configuration, initialize transforms and model data, initialize the selected hardware interface, read joint states, verify encoders and controller health, confirm the safety state, start planning and vision services, then expose user interfaces. Motors must remain disabled until a deliberate enable operation succeeds.

## Shutdown sequence

On shutdown, cancel the active task, stop trajectory execution, command a safe state, disable motors, close the hardware transport, stop ROS nodes and services, flush structured logs, and close the database. If any shutdown step fails, retain the safest available state and surface the fault for operator review.

## Container and host boundaries

The C++ core can be built in a normal Linux environment. ROS 2, Gazebo, GPU camera drivers, real-time device access, and EtherCAT/CAN interfaces may require host installation or carefully configured containers. Device access and network privileges must be explicit. Do not expose a physical motor transport directly to an internet-facing service.

## Monitoring

Production deployments should monitor command age, controller heartbeat, communication latency, motor temperature/current, joint tracking error, active faults, camera freshness, CPU/memory pressure, and safety state. Any missing or stale safety-relevant signal must result in a safe stop or fault rather than silent continuation.
