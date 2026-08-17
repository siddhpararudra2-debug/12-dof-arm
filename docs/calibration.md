# Camera-to-Robot Calibration

Calibration establishes the rigid transform between a camera frame and a robot frame. The deployment must record whether the camera is eye-in-hand or eye-to-hand, which robot link is the reference, the camera serial or identifier, image resolution, intrinsic matrix, distortion coefficients, sample count, residual error, and calibration timestamp.

## Recommended workflow

1. Mount the camera rigidly and record its frame name and physical mounting arrangement.
2. Calibrate camera intrinsics and distortion at the intended resolution.
3. Capture multiple marker observations over the reachable workspace using AprilTag or ArUco targets.
4. Record synchronized robot poses and marker observations.
5. Solve the hand-eye transform using a reviewed calibration method.
6. Validate the transform on held-out samples and measure position/orientation residuals.
7. Store the transform and report in a versioned calibration record.
8. Recheck the transform after any camera, tool, robot-base, or mounting change.

## Validation

A calibration record must be rejected when the sample count is too small, residual error exceeds the configured threshold, timestamps are inconsistent, or the transform contains non-finite values. The task planner must treat a missing or stale calibration as a fault, not as an identity transform.

A report should contain a homogeneous transformation matrix, calibration mode, camera and robot frame names, residual error, number of samples, timestamp, and software/calibration target identifiers. The transform direction must be explicit, for example `T_robot_camera`, so that consumers do not accidentally apply its inverse.

## Safety

Calibration motions must use a restricted workspace, low speed, collision checking, and an independent E-stop. A valid calibration does not make an arbitrary object pose safe or reachable. Every transformed object target must still pass workspace, IK, collision, trajectory, and safety validation before execution.
