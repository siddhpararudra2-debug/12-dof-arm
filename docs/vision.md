# Vision Integration

The standalone C++ core does not require a camera and does not run object detection. A future `aether12_vision` package should run outside the real-time control loop and publish timestamped detections to the task-planning layer.

## Expected data

Each detection should include an identifier, class name, confidence, image coordinates, depth when available, camera-frame position, orientation when estimated, and timestamp. RGB, depth, and camera calibration data should be kept together so that a task planner can reject stale or low-confidence observations.

## Processing pipeline

```text
camera driver
    -> image and depth acquisition
    -> intrinsic and distortion correction
    -> object detection or segmentation
    -> depth validation
    -> 3D camera-frame localization
    -> camera-to-robot transform
    -> confidence and freshness checks
    -> task planner
```

Object detection must not directly command motors. A pick-and-place task should verify that the object remains visible, the pose confidence is sufficient, the transformed target is reachable, the path is collision-free, and the robot remains in an allowed safety state.

## Detector choices

The integration may use OpenCV for calibration, segmentation, contours, ArUco, or AprilTag processing, with an optional YOLO-compatible detector for learned object classes. Heavy inference must remain outside the real-time command loop. The detector should be replaceable without changing the kinematics or safety modules.

## Coordinate conventions

Document camera optical-frame conventions, depth units, timestamp source, and transform direction. Every camera-to-robot transform must be versioned with its calibration record. Reject detections when calibration is missing, stale, inconsistent with the selected camera, or outside configured workspace bounds.
