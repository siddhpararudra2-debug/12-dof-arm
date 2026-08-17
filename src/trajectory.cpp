#include "aether12/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace aether12 {

JointTrajectory TrajectoryPlanner::planJointSpace(const std::array<double, kDof>& start, const std::array<double, kDof>& goal,
                                                  double duration, std::size_t samples) const {
  JointTrajectory trajectory;
  if (duration <= 0.0 || samples < 2) { trajectory.error = "duration must be positive and samples must be at least two"; return trajectory; }
  if (!model_.validateJointVector(start, &trajectory.error) || !model_.validateJointVector(goal, &trajectory.error)) return trajectory;
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    const double distance = goal[joint] - start[joint];
    const auto& limit = model_.config().joints[joint].limits;
    if (std::abs(distance) / duration > limit.max_velocity + 1e-9) { trajectory.error = "joint-space trajectory exceeds velocity limit"; return trajectory; }
    if (std::abs(6.0 * distance / (duration * duration)) > limit.max_acceleration + 1e-9) { trajectory.error = "joint-space trajectory exceeds acceleration limit"; return trajectory; }
  }
  trajectory.duration = duration;
  trajectory.points.reserve(samples);
  for (std::size_t sample = 0; sample < samples; ++sample) {
    const double time = duration * static_cast<double>(sample) / static_cast<double>(samples - 1);
    const double tau = time / duration;
    const double s = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
    const double ds = (30.0 * std::pow(tau, 2) - 60.0 * std::pow(tau, 3) + 30.0 * std::pow(tau, 4)) / duration;
    const double dds = (60.0 * tau - 180.0 * std::pow(tau, 2) + 120.0 * std::pow(tau, 3)) / (duration * duration);
    TrajectoryPoint point;
    point.time = time;
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const double distance = goal[joint] - start[joint];
      point.position[joint] = start[joint] + distance * s;
      point.velocity[joint] = distance * ds;
      point.acceleration[joint] = distance * dds;
    }
    trajectory.points.push_back(point);
  }
  trajectory.valid = true;
  return trajectory;
}

CartesianTrajectory TrajectoryPlanner::planCartesian(const Transform& start, const Transform& goal, const std::array<double, kDof>& seed,
                                                     double duration, std::size_t samples, const IKOptions& ik_options) const {
  CartesianTrajectory trajectory;
  if (duration <= 0.0 || samples < 2) { trajectory.error = "duration must be positive and samples must be at least two"; return trajectory; }
  if (!model_.validateJointVector(seed, &trajectory.error)) return trajectory;
  const Quaternion start_q = quaternionFromRotation(start.rotation);
  const Quaternion goal_q = quaternionFromRotation(goal.rotation);
  auto previous_joints = seed;
  trajectory.points.reserve(samples);
  for (std::size_t sample = 0; sample < samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples - 1);
    Transform pose;
    pose.translation = start.translation * (1.0 - ratio) + goal.translation * ratio;
    pose.rotation = rotationFromQuaternion(slerp(start_q, goal_q, ratio));
    const IKResult ik = kinematics_.inverseKinematics({pose, false}, previous_joints, ik_options);
    if (!ik.converged) { trajectory.error = "Cartesian waypoint " + std::to_string(sample) + " failed IK: " + ik.message; return trajectory; }
    CartesianWaypoint waypoint;
    waypoint.time = duration * ratio;
    waypoint.pose = pose;
    waypoint.joints = ik.joints;
    trajectory.points.push_back(waypoint);
    previous_joints = ik.joints;
  }
  trajectory.valid = true;
  return trajectory;
}

}  // namespace aether12
