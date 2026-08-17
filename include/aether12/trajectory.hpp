#pragma once

#include "aether12/kinematics.hpp"

#include <array>
#include <string>
#include <vector>

namespace aether12 {

struct TrajectoryPoint {
  double time{0.0};
  std::array<double, kDof> position{};
  std::array<double, kDof> velocity{};
  std::array<double, kDof> acceleration{};
};

struct JointTrajectory {
  std::vector<TrajectoryPoint> points;
  double duration{0.0};
  bool valid{false};
  std::string error;
};

struct CartesianWaypoint {
  double time{0.0};
  Transform pose{};
  std::array<double, kDof> joints{};
};

struct CartesianTrajectory {
  std::vector<CartesianWaypoint> points;
  bool valid{false};
  std::string error;
};

class TrajectoryPlanner {
 public:
  explicit TrajectoryPlanner(const RobotModel& model) : model_(model), kinematics_(model) {}

  JointTrajectory planJointSpace(const std::array<double, kDof>& start, const std::array<double, kDof>& goal,
                                 double duration, std::size_t samples = 101) const;
  CartesianTrajectory planCartesian(const Transform& start, const Transform& goal, const std::array<double, kDof>& seed,
                                    double duration, std::size_t samples = 101, const IKOptions& ik_options = {}) const;

 private:
  const RobotModel& model_;
  Kinematics kinematics_;
};

}  // namespace aether12
