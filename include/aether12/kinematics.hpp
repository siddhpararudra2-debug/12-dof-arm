#pragma once

#include "aether12/robot_model.hpp"

#include <array>
#include <string>

namespace aether12 {

enum class IKErrorCode {
  None,
  InvalidInput,
  Unreachable,
  JointLimitViolation,
  Singular,
  NumericalInstability,
  MaxIterations
};

struct PoseTarget {
  Transform pose{};
  bool position_only{false};
};

struct IKOptions {
  std::size_t max_iterations{300};
  double position_tolerance{1e-4};
  double orientation_tolerance{1e-4};
  double step_size{0.65};
  double damping{0.03};
  double nullspace_gain{0.03};
  double singularity_threshold{1e-5};
};

struct IKResult {
  std::array<double, kDof> joints{};
  IKErrorCode code{IKErrorCode::None};
  std::string message;
  std::size_t iterations{0};
  double position_error{0.0};
  double orientation_error{0.0};
  bool converged{false};
};

class Kinematics {
 public:
  explicit Kinematics(const RobotModel& model) : model_(model) {}

  FKResult forwardKinematics(const std::array<double, kDof>& joints) const;
  JacobianResult jacobian(const std::array<double, kDof>& joints) const;
  IKResult inverseKinematics(const PoseTarget& target, const std::array<double, kDof>& seed,
                             const IKOptions& options = {}) const;

 private:
  const RobotModel& model_;
};

}  // namespace aether12
