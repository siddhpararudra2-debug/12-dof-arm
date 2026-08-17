#pragma once

#include "aether12/math.hpp"

#include <array>
#include <string>
#include <vector>

namespace aether12 {

enum class JointType { Revolute };

struct JointLimit {
  double min_position{-3.141592653589793};
  double max_position{3.141592653589793};
  double max_velocity{2.0};
  double max_acceleration{5.0};
  double max_torque{20.0};
};

struct JointSpec {
  JointType type{JointType::Revolute};
  double a{0.0};
  double alpha{0.0};
  double d{0.0};
  double theta_offset{0.0};
  JointLimit limits{};
  int motor_id{0};
  int controller_id{0};
  int direction{1};
  double zero_offset{0.0};
};

struct RobotConfig {
  std::array<JointSpec, kDof> joints{};
  Transform end_effector{Transform::identity()};
  double collision_radius{0.045};
  double base_radius{0.12};
  static RobotConfig defaultConfig();
};

struct FKResult {
  Transform pose{};
  std::array<Transform, kDof + 1> frames{};
  bool valid{false};
  std::string error;
};

struct JacobianResult {
  Matrix<6, kDof> value{};
  Matrix<3, kDof> linear{};
  Matrix<3, kDof> angular{};
  double manipulability{0.0};
  double condition_number{0.0};
  std::array<double, 6> singular_values{};
  bool valid{false};
  std::string error;
};

class RobotModel {
 public:
  explicit RobotModel(RobotConfig config = RobotConfig::defaultConfig());

  const RobotConfig& config() const { return config_; }
  bool validateJointVector(const std::array<double, kDof>& q, std::string* error = nullptr) const;
  std::array<double, kDof> clampToLimits(const std::array<double, kDof>& q) const;
  FKResult forwardKinematics(const std::array<double, kDof>& q) const;
  JacobianResult jacobian(const std::array<double, kDof>& q) const;

 private:
  RobotConfig config_;
};

}  // namespace aether12
