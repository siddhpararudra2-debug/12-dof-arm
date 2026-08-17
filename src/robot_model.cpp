#include "aether12/robot_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace aether12 {
namespace {

Transform dhTransform(double a, double alpha, double d, double theta) {
  const double ct = std::cos(theta), st = std::sin(theta);
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  Transform result;
  result.rotation[0][0] = ct;
  result.rotation[0][1] = -st * ca;
  result.rotation[0][2] = st * sa;
  result.rotation[1][0] = st;
  result.rotation[1][1] = ct * ca;
  result.rotation[1][2] = -ct * sa;
  result.rotation[2][0] = 0.0;
  result.rotation[2][1] = sa;
  result.rotation[2][2] = ca;
  result.translation = {a * ct, a * st, d};
  return result;
}

bool finite(double value) { return std::isfinite(value); }

std::array<double, 6> eigenvaluesSymmetric6(Matrix<6, 6> matrix) {
  for (std::size_t iteration = 0; iteration < 80; ++iteration) {
    std::size_t p = 0, q = 1;
    double largest = 0.0;
    for (std::size_t row = 0; row < 6; ++row) {
      for (std::size_t column = row + 1; column < 6; ++column) {
        if (std::abs(matrix[row][column]) > largest) { largest = std::abs(matrix[row][column]); p = row; q = column; }
      }
    }
    if (largest < 1e-12) break;
    const double angle = 0.5 * std::atan2(2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
    const double c = std::cos(angle), s = std::sin(angle);
    for (std::size_t index = 0; index < 6; ++index) {
      if (index == p || index == q) continue;
      const double aip = matrix[index][p], aiq = matrix[index][q];
      matrix[index][p] = c * aip - s * aiq;
      matrix[p][index] = matrix[index][p];
      matrix[index][q] = s * aip + c * aiq;
      matrix[q][index] = matrix[index][q];
    }
    const double app = matrix[p][p], aqq = matrix[q][q], apq = matrix[p][q];
    matrix[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
    matrix[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
    matrix[p][q] = matrix[q][p] = 0.0;
  }
  std::array<double, 6> values{};
  for (std::size_t i = 0; i < 6; ++i) values[i] = std::max(0.0, matrix[i][i]);
  std::sort(values.begin(), values.end(), std::greater<double>());
  for (double& value : values) value = std::sqrt(value);
  return values;
}

}  // namespace

RobotConfig RobotConfig::defaultConfig() {
  RobotConfig config;
  const std::array<double, kDof> link_lengths{0.16, 0.20, 0.18, 0.16, 0.14, 0.12, 0.10, 0.09, 0.08, 0.07, 0.06, 0.05};
  const std::array<double, kDof> twists{1.5707963267948966, 0.0, 1.5707963267948966, -1.5707963267948966,
                                        1.5707963267948966, 0.0, 1.5707963267948966, -1.5707963267948966,
                                        1.5707963267948966, 0.0, 1.5707963267948966, 0.0};
  for (std::size_t i = 0; i < kDof; ++i) {
    config.joints[i].a = link_lengths[i];
    config.joints[i].alpha = twists[i];
    config.joints[i].d = i == 0 ? 0.18 : 0.0;
    config.joints[i].limits = {-3.141592653589793, 3.141592653589793, 2.0, 6.0, 30.0};
    config.joints[i].motor_id = static_cast<int>(i + 1);
    config.joints[i].controller_id = 1;
    config.joints[i].direction = 1;
    config.joints[i].zero_offset = 0.0;
  }
  config.collision_radius = 0.045;
  config.base_radius = 0.12;
  return config;
}

RobotModel::RobotModel(RobotConfig config) : config_(std::move(config)) {}

bool RobotModel::validateJointVector(const std::array<double, kDof>& q, std::string* error) const {
  for (std::size_t i = 0; i < kDof; ++i) {
    if (!finite(q[i])) {
      if (error) *error = "joint " + std::to_string(i + 1) + " is not finite";
      return false;
    }
    const auto& limit = config_.joints[i].limits;
    if (q[i] < limit.min_position || q[i] > limit.max_position) {
      if (error) *error = "joint " + std::to_string(i + 1) + " exceeds position limits";
      return false;
    }
  }
  return true;
}

std::array<double, kDof> RobotModel::clampToLimits(const std::array<double, kDof>& q) const {
  auto result = q;
  for (std::size_t i = 0; i < kDof; ++i) result[i] = clamp(result[i], config_.joints[i].limits.min_position, config_.joints[i].limits.max_position);
  return result;
}

FKResult RobotModel::forwardKinematics(const std::array<double, kDof>& q) const {
  FKResult result;
  if (!validateJointVector(q, &result.error)) return result;
  Transform current = Transform::identity();
  result.frames[0] = current;
  for (std::size_t i = 0; i < kDof; ++i) {
    const auto& joint = config_.joints[i];
    const double theta = static_cast<double>(joint.direction) * (q[i] + joint.zero_offset) + joint.theta_offset;
    current = current * dhTransform(joint.a, joint.alpha, joint.d, theta);
    result.frames[i + 1] = current;
  }
  result.pose = current * config_.end_effector;
  result.valid = true;
  return result;
}

JacobianResult RobotModel::jacobian(const std::array<double, kDof>& q) const {
  JacobianResult result;
  const FKResult fk = forwardKinematics(q);
  if (!fk.valid) { result.error = fk.error; return result; }
  const Vec3 end_position = fk.pose.translation;
  Matrix<6, 6> gram{};
  for (std::size_t i = 0; i < kDof; ++i) {
    const Transform& frame = fk.frames[i];
    const Vec3 axis{frame.rotation[0][2], frame.rotation[1][2], frame.rotation[2][2]};
    const Vec3 offset = end_position - frame.translation;
    const Vec3 linear = cross(axis, offset) * static_cast<double>(config_.joints[i].direction);
    const Vec3 angular = axis * static_cast<double>(config_.joints[i].direction);
    result.linear[0][i] = linear.x; result.linear[1][i] = linear.y; result.linear[2][i] = linear.z;
    result.angular[0][i] = angular.x; result.angular[1][i] = angular.y; result.angular[2][i] = angular.z;
    result.value[0][i] = linear.x; result.value[1][i] = linear.y; result.value[2][i] = linear.z;
    result.value[3][i] = angular.x; result.value[4][i] = angular.y; result.value[5][i] = angular.z;
  }
  for (std::size_t row = 0; row < 6; ++row) for (std::size_t column = 0; column < 6; ++column)
    for (std::size_t k = 0; k < kDof; ++k) gram[row][column] += result.value[row][k] * result.value[column][k];
  result.singular_values = eigenvaluesSymmetric6(gram);
  const double product = std::accumulate(result.singular_values.begin(), result.singular_values.end(), 1.0, std::multiplies<double>());
  result.manipulability = product;
  result.condition_number = result.singular_values[5] < kEpsilon ? std::numeric_limits<double>::infinity() : result.singular_values[0] / result.singular_values[5];
  result.valid = true;
  return result;
}

}  // namespace aether12
