#include "aether12/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aether12 {
namespace {

bool invert6(const Matrix<6, 6>& input, Matrix<6, 6>* output) {
  Matrix<6, 12> augmented{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) augmented[row][column] = input[row][column];
    augmented[row][row + 6] = 1.0;
  }
  for (std::size_t pivot = 0; pivot < 6; ++pivot) {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1; row < 6; ++row) if (std::abs(augmented[row][pivot]) > std::abs(augmented[best][pivot])) best = row;
    if (std::abs(augmented[best][pivot]) < 1e-12) return false;
    if (best != pivot) std::swap(augmented[best], augmented[pivot]);
    const double divisor = augmented[pivot][pivot];
    for (double& value : augmented[pivot]) value /= divisor;
    for (std::size_t row = 0; row < 6; ++row) {
      if (row == pivot) continue;
      const double factor = augmented[row][pivot];
      for (std::size_t column = 0; column < 12; ++column) augmented[row][column] -= factor * augmented[pivot][column];
    }
  }
  for (std::size_t row = 0; row < 6; ++row) for (std::size_t column = 0; column < 6; ++column) (*output)[row][column] = augmented[row][column + 6];
  return true;
}

std::array<double, 6> errorVector(const Transform& current, const PoseTarget& target) {
  const Vec3 position = target.pose.translation - current.translation;
  const Vec3 orientation = target.position_only ? Vec3{} : rotationError(current.rotation, target.pose.rotation);
  return {position.x, position.y, position.z, orientation.x, orientation.y, orientation.z};
}

}  // namespace

FKResult Kinematics::forwardKinematics(const std::array<double, kDof>& joints) const { return model_.forwardKinematics(joints); }
JacobianResult Kinematics::jacobian(const std::array<double, kDof>& joints) const { return model_.jacobian(joints); }

IKResult Kinematics::inverseKinematics(const PoseTarget& target, const std::array<double, kDof>& seed, const IKOptions& options) const {
  IKResult result;
  result.joints = seed;
  if (!model_.validateJointVector(seed, &result.message)) { result.code = IKErrorCode::InvalidInput; return result; }
  if (!std::isfinite(options.damping) || options.damping <= 0.0 || options.max_iterations == 0) {
    result.code = IKErrorCode::InvalidInput; result.message = "IK options are invalid"; return result;
  }
  for (std::size_t iteration = 0; iteration < options.max_iterations; ++iteration) {
    result.iterations = iteration + 1;
    const FKResult fk = model_.forwardKinematics(result.joints);
    const JacobianResult jacobian = model_.jacobian(result.joints);
    if (!fk.valid || !jacobian.valid) { result.code = IKErrorCode::NumericalInstability; result.message = "FK or Jacobian failed"; return result; }
    const auto error = errorVector(fk.pose, target);
    result.position_error = std::sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);
    result.orientation_error = std::sqrt(error[3] * error[3] + error[4] * error[4] + error[5] * error[5]);
    if (result.position_error <= options.position_tolerance && (target.position_only || result.orientation_error <= options.orientation_tolerance)) {
      result.code = IKErrorCode::None; result.converged = true; result.message = "IK converged"; return result;
    }
    const bool near_singularity = jacobian.condition_number > 1.0 / options.singularity_threshold;
    const double damping = near_singularity ? std::max(options.damping, 0.15) : options.damping;
    Matrix<6, 6> system{};
    for (std::size_t row = 0; row < 6; ++row) {
      for (std::size_t column = 0; column < 6; ++column) {
        for (std::size_t k = 0; k < kDof; ++k) system[row][column] += jacobian.value[row][k] * jacobian.value[column][k];
        if (row == column) system[row][column] += damping * damping;
      }
    }
    Matrix<6, 6> inverse_system{};
    if (!invert6(system, &inverse_system)) { result.code = IKErrorCode::NumericalInstability; result.message = "Damped normal matrix is singular"; return result; }
    Matrix<kDof, 6> pseudoinverse{};
    for (std::size_t joint = 0; joint < kDof; ++joint) for (std::size_t row = 0; row < 6; ++row)
      for (std::size_t column = 0; column < 6; ++column) pseudoinverse[joint][row] += jacobian.value[column][joint] * inverse_system[column][row];
    std::array<double, kDof> delta{};
    for (std::size_t joint = 0; joint < kDof; ++joint) for (std::size_t row = 0; row < 6; ++row) delta[joint] += pseudoinverse[joint][row] * error[row];

    std::array<double, kDof> center_bias{};
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const auto& limit = model_.config().joints[joint].limits;
      const double midpoint = 0.5 * (limit.min_position + limit.max_position);
      const double half_range = 0.5 * (limit.max_position - limit.min_position);
      center_bias[joint] = (midpoint - result.joints[joint]) / std::max(half_range * half_range, 1e-6);
    }
    std::array<double, 6> projected{};
    for (std::size_t row = 0; row < 6; ++row) for (std::size_t joint = 0; joint < kDof; ++joint) projected[row] += jacobian.value[row][joint] * center_bias[joint];
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      double nullspace = center_bias[joint];
      for (std::size_t row = 0; row < 6; ++row) nullspace -= pseudoinverse[joint][row] * projected[row];
      delta[joint] += options.nullspace_gain * nullspace;
    }
    double step_norm = 0.0;
    for (double value : delta) step_norm += value * value;
    step_norm = std::sqrt(step_norm);
    if (!std::isfinite(step_norm)) { result.code = IKErrorCode::NumericalInstability; result.message = "IK step became non-finite"; return result; }
    const double scale = step_norm > 0.25 ? 0.25 / step_norm : 1.0;
    for (std::size_t joint = 0; joint < kDof; ++joint) result.joints[joint] += options.step_size * scale * delta[joint];
    const auto clamped = model_.clampToLimits(result.joints);
    bool hit_limit = false;
    for (std::size_t joint = 0; joint < kDof; ++joint) { hit_limit = hit_limit || std::abs(clamped[joint] - result.joints[joint]) > 1e-9; result.joints[joint] = clamped[joint]; }
    if (hit_limit && step_norm < options.position_tolerance) { result.code = IKErrorCode::JointLimitViolation; result.message = "IK stalled at a joint limit"; return result; }
  }
  result.code = IKErrorCode::MaxIterations;
  result.message = "IK did not converge within the iteration limit";
  return result;
}

}  // namespace aether12
