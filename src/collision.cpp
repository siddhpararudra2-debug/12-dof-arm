#include "aether12/collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aether12 {
namespace {

double vectorDistance(const Vec3& first, const Vec3& second) { return (first - second).norm(); }

double pointDistance(const Vec3& point, const ObstacleShape& shape) {
  return std::visit([&point](const auto& obstacle) -> double {
    using T = std::decay_t<decltype(obstacle)>;
    if constexpr (std::is_same_v<T, SphereObstacle>) {
      return vectorDistance(point, obstacle.center) - obstacle.radius;
    } else if constexpr (std::is_same_v<T, BoxObstacle>) {
      const Vec3 delta{std::abs(point.x - obstacle.center.x) - obstacle.half_extents.x,
                       std::abs(point.y - obstacle.center.y) - obstacle.half_extents.y,
                       std::abs(point.z - obstacle.center.z) - obstacle.half_extents.z};
      const Vec3 outside{std::max(delta.x, 0.0), std::max(delta.y, 0.0), std::max(delta.z, 0.0)};
      const double outside_distance = outside.norm();
      const double inside_distance = std::min({std::max(delta.x, delta.y), std::max(delta.y, delta.z), std::max(delta.x, delta.z), 0.0});
      return outside_distance + inside_distance;
    } else if constexpr (std::is_same_v<T, CylinderObstacle>) {
      const double radial = std::hypot(point.x - obstacle.center.x, point.y - obstacle.center.y) - obstacle.radius;
      const double axial = std::abs(point.z - obstacle.center.z) - obstacle.half_height;
      return std::max(radial, 0.0) + std::max(axial, 0.0) + std::min(std::max(radial, axial), 0.0);
    } else {
      return dot(point - obstacle.point, obstacle.normal.normalized());
    }
  }, shape);
}

}  // namespace

int CollisionChecker::addObstacle(const ObstacleShape& shape) {
  obstacles_.push_back({next_id_, shape});
  return next_id_++;
}

bool CollisionChecker::removeObstacle(int id) {
  const auto old_size = obstacles_.size();
  obstacles_.erase(std::remove_if(obstacles_.begin(), obstacles_.end(), [id](const Obstacle& obstacle) { return obstacle.id == id; }), obstacles_.end());
  return old_size != obstacles_.size();
}

void CollisionChecker::clearObstacles() { obstacles_.clear(); }

CollisionResult CollisionChecker::check(const std::array<double, kDof>& joints) const {
  CollisionResult result;
  const FKResult fk = model_.forwardKinematics(joints);
  if (!fk.valid) { result.collision = true; result.minimum_distance = -std::numeric_limits<double>::infinity(); result.reason = fk.error; return result; }
  std::vector<Vec3> points;
  points.reserve(kDof + 2);
  for (const auto& frame : fk.frames) points.push_back(frame.translation);
  points.push_back(fk.pose.translation);
  result.minimum_distance = std::numeric_limits<double>::infinity();
  for (const auto& obstacle : obstacles_) {
    for (const Vec3& point : points) {
      const double distance = pointDistance(point, obstacle.shape) - model_.config().collision_radius;
      if (distance < result.minimum_distance) { result.minimum_distance = distance; result.obstacle_id = obstacle.id; }
      if (distance <= 0.0) { result.collision = true; result.reason = "robot link sample intersects obstacle"; return result; }
    }
  }
  if (obstacles_.empty()) result.minimum_distance = std::numeric_limits<double>::infinity();
  return result;
}

}  // namespace aether12
