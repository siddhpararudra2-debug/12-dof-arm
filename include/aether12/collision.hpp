#pragma once

#include "aether12/robot_model.hpp"

#include <array>
#include <string>
#include <variant>
#include <vector>

namespace aether12 {

enum class ObstacleType { Sphere, Box, Cylinder, Plane };

struct SphereObstacle { Vec3 center{}; double radius{0.1}; };
struct BoxObstacle { Vec3 center{}; Vec3 half_extents{0.1, 0.1, 0.1}; };
struct CylinderObstacle { Vec3 center{}; double radius{0.1}; double half_height{0.1}; };
struct PlaneObstacle { Vec3 point{}; Vec3 normal{0.0, 0.0, 1.0}; };
using ObstacleShape = std::variant<SphereObstacle, BoxObstacle, CylinderObstacle, PlaneObstacle>;

struct Obstacle { int id{0}; ObstacleShape shape{SphereObstacle{}}; };

struct CollisionResult {
  bool collision{false};
  double minimum_distance{0.0};
  int obstacle_id{-1};
  std::string reason;
};

class CollisionChecker {
 public:
  explicit CollisionChecker(const RobotModel& model) : model_(model) {}
  int addObstacle(const ObstacleShape& shape);
  bool removeObstacle(int id);
  void clearObstacles();
  CollisionResult check(const std::array<double, kDof>& joints) const;
  const std::vector<Obstacle>& obstacles() const { return obstacles_; }

 private:
  const RobotModel& model_;
  std::vector<Obstacle> obstacles_;
  int next_id_{1};
};

}  // namespace aether12
