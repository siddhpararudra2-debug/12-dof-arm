#pragma once

#include "aether12/robot_model.hpp"

#include <array>
#include <chrono>
#include <string>

namespace aether12 {

enum class SafetyState { Init, Idle, Ready, Moving, Paused, Fault, Estop, Shutdown };

enum class SafetyEvent { ConfigurationValid, Enable, BeginMotion, Pause, Resume, FaultDetected, EmergencyStop, Reset, Shutdown };

struct SafetyLimits {
  double command_timeout_seconds{0.25};
  double collision_distance{0.02};
};

struct SafetyStatus {
  SafetyState state{SafetyState::Init};
  bool motion_allowed{false};
  bool estop_latched{false};
  std::string reason;
};

class SafetyManager {
 public:
  explicit SafetyManager(const RobotModel& model, SafetyLimits limits = {});

  bool transition(SafetyEvent event, const std::string& reason = {});
  bool validateCommand(const std::array<double, kDof>& position, const std::array<double, kDof>& velocity,
                       double age_seconds, std::string* error = nullptr) const;
  void latchEmergencyStop(const std::string& reason = "Emergency stop asserted");
  SafetyStatus status() const { return status_; }
  SafetyState state() const { return status_.state; }

 private:
  const RobotModel& model_;
  SafetyLimits limits_;
  SafetyStatus status_;
};

std::string toString(SafetyState state);

}  // namespace aether12
