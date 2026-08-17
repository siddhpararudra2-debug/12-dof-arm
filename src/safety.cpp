#include "aether12/safety.hpp"

#include <cmath>

namespace aether12 {

SafetyManager::SafetyManager(const RobotModel& model, SafetyLimits limits) : model_(model), limits_(limits) {}

bool SafetyManager::transition(SafetyEvent event, const std::string& reason) {
  if (status_.state == SafetyState::Shutdown && event != SafetyEvent::Shutdown) return false;
  switch (event) {
    case SafetyEvent::ConfigurationValid:
      if (status_.state != SafetyState::Init) return false;
      status_.state = SafetyState::Idle; status_.reason = reason; break;
    case SafetyEvent::Enable:
      if (status_.state != SafetyState::Idle) return false;
      status_.state = SafetyState::Ready; status_.motion_allowed = true; status_.reason = reason; break;
    case SafetyEvent::BeginMotion:
      if (status_.state != SafetyState::Ready) return false;
      status_.state = SafetyState::Moving; status_.reason = reason; break;
    case SafetyEvent::Pause:
      if (status_.state != SafetyState::Moving) return false;
      status_.state = SafetyState::Paused; status_.reason = reason; break;
    case SafetyEvent::Resume:
      if (status_.state != SafetyState::Paused) return false;
      status_.state = SafetyState::Moving; status_.reason = reason; break;
    case SafetyEvent::FaultDetected:
      if (status_.state == SafetyState::Estop) return false;
      status_.state = SafetyState::Fault; status_.motion_allowed = false; status_.reason = reason; break;
    case SafetyEvent::EmergencyStop:
      latchEmergencyStop(reason.empty() ? "Emergency stop asserted" : reason); break;
    case SafetyEvent::Reset:
      if (status_.state != SafetyState::Estop && status_.state != SafetyState::Fault) return false;
      status_.state = SafetyState::Idle; status_.motion_allowed = false; status_.estop_latched = false; status_.reason = reason.empty() ? "Safety reset" : reason; break;
    case SafetyEvent::Shutdown:
      status_.state = SafetyState::Shutdown; status_.motion_allowed = false; status_.reason = reason; break;
  }
  return true;
}

void SafetyManager::latchEmergencyStop(const std::string& reason) {
  status_.state = SafetyState::Estop;
  status_.motion_allowed = false;
  status_.estop_latched = true;
  status_.reason = reason;
}

bool SafetyManager::validateCommand(const std::array<double, kDof>& position, const std::array<double, kDof>& velocity,
                                    double age_seconds, std::string* error) const {
  if (!status_.motion_allowed || status_.state == SafetyState::Estop || status_.state == SafetyState::Fault) {
    if (error) *error = "motion is not allowed in safety state " + toString(status_.state);
    return false;
  }
  if (!std::isfinite(age_seconds) || age_seconds < 0.0 || age_seconds > limits_.command_timeout_seconds) {
    if (error) *error = "command is stale or has an invalid timestamp";
    return false;
  }
  if (!model_.validateJointVector(position, error)) return false;
  for (std::size_t i = 0; i < kDof; ++i) {
    if (!std::isfinite(velocity[i]) || std::abs(velocity[i]) > model_.config().joints[i].limits.max_velocity) {
      if (error) *error = "joint " + std::to_string(i + 1) + " exceeds velocity limits";
      return false;
    }
  }
  return true;
}

std::string toString(SafetyState state) {
  switch (state) {
    case SafetyState::Init: return "INIT";
    case SafetyState::Idle: return "IDLE";
    case SafetyState::Ready: return "READY";
    case SafetyState::Moving: return "MOVING";
    case SafetyState::Paused: return "PAUSED";
    case SafetyState::Fault: return "FAULT";
    case SafetyState::Estop: return "ESTOP";
    case SafetyState::Shutdown: return "SHUTDOWN";
  }
  return "UNKNOWN";
}

}  // namespace aether12
