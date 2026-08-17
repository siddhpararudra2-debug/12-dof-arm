#include "aether12/hardware.hpp"

#include <algorithm>
#include <cmath>

namespace aether12 {

SimulatedHardware::SimulatedHardware(double time_constant) : time_constant_(std::max(1e-3, time_constant)) {
  for (auto& command : commands_) command.mode = ControlMode::Disabled;
}

bool SimulatedHardware::connect(std::string*) { connected_ = true; estop_ = false; return true; }
void SimulatedHardware::disconnect() { connected_ = false; disable(); }

bool SimulatedHardware::enable(std::string* error) {
  if (!connected_) { if (error) *error = "simulation hardware is not connected"; return false; }
  if (estop_) { if (error) *error = "simulation hardware E-stop is latched"; return false; }
  for (auto& state : states_) state.enabled = true;
  return true;
}

void SimulatedHardware::disable() {
  for (auto& state : states_) state.enabled = false;
  for (auto& command : commands_) command.mode = ControlMode::Disabled;
}

bool SimulatedHardware::sendJointCommands(const std::array<MotorCommand, kDof>& commands, std::string* error) {
  if (!connected_ || estop_) { if (error) *error = "simulated hardware is disconnected or E-stopped"; return false; }
  for (std::size_t i = 0; i < kDof; ++i) {
    if (!std::isfinite(commands[i].target_position) || !std::isfinite(commands[i].target_velocity)) { if (error) *error = "command contains a non-finite value"; return false; }
  }
  commands_ = commands;
  return true;
}

void SimulatedHardware::stop() {
  for (auto& command : commands_) { command.mode = ControlMode::Position; command.target_velocity = 0.0; }
}

void SimulatedHardware::emergencyStop() { estop_ = true; stop(); disable(); }

void SimulatedHardware::update(double dt_seconds) {
  if (!connected_ || estop_ || dt_seconds <= 0.0) return;
  const double alpha = 1.0 - std::exp(-dt_seconds / time_constant_);
  for (std::size_t i = 0; i < kDof; ++i) {
    if (!states_[i].enabled || commands_[i].mode != ControlMode::Position) { states_[i].velocity = 0.0; continue; }
    const double previous = states_[i].position;
    states_[i].position += alpha * (commands_[i].target_position - states_[i].position);
    states_[i].velocity = (states_[i].position - previous) / dt_seconds;
    states_[i].acceleration = 0.0;
    states_[i].current = std::abs(states_[i].velocity) * 0.5;
  }
}

bool RealHardware::connect(std::string* error) {
  if (error) *error = "no physical transport adapter configured; refusing unsafe implicit hardware access";
  return false;
}

bool RealHardware::enable(std::string* error) {
  if (error) *error = "physical hardware requires an explicit vendor-neutral transport adapter";
  return false;
}

bool RealHardware::sendJointCommands(const std::array<MotorCommand, kDof>&, std::string* error) {
  if (error) *error = "physical command dispatch is unavailable until a reviewed transport adapter is installed";
  return false;
}

}  // namespace aether12
