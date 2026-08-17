#pragma once

#include "aether12/robot_model.hpp"

#include <array>
#include <string>

namespace aether12 {

enum class ControlMode { Position, Velocity, Torque, Disabled };

struct MotorState {
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double torque{0.0};
  double temperature{25.0};
  double voltage{24.0};
  double current{0.0};
  bool enabled{false};
  bool fault{false};
};

struct MotorCommand {
  double target_position{0.0};
  double target_velocity{0.0};
  double target_torque{0.0};
  ControlMode mode{ControlMode::Disabled};
};

class RobotHardwareInterface {
 public:
  virtual ~RobotHardwareInterface() = default;
  virtual bool connect(std::string* error = nullptr) = 0;
  virtual void disconnect() = 0;
  virtual bool enable(std::string* error = nullptr) = 0;
  virtual void disable() = 0;
  virtual std::array<MotorState, kDof> readJointStates() const = 0;
  virtual bool sendJointCommands(const std::array<MotorCommand, kDof>& commands, std::string* error = nullptr) = 0;
  virtual void stop() = 0;
  virtual void emergencyStop() = 0;
};

class SimulatedHardware final : public RobotHardwareInterface {
 public:
  explicit SimulatedHardware(double time_constant = 0.08);
  bool connect(std::string* error = nullptr) override;
  void disconnect() override;
  bool enable(std::string* error = nullptr) override;
  void disable() override;
  std::array<MotorState, kDof> readJointStates() const override { return states_; }
  bool sendJointCommands(const std::array<MotorCommand, kDof>& commands, std::string* error = nullptr) override;
  void stop() override;
  void emergencyStop() override;
  void update(double dt_seconds);

 private:
  std::array<MotorState, kDof> states_{};
  std::array<MotorCommand, kDof> commands_{};
  double time_constant_;
  bool connected_{false};
  bool estop_{false};
};

class RealHardware final : public RobotHardwareInterface {
 public:
  bool connect(std::string* error = nullptr) override;
  void disconnect() override {}
  bool enable(std::string* error = nullptr) override;
  void disable() override {}
  std::array<MotorState, kDof> readJointStates() const override { return {}; }
  bool sendJointCommands(const std::array<MotorCommand, kDof>&, std::string* error = nullptr) override;
  void stop() override {}
  void emergencyStop() override {}
};

}  // namespace aether12
