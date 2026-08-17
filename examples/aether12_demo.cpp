#include "aether12/collision.hpp"
#include "aether12/hardware.hpp"
#include "aether12/kinematics.hpp"
#include "aether12/safety.hpp"
#include "aether12/trajectory.hpp"

#include <iomanip>
#include <iostream>

int main() {
  using namespace aether12;
  RobotModel model;
  Kinematics kinematics(model);
  std::array<double, kDof> joints{};
  joints[1] = 0.25;
  joints[2] = -0.35;

  const FKResult fk = kinematics.forwardKinematics(joints);
  if (!fk.valid) {
    std::cerr << "FK failed: " << fk.error << '\n';
    return 1;
  }
  std::cout << std::fixed << std::setprecision(4)
            << "AETHER-12 demo\n"
            << "End-effector position: " << fk.pose.translation.x << ", " << fk.pose.translation.y << ", " << fk.pose.translation.z << '\n';

  const JacobianResult jacobian = kinematics.jacobian(joints);
  std::cout << "Jacobian condition number: " << jacobian.condition_number << '\n'
            << "Manipulability: " << jacobian.manipulability << '\n';

  TrajectoryPlanner planner(model);
  std::array<double, kDof> goal = joints;
  goal[0] = 0.4;
  const JointTrajectory trajectory = planner.planJointSpace(joints, goal, 2.0);
  std::cout << "Joint trajectory: " << (trajectory.valid ? "valid" : trajectory.error) << '\n';

  SafetyManager safety(model);
  safety.transition(SafetyEvent::ConfigurationValid);
  safety.transition(SafetyEvent::Enable);
  safety.transition(SafetyEvent::BeginMotion);
  std::cout << "Safety state: " << toString(safety.state()) << '\n';

  SimulatedHardware hardware;
  std::string error;
  hardware.connect(&error);
  hardware.enable(&error);
  std::array<MotorCommand, kDof> commands{};
  for (std::size_t i = 0; i < kDof; ++i) { commands[i].mode = ControlMode::Position; commands[i].target_position = goal[i]; }
  hardware.sendJointCommands(commands, &error);
  hardware.update(0.1);
  std::cout << "Simulated joint 1 after 100 ms: " << hardware.readJointStates()[0].position << '\n';
  return 0;
}
