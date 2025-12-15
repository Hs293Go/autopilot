#include "autopilot/core/definitions.hpp"
#include "spdlog/spdlog.h"

int main() {
  using namespace autopilot;

  // Example usage of format types
  Eigen::Vector3d position(1.0, 2.0, 3.0);

  QuadrotorState state;
  state.odometry.pose().translation() = position;
  state.odometry.pose().rotation() =
      Eigen::AngleAxis(0.5, Eigen::Vector3d::UnitZ());
  state.odometry.twist().linear() = {0.1, 0.2, 0.3};
  state.odometry.twist().angular() = {0.01, 0.02, 0.03};
  state.accel.linear() = {0.0, 0.0, -9.81};
  state.accel.angular() = {0.0, 0.0, 0.0};

  spdlog::info("Quadrotor state: {}", state);

  QuadrotorCommand command;
  std::ignore = command.setBodyRate(Eigen::Vector3d(0.1, 0.2, 0.3));
  std::ignore = command.setCollectiveThrust(15.0);
  spdlog::info("Quadrotor command: {}", command);

  return 0;
}
