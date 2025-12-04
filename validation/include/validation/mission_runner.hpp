#ifndef VALIDATION_MISSION_RUNNER_HPP_
#define VALIDATION_MISSION_RUNNER_HPP_

#include <vector>

#include "autopilot/base.hpp"
#include "autopilot/quadrotor_simulator.hpp"

namespace autopilot {

struct MissionWaypoint {
  Eigen::Vector3d position;
  double yaw;
};

struct SimulationResult {
  bool completed;
  std::vector<double> time_history;
  std::vector<QuadrotorState> state_history;
  std::vector<QuadrotorCommand> command_history;
};

struct MissionRunnerConfig {
  double dt_control = 0.01;
  int sim_substeps = 10;
  double acceptance_radius = 0.1;
  int max_steps = 5000;
};

class MissionRunner {
 public:
  using Config = MissionRunnerConfig;

  MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                std::shared_ptr<ControllerBase> ctrl,
                std::span<const MissionWaypoint> mission,
                Config config = Config(),
                std::shared_ptr<spdlog::logger> logger = nullptr);

  SimulationResult run();

 private:
  std::shared_ptr<QuadrotorSimulator> sim_;
  std::shared_ptr<ControllerBase> ctrl_;
  std::vector<MissionWaypoint> mission_;
  Config cfg_;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace autopilot
#endif  // VALIDATION_MISSION_RUNNER_HPP_
