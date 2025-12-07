#ifndef VALIDATION_MISSION_RUNNER_HPP_
#define VALIDATION_MISSION_RUNNER_HPP_

#include <utility>
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
  double dt_sim = 0.001;
  double dt_gps = 0.10;
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

  MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                std::shared_ptr<ControllerBase> ctrl,
                std::shared_ptr<EstimatorBase> est,
                std::span<const MissionWaypoint> mission,
                Config config = Config(),
                std::shared_ptr<spdlog::logger> logger = nullptr);

  SimulationResult run();

 private:
  [[nodiscard]] QuadrotorState getCurrentState(int step) const;

  [[nodiscard]] bool isMissionComplete(const QuadrotorState& state,
                                       size_t& wp_idx) const;

  void pushEstimatorData(double& last_gps_time, double curr_time);

  std::shared_ptr<QuadrotorSimulator> sim_;
  std::shared_ptr<ControllerBase> ctrl_;

  std::shared_ptr<EstimatorBase> est_ = nullptr;
  std::vector<MissionWaypoint> mission_;
  Config cfg_;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace autopilot
#endif  // VALIDATION_MISSION_RUNNER_HPP_
