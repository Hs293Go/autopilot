#ifndef VALIDATION_MISSION_RUNNER_HPP_
#define VALIDATION_MISSION_RUNNER_HPP_

#include <utility>
#include <vector>

#include "autopilot/base/config_base.hpp"
#include "autopilot/base/controller_base.hpp"
#include "autopilot/estimators/estimator_driver_base.hpp"
#include "autopilot/simulator/quadrotor_simulator.hpp"

namespace autopilot {

struct MissionWaypoint {
  Eigen::Vector3d position;
  double yaw;
};

struct History {
  QuadrotorState real_state;
  QuadrotorState estimated_state;
  QuadrotorCommand command;
};

struct SimulationResult {
  bool completed;
  std::vector<double> time;
  std::vector<History> hist;
};

struct MissionRunnerConfig {
  double dt_control = 0.01;
  double dt_sim = 0.001;
  double dt_gps = 0.10;
  double acceptance_radius = 0.1;
  int max_steps = 5000;
  Eigen::Vector3d geofence_min = Eigen::Vector3d(-10.0, -10.0, -1.0);
  Eigen::Vector3d geofence_max = Eigen::Vector3d(10.0, 10.0, 20.0);
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
                std::shared_ptr<EstimatorDriverBase> est,
                std::span<const MissionWaypoint> mission,
                Config config = Config(),
                std::shared_ptr<spdlog::logger> logger = nullptr);

  SimulationResult run();

 private:
  [[nodiscard]] QuadrotorState getStateEstimate(int step) const;

  [[nodiscard]] bool isMissionComplete(const QuadrotorState& state,
                                       size_t& wp_idx) const;

  void pushEstimatorData(double& last_gps_time, double curr_time);

  bool detectGeofenceViolation(const QuadrotorState& state) const;

  std::shared_ptr<QuadrotorSimulator> sim_;
  std::shared_ptr<ControllerBase> ctrl_;

  std::shared_ptr<EstimatorDriverBase> est_ = nullptr;
  std::vector<MissionWaypoint> mission_;
  Config cfg_;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace autopilot
#endif  // VALIDATION_MISSION_RUNNER_HPP_
