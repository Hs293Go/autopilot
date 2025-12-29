#ifndef VALIDATION_MISSION_RUNNER_HPP_
#define VALIDATION_MISSION_RUNNER_HPP_

#include <utility>
#include <vector>

#include "autopilot/base/config_base.hpp"
#include "autopilot/base/controller_base.hpp"
#include "autopilot/core/butterworth_filter.hpp"
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
  Eigen::VectorXd est_variance = Eigen::VectorXd::Zero(15);
};

struct SimulationResult {
  bool completed;
  std::vector<double> time;
  std::vector<History> hist;
};

struct MissionRunnerConfig : ReflectiveConfigBase<MissionRunnerConfig> {
  MissionRunnerConfig() = default;
  std::string_view name() const override { return "MissionRunnerConfig"; }
  double dt_control = 0.01;
  double dt_sim = 0.001;
  double dt_gps = 0.10;
  double acceptance_radius = 0.1;
  std::int64_t max_steps = 5000;
  Eigen::Vector3d geofence_min = Eigen::Vector3d(-10.0, -10.0, -1.0);
  Eigen::Vector3d geofence_max = Eigen::Vector3d(10.0, 10.0, 20.0);
  bool filter_accel = true;
  double accel_cutoff_hz = 40.0;
  bool filter_gyro = true;
  double gyro_cutoff_hz = 40.0;

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("dt_control", &MissionRunnerConfig::dt_control,
               F64Properties{.desc = "Control timestep (s)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("dt_sim", &MissionRunnerConfig::dt_sim,
               F64Properties{.desc = "Simulation timestep (s)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("dt_gps", &MissionRunnerConfig::dt_gps,
               F64Properties{.desc = "GPS measurement interval (s)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("acceptance_radius", &MissionRunnerConfig::acceptance_radius,
               F64Properties{.desc = "Radius to accept waypoint (m)",
                             .bounds = Bounds<double>::NonNegative()}),
      Describe(
          "max_steps", &MissionRunnerConfig::max_steps,
          I64Properties{.desc = "Maximum number of control steps to simulate",
                        .bounds = Bounds<std::int64_t>::AtLeast(1)}),
      Describe("geofence_min", &MissionRunnerConfig::geofence_min,
               F64Properties{.desc = "Minimum geofence boundary (m)"}),
      Describe("geofence_max", &MissionRunnerConfig::geofence_max,
               F64Properties{.desc = "Maximum geofence boundary (m)"}),
      Describe("accel_cutoff_hz", &MissionRunnerConfig::accel_cutoff_hz,
               F64Properties{.desc = "Cutoff frequency for Accel Butterworth "
                                     "filter (Hz); Set to 0 to disable",
                             .bounds = Bounds<double>::NonNegative()}),
      Describe("gyro_cutoff_hz", &MissionRunnerConfig::gyro_cutoff_hz,
               F64Properties{
                   .desc = "Cutoff frequency for Gyro Butterworth filter (Hz); "
                           "Set to 0 to disable",
                   .bounds = Bounds<double>::NonNegative()}));
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
  ButterworthFilter<double, 3> gyro_filter_;
  ButterworthFilter<double, 3> accel_filter_;
};

}  // namespace autopilot
#endif  // VALIDATION_MISSION_RUNNER_HPP_
