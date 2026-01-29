#ifndef VALIDATION_MISSION_RUNNER_HPP_
#define VALIDATION_MISSION_RUNNER_HPP_

#include <vector>

#include "autopilot/base/config_base.hpp"
#include "autopilot/base/controller_base.hpp"
#include "autopilot/core/butterworth_filter.hpp"
#include "autopilot/estimators/estimator_driver_base.hpp"
#include "autopilot/planning/mission.hpp"
#include "autopilot/planning/sampler_base.hpp"
#include "autopilot/planning/trajectory.hpp"
#include "autopilot/simulator/quadrotor_simulator.hpp"

namespace autopilot {

struct History {
  QuadrotorState real_state;
  QuadrotorState estimated_state;
  QuadrotorCommand command;
  Eigen::VectorXd est_variance = Eigen::VectorXd::Zero(15);
};

struct SimulationResult {
  bool completed = false;
  std::vector<double> time;
  std::vector<History> hist;
};

using namespace autopilot::literals;

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

class RunnerBase {
 public:
  struct StateAndCovariance {
    QuadrotorState state;
    std::optional<EstimatorCovarianceMatrix> covariance;
  };
  using Config = MissionRunnerConfig;
  RunnerBase(std::shared_ptr<QuadrotorSimulator> sim,
             std::shared_ptr<ControllerBase> ctrl, Config config = Config(),
             std::shared_ptr<spdlog::logger> logger = nullptr);

  virtual ~RunnerBase() = default;

  virtual StateAndCovariance getStateEstimate(int /*step*/) const {
    return {sim_->state(), std::nullopt};
  }

  virtual std::expected<bool, AutopilotErrc> getCurrentCommand(
      const QuadrotorState& state, std::span<QuadrotorCommand> commands) = 0;

  SimulationResult run();

  virtual bool onSimStepStart(int /*step*/) { return true; }
  virtual bool onSimStepEnd(int /*step*/) { return true; };

  virtual bool onStepStart(int /*step*/) { return true; };
  virtual bool onStepEnd(int /*step*/) { return true; };

 protected:
  std::shared_ptr<QuadrotorSimulator> sim_;
  std::shared_ptr<ControllerBase> ctrl_;
  Config cfg_;
  std::shared_ptr<spdlog::logger> logger_;

 private:
  bool detectGeofenceViolation(const QuadrotorState& state) const;
};

class TrajectoryRunner : public RunnerBase {
 public:
  using Config = MissionRunnerConfig;

  TrajectoryRunner(std::shared_ptr<QuadrotorSimulator> sim,
                   std::shared_ptr<ControllerBase> ctrl,
                   std::shared_ptr<TrajectoryBase> trajectory,
                   Config config = Config(),
                   std::shared_ptr<spdlog::logger> logger = nullptr);

 private:
  std::expected<bool, AutopilotErrc> getCurrentCommand(
      const QuadrotorState& state,
      std::span<QuadrotorCommand> commands) override;

  std::shared_ptr<SamplerBase> sampler_;
  std::shared_ptr<TrajectoryBase> trajectory_;
};

class MissionRunner : public RunnerBase {
 public:
  using Config = MissionRunnerConfig;

  MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                std::shared_ptr<ControllerBase> ctrl, Mission mission,
                Config config = Config(),
                std::shared_ptr<spdlog::logger> logger = nullptr);

 private:
  std::expected<bool, AutopilotErrc> getCurrentCommand(
      const QuadrotorState& state,
      std::span<QuadrotorCommand> commands) override;

  std::shared_ptr<SamplerBase> sampler_;
  Mission mission_;
  bool last_sample_finished_ = false;
};

class EstimationControlMissionRunner : public MissionRunner {
 public:
  using Config = MissionRunnerConfig;

  EstimationControlMissionRunner(
      std::shared_ptr<QuadrotorSimulator> sim,
      std::shared_ptr<ControllerBase> ctrl,
      std::shared_ptr<EstimatorDriverBase> est, Mission trajectory,
      Config config = Config(),
      std::shared_ptr<spdlog::logger> logger = nullptr);

 private:
  bool onStepStart(int step) override;

  bool onStepEnd(int step) override;

  bool onSimStepEnd(int step) override;

  [[nodiscard]] StateAndCovariance getStateEstimate(int step) const override;

  std::shared_ptr<EstimatorDriverBase> est_ = nullptr;

  ButterworthFilter<double, 3> gyro_filter_;
  ButterworthFilter<double, 3> accel_filter_;
  double last_gps_time_ = -1;
};

}  // namespace autopilot
#endif  // VALIDATION_MISSION_RUNNER_HPP_
