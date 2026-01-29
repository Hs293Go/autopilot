#include "validation/mission_runner.hpp"

#include <utility>

#include "autopilot/planning/preset_trajectories.hpp"
#include "autopilot/planning/time_sampler.hpp"
#include "fmt/ranges.h"

namespace autopilot {

RunnerBase::RunnerBase(std::shared_ptr<QuadrotorSimulator> sim,
                       std::shared_ptr<ControllerBase> ctrl, Config config,
                       std::shared_ptr<spdlog::logger> logger)
    : sim_(std::move(sim)),
      ctrl_(std::move(ctrl)),
      cfg_(std::move(config)),
      logger_(logger ? std::move(logger) : spdlog::default_logger()) {}

SimulationResult RunnerBase::run() {
  SimulationResult res;

  // Future proof: Query controller for number of setpoints (prediction horizon)
  std::vector<QuadrotorCommand> sp_buf(1);
  std::vector<QuadrotorCommand> out_buf(1);

  const int sim_substeps =
      static_cast<int>(std::ceil(cfg_.dt_control / cfg_.dt_sim));
  spdlog::info(
      "Starting Mission Runner for up to {} steps, with {} simulation substeps",
      cfg_.max_steps, sim_substeps);

  for (int step = 0; step < cfg_.max_steps; ++step) {
    if (!onStepStart(step)) {
      break;
    }

    auto [state_est, cov] = getStateEstimate(step);
    auto state = sim_->state();

    // 1. Check Waypoint
    if (auto has_command = getCurrentCommand(state_est, sp_buf)) {
      if (has_command.value()) {
        res.completed = true;
        break;
      }

    } else {
      res.completed = false;
      break;
    }

    if (auto result = ctrl_->compute(state_est, sp_buf, out_buf); !result) {
      logger_->error("Controller failed at t={:.2f}s, because: {}",
                     state.timestamp_secs, result.error());
      break;
    }

    // 3. Sim
    for (int i = 0; i < sim_substeps; ++i) {
      onStepStart(step);
      sim_->step(out_buf[0], cfg_.dt_sim);
      onSimStepEnd(step);
    }

    if (detectGeofenceViolation(state)) {
      break;
    }

    // 4. Record
    res.time.push_back(state.timestamp_secs);
    History history = {
        .real_state = state,
        .estimated_state = state_est,
        .command = sp_buf[0],
    };
    if (cov.has_value()) {
      history.est_variance = cov->diagonal();
    }
    res.hist.push_back(history);

    if (!onStepEnd(step)) {
      break;
    }
  }
  return res;
}

bool RunnerBase::detectGeofenceViolation(const QuadrotorState& state) const {
  if ((state.odometry.pose().translation().array() < cfg_.geofence_min.array())
          .any()) {
    logger_->error(
        "Geofence violation at t={:.2f}s: position {} below "
        "min {}",
        state.timestamp_secs, state.odometry.pose().translation().transpose(),
        cfg_.geofence_min.transpose());
    return true;
  }
  if ((state.odometry.pose().translation().array() > cfg_.geofence_max.array())
          .any()) {
    logger_->error(
        "Geofence violation at t={:.2f}s: position {} exceeds "
        "max {}",
        state.timestamp_secs, state.odometry.pose().translation().transpose(),
        cfg_.geofence_max.transpose());
    return true;
  }
  return false;
}

TrajectoryRunner::TrajectoryRunner(std::shared_ptr<QuadrotorSimulator> sim,
                                   std::shared_ptr<ControllerBase> ctrl,
                                   std::shared_ptr<TrajectoryBase> trajectory,
                                   Config config,
                                   std::shared_ptr<spdlog::logger> logger)
    : RunnerBase(std::move(sim), std::move(ctrl), std::move(config),
                 std::move(logger)),
      sampler_(std::make_shared<TimeSampler>(sim_->model())),
      trajectory_(std::move(trajectory)) {}

MissionRunner::MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                             std::shared_ptr<ControllerBase> ctrl,
                             Mission mission, Config config,
                             std::shared_ptr<spdlog::logger> logger)
    : RunnerBase(std::move(sim), std::move(ctrl), std::move(config),
                 std::move(logger)),
      sampler_(std::make_shared<TimeSampler>(sim_->model())),
      mission_(std::move(mission)) {}

std::expected<bool, AutopilotErrc> TrajectoryRunner::getCurrentCommand(
    const QuadrotorState& state, std::span<QuadrotorCommand> commands) {
  SampleContext sample_context{.num_samples = 1,
                               .sampling_interval = cfg_.dt_control};
  auto sample = sampler_->getSetpoint(std::views::single(trajectory_), state,
                                      sample_context, commands);
  if (!sample.has_value()) {
    logger_->error("Sampler failed at t={:.2f}s: {}", state.timestamp_secs,
                   sample.error());
    return std::unexpected(sample.error());
  }

  if (trajectory_->requiresEquilibrium()) {
    return trajectory_->checkEquilibrium(state, {}).state !=
           EquilibriumState::kNone;
  }
  return sample->all_finished;
}

std::expected<bool, AutopilotErrc> MissionRunner::getCurrentCommand(
    const QuadrotorState& state, std::span<QuadrotorCommand> commands) {
  SampleContext sample_context{.num_samples = 1,
                               .sampling_interval = cfg_.dt_control};

  // 1. Get the Intent (Horizon)
  // This handles the equilibrium gating internally using last_sample_finished_
  auto horizon = mission_.getUpdatedTrajectory(state, last_sample_finished_);

  // 2. Interpret the Intent (Sampling)
  // The sampler is free to iterate across the 'horizon' vector.
  auto sample_res =
      sampler_->getSetpoint(horizon, state, sample_context, commands);
  if (!sample_res) {
    return std::unexpected(sample_res.error());
  }

  // Update feedback for next iteration
  last_sample_finished_ = sample_res->all_finished;

  // 3. Global Completion Check
  // We are done if the planned queue is dry AND we've settled at the fallback.
  if (mission_.empty()) {
    // horizon[0] is the fallback_hover_ here.
    return horizon[0]
               ->checkEquilibrium(state,
                                  {.position_tol = cfg_.acceptance_radius})
               .state != EquilibriumState::kNone;
  }

  return false;
}

EstimationControlMissionRunner::EstimationControlMissionRunner(
    std::shared_ptr<QuadrotorSimulator> sim,
    std::shared_ptr<ControllerBase> ctrl,
    std::shared_ptr<EstimatorDriverBase> est, Mission trajectory, Config config,
    std::shared_ptr<spdlog::logger> logger)
    : MissionRunner(std::move(sim), std::move(ctrl), std::move(trajectory),
                    std::move(config), std::move(logger)),

      est_(std::move(est)) {
  logger_->info("Using sampling frequency {:.2f} Hz for IMU filter",
                1.0 / cfg_.dt_sim);
  if (cfg_.gyro_cutoff_hz > 0.0) {
    std::ignore =
        gyro_filter_.initialize(cfg_.gyro_cutoff_hz, 1.0 / cfg_.dt_sim);
  }
  if (cfg_.accel_cutoff_hz > 0.0) {
    std::ignore =
        accel_filter_.initialize(cfg_.accel_cutoff_hz, 1.0 / cfg_.dt_sim);
  }

  logger_->info("Estimator attached: {}", est_->name());
  est_->start();
}

auto EstimationControlMissionRunner::getStateEstimate(int step) const
    -> StateAndCovariance {
  auto x_res = est_->getStateAt(sim_->state().timestamp_secs);
  if (!x_res.has_value()) {
    logger_->error("Estimator failed to provide state at step {}: {}. ", step,
                   x_res.error());
  }
  return {x_res.value_or(sim_->state()), est_->getCovariance()};
}

bool EstimationControlMissionRunner::onStepStart(int step) {
  if (!est_->isHealthy()) {
    spdlog::error("Estimator became unhealthy at step {}, time={}", step,
                  sim_->state().timestamp_secs);
    return false;
  }
  return true;
}

bool EstimationControlMissionRunner::onSimStepEnd(int /*step*/) {
  const auto curr_time = sim_->state().timestamp_secs;
  auto imu = sim_->getImuMeasurement(cfg_.dt_sim);
  if (cfg_.gyro_cutoff_hz > 0.0) {
    imu->gyro = gyro_filter_.compute(imu->gyro);
  }
  if (cfg_.accel_cutoff_hz > 0.0) {
    imu->accel = accel_filter_.compute(imu->accel);
  }
  // B. IMU Input (Fast: e.g. 1000Hz)
  // Note: In reality, IMU data comes *from* the step integration.
  // Ensure sim_ generates IMU data corresponding to the interval we just
  // stepped.
  est_->push(imu);

  if (curr_time - last_gps_time_ >= cfg_.dt_gps) {
    // C. GPS Measurement (Slow: e.g. 10Hz)
    // Check if enough simulation time has passed for a GPS hit
    auto gps_data =
        sim_->getLocalPositionMeasurement();  // Generate from current_state
    est_->push(gps_data);                     // Push Measurement (Correction)

    last_gps_time_ = curr_time;

    // Optional: Add some jitter to last_gps_time here to simulate
    // non-deterministic arrival, testing your AsyncEstimator's queue.
  }
  return true;
}

bool EstimationControlMissionRunner::onStepEnd(int /*step*/) {
  est_->wait();
  return true;
}
}  // namespace autopilot
