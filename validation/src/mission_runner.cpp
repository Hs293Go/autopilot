#include "validation/mission_runner.hpp"

#include <thread>
#include <utility>

#include "fmt/ranges.h"

namespace autopilot {

MissionRunner::MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                             std::shared_ptr<ControllerBase> ctrl,
                             std::shared_ptr<EstimatorDriverBase> est,
                             PolynomialTrajectory trajectory, Config config,
                             std::shared_ptr<spdlog::logger> logger)
    : sim_(std::move(sim)),
      ctrl_(std::move(ctrl)),
      est_(std::move(est)),
      cfg_(std::move(config)),
      logger_(logger ? std::move(logger) : spdlog::default_logger()),
      current_trajectory_(std::move(trajectory)),
      flatness_map_(sim_->model()) {
  if (est_) {
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
}

MissionRunner::MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                             std::shared_ptr<ControllerBase> ctrl,
                             PolynomialTrajectory trajectory, Config config,
                             std::shared_ptr<spdlog::logger> logger)
    : MissionRunner(std::move(sim), std::move(ctrl), nullptr,
                    std::move(trajectory), std::move(config),
                    std::move(logger)) {}

void MissionRunner::pushEstimatorData(double& last_gps_time, double curr_time) {
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

  if (curr_time - last_gps_time >= cfg_.dt_gps) {
    // C. GPS Measurement (Slow: e.g. 10Hz)
    // Check if enough simulation time has passed for a GPS hit
    auto gps_data =
        sim_->getLocalPositionMeasurement();  // Generate from current_state
    est_->push(gps_data);                     // Push Measurement (Correction)

    last_gps_time = curr_time;

    // Optional: Add some jitter to last_gps_time here to simulate
    // non-deterministic arrival, testing your AsyncEstimator's queue.
  }
}

QuadrotorState MissionRunner::getStateEstimate(int step) const {
  if (est_) {
    auto x_res = est_->getStateAt(sim_->state().timestamp_secs);
    if (!x_res.has_value()) {
      logger_->error("Estimator failed to provide state at step {}: {}. ", step,
                     x_res.error().message());
    }
    return x_res.value_or(sim_->state());
  }
  return sim_->state();
}

bool MissionRunner::detectGeofenceViolation(const QuadrotorState& state) const {
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

SimulationResult MissionRunner::run() {
  SimulationResult res;
  QuadrotorCommand cmd;

  // Future proof: Query controller for number of setpoints (prediction horizon)
  std::vector<QuadrotorCommand> sp_buf(1);
  std::vector<QuadrotorCommand> out_buf(1);

  double last_gps_time = -cfg_.dt_gps;
  const int sim_substeps =
      static_cast<int>(std::ceil(cfg_.dt_control / cfg_.dt_sim));
  spdlog::info(
      "Starting Mission Runner for up to {} steps, with {} simulation substeps",
      cfg_.max_steps, sim_substeps);

  for (int step = 0; step < cfg_.max_steps; ++step) {
    if (est_) {
      if (!est_->isHealthy()) {
        spdlog::error("Estimator became unhealthy at step {}", step);
        break;
      }
    }
    auto state_est = getStateEstimate(step);
    auto state = sim_->state();

    // 1. Check Waypoint
    auto kinematic_state =
        current_trajectory_.sample(sim_->state().timestamp_secs);
    auto setpoint = flatness_map_.compute(kinematic_state).value();
    if (state.timestamp_secs >= current_trajectory_.duration()) {
      res.completed = true;
      break;
    }

    // 2. Control
    cmd.reset(state.timestamp_secs);
    std::ignore = cmd.setPosition(setpoint.position());
    // std::ignore = cmd.setYaw(mission_[wp_idx].yaw.value_or(0.0));

    sp_buf[0] = cmd;
    if (auto result = ctrl_->compute(state_est, sp_buf, out_buf); !result) {
      logger_->error("Controller failed at t={:.2f}s, because: {}",
                     state.timestamp_secs, result.error().message());
      break;
    }

    // 3. Sim
    for (int i = 0; i < sim_substeps; ++i) {
      sim_->step(out_buf[0], cfg_.dt_sim);

      auto curr_time = sim_->state().timestamp_secs;

      if (est_) {
        pushEstimatorData(last_gps_time, curr_time);
      }
    }

    if (detectGeofenceViolation(state)) {
      break;
    }

    // 4. Record
    res.time.push_back(state.timestamp_secs);
    if (est_) {
      res.hist.push_back(History{
          .real_state = state,
          .estimated_state = state_est,
          .command = sp_buf[0],
          .est_variance = est_->getCovariance().diagonal(),
      });
    } else {
      res.hist.push_back(History{.real_state = state,
                                 .estimated_state = state_est,
                                 .command = sp_buf[0]});
    }
    if (est_) {
      est_->wait();
    }
  }
  return res;
}
}  // namespace autopilot
