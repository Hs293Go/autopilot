#include "validation/mission_runner.hpp"

#include "fmt/ranges.h"

namespace autopilot {

MissionRunner::MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                             std::shared_ptr<ControllerBase> ctrl,
                             std::shared_ptr<EstimatorBase> est,
                             std::span<const MissionWaypoint> mission,
                             Config config,
                             std::shared_ptr<spdlog::logger> logger)
    : sim_(std::move(sim)),
      ctrl_(std::move(ctrl)),
      est_(std::move(est)),
      mission_(mission.begin(), mission.end()),
      cfg_(config),
      logger_(logger ? std::move(logger) : spdlog::default_logger()) {
  if (est_) {
    logger_->info("Estimator attached: {}", est_->name());
    est_->start();
  }
}

MissionRunner::MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                             std::shared_ptr<ControllerBase> ctrl,
                             std::span<const MissionWaypoint> mission,
                             Config config,
                             std::shared_ptr<spdlog::logger> logger)
    : MissionRunner(std::move(sim), std::move(ctrl), nullptr, mission, config,
                    std::move(logger)) {}

void MissionRunner::pushEstimatorData(double& last_gps_time, double curr_time) {
  auto imu = sim_->getImuMeasurement(cfg_.dt_sim);
  // B. IMU Input (Fast: e.g. 1000Hz)
  // Note: In reality, IMU data comes *from* the step integration.
  // Ensure sim_ generates IMU data corresponding to the interval we just
  // stepped.
  est_->push(imu);

  if (curr_time - last_gps_time >= cfg_.dt_gps) {
    // C. GPS Measurement (Slow: e.g. 10Hz)
    // Check if enough simulation time has passed for a GPS hit
    auto gps_data = sim_->getGpsMeasurement();  // Generate from current_state
    est_->push(gps_data);                       // Push Measurement (Correction)

    last_gps_time = curr_time;

    // Optional: Add some jitter to last_gps_time here to simulate
    // non-deterministic arrival, testing your AsyncEstimator's queue.
  }
}

QuadrotorState MissionRunner::getCurrentState(int step) const {
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

bool MissionRunner::isMissionComplete(const QuadrotorState& state,
                                      size_t& wp_idx) const {
  auto dist =
      (state.odometry.pose().translation() - mission_[wp_idx].position).norm();
  if (dist < cfg_.acceptance_radius) {
    logger_->info("Waypoint {} reached at t={:.2f}s", wp_idx,
                  state.timestamp_secs);

    wp_idx++;
    if (wp_idx >= mission_.size()) {
      if (logger_) {
        logger_->info("Mission Complete.");
      }
      return true;
    }
  }
  return false;
}

SimulationResult MissionRunner::run() {
  SimulationResult res;
  size_t wp_idx = 0;
  QuadrotorCommand cmd;

  // Future proof: Query controller for number of setpoints (prediction horizon)
  std::vector<QuadrotorCommand> sp_buf(1);
  std::vector<QuadrotorCommand> out_buf(1);

  double last_gps_time = -cfg_.dt_gps;
  QuadrotorState state = sim_->state();
  for (int step = 0; step < cfg_.max_steps; ++step) {
    state = getCurrentState(step);

    // 1. Check Waypoint
    if (isMissionComplete(state, wp_idx)) {
      res.completed = true;
      break;
    }

    // 2. Control
    cmd.reset(state.timestamp_secs);
    std::ignore = cmd.setPosition(mission_[wp_idx].position);
    std::ignore = cmd.setYaw(mission_[wp_idx].yaw);

    sp_buf[0] = cmd;
    if (auto result = ctrl_->compute(state, sp_buf, out_buf); !result) {
      logger_->error("Controller failed at t={:.2f}s, because: {}",
                     state.timestamp_secs, result.error().message());
      break;
    }

    // 3. Sim
    const int sim_substeps =
        static_cast<int>(std::ceil(cfg_.dt_control / cfg_.dt_sim));
    for (int i = 0; i < sim_substeps; ++i) {
      sim_->step(out_buf[0], cfg_.dt_sim);

      auto curr_time = sim_->state().timestamp_secs;

      if (est_) {
        pushEstimatorData(last_gps_time, curr_time);
      }
    }

    // 4. Record
    res.time_history.push_back(state.timestamp_secs);
    res.state_history.push_back(state);
    res.command_history.push_back(sp_buf[0]);
  }
  return res;
}
}  // namespace autopilot
