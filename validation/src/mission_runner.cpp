#include "validation/mission_runner.hpp"

namespace autopilot {

MissionRunner::MissionRunner(std::shared_ptr<QuadrotorSimulator> sim,
                             std::shared_ptr<ControllerBase> ctrl,
                             std::span<const MissionWaypoint> mission,
                             Config config,
                             std::shared_ptr<spdlog::logger> logger)
    : sim_(std::move(sim)),
      ctrl_(std::move(ctrl)),
      mission_(mission.begin(), mission.end()),
      cfg_(config),
      logger_(logger ? std::move(logger) : spdlog::default_logger()) {}

SimulationResult MissionRunner::run() {
  SimulationResult res;
  size_t wp_idx = 0;
  QuadrotorCommand cmd;
  std::vector<QuadrotorCommand> sp_buf(1), out_buf(1);

  for (int step = 0; step < cfg_.max_steps; ++step) {
    const auto& state = sim_->state();

    // 1. Check Waypoint
    auto dist =
        (state.odometry.pose().translation() - mission_[wp_idx].position)
            .norm();
    if (dist < cfg_.acceptance_radius) {
      logger_->info("Waypoint {} reached at t={:.2f}s", wp_idx,
                    state.timestamp_secs);

      wp_idx++;
      if (wp_idx >= mission_.size()) {
        if (logger_) {
          logger_->info("Mission Complete.");
        }
        res.completed = true;
        break;
      }
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
    double dt_sim = cfg_.dt_control / cfg_.sim_substeps;
    for (int i = 0; i < cfg_.sim_substeps; ++i) {
      sim_->step(out_buf[0], dt_sim);
    }

    // 4. Record
    res.time_history.push_back(state.timestamp_secs);
    res.state_history.push_back(state);
    res.command_history.push_back(sp_buf[0]);
  }
  return res;
}
}  // namespace autopilot
