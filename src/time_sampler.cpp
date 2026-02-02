#include "autopilot/planning/time_sampler.hpp"

namespace autopilot {

std::expected<SampleResult, AutopilotErrc> TimeSampler::getSetpoint(
    std::span<const std::shared_ptr<TrajectoryBase>> traj,
    const QuadrotorState& state, const SampleContext& context,
    std::span<QuadrotorCommand> sampled_commands) {
  const double start_time = state.timestamp_secs;
  const double first_traj_end = traj.front()->endTime();

  // 1. Snapshot the "Executive" state
  const bool all_finished = (start_time >= first_traj_end);

  bool complete_in_horizon = all_finished;
  std::size_t complete_at_idx =
      all_finished ? 0 : std::numeric_limits<std::size_t>::max();

  double curr_time_secs = start_time;
  size_t h_idx = 0;
  size_t i = 0;

  for (; i < context.num_samples; ++i) {
    // 2. Spill-over logic
    while (h_idx + 1 < traj.size() &&
           curr_time_secs >= traj[h_idx]->endTime()) {
      h_idx++;
    }

    const auto kinematics = traj[h_idx]->sample(curr_time_secs);
    auto setpoint = mapper_.compute(kinematics);
    if (!setpoint.has_value()) {
      return std::unexpected(setpoint.error());
    }

    // 3. Track the "Horizon" finish line
    // We only care about the transition of the FIRST segment (traj[0])
    if (!complete_in_horizon && curr_time_secs >= first_traj_end) {
      complete_in_horizon = true;
      complete_at_idx = i;
    }

    sampled_commands[i] = setpoint.value();
    curr_time_secs += context.sampling_interval;
  }

  return SampleResult{
      .finish_in_horizon = complete_in_horizon,
      .all_finished = all_finished,
      .finish_at_idx = complete_at_idx,
      .num_computed_samples = i,
  };
}

}  // namespace autopilot
