#include "autopilot/planning/time_sampler.hpp"

namespace autopilot {

std::expected<SampleResult, AutopilotErrc> TimeSampler::getSetpoint(
    const TrajectoryBase& traj, const QuadrotorState& state,
    const SampleContext& context,
    std::span<QuadrotorCommand> sampled_commands) {
  if (context.num_samples != sampled_commands.size()) {
    return std::unexpected(AutopilotErrc::kInvalidBufferSize);
  }

  // Standard time-based sampling
  double curr_time_secs = state.timestamp_secs;
  std::size_t i;
  bool complete_in_horizon = false;
  std::size_t complete_at_idx = std::numeric_limits<std::size_t>::max();
  for (i = 0; i < context.num_samples; ++i) {
    const KinematicState kinematics = traj.sample(curr_time_secs);
    auto setpoint = mapper_.compute(kinematics);
    if (!setpoint.has_value()) {
      return std::unexpected(setpoint.error());
    }

    if (!complete_in_horizon && curr_time_secs >= traj.endTime()) {
      complete_in_horizon = true;
      complete_at_idx = i;
    }

    sampled_commands[i] = setpoint.value();
    curr_time_secs += context.sampling_interval;
  }

  return SampleResult{
      .finish_in_horizon = complete_in_horizon,
      .all_finished = (complete_at_idx == 0),
      .finish_at_idx = complete_at_idx,
      .num_computed_samples = i,
  };
}

}  // namespace autopilot
