#include "autopilot/planning/time_sampler.hpp"

namespace autopilot {

std::expected<Sample, AutopilotErrc> TimeSampler::getSetpoint(
    const TrajectoryBase& traj, const QuadrotorState& state) {
  // Standard time-based sampling
  const double curr_time_secs = state.timestamp_secs;
  const KinematicState kinematics = traj.sample(curr_time_secs);

  // We can inject yaw logic here (e.g., look-ahead or fixed)

  auto setpoint = mapper_.compute(kinematics);
  if (!setpoint.has_value()) {
    return std::unexpected(setpoint.error());
  }

  return Sample{
      .command = QuadrotorCommand(setpoint.value()),
      .is_finished = traj.checkComplete(state),
      .is_hover = false  // WIP: Implement hover detection
  };
}

}  // namespace autopilot
