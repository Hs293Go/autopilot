#ifndef AUTOPILOT_PLANNING_SAMPLER_BASE_HPP_
#define AUTOPILOT_PLANNING_SAMPLER_BASE_HPP_

#include <expected>

#include "autopilot/planning/trajectory.hpp"

namespace autopilot {

struct SampleResult {
  bool finish_in_horizon = false;
  bool all_finished = false;
  std::size_t finish_at_idx = std::numeric_limits<std::size_t>::max();
  std::size_t num_computed_samples = 0;
};

struct SampleContext {
  std::size_t num_samples = 1;
  double sampling_interval;
};

class SamplerBase {
 public:
  virtual ~SamplerBase() = default;

  // The Sampler returns a command based on the provided trajectory and current
  // state.
  virtual std::expected<SampleResult, AutopilotErrc> getSetpoint(
      std::span<const std::shared_ptr<TrajectoryBase>> traj,
      const QuadrotorState& state, const SampleContext& context,
      std::span<QuadrotorCommand> sampled_commands) = 0;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_SAMPLER_BASE_HPP_
