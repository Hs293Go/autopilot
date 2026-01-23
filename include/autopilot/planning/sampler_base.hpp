#ifndef AUTOPILOT_PLANNING_SAMPLER_BASE_HPP_
#define AUTOPILOT_PLANNING_SAMPLER_BASE_HPP_

#include <expected>

#include "autopilot/base/config_base.hpp"
#include "autopilot/planning/trajectory.hpp"

namespace autopilot {

struct Sample {
  QuadrotorCommand command;
  bool is_finished = false;
  bool is_hover = false;
};

class SamplerBase {
 public:
  virtual ~SamplerBase() = default;

  // The Sampler returns a command based on the provided trajectory and current
  // state.
  virtual std::expected<Sample, AutopilotErrc> getSetpoint(
      const PolynomialTrajectory& traj, const QuadrotorState& state) = 0;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_SAMPLER_BASE_HPP_
