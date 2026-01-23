#ifndef AUTOPILOT_PLANNING_TIME_SAMPLER_HPP_
#define AUTOPILOT_PLANNING_TIME_SAMPLER_HPP_

#include "autopilot/planning/flatness_map.hpp"
#include "autopilot/planning/sampler_base.hpp"

namespace autopilot {

class TimeSampler : public SamplerBase {
 public:
  explicit TimeSampler(std::shared_ptr<QuadrotorModel> model)
      : mapper_(std::move(model)) {}

  std::expected<Sample, AutopilotErrc> getSetpoint(
      const PolynomialTrajectory& traj, const QuadrotorState& state) override;

 private:
  FlatnessMap mapper_;
};
}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_TIME_SAMPLER_HPP_
