#ifndef AUTOPILOT_PLANNING_FLATNESS_MAP_HPP_
#define AUTOPILOT_PLANNING_FLATNESS_MAP_HPP_

#include <expected>

#include "autopilot/core/definitions.hpp"
#include "autopilot/core/quadrotor_model.hpp"

namespace autopilot {

class FlatnessMap {
 public:
  explicit FlatnessMap(std::shared_ptr<const QuadrotorModel> model)
      : model_(std::move(model)) {}

  /**
   * @brief Converts kinematic derivatives to a full quadrotor command.
   * @param kinematics Position, velocity, acceleration, jerk, and snap.
   * @param yaw Desired heading (rad).
   * @param yaw_rate Desired heading rate (rad/s).
   */
  [[nodiscard]] std::expected<QuadrotorCommand, AutopilotErrc> compute(
      const KinematicState& kinematics) const;

 private:
  std::shared_ptr<const QuadrotorModel> model_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_FLATNESS_MAP_HPP_
