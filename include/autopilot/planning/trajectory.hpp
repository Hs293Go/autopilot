#ifndef AUTOPILOT_PLANNING_TRAJECTORY_HPP_
#define AUTOPILOT_PLANNING_TRAJECTORY_HPP_

#include "autopilot/core/definitions.hpp"

namespace autopilot {

class TrajectoryBase {
 public:
  virtual ~TrajectoryBase() = default;

  [[nodiscard]] virtual KinematicState sample(double timestamp) const = 0;

  [[nodiscard]] virtual double duration() const = 0;

  [[nodiscard]] virtual double startTime() const = 0;

  [[nodiscard]] virtual double endTime() const {
    return startTime() + duration();
  }

  [[nodiscard]] virtual bool checkComplete(
      const QuadrotorState& state) const = 0;
};

}  // namespace autopilot
#endif
