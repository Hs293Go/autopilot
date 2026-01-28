#ifndef AUTOPILOT_PLANNING_TRAJECTORY_HPP_
#define AUTOPILOT_PLANNING_TRAJECTORY_HPP_

#include "autopilot/core/definitions.hpp"

namespace autopilot {

enum class EquilibriumState {
  kNone,     // High tracking error / Not yet arrived
  kDynamic,  // Following a moving path within tolerance
  kStatic,   // Reached a stationary hover point and settled
  kSingular  // Specific force near zero (falling), math undefined
};

struct EquilibriumStatus {
  EquilibriumState state = EquilibriumState::kNone;

  // Allow limited feedback on the magnitude of errors
  double position_error_mag = 0.0;
  double velocity_error_mag = 0.0;
  double angle_error_mag = 0.0;
};

struct EquilibriumTolerances {
  double position_tol = 0.1;  // meters, also known as acceptance radius
  double velocity_tol = 0.1;  // meters/second
  double angle_tol = 0.1;     // radians
};

class TrajectoryBase {
 public:
  virtual ~TrajectoryBase() = default;

  [[nodiscard]] virtual KinematicState sample(double timestamp) const = 0;

  [[nodiscard]] virtual double duration() const = 0;

  virtual AutopilotErrc truncate(double new_end_time_secs) = 0;

  [[nodiscard]] virtual double startTime() const = 0;

  virtual AutopilotErrc shiftStartTime(double shift_duration) = 0;

  [[nodiscard]] virtual double endTime() const {
    return startTime() + duration();
  }

  virtual bool requiresEquilibrium() const = 0;

  [[nodiscard]] virtual EquilibriumStatus checkEquilibrium(
      const QuadrotorState& state, const EquilibriumTolerances& tols) const = 0;
};

}  // namespace autopilot
#endif
