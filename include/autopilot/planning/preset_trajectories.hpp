#ifndef AUTOPILOT_PLANNING_PRESET_TRAJECTORIES_HPP_
#define AUTOPILOT_PLANNING_PRESET_TRAJECTORIES_HPP_

#include "autopilot/planning/polynomial.hpp"
#include "autopilot/planning/trajectory.hpp"

namespace autopilot {

class Hover final : public TrajectoryBase {
 public:
  template <typename Derived>
  Hover(const Eigen::MatrixBase<Derived>& position, double start_time,
        double duration = std::numeric_limits<double>::infinity(),
        double yaw = 0.0)
      : pos_(position), start_t_(start_time), duration_(duration), yaw_(yaw) {}

  bool requiresEquilibrium() const override { return true; }

  KinematicState sample(double timestamp) const override;

  double duration() const override { return duration_; }

  AutopilotErrc truncate(double new_end_time_secs) override;

  double startTime() const override { return start_t_; }

  AutopilotErrc shiftStartTime(double shift_duration) override;

  EquilibriumStatus checkEquilibrium(
      const QuadrotorState& state,
      const EquilibriumTolerances& tols) const override;

 private:
  Eigen::Vector3d pos_;
  double start_t_;
  double duration_;
  double yaw_;
};

class HeadingChange final : public TrajectoryBase {
 public:
  // Using a 3rd order polynomial for yaw to ensure zero velocity at ends
  template <typename Derived>
  HeadingChange(const Eigen::MatrixBase<Derived>& position, double start_yaw,
                double end_yaw, double start_time, double duration)
      : pos_(position),
        start_t_(start_time),
        duration_(duration),
        start_yaw_(start_yaw),
        end_yaw_(end_yaw),
        yaw_poly_(CreateYawPolynomial(start_yaw_, end_yaw_, duration_)) {}

  bool requiresEquilibrium() const override { return true; }

  KinematicState sample(double timestamp) const override;

  double duration() const override { return duration_; }

  AutopilotErrc truncate(double new_end_time_secs) override;

  double startTime() const override { return start_t_; }

  AutopilotErrc shiftStartTime(double shift_duration) override;

  EquilibriumStatus checkEquilibrium(
      const QuadrotorState& state,
      const EquilibriumTolerances& tols) const override;

 private:
  static Polynomial<double, 3> CreateYawPolynomial(double start_yaw,
                                                   double end_yaw,
                                                   double duration);

  Eigen::Vector3d pos_;
  double start_t_;
  double duration_;
  double start_yaw_ = 0;
  double end_yaw_ = 0;
  Polynomial<double, 3> yaw_poly_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_PRESET_TRAJECTORIES_HPP_
