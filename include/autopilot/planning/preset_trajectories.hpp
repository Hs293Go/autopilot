#ifndef AUTOPILOT_PLANNING_PRESET_TRAJECTORIES_HPP_
#define AUTOPILOT_PLANNING_PRESET_TRAJECTORIES_HPP_

#include "autopilot/core/math.hpp"
#include "autopilot/planning/polynomial.hpp"
#include "autopilot/planning/trajectory.hpp"

namespace autopilot {

class Hover final : public TrajectoryBase {
 public:
  template <typename Derived>

  Hover(const Eigen::MatrixBase<Derived>& position, double start_time,
        double yaw = 0.0)
      : pos_(position), yaw_(yaw), start_t_(start_time) {}

  std::unique_ptr<TrajectoryBase> clone() const override {
    return std::make_unique<Hover>(*this);
  }

  KinematicState sample(double timestamp) const override {
    return {.timestamp_secs = timestamp, .position = pos_, .yaw = yaw_};
  }

  double duration() const override {
    return std::numeric_limits<double>::infinity();
  }

  double startTime() const override { return start_t_; }

  bool checkExpiry(const QuadrotorState& /*state*/) const override {
    return false;  // Never expires
  }

  EquilibriumStatus checkEquilibrium(
      const QuadrotorState& state,
      const EquilibriumTolerances& tols) const override {
    const double pe = (state.odometry.pose().translation() - pos_).norm();
    const double ve = state.odometry.twist().linear().norm();

    if (pe <= tols.position_tol && ve <= tols.velocity_tol) {
      return {.state = EquilibriumState::kStatic,
              .position_error_mag = pe,
              .velocity_error_mag = ve};
    }

    return {.state = EquilibriumState::kNone,
            .position_error_mag = pe,
            .velocity_error_mag = ve};
  }

 private:
  Eigen::Vector3d pos_;
  double yaw_;
  double start_t_;
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

  std::unique_ptr<TrajectoryBase> clone() const override {
    return std::make_unique<HeadingChange>(*this);
  }

  KinematicState sample(double timestamp) const override {
    const double t = std::clamp(timestamp - start_t_, 0.0, duration_);

    KinematicState ks;
    ks.timestamp_secs = timestamp;
    ks.position = pos_;

    // Evaluate 1D Yaw Polynomial
    ks.yaw = yaw_poly_(t);
    ks.yaw_rate = yaw_poly_.derivVal(t, 1);
    ks.yaw_acceleration = yaw_poly_.derivVal(t, 2);

    return ks;
  }

  double duration() const override { return duration_; }
  double startTime() const override { return start_t_; }

  bool checkExpiry(const QuadrotorState& state) const override {
    return state.timestamp_secs >= endTime();
  }

  EquilibriumStatus checkEquilibrium(
      const QuadrotorState& state,
      const EquilibriumTolerances& tols) const override {
    const double pe = (state.odometry.pose().translation() - pos_).norm();
    const double ae = std::abs(
        QuaternionToRollPitchYaw(state.odometry.pose().rotation()).z() -
        end_yaw_);

    if (pe <= tols.position_tol && ae <= tols.angle_tol) {
      return {.state = EquilibriumState::kStatic,
              .position_error_mag = pe,
              .angle_error_mag = ae};
    }

    return {.state = EquilibriumState::kNone,
            .position_error_mag = pe,
            .angle_error_mag = ae};
  }

 private:
  static Polynomial<double, 3> CreateYawPolynomial(double start_yaw,
                                                   double end_yaw,
                                                   double duration) {
    // Cubic polynomial coefficients for yaw
    const double delta = (end_yaw - start_yaw);
    const double duration_sq = duration * duration;
    return Polynomial<double, 3>(start_yaw, 0.0, 3.0 * delta / duration_sq,
                                 -2.0 * delta / (duration_sq * duration));
  }

  Eigen::Vector3d pos_;
  double start_t_;
  double duration_;
  double start_yaw_ = 0;
  double end_yaw_ = 0;
  Polynomial<double, 3> yaw_poly_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_PRESET_TRAJECTORIES_HPP_
