#include "autopilot/planning/preset_trajectories.hpp"

#include "autopilot/core/math.hpp"

namespace autopilot {

KinematicState Hover::sample(double timestamp) const {
  return {.timestamp_secs = timestamp, .position = pos_, .yaw = yaw_};
}

AutopilotErrc Hover::truncate(double new_end_time_secs) {
  if (new_end_time_secs < start_t_) {
    return AutopilotErrc::kTimestampOutOfOrder;
  }
  duration_ = new_end_time_secs - start_t_;
  return AutopilotErrc::kNone;
}

AutopilotErrc Hover::shiftStartTime(double shift_duration) {
  if (shift_duration < 0) {
    return AutopilotErrc::kOutOfBounds;
  }
  start_t_ += shift_duration;
  return AutopilotErrc::kNone;
}

EquilibriumStatus Hover::checkEquilibrium(
    const QuadrotorState& state, const EquilibriumTolerances& tols) const {
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

KinematicState HeadingChange::sample(double timestamp) const {
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

AutopilotErrc HeadingChange::truncate(double new_end_time_secs) {
  if (new_end_time_secs < start_t_) {
    return AutopilotErrc::kTimestampOutOfOrder;
  }

  auto new_end_state = sample(new_end_time_secs);
  end_yaw_ = new_end_state.yaw;
  duration_ = new_end_time_secs - start_t_;
  return AutopilotErrc::kNone;
}

AutopilotErrc HeadingChange::shiftStartTime(double shift_duration) {
  if (shift_duration < 0) {
    return AutopilotErrc::kOutOfBounds;
  }
  start_t_ += shift_duration;
  return AutopilotErrc::kNone;
}

EquilibriumStatus HeadingChange::checkEquilibrium(
    const QuadrotorState& state, const EquilibriumTolerances& tols) const {
  const double pe = (state.odometry.pose().translation() - pos_).norm();
  const double ae = std::abs(
      wrapToPi(QuaternionToRollPitchYaw(state.odometry.pose().rotation()).z() -
               end_yaw_));

  if (pe <= tols.position_tol && ae <= tols.angle_tol) {
    return {.state = EquilibriumState::kStatic,
            .position_error_mag = pe,
            .angle_error_mag = ae};
  }

  return {.state = EquilibriumState::kNone,
          .position_error_mag = pe,
          .angle_error_mag = ae};
}

Polynomial<double, 3> HeadingChange::CreateYawPolynomial(double start_yaw,
                                                         double end_yaw,
                                                         double duration) {
  // Cubic polynomial coefficients for yaw
  const double delta = wrapToPi(end_yaw - start_yaw);
  const double duration_sq = duration * duration;
  return Polynomial<double, 3>(start_yaw, 0.0, 3.0 * delta / duration_sq,
                               -2.0 * delta / (duration_sq * duration));
}

}  // namespace autopilot
