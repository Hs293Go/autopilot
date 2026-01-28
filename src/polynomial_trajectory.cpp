#include "autopilot/planning/polynomial_trajectory.hpp"

#include <numeric>

#include "autopilot/core/math.hpp"

namespace autopilot {

AutopilotErrc TrajectorySegment::setDuration(double new_duration) {
  if (new_duration < 0) {
    return AutopilotErrc::kOutOfBounds;
  }
  duration_ = new_duration;
  return AutopilotErrc::kNone;
}

[[nodiscard]] KinematicState TrajectorySegment::sample(double t) const {
  // Position
  return {
      .position = std::apply(
          [t](auto&&... ax) { return Eigen::Vector3d(ax(t)...); }, axes_),
      .velocity = std::apply(
          [t](auto&&... ax) { return Eigen::Vector3d(ax.derivVal(t, 1)...); },
          axes_),
      .acceleration = std::apply(
          [t](auto&&... ax) { return Eigen::Vector3d(ax.derivVal(t, 2)...); },
          axes_),
      .jerk = std::apply(
          [t](auto&&... ax) { return Eigen::Vector3d(ax.derivVal(t, 3)...); },
          axes_),
      .snap = std::apply(
          [t](auto&&... ax) { return Eigen::Vector3d(ax.derivVal(t, 4)...); },
          axes_),
  };
}

void PolynomialTrajectory::updateCumulativeTimes() {
  cumulative_times_.reserve(segments_.size() + 1);
  cumulative_times_.emplace_back(0.0);
  std::transform_inclusive_scan(
      segments_.begin(), segments_.end(), std::back_inserter(cumulative_times_),
      std::plus(), std::mem_fn(&TrajectorySegment::duration));
}

PolynomialTrajectory::PolynomialTrajectory(
    std::span<const TrajectorySegment> segments, double start_time,
    const HeadingPolicy& policy)
    : segments_(segments.begin(), segments.end()),
      start_time_(start_time),
      heading_policy_(policy) {
  updateCumulativeTimes();
}

[[nodiscard]] KinematicState PolynomialTrajectory::sample(
    double timestamp) const {
  double t_rel = timestamp - start_time_;
  double total_duration = cumulative_times_.back();

  // Boundary handling matching Python's sample logic
  KinematicState sample;
  if (t_rel <= 0) {
    sample = segments_.front().sample(0);
  } else if (t_rel >= total_duration) {
    sample = segments_.back().sample(segments_.back().duration());
  } else {
    // Binary search for the correct segment (more efficient than Python loop)
    auto it = std::ranges::upper_bound(cumulative_times_, t_rel);
    auto idx = static_cast<std::size_t>(
        std::distance(cumulative_times_.begin(), it) - 1);

    double t_local = t_rel - cumulative_times_[idx];
    sample = segments_[idx].sample(t_local);
  }

  sample = ResolveYawState(sample, heading_policy_);
  sample.timestamp_secs = timestamp;
  return sample;
}

AutopilotErrc PolynomialTrajectory::truncate(double new_end_time_secs) {
  if (new_end_time_secs < start_time_) {
    return AutopilotErrc::kTimestampOutOfOrder;
  }

  double t_rel = new_end_time_secs - start_time_;

  // 1. Locate the segment containing the truncation point
  auto it = std::ranges::upper_bound(cumulative_times_, t_rel);
  if (it == cumulative_times_.end()) {
    // Requested truncation is after the current end.
    // This is technically a "no-op" or an extension we don't support.
    return AutopilotErrc::kNone;
  }

  auto offset = std::distance(cumulative_times_.begin(), it) - 1;
  auto idx = static_cast<std::size_t>(offset);

  // 2. Shorten the specific segment
  double t_local = t_rel - cumulative_times_[idx];
  segments_[idx].setDuration(t_local);

  // 3. Discard all subsequent segments
  segments_.erase(segments_.begin() + offset + 1, segments_.end());

  // 4. Rebuild the cumulative timeline
  cumulative_times_.clear();
  updateCumulativeTimes();

  return AutopilotErrc::kNone;
}

AutopilotErrc PolynomialTrajectory::shiftStartTime(double shift_duration) {
  if (shift_duration < 0) {
    return AutopilotErrc::kTimestampOutOfOrder;
  }
  start_time_ += shift_duration;
  return AutopilotErrc::kNone;
}

EquilibriumStatus PolynomialTrajectory::checkEquilibrium(
    const QuadrotorState& state, const EquilibriumTolerances& tols) const {
  const auto ref = sample(state.timestamp_secs);
  const double d_p =
      (state.odometry.pose().translation() - ref.position).norm();
  const double d_v = (state.odometry.twist().linear() - ref.velocity).norm();

  // Dynamic equilibrium: Tracking within 10cm and 0.2m/s
  if (d_p < tols.position_tol && d_v < tols.velocity_tol) {
    return {.state = EquilibriumState::kDynamic,
            .position_error_mag = d_p,
            .velocity_error_mag = d_v};
  }
  return {.state = EquilibriumState::kNone,
          .position_error_mag = d_p,
          .velocity_error_mag = d_v};
}

}  // namespace autopilot
