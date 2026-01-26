#include "autopilot/planning/polynomial_trajectory.hpp"

#include <numeric>

#include "autopilot/core/math.hpp"

namespace autopilot {
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

PolynomialTrajectory::PolynomialTrajectory(
    std::span<const TrajectorySegment> segments, double start_time,
    const HeadingPolicy& policy)
    : segments_(segments.begin(), segments.end()),
      start_time_(start_time),
      heading_policy_(policy) {
  // Precompute cumulative times for efficient segment lookups
  cumulative_times_.reserve(segments_.size() + 1);
  cumulative_times_.push_back(0.0);
  std::transform_inclusive_scan(
      segments_.begin(), segments_.end(), std::back_inserter(cumulative_times_),
      std::plus(), std::mem_fn(&TrajectorySegment::duration));
}

[[nodiscard]] KinematicState PolynomialTrajectory::sample(
    double timestamp) const {
  double t_rel = timestamp - start_time_;
  double total_duration = cumulative_times_.back();

  // Boundary handling matching Python's sample logic
  if (t_rel <= 0) {
    return segments_.front().sample(0);
  }
  if (t_rel >= total_duration) {
    return segments_.back().sample(segments_.back().duration());
  }

  // Binary search for the correct segment (more efficient than Python loop)
  auto it = std::ranges::upper_bound(cumulative_times_, t_rel);
  auto idx = static_cast<std::size_t>(
      std::distance(cumulative_times_.begin(), it) - 1);

  double t_local = t_rel - cumulative_times_[idx];
  auto sample = segments_[idx].sample(t_local);
  sample = ResolveYawState(sample, heading_policy_);
  return sample;
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
