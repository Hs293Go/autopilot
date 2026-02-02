#ifndef AUTOPILOT_PLANNING_MINIMUM_SNAP_SOLVER_HPP_
#define AUTOPILOT_PLANNING_MINIMUM_SNAP_SOLVER_HPP_

#include <expected>

#include "Eigen/Dense"
#include "autopilot/planning/polynomial_trajectory.hpp"

namespace autopilot {

static_assert(kPolynomialDegree % 2 == 1,
              "Polynomial degree must be odd for minimum snap solver.");
static constexpr Eigen::Index kContinuousDegrees =
    (kPolynomialDegree - 1) / 2 + 1;

class MinimumSnapSolver {
 public:
  // Port of generate_trajectory / _solve_closed_form
  std::expected<PolynomialTrajectory, AutopilotErrc> solve(
      std::span<const TrajectoryWaypoint> waypoints,
      const HeadingPolicy& policy = FollowVelocity{}) const;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_MINIMUM_SNAP_SOLVER_HPP_
