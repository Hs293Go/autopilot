#ifndef AUTOPILOT_PLANNING_MINIMUM_SNAP_SOLVER_HPP_
#define AUTOPILOT_PLANNING_MINIMUM_SNAP_SOLVER_HPP_

#include <expected>

#include "Eigen/Dense"
#include "autopilot/planning/trajectory.hpp"

namespace autopilot {

static_assert(kPolynomialDegree % 2 == 1,
              "Polynomial degree must be odd for minimum snap solver.");
static constexpr Eigen::Index kContinuousDegrees =
    (kPolynomialDegree - 1) / 2 + 1;

class MinimumSnapSolver {
 public:
  // Port of generate_trajectory / _solve_closed_form
  std::expected<PolynomialTrajectory, std::error_code> solve(
      std::span<const TrajectoryWaypoint> waypoints) const;

 private:
  // Helpers mirroring Python _compute_Q, _compute_tvec
  Eigen::MatrixXd computeQ(double duration, int n_coeffs) const;
  Eigen::RowVectorXd computeTVector(double t, int r, int n_coeffs) const;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_MINIMUM_SNAP_SOLVER_HPP_
