#ifndef AUTOPILOT_PLANNING_TRAJECTORY_HPP_
#define AUTOPILOT_PLANNING_TRAJECTORY_HPP_

#include <vector>

#include "autopilot/core/definitions.hpp"
#include "autopilot/planning/polynomial.hpp"  // The new API class

#ifndef AUTOPILOT_PLANNING_POLYNOMIAL_DEGREE
#define AUTOPILOT_PLANNING_POLYNOMIAL_DEGREE 7
#endif

namespace autopilot {

inline constexpr Eigen::Index kPolynomialDegree =
    AUTOPILOT_PLANNING_POLYNOMIAL_DEGREE;

using PlanningPolynomial =
    Polynomial<double, kPolynomialDegree>;  // Alias for easier usage

/**
 * @brief Represents a multi-dimensional (3D) piecewise segment.
 * Each segment is defined for a specific duration [0, double].
 */
class TrajectorySegment {
 public:
  TrajectorySegment(double duration, std::array<PlanningPolynomial, 3> axes)
      : duration_(duration), axes_(std::move(axes)) {}

  [[nodiscard]] double duration() const { return duration_; }

  /**
   * @brief Samples the segment to produce a QuadrotorState.
   * Leverages the Polynomial::derivVal API for feed-forward terms.
   */
  [[nodiscard]] KinematicState sample(double t) const;

 private:
  double duration_;
  std::array<Polynomial<double, kPolynomialDegree>, 3> axes_;
};

/**
 * @brief Redesign of PiecewisePolynomialTrajectory from Python.
 * Manages the sequence of segments and global timing.
 */
class PolynomialTrajectory {
 public:
  PolynomialTrajectory(std::span<const TrajectorySegment> segments,
                       double start_time = 0.0);

  [[nodiscard]] KinematicState sample(double timestamp) const;

  [[nodiscard]] double duration() const { return cumulative_times_.back(); }

 private:
  std::vector<TrajectorySegment> segments_;
  std::vector<double> cumulative_times_;
  double start_time_;
};

}  // namespace autopilot
#endif
