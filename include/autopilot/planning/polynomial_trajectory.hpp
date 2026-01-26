#ifndef AUTOPILOT_PLANNING_POLYNOMIAL_TRAJECTORY_HPP_
#define AUTOPILOT_PLANNING_POLYNOMIAL_TRAJECTORY_HPP_

#include <span>

#include "autopilot/planning/polynomial.hpp"  // The new API class
#include "autopilot/planning/trajectory.hpp"

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

/// @brief Redesign of PiecewisePolynomialTrajectory from Python.
/// Manages the sequence of segments and global timing.
class PolynomialTrajectory final : public TrajectoryBase {
 public:
  PolynomialTrajectory(std::span<const TrajectorySegment> segments,
                       double start_time = 0.0,
                       const HeadingPolicy& policy = {});

  std::unique_ptr<TrajectoryBase> clone() const override {
    return std::make_unique<PolynomialTrajectory>(*this);
  }

  KinematicState sample(double timestamp) const override;

  double duration() const override { return cumulative_times_.back(); }

  double startTime() const override { return start_time_; }

  bool checkExpiry(const QuadrotorState& state) const override;

  EquilibriumStatus checkEquilibrium(
      const QuadrotorState& state,
      const EquilibriumTolerances& tols) const override;

 private:
  std::vector<TrajectorySegment> segments_;
  std::vector<double> cumulative_times_;
  double start_time_;
  HeadingPolicy heading_policy_;
  mutable KinematicState cache_sample_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_POLYNOMIAL_TRAJECTORY_HPP_
