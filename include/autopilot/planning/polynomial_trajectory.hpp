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

  AutopilotErrc setDuration(double new_duration);

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
class PolynomialTrajectory : public TrajectoryBase {
 public:
  PolynomialTrajectory(std::span<const TrajectorySegment> segments,
                       double start_time = 0.0,
                       const HeadingPolicy& policy = {});

  bool requiresEquilibrium() const override {
    return requires_equilibrium_check_;
  }

  void setRequiresEquilibrium(bool requires_check) {
    requires_equilibrium_check_ = requires_check;
  }

  KinematicState sample(double timestamp) const override;

  double duration() const override { return cumulative_times_.back(); }

  AutopilotErrc truncate(double new_end_time_secs) override;

  double startTime() const override { return start_time_; }

  AutopilotErrc shiftStartTime(double shift_duration) override;

  EquilibriumStatus checkEquilibrium(
      const QuadrotorState& state,
      const EquilibriumTolerances& tols) const override;

 private:
  void updateCumulativeTimes();

  std::vector<TrajectorySegment> segments_;
  std::vector<double> cumulative_times_;
  double start_time_;
  HeadingPolicy heading_policy_;
  bool requires_equilibrium_check_ = false;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_POLYNOMIAL_TRAJECTORY_HPP_
