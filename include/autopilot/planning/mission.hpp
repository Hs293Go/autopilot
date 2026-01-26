#ifndef AUTOPILOT_PLANNING_MISSION_HPP_
#define AUTOPILOT_PLANNING_MISSION_HPP_

#include <deque>

#include "autopilot/base/config_base.hpp"
#include "autopilot/core/math.hpp"
#include "autopilot/planning/preset_trajectories.hpp"
#include "autopilot/planning/sampler_base.hpp"

namespace autopilot {
struct Result {
  SampleResult sample;
  bool is_complete = false;
};

struct MissionCfg : public ReflectiveConfigBase<MissionCfg> {
  double position_tol = 0.1;
  double velocity_tol = 0.1;
  double angle_tol = rad2deg(5.0);  // 5 degrees in radians

  std::string_view name() const override { return "MissionCfg"; }

  EquilibriumTolerances getTolerances() const {
    return {.position_tol = position_tol,
            .velocity_tol = velocity_tol,
            .angle_tol = angle_tol};
  }

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("position_tol", &MissionCfg::position_tol,
               F64Properties{.desc = "Position tolerance for waypoint (m)",
                             .bounds = Bounds<double>::NonNegative()}),
      Describe("velocity_tol", &MissionCfg::velocity_tol,
               F64Properties{.desc = "Velocity tolerance for waypoint (m/s)",
                             .bounds = Bounds<double>::NonNegative()}),
      Describe("angle_tol", &MissionCfg::angle_tol,
               F64Properties{.desc = "Angle tolerance for waypoint (rad)",
                             .bounds = Bounds<double>::NonNegative()}));
};

class Mission {
 public:
  Mission() = default;

  Mission(std::shared_ptr<MissionCfg> cfg) : cfg_(std::move(cfg)) {}

  void append(std::unique_ptr<TrajectoryBase>&& traj) {
    if (traj) {
      queue_.push_back(std::move(traj));
    }
  }

  template <std::derived_from<TrajectoryBase> T>
  void append(const T& traj)
    requires(!std::is_same_v<T, TrajectoryBase>)
  {
    // Note: This requires TrajectoryBase to have a proper copy constructor
    queue_.emplace_back(std::make_shared<T>(traj));
  }

  std::shared_ptr<TrajectoryBase> current() const {
    return queue_.empty() ? nullptr : queue_.front();
  }

  /// @brief Get the updated trajectory segment, advancing if finished_hint is
  /// true
  ///
  /// @param state Current quadrotor state
  /// @param finished_hint Hint that the current segment is finished
  /// @param tols Equilibrium tolerances
  /// @return Updated trajectory segment or fallback hover
  std::vector<std::shared_ptr<TrajectoryBase>> getUpdatedTrajectory(
      const QuadrotorState& state, bool finished_hint) {
    if (finished_hint && !queue_.empty()) {
      // We may fail to advance if the equilibrium check fails. In that case, we
      // return only the current segment to allow settling.
      if (!advance(state, cfg_->getTolerances())) {
        return {queue_.empty() ? std::make_shared<Hover>(fallback_hover_)
                               : queue_.front()};
      }
    }

    if (queue_.empty()) {
      // If the queue is dry, serve the Infinite Safety Anchor
      return {std::make_shared<Hover>(fallback_hover_)};
    }

    return {queue_.begin(), queue_.end()};
  }

  void pop() { queue_.pop_front(); }

  bool empty() const { return queue_.empty(); }

  size_t size() const { return queue_.size(); }

 private:
  bool advance(const QuadrotorState& state, const EquilibriumTolerances& tols) {
    const auto& curr = queue_.front();

    // Completion of spline/polynomial dynamic trajectories is usually not gated
    // on equilibrium, so they can always be popped when time is up.
    // Otherwise, we check if we have reached equilibrium before popping.
    bool can_pop =
        !curr->requiresEquilibriumCheck() ||
        curr->checkEquilibrium(state, tols).state != EquilibriumState::kNone;

    // Signal that we could not advance due to equilibrium not met. The mission
    // will hand out a restricted trajectory to allow settling.
    if (!can_pop) {
      return false;
    }
    // Kinematic Handover: Update the fallback before we lose the intent
    const auto final_ks = curr->sample(curr->endTime());
    updateFallback(final_ks, state.timestamp_secs);

    queue_.pop_front();

    // Recursive check: Handle zero-duration or already-expired next segments.
    if (!queue_.empty()) {
      double end_t = queue_.front()->endTime();
      bool already_done = std::isfinite(end_t) && state.timestamp_secs >= end_t;
      if (already_done) {
        advance(state, tols);
      }
    }
    return true;
  }

  void updateFallback(const KinematicState& end_state, double time) {
    // We capture the exact position and yaw where the last segment finished.
    // Note: Since end_state.velocity should be near-zero (due to equilibrium
    // gating), this creates a perfect C0/C1 handover.
    fallback_hover_ =
        Hover(end_state.position, time, std::numeric_limits<double>::infinity(),
              end_state.yaw);
  }

  std::shared_ptr<MissionCfg> cfg_ = std::make_shared<MissionCfg>();
  std::deque<std::shared_ptr<TrajectoryBase>> queue_;
  Hover fallback_hover_{Eigen::Vector3d::Zero(), 0.0,
                        std::numeric_limits<double>::infinity(), 0.0};
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_MISSION_HPP_
