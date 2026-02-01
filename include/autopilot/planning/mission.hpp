#ifndef AUTOPILOT_PLANNING_MISSION_HPP_
#define AUTOPILOT_PLANNING_MISSION_HPP_

#include <queue>

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
    queue_.emplace(std::move(traj));
  }

  template <std::derived_from<TrajectoryBase> T>
  void append(const T& traj)
    requires(!std::is_same_v<T, TrajectoryBase>)
  {
    // Note: This requires TrajectoryBase to have a proper copy constructor
    queue_.emplace(std::make_unique<T>(traj));
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
  std::shared_ptr<TrajectoryBase> getUpdatedTrajectory(
      const QuadrotorState& state, bool finished_hint,
      const EquilibriumTolerances& tols = {}) {
    if (queue_.empty()) {
      return std::make_shared<Hover>(fallback_hover_);
    }

    if (finished_hint) {
      if (auto updated_traj = advance(state, tols)) {
        return updated_traj;
      }
    }

    return queue_.front();
  }

  /// @brief Overload that infers finished_hint from state time
  ///
  /// @param state Current quadrotor state
  /// @param tols Equilibrium tolerances
  /// @return Updated trajectory segment or fallback hover
  std::shared_ptr<TrajectoryBase> getUpdatedTrajectory(
      const QuadrotorState& state, const EquilibriumTolerances& tols = {}) {
    return getUpdatedTrajectory(
        state, state.timestamp_secs >= queue_.front()->endTime(), tols);
  }

  void pop() { queue_.pop(); }

  bool empty() const { return queue_.empty(); }

  size_t size() const { return queue_.size(); }

 private:
  std::shared_ptr<TrajectoryBase> advance(const QuadrotorState& exit_state,
                                          const EquilibriumTolerances& tols) {
    const auto& curr = queue_.front();

    if (!curr->requiresEquilibriumCheck() ||
        curr->checkEquilibrium(exit_state, tols).state !=
            EquilibriumState::kNone) {
      // Transition: Capture the current end-point for a smooth handover
      const auto final_ks = curr->sample(curr->endTime());
      updateFallback(final_ks, exit_state.timestamp_secs);

      queue_.pop();

      // Recursive call handles zero-duration segments or immediate
      // back-to-back logic We pass a 'dummy' result to prevent infinite loops
      // if the new segment is also done
      return getUpdatedTrajectory(exit_state, false);
    }
    return nullptr;
  }

  void updateFallback(const KinematicState& end_state, double time) {
    // We capture the exact position and yaw where the last segment finished.
    // Note: Since end_state.velocity should be near-zero (due to equilibrium
    // gating), this creates a perfect C0/C1 handover.
    fallback_hover_ = Hover(end_state.position, end_state.yaw, time);
  }

  std::shared_ptr<MissionCfg> cfg_;
  std::queue<std::shared_ptr<TrajectoryBase>> queue_;
  Hover fallback_hover_{Eigen::Vector3d::Zero(), 0.0, 0.0};
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_MISSION_HPP_
