#ifndef AUTOPILOT_PLANNING_MISSION_HPP_
#define AUTOPILOT_PLANNING_MISSION_HPP_

#include <deque>

#include "autopilot/base/config_base.hpp"
#include "autopilot/core/math.hpp"
#include "autopilot/planning/minimum_snap_solver.hpp"
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
  /// Construct an empty mission, optionally initializing the starting time
  /// reference for future trajectory additions.
  explicit Mission(double init_time = 0.0) : now_secs_(init_time) {}

  /// Construct a mission with a given configuration, optionally initializing
  /// the starting time reference for future trajectory additions.
  Mission(std::shared_ptr<MissionCfg> cfg, double init_time = 0.0)
      : cfg_(std::move(cfg)), now_secs_(init_time) {}

  template <typename T>
    requires(std::derived_from<std::remove_cvref_t<T>, TrajectoryBase> &&
             !std::is_same_v<T, TrajectoryBase>)
  void splice(T&& traj) {
    // Surgical cut of the timeline to make room for the new intent
    pruneFuture(traj.startTime());

    using Derived = std::remove_cvref_t<T>;
    // Logic: Ensure the new segment starts exactly where the old one was cut
    // (Assuming your high-level helpers like lineTo have already
    // calculated start_state/start_time correctly).
    queue_.emplace_back(std::make_shared<Derived>(std::forward<T>(traj)));
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
  [[nodiscard]] std::vector<std::shared_ptr<TrajectoryBase>>
  getUpdatedTrajectory(const QuadrotorState& state, bool finished_hint);

  bool lineTo(const Eigen::Vector3d& pos, double duration,
              MinimumSnapSolver& solver, bool interrupt = false,
              const HeadingPolicy& heading_policy = FollowVelocity(),
              bool stop_at_end = false);

  void yawTo(double end_yaw, double duration);

  void yawBy(double relative_yaw, double duration);

  void pop() { queue_.pop_front(); }

  bool empty() const { return queue_.empty(); }

  std::shared_ptr<const TrajectoryBase> operator[](size_t index) const {
    return queue_[index];
  }

  std::shared_ptr<const TrajectoryBase> at(size_t index) const {
    return queue_.at(index);
  }

  size_t size() const { return queue_.size(); }

  double getTailTime() const;

  KinematicState getNowState() const;

 private:
  void pruneFuture(double time);

  KinematicState getTailState() const;

  bool advance(const QuadrotorState& state, const EquilibriumTolerances& tols);

  void updateFallback(const KinematicState& end_state, double time);

  std::shared_ptr<MissionCfg> cfg_ = std::make_shared<MissionCfg>();
  double now_secs_ = 0.0;
  std::deque<std::shared_ptr<TrajectoryBase>> queue_;
  Hover fallback_hover_{Eigen::Vector3d::Zero(), 0.0,
                        std::numeric_limits<double>::infinity(), 0.0};
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_MISSION_HPP_
