#ifndef AUTOPILOT_PLANNING_MISSION_HPP_
#define AUTOPILOT_PLANNING_MISSION_HPP_

#include <deque>

#include "autopilot/base/config_base.hpp"
#include "autopilot/core/math.hpp"
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

  Mission(const Mission& other) {
    queue_.clear();
    queue_.resize(0);
    for (auto const& traj : other.queue_) {
      queue_.push_back(traj ? traj->clone() : nullptr);
    }
  }

  friend void swap(Mission& self, Mission& other) noexcept {
    self.queue_.swap(other.queue_);
  }

  Mission& operator=(const Mission& other) {
    using std::swap;
    Mission temp(other);
    swap(*this, temp);
    return *this;
  }

  Mission(Mission&&) = default;
  Mission& operator=(Mission&&) = default;

  void append(std::unique_ptr<TrajectoryBase>&& traj) {
    queue_.push_back(std::move(traj));
  }

  template <std::derived_from<TrajectoryBase> T>
  void append(const T& traj)
    requires(!std::is_same_v<T, TrajectoryBase>)
  {
    // Note: This requires TrajectoryBase to have a proper copy constructor
    queue_.emplace_back(std::make_unique<T>(traj));
  }

  TrajectoryBase* current() const {
    return queue_.empty() ? nullptr : queue_.front().get();
  }

  void tryAdvance(const QuadrotorState& state) {
    if (queue_.empty()) {
      return;
    }

    // We only advance if the Trajectory says the drone is stable
    // AND the Sampler (externally) reported the time is up.
    auto status =
        queue_.front()->checkEquilibrium(state, cfg_->getTolerances());

    if (status.state != EquilibriumState::kNone) {
      queue_.pop_front();
    }
  }

  void pop() { queue_.pop_front(); }

  bool empty() const { return queue_.empty(); }

 private:
  std::shared_ptr<MissionCfg> cfg_;
  Eigen::Vector3d last_position_;
  std::deque<std::unique_ptr<TrajectoryBase>> queue_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_MISSION_HPP_
