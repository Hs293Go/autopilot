#ifndef AUTOPILOT_PLANNING_TRAJECTORY_HPP_
#define AUTOPILOT_PLANNING_TRAJECTORY_HPP_

#include <deque>
#include <memory>

#include "autopilot/core/definitions.hpp"

namespace autopilot {

enum class EquilibriumState {
  kNone,     // High tracking error / Not yet arrived
  kDynamic,  // Following a moving path within tolerance
  kStatic,   // Reached a stationary hover point and settled
  kSingular  // Specific force near zero (falling), math undefined
};

struct EquilibriumStatus {
  EquilibriumState state = EquilibriumState::kNone;

  // Allow limited feedback on the magnitude of errors
  double position_error_mag = 0.0;
  double velocity_error_mag = 0.0;
  double angle_error_mag = 0.0;
};

struct EquilibriumTolerances {
  double position_tol = 0.1;  // meters, also known as acceptance radius
  double velocity_tol = 0.1;  // meters/second
  double angle_tol = 0.1;     // radians
};

class TrajectoryBase {
 public:
  virtual ~TrajectoryBase() = default;

  virtual std::unique_ptr<TrajectoryBase> clone() const = 0;

  [[nodiscard]] virtual KinematicState sample(double timestamp) const = 0;

  [[nodiscard]] virtual double duration() const = 0;

  [[nodiscard]] virtual double startTime() const = 0;

  [[nodiscard]] virtual double endTime() const {
    return startTime() + duration();
  }

  [[nodiscard]] virtual bool checkExpiry(const QuadrotorState& state) const = 0;

  [[nodiscard]] virtual EquilibriumStatus checkEquilibrium(
      const QuadrotorState& state, const EquilibriumTolerances& tols) const = 0;
};

class Mission {
 public:
  Mission() = default;

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

  void pop() { queue_.pop_front(); }

  bool empty() const { return queue_.empty(); }

 private:
  std::deque<std::unique_ptr<TrajectoryBase>> queue_;
};

}  // namespace autopilot
#endif
