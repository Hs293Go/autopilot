#include "autopilot/planning/mission.hpp"

namespace autopilot {
bool Mission::append(std::shared_ptr<TrajectoryBase> traj) {
  if (traj) {
    queue_.emplace_back(traj);
    return true;
  }
  return false;
}

std::vector<std::shared_ptr<TrajectoryBase>> Mission::getUpdatedTrajectory(
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

  // Cache time of query
  now_secs_ = state.timestamp_secs;

  return {queue_.begin(), queue_.end()};
}

bool Mission::lineTo(const Eigen::Vector3d& pos, double duration,
                     MinimumSnapSolver& solver, bool stop_at_end) {
  // 1. Resolve starting conditions
  double start_time = getTailTime();
  const KinematicState& start_state = getTailState();

  // 2. Build the trajectory using your solver
  const TrajectoryWaypoint wps[] = {
      {.time_from_start_secs = start_time,
       .position = start_state.position,
       .yaw = start_state.yaw},
      {.time_from_start_secs = start_time + duration,
       .position = pos,
       .yaw = start_state.yaw}};

  if (auto traj = solver.solve(wps)) {
    // Custom behavior: Polynomials usually don't gate unless we force them
    traj->setRequiresEquilibrium(stop_at_end);
    this->append(std::move(traj.value()));
    return true;
  }
  return false;
}

void Mission::yawTo(double end_yaw, double duration) {
  auto tail_state = getTailState();
  auto rotate = std::make_unique<HeadingChange>(
      tail_state.position, tail_state.yaw, end_yaw, getTailTime(), duration);
  append(std::move(rotate));
}

void Mission::yawBy(double relative_yaw, double duration) {
  auto tail_state = getTailState();
  auto end_yaw = wrapToPi(tail_state.yaw + relative_yaw);
  auto rotate = std::make_unique<HeadingChange>(
      tail_state.position, tail_state.yaw, end_yaw, getTailTime(), duration);
  append(std::move(rotate));
}

double Mission::getTailTime() const {
  return queue_.empty() ? now_secs_ : queue_.back()->endTime();
}

KinematicState Mission::getTailState() const {
  if (queue_.empty()) {
    // Sample from the fallback if no plan exists
    return fallback_hover_.sample(now_secs_);
  }
  return queue_.back()->sample(queue_.back()->endTime());
}

bool Mission::advance(const QuadrotorState& state,
                      const EquilibriumTolerances& tols) {
  const auto& curr = queue_.front();

  // Completion of spline/polynomial dynamic trajectories is usually not gated
  // on equilibrium, so they can always be popped when time is up.
  // Otherwise, we check if we have reached equilibrium before popping.
  bool can_pop =
      !curr->requiresEquilibrium() ||
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

void Mission::updateFallback(const KinematicState& end_state, double time) {
  // We capture the exact position and yaw where the last segment finished.
  // Note: Since end_state.velocity should be near-zero (due to equilibrium
  // gating), this creates a perfect C0/C1 handover.
  fallback_hover_ =
      Hover(end_state.position, time, std::numeric_limits<double>::infinity(),
            end_state.yaw);
}
}  // namespace autopilot
