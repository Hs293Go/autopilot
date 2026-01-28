#include "autopilot/planning/mission.hpp"

namespace autopilot {

std::vector<std::shared_ptr<TrajectoryBase>> Mission::getUpdatedTrajectory(
    const QuadrotorState& state, bool finished_hint) {
  // Cache time of query
  now_secs_ = state.timestamp_secs;

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

bool Mission::lineTo(const Eigen::Vector3d& pos, double duration,
                     MinimumSnapSolver& solver, bool interrupt,
                     const HeadingPolicy& heading_policy, bool stop_at_end) {
  // 1. Resolve starting conditions
  const auto start_time = interrupt ? now_secs_ : getTailTime();
  const auto& start_state = interrupt ? getNowState() : getTailState();

  // 2. Build the trajectory using your solver
  const TrajectoryWaypoint wps[] = {
      {.time_from_start_secs = start_time,
       .position = start_state.position,
       .yaw = start_state.yaw},
      {.time_from_start_secs = start_time + duration,
       .position = pos,
       .yaw = start_state.yaw}};

  if (auto traj = solver.solve(wps, heading_policy)) {
    // Custom behavior: Polynomials usually don't gate unless we force them
    auto tail_state = traj->sample(traj->endTime());
    splice(std::move(traj.value()));
    if (stop_at_end) {
      queue_.emplace_back(std::make_shared<Hover>(
          tail_state.position, tail_state.timestamp_secs,
          std::numeric_limits<double>::infinity(), tail_state.yaw));
    }
    return true;
  }
  return false;
}

void Mission::yawTo(double end_yaw, double duration) {
  auto tail_state = getTailState();
  splice(HeadingChange(tail_state.position, tail_state.yaw, end_yaw,
                       getTailTime(), duration));
}

void Mission::yawBy(double relative_yaw, double duration) {
  auto tail_state = getTailState();
  auto end_yaw = wrapToPi(tail_state.yaw + relative_yaw);
  splice(HeadingChange(tail_state.position, tail_state.yaw, end_yaw,
                       getTailTime(), duration));
}

void Mission::pruneFuture(double time) {
  if (queue_.empty()) {
    return;
  }

  // 1. Remove all segments that start AFTER the target time
  while (!queue_.empty() && queue_.back()->startTime() > time) {
    queue_.pop_back();
  }

  // 2. If the now-last segment overlaps the target time, truncate it
  if (!queue_.empty() && queue_.back()->endTime() > time) {
    queue_.back()->truncate(time);
  }
}

double Mission::getTailTime() const {
  if (queue_.empty()) {
    return now_secs_;
  }
  const auto& tail = *queue_.back();
  if (std::isinf(tail.endTime())) {
    // If the last segment is infinite, we consider the tail time to be now.
    return tail.startTime();
  }
  return tail.endTime();
}

KinematicState Mission::getTailState() const {
  if (queue_.empty()) {
    // Sample from the fallback if no plan exists
    return fallback_hover_.sample(now_secs_);
  }
  return queue_.back()->sample(queue_.back()->endTime());
}

KinematicState Mission::getNowState() const {
  if (queue_.empty()) {
    return fallback_hover_.sample(now_secs_);
  }
  return queue_.front()->sample(now_secs_);
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

  double logical_end = curr->endTime();
  double actual_now = state.timestamp_secs;
  if (actual_now > logical_end && std::isfinite(logical_end)) {
    const double delay = actual_now - logical_end;

    queue_.pop_front();

    // SHIFT the rest of the timeline so the next segment starts NOW
    // (actual_now) instead of in the past (logical_end).
    for (auto& traj : queue_) {
      traj->shiftStartTime(delay);
    }
  } else {
    queue_.pop_front();
  }

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
