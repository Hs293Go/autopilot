#include "autopilot/estimators/async_estimator.hpp"

#include <utility>

namespace autopilot {

AsyncEstimator::AsyncEstimator(std::shared_ptr<EstimatorBase> estimator)
    : EstimatorDriverBase(std::move(estimator)) {
  // 1. Factory: Allocate the opaque memory context for the specific algorithm
  context_ = estimator_->createContext();
}

AsyncEstimator::~AsyncEstimator() {
  running_ = false;
  cv_.notify_all();
}

void AsyncEstimator::start() {
  if (!running_) {
    running_ = true;
    worker_ = std::jthread(&AsyncEstimator::workerLoop, this);
  }
}

void AsyncEstimator::wait() {
  std::unique_lock lock(queue_mutex_);
  // Wait until the queue is empty AND the worker has finished the last job
  drain_cv_.wait(lock, [this] { return queue_.empty() && !busy_; });
}

void AsyncEstimator::push(const std::shared_ptr<const EstimatorData>& data) {
  {
    std::scoped_lock lock(queue_mutex_);
    // Wrap in packet for sorting
    queue_.emplace(data->timestamp_secs(), data);
  }
  cv_.notify_one();  // Wake up the worker
}

std::expected<QuadrotorState, AutopilotErrc> AsyncEstimator::getStateAt(
    double timestamp) const {
  // A. Read the latest committed belief state (Thread-Safe Copy)
  QuadrotorState belief_snapshot;
  {
    std::shared_lock lock(state_mutex_);  // Reader lock (C++17)
    belief_snapshot = committed_state_;
  }

  // B. Extrapolate (Lightweight Logic)
  // If the filter is lagging (belief time < requested time), predict
  // forward. This happens entirely on the caller's thread budget.
  if (timestamp > belief_snapshot.timestamp_secs) {
    return extrapolateState(belief_snapshot, timestamp);
  }
  return belief_snapshot;
}

AutopilotErrc AsyncEstimator::processInput(
    const std::shared_ptr<const InputBase>& input) {
  // Lock Context (Write Access)
  std::scoped_lock lock(context_mutex_);

  // Delegate Math to Core
  // Note: nominal_state_ is updated IN PLACE by the core if successful?
  // Your interface defined predict(state&, ...) -> error_code.
  // This implies the Core modifies the state reference passed to it.

  if (auto ec = estimator_->predict(nominal_state_, *context_, input);
      ec != AutopilotErrc::kNone) {
    return ec;
  }

  // Publish
  updateCommittedState(nominal_state_);
  return {};
}

AutopilotErrc AsyncEstimator::processMeasurement(
    const std::shared_ptr<const MeasurementBase>& meas) {
  // Lock Context (Write Access)
  std::lock_guard<std::mutex> lock(context_mutex_);

  if (auto ec = estimator_->correct(nominal_state_, *context_, meas);
      ec != AutopilotErrc::kNone) {
    return ec;
  }

  // Publish
  updateCommittedState(nominal_state_);
  return {};
}

void AsyncEstimator::workerLoop() {
  while (running_) {
    std::unique_lock lock(queue_mutex_);
    cv_.wait(lock, [this] { return !queue_.empty() || !running_; });

    if (!running_) {
      break;
    }

    // Extract the oldest packet
    busy_ = true;
    const auto packet = queue_.top();
    queue_.pop();
    lock.unlock();  // Release lock while processing math

    // Dispatch based on type (Input vs Measurement)
    // We use dynamic_cast because we erased the type in the queue
    AutopilotErrc ec;
    switch (packet.data->type()) {
      case EstimatorData::Type::kInput:
        ec = processInput(
            std::static_pointer_cast<const InputBase>(packet.data));
        break;
      case EstimatorData::Type::kMeasurement:
        ec = processMeasurement(
            std::static_pointer_cast<const MeasurementBase>(packet.data));
        break;
    }

    if (ec != AutopilotErrc::kNone) {
      logger()->error("AsyncEstimator error: {}", ec);
    }

    // Cleanup
    lock.lock();
    busy_ = false;
    if (queue_.empty()) {
      drain_cv_.notify_all();
    }
  }
}

void AsyncEstimator::updateCommittedState(const QuadrotorState& new_state) {
  std::unique_lock lock(state_mutex_);
  committed_state_ = new_state;
}

QuadrotorState AsyncEstimator::extrapolateState(const QuadrotorState& state,
                                                double t) const {
  // Lock Context (Read Access to Biases/Cache inside context)
  std::lock_guard<std::mutex> lock(context_mutex_);
  return estimator_->extrapolate(state, *context_, t);
}

EstimatorCovarianceMatrix AsyncEstimator::getCovariance() const {
  std::lock_guard<std::mutex> lock(context_mutex_);
  return context_->covariance();
}

AutopilotErrc AsyncEstimator::reset(
    const QuadrotorState& initial_state,
    const Eigen::Ref<const Eigen::MatrixXd>& initial_cov) {
  // Reset all states
  {
    std::unique_lock state_lock(state_mutex_);
    committed_state_ = initial_state;
  }

  {
    std::lock_guard<std::mutex> ctx_lock(context_mutex_);
    nominal_state_ = initial_state;
    // Core resets the context (P, biases, initialization flags)
    return estimator_->reset(*context_, initial_state, initial_cov);
  }
}

bool AsyncEstimator::isHealthy() const {
  std::lock_guard<std::mutex> lock(context_mutex_);
  return estimator_->isHealthy(*context_);
}

}  // namespace autopilot
