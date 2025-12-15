#include "autopilot/estimators/async_estimator.hpp"

#include <shared_mutex>
#include <utility>

namespace autopilot {

AsyncEstimator::AsyncEstimator(const std::string& name,
                               std::shared_ptr<QuadrotorModel> model,
                               std::shared_ptr<spdlog::logger> logger)
    : EstimatorBase(fmt::format("AsyncEstimator.{}", name), std::move(model),
                    std::move(logger)) {}

AsyncEstimator::~AsyncEstimator() {
  running_ = false;
  cv_.notify_all();
}

void AsyncEstimator::start() {
  running_ = true;
  worker_ = std::jthread(&AsyncEstimator::workerLoop, this);
}

void AsyncEstimator::wait() {
  std::unique_lock lock(queue_mutex_);
  // Wait until the queue is empty AND the worker has finished the last job
  drain_cv_.wait(lock, [this] { return queue_.empty() && !busy_; });
}

void AsyncEstimator::push(const std::shared_ptr<const EstimatorData>& data) {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    // Wrap in packet for sorting
    queue_.emplace(data->timestamp_secs(), data);
  }
  cv_.notify_one();  // Wake up the worker
}

expected<QuadrotorState, std::error_code> AsyncEstimator::getStateAt(
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
    switch (packet.data->type()) {
      case EstimatorData::Type::kInput:
        if (auto ec = processInput(
                std::static_pointer_cast<const InputBase>(packet.data))) {
          logger()->error("AsyncEstimator processInput failed: {}",
                          ec.message());
        }
        break;
      case EstimatorData::Type::kMeasurement:
        if (auto ec = processMeasurement(
                std::static_pointer_cast<const MeasurementBase>(packet.data))) {
          logger()->error("AsyncEstimator processMeasurement failed: {}",
                          ec.message());
        }
        break;
      default:
        logger()->error("AsyncEstimator received unknown data type");
        break;
    }

    lock.lock();
    busy_ = false;
    if (queue_.empty()) {
      drain_cv_.notify_all();
    }
  }
}

}  // namespace autopilot
