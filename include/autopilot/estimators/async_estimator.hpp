#ifndef INCLUDE_AUTOPILOT_ASYNC_ESTIMATOR_BASE_HPP_
#define INCLUDE_AUTOPILOT_ASYNC_ESTIMATOR_BASE_HPP_

#include <condition_variable>
#include <memory>
#include <queue>
#include <shared_mutex>
#include <thread>

#include "autopilot/base/estimator_base.hpp"

namespace autopilot {
// A type-erased container for the priority queue
struct QueuedPacket {
  double timestamp;
  // We use shared_ptr to avoid slicing and copying
  std::shared_ptr<const EstimatorData> data;

  // For priority_queue: smallest timestamp (oldest data) is on top
  bool operator>(const QueuedPacket& other) const {
    return timestamp > other.timestamp;
  }
};

class AsyncEstimator : public EstimatorBase {
 public:
  AsyncEstimator(const std::string& name, std::shared_ptr<QuadrotorModel> model,
                 std::shared_ptr<spdlog::logger> = nullptr);

  void start() override;

  // Blocks until the queue is empty AND the worker is idle
  void wait() override;

  ~AsyncEstimator() override;

  // ---------------------------------------------------------------------------
  // 1. Ingestion (Runs on Sensor Threads)
  // ---------------------------------------------------------------------------
  void push(const std::shared_ptr<const EstimatorData>& data) override;

  // ---------------------------------------------------------------------------
  // 2. Query (Runs on Control Thread)
  // ---------------------------------------------------------------------------
  std::expected<QuadrotorState, std::error_code> getStateAt(
      double timestamp) const override;

 protected:
  // Derived classes implement the specific math here
  // virtual void processInput(const InputBase& u) = 0;
  // virtual void processMeasurement(const MeasurementBase& z) = 0;

  // Lightweight kinematic prediction for the query path
  virtual QuadrotorState extrapolateState(const QuadrotorState& state,
                                          double t) const = 0;

  // Direct access for derived classes to update the "Official" state
  void updateCommittedState(const QuadrotorState& new_state) {
    std::unique_lock lock(state_mutex_);  // Writer lock
    committed_state_ = new_state;
  }

 private:
  // ---------------------------------------------------------------------------
  // 3. Maintenance (Runs on Worker Thread)
  // ---------------------------------------------------------------------------
  void workerLoop();

  // Threading Primitives
  std::atomic<bool> running_;
  std::jthread worker_;

  std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::priority_queue<QueuedPacket, std::vector<QueuedPacket>, std::greater<>>
      queue_;

  std::condition_variable drain_cv_;
  bool busy_ = false;

  mutable std::shared_mutex state_mutex_;  // Allows multiple readers
                                           // (controllers), one writer (worker)
  QuadrotorState committed_state_;
};
}  // namespace autopilot

#endif  // INCLUDE_AUTOPILOT_ASYNC_ESTIMATOR_BASE_HPP_
