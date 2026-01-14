#ifndef AUTOPILOT_ESTIMATORS_ASYNC_ESTIMATOR_HPP_
#define AUTOPILOT_ESTIMATORS_ASYNC_ESTIMATOR_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <thread>

#include "autopilot/estimators/estimator_driver_base.hpp"

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

class AsyncEstimator final : public EstimatorDriverBase {
 public:
  AsyncEstimator(std::shared_ptr<EstimatorBase> estimator);

  void start() override;

  // Blocks until the queue is empty AND the worker is idle
  void wait() override;

  ~AsyncEstimator() override;

  // ---------------------------------------------------------------------------
  // 1. Ingestion (Runs on Sensor Threads)
  // ---------------------------------------------------------------------------
  void push(const std::shared_ptr<const EstimatorData>& data) override;

  // 2. Processing (Worker Thread)
  std::error_code processInput(
      const std::shared_ptr<const InputBase>& input) override;

  std::error_code processMeasurement(
      const std::shared_ptr<const MeasurementBase>& meas) override;

  // 3. Query (Control Thread)
  std::expected<QuadrotorState, std::error_code> getStateAt(
      double timestamp_s = 0.0) const override;

  EstimatorCovarianceMatrix getCovariance() const override;

  // 4. Lifecycle
  std::error_code reset(
      const QuadrotorState& initial_state,
      const Eigen::Ref<const Eigen::MatrixXd>& initial_cov) override;

  [[nodiscard]] bool isHealthy() const override;

 private:
  // ---------------------------------------------------------------------------
  // 3. Maintenance (Runs on Worker Thread)
  // ---------------------------------------------------------------------------
  void workerLoop();
  void updateCommittedState(const QuadrotorState& new_state);

  // Helper for getStateAt
  QuadrotorState extrapolateState(const QuadrotorState& state, double t) const;

  // --- State Management ---
  // The Driver OWNS the memory for the algorithm
  std::shared_ptr<EstimatorContext> context_;
  // Protects 'context_' from concurrent access (Worker Write vs Control Read)
  mutable std::mutex context_mutex_;

  // Worker-local state (no lock needed inside worker)
  QuadrotorState nominal_state_;

  // Committed state (Shared between Worker Write and Control Read)
  mutable std::shared_mutex state_mutex_;
  QuadrotorState committed_state_;

  // Threading Primitives
  std::atomic_bool running_ = false;
  std::jthread worker_;

  std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::priority_queue<QueuedPacket, std::vector<QueuedPacket>, std::greater<>>
      queue_;

  std::condition_variable drain_cv_;
  bool busy_ = false;
};
}  // namespace autopilot

#endif  // AUTOPILOT_ESTIMATORS_ASYNC_ESTIMATOR_HPP_
