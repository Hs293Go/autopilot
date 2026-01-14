#ifndef AUTOPILOT_ESTIMATORS_ESTIMATOR_DRIVER_BASE_HPP_
#define AUTOPILOT_ESTIMATORS_ESTIMATOR_DRIVER_BASE_HPP_

#include <memory>

#include "autopilot/base/estimator_base.hpp"

namespace autopilot {

class EstimatorDriverBase {
 public:
  explicit EstimatorDriverBase(std::shared_ptr<EstimatorBase>&& estimator)
      : estimator_(std::move(estimator)) {}

  /// Forwards the name from the underlying estimator
  std::string_view name() const { return estimator_->name(); }

  /// Forwards the logger from the underlying estimator
  std::shared_ptr<spdlog::logger> logger() const {
    return estimator_->logger();
  }

  virtual ~EstimatorDriverBase() = default;

  virtual void start() {}

  /** Pushes new data into the estimator (The "Push")
   *
   * This function is called whenever new sensor data or control inputs arrive.
   * The estimator should process this data and update its internal state
   * accordingly.
   *
   * @details A naive implementation might simply call processInput or
   * processMeasurement based on the type of data. More sophisticated
   * implementations might buffer data, handle out-of-order timestamps, or
   * perform other preprocessing steps.
   *
   * @param data A shared pointer to the EstimatorData object containing the
   *             new data.
   */
  virtual void push(const std::shared_ptr<const EstimatorData>& data) = 0;

  /** Processes control input data.
   *
   * This pure virtual function must be implemented by derived classes to
   * handle control input data.
   *
   * @param input A reference to the InputBase object containing the control
   *              input data.
   * @return An std::error_code indicating success or failure of the processing.
   */
  virtual std::error_code processInput(
      const std::shared_ptr<const InputBase>& input) = 0;

  /** Processes measurement data.
   *
   * This pure virtual function must be implemented by derived classes to
   * handle measurement data.
   *
   * @param meas A reference to the MeasurementBase object containing the
   *             measurement data.
   * @return An std::error_code indicating success or failure of the processing.
   */
  virtual std::error_code processMeasurement(
      const std::shared_ptr<const MeasurementBase>& meas) = 0;

  // 2. State Retrieval (The "Pull")
  // "Give me the state at time 't'".
  // If t=0, return latest.
  // If t > latest, EXTRAPOLATE (Predict forward).
  // If t < latest, INTERPOLATE (delayed query).
  virtual std::expected<QuadrotorState, std::error_code> getStateAt(
      double timestamp_s = 0.0) const = 0;

  virtual EstimatorCovarianceMatrix getCovariance() const = 0;

  // 3. Lifecycle
  // Reset filter to a known initial state (e.g., waiting on tarmac)
  virtual std::error_code reset(
      const QuadrotorState& initial_state,
      const Eigen::Ref<const Eigen::MatrixXd>& initial_cov) = 0;

  virtual std::error_code resetState(const QuadrotorState& new_state) {
    const auto size = static_cast<Eigen::Index>(estimator_->tangentDim());
    return reset(new_state,
                 EstimatorCovarianceMatrix::Identity(size, size));
  }

  // Check if the filter has converged enough to fly
  [[nodiscard]] virtual bool isHealthy() const = 0;

  virtual void wait() {}

 protected:
  std::shared_ptr<EstimatorBase> estimator_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_ESTIMATORS_ESTIMATOR_DRIVER_BASE_HPP_
