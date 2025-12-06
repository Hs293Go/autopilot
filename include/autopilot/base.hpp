#ifndef AUTOPILOT_BASE_HPP_
#define AUTOPILOT_BASE_HPP_

#include <memory>
#include <span>

#include "autopilot/definitions.hpp"
#include "autopilot/expected.hpp"
#include "autopilot/module.hpp"
#include "autopilot/quadrotor_model.hpp"

namespace autopilot {

class ControllerBase : public Module {
 public:
  ControllerBase(const std::string& name, std::shared_ptr<QuadrotorModel> model,
                 std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("Controller.{}", name), std::move(model),
               std::move(logger)) {}

  virtual expected<std::size_t, std::error_code> compute(
      const QuadrotorState& state, std::span<const QuadrotorCommand> setpoints,
      std::span<QuadrotorCommand> outputs) = 0;

  virtual void reset() = 0;
};

struct EstimatorData {
  enum class Type {
    kInput,
    kMeasurement,
  };
  virtual ~EstimatorData() = default;

  // 1. Identification & Timing
  [[nodiscard]] virtual double timestamp_secs() const = 0;
  [[nodiscard]] virtual std::string sourceId() const = 0;  // e.g., "gps_1"
  [[nodiscard]] virtual std::string frameId() const = 0;   // e.g., "base_link"

  // 2. Lifecycle
  // Allows the estimator to buffer this data locally
  [[nodiscard]] virtual std::unique_ptr<EstimatorData> clone() const = 0;

  // 3. Dimensionality (For logging/allocators)
  // Storage dimension
  [[nodiscard]] virtual std::size_t dim() const = 0;

  // Minimal/Tangent dimension
  [[nodiscard]] virtual std::size_t tangentDim() const { return dim(); };

  // 4. Validity Check
  [[nodiscard]] virtual bool isValid() const { return true; }

  [[nodiscard]] virtual Type type() const = 0;
};

class InputBase : public EstimatorData {
 public:
  Type type() const final { return Type::kInput; }
};

class MeasurementBase : public EstimatorData {
 public:
  Type type() const final { return Type::kMeasurement; }

  /** Returns the measurement covariance matrix.
   *
   * This pure virtual function must be implemented by derived classes to
   * provide a view of a concrete covariance matrix associated with the
   * measurement.
   *
   * @return An Eigen::Ref to a constant Eigen::MatrixXd representing the
   *         measurement covariance.
   */
  virtual Eigen::Ref<const Eigen::MatrixXd> covariance() const = 0;
};

class EstimatorBase : public Module {
 public:
  EstimatorBase(const std::string& name, std::shared_ptr<QuadrotorModel> model,
                std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("Estimator.{}", name), std::move(model),
               std::move(logger)) {}

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
  virtual expected<QuadrotorState, std::error_code> getStateAt(
      double timestamp_s = 0.0) const = 0;

  // 3. Lifecycle
  // Reset filter to a known initial state (e.g., waiting on tarmac)
  virtual void reset(const QuadrotorState& initial_state) = 0;

  // Check if the filter has converged enough to fly
  [[nodiscard]] virtual bool isHealthy() const = 0;
};
}  // namespace autopilot

#endif  // AUTOPILOT_BASE_HPP_
