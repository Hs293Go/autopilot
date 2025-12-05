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
};

class EstimatorBase : public Module {
 public:
  EstimatorBase(const std::string& name, std::shared_ptr<QuadrotorModel> model,
                std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("Estimator.{}", name), std::move(model),
               std::move(logger)) {}

  // 1. Data Ingestion (The "Push")
  // We separate Prediction inputs (IMU/Control) from Corrections (GPS/Mag)
  // because they fundamentally drive the filter phases differently.
  virtual std::error_code processInput(const InputBase& input) = 0;

  virtual std::error_code processMeasurement(const MeasurementBase& meas) = 0;

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
