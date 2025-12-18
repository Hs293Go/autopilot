#ifndef AUTOPILOT_ESTIMATOR_BASE_HPP_
#define AUTOPILOT_ESTIMATOR_BASE_HPP_

#include <expected>

#include "autopilot/base/factory.hpp"
#include "autopilot/base/module.hpp"
#include "autopilot/core/definitions.hpp"

namespace autopilot {

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
//
// Opaque handle for algorithm-specific state (Covariances, Biases, Buffers)
struct EstimatorContext {
  virtual ~EstimatorContext() = default;

  virtual std::string_view coreName() const = 0;

  virtual bool isInitialized() const = 0;

  virtual Eigen::Ref<const Eigen::MatrixXd> covariance() const = 0;
};

class EstimatorBase : public Module {
 public:
  static constexpr std::string_view kModuleRootType = "EstimatorData";

  EstimatorBase(const std::string& name, std::shared_ptr<QuadrotorModel> model,
                std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("Estimator.{}", name), std::move(model),
               std::move(logger)) {}

  // Factory: Allocates the algorithm-specific memory
  virtual std::unique_ptr<EstimatorContext> createContext() const = 0;

  // 1. Math Operations (Stateless Logic)
  // Input: Last State, Context, Data
  // Output: New State, Updated Context
  virtual std::error_code predict(
      QuadrotorState& state, EstimatorContext& context,
      const std::shared_ptr<const InputBase>& input) const = 0;

  virtual std::error_code correct(
      QuadrotorState& state, EstimatorContext& context,
      const std::shared_ptr<const MeasurementBase>& meas) const = 0;

  // 2. Query Logic (Stateless Extrapolation)
  virtual QuadrotorState extrapolate(const QuadrotorState& state,
                                     const EstimatorContext& context,
                                     double time) const = 0;

  // 3. Lifecycle
  virtual std::error_code reset(
      EstimatorContext& context, const QuadrotorState& initial_state,
      const Eigen::Ref<const Eigen::MatrixXd>& initial_cov) const = 0;

  virtual bool isHealthy(const EstimatorContext& context) const = 0;
};

using EstimatorFactory = GenericFactory<EstimatorBase>;

}  // namespace autopilot

#define REGISTER_ESTIMATOR(ConcreteType, KeyName)                              \
  static const autopilot::Registrar<ConcreteType, autopilot::EstimatorFactory> \
      kRegistrarFor##ConcreteType(KeyName)

#endif /* end of include guard: AUTOPILOT_ESTIMATOR_BASE_HPP_ */
