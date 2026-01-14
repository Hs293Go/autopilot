#ifndef AUTOPILOT_ESTIMATORS_EKFIMU_ESTIMATOR_HPP_
#define AUTOPILOT_ESTIMATORS_EKFIMU_ESTIMATOR_HPP_

#include "autopilot/base/estimator_base.hpp"

namespace autopilot {

class EkfEstimator final : public EstimatorBase {
 public:
  static constexpr char kName[] = "EkfEstimator";

  // State Block Definitions (Full 16-state vector)
  static constexpr BlockDef<0, 3> kPosition{};
  static constexpr auto kOrientation =
      NextBlock<4>(kPosition);  // 4D Quaternion
  static constexpr auto kVelocity = NextBlock<3>(kOrientation);
  static constexpr auto kAccelBias = NextBlock<3>(kVelocity);
  static constexpr auto kGyroBias = NextBlock<3>(kAccelBias);

  static constexpr auto kAccel = BlockDef<0, 3>{};
  static constexpr auto kGyro = NextBlock<3>(kAccel);
  static constexpr int kNumStates = 16;

  using StateMatrix = Eigen::Matrix<double, kNumStates, kNumStates>;

  struct Context : EstimatorContext {
    std::string_view coreName() const override { return kName; }
    bool isInitialized() const override { return initialized; }
    EstimatorCovarianceMatrix covariance() const override { return P; }

    StateMatrix P = StateMatrix::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();

    // For extrapolation and integration
    Eigen::Vector3d last_accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();
    bool initialized = false;
  };

  struct Config : ReflectiveConfigBase<Config> {
    // Noise Densities
    Eigen::Vector3d pos_proc_var = Eigen::Vector3d::Constant(1e-3);
    Eigen::Vector4d att_proc_var = Eigen::Vector4d::Constant(1);
    Eigen::Vector3d vel_proc_var = Eigen::Vector3d::Constant(1e-3);
    Eigen::Vector3d gyro_bias_proc_var = Eigen::Vector3d::Constant(1e-6);
    Eigen::Vector3d accel_bias_proc_var = Eigen::Vector3d::Constant(1e-6);
    double accel_noise_density = 0.1;
    double gyro_noise_density = 0.1;

    std::string_view name() const override { return kName; }

    static constexpr auto kDescriptors = std::make_tuple(
        Describe(
            "pos_proc_var", &Config::pos_proc_var,
            F64Properties{
                .desc = "Process noise variance for position states (m^2/s)",
                .bounds = Bounds<double>::Positive()}),
        Describe(
            "att_proc_var", &Config::att_proc_var,
            F64Properties{.desc = "Process noise variance for attitude states",
                          .bounds = Bounds<double>::Positive()}),
        Describe(
            "vel_proc_var", &Config::vel_proc_var,
            F64Properties{
                .desc = "Process noise variance for velocity states (m^2/s^3)",
                .bounds = Bounds<double>::Positive()}),
        Describe("gyro_bias_proc_var", &Config::gyro_bias_proc_var,
                 F64Properties{
                     .desc = "Process noise variance for gyroscope bias states",
                     .bounds = Bounds<double>::Positive()}),
        Describe(
            "accel_bias_proc_var", &Config::accel_bias_proc_var,
            F64Properties{
                .desc = "Process noise variance for accelerometer bias states",
                .bounds = Bounds<double>::Positive()}),
        Describe(
            "accel_noise_density", &Config::accel_noise_density,
            F64Properties{.desc = "Accelerometer noise density (m/s^2)/√Hz",
                          .bounds = Bounds<double>::Positive()}),
        Describe("gyro_noise_density", &Config::gyro_noise_density,
                 F64Properties{.desc = "Gyroscope noise density (rad/s)/√Hz",
                               .bounds = Bounds<double>::Positive()}));
  };

  EkfEstimator(std::shared_ptr<QuadrotorModel> model,
               std::shared_ptr<Config> config,
               std::shared_ptr<spdlog::logger> logger = nullptr);

  std::string_view name() const override { return kName; }
  std::unique_ptr<EstimatorContext> createContext() const override;

  // Interface Implementation
  std::error_code predict(
      QuadrotorState& state, EstimatorContext& context,
      const std::shared_ptr<const InputBase>& u) const override;

  std::error_code correct(
      QuadrotorState& state, EstimatorContext& context,
      const std::shared_ptr<const MeasurementBase>& z) const override;

  QuadrotorState extrapolate(const QuadrotorState& state,
                             const EstimatorContext& context,
                             double t) const override;

  std::error_code reset(
      EstimatorContext& context, const QuadrotorState& initial_state,
      const Eigen::Ref<const Eigen::MatrixXd>& initial_cov) const override;

  bool isHealthy(const EstimatorContext& context) const override;

  std::size_t stateDim() const override { return 16; }

 private:
  std::shared_ptr<Config> cfg_;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace autopilot

#endif
