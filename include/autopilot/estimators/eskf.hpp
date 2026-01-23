#ifndef AUTOPILOT_ESKF_HPP_
#define AUTOPILOT_ESKF_HPP_

#include "autopilot/base/estimator_base.hpp"
#include "autopilot/simulator/sensors.hpp"

namespace autopilot {

class EskfEstimator final : public EstimatorBase {
 public:
  static constexpr char kName[] = "EskfEstimator";

  // Key intrinsic dimensions related to state and data layouts
  static constexpr auto kPositionError = BlockDef<0, 3>{};
  static constexpr auto kRotationError = NextBlock<3>(kPositionError);
  static constexpr auto kVelocityError = NextBlock<3>(kRotationError);
  static constexpr auto kAccelBiasError = NextBlock<3>(kVelocityError);
  static constexpr auto kGyroBiasError = NextBlock<3>(kAccelBiasError);

  static constexpr int kNumErrorStates =
      SumSizes(kPositionError, kRotationError, kVelocityError, kAccelBiasError,
               kGyroBiasError);

  using ErrorState = Eigen::Vector<double, kNumErrorStates>;
  using ErrorCov = Eigen::Matrix<double, kNumErrorStates, kNumErrorStates>;

  struct Context : EstimatorContext {
    std::string_view coreName() const override { return kName; }

    EstimatorCovarianceMatrix covariance() const override { return P; }

    bool isInitialized() const override { return is_initialized; }

    ErrorCov P = ErrorCov::Identity();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();

    // Cache for extrapolation
    Eigen::Vector3d last_accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();

    bool is_initialized = false;
  };

  struct Config final : ReflectiveConfigBase<Config> {
    // Process Noise (Continuous Time)
    double accel_noise_density = 0.1;
    double gyro_noise_density = 0.1;
    double accel_bias_random_walk = 0.01;
    double gyro_bias_random_walk = 0.01;
    double gps_confidence_level_warning = 0.97;
    double gps_confidence_level_error = 0.95;  // 3-sigma
    double mag_confidence_level_warning = 0.97;
    double mag_confidence_level_error = 0.95;  // 3-sigma
    double max_sum_error_variance = 1e6;

    std::string_view name() const override { return kName; }

    static constexpr auto kDescriptors = std::make_tuple(
        Describe(
            "accel_noise_density", &Config::accel_noise_density,
            F64Properties{.desc = "Accelerometer noise density (m/s^2)/√Hz",
                          .bounds = Bounds<double>::GreaterThan(0.0)}),
        Describe("gyro_noise_density", &Config::gyro_noise_density,
                 F64Properties{.desc = "Gyroscope noise density (rad/s)/√Hz",
                               .bounds = Bounds<double>::GreaterThan(0.0)}),
        Describe(
            "accel_bias_random_walk", &Config::accel_bias_random_walk,
            F64Properties{.desc = "Accelerometer bias random walk (m/s^2)/√Hz",
                          .bounds = Bounds<double>::GreaterThan(0.0)}),
        Describe("gyro_bias_random_walk", &Config::gyro_bias_random_walk,
                 F64Properties{.desc = "Gyroscope bias random walk (rad/s)/√Hz",
                               .bounds = Bounds<double>::GreaterThan(0.0)}),
        Describe(
            "gps_confidence_level_warning",
            &Config::gps_confidence_level_warning,
            F64Properties{.desc = "GPS confidence level for warning threshold",
                          .bounds = Bounds<double>::ClosedInterval(0.5, 1.0)}),
        Describe(
            "gps_confidence_level_error", &Config::gps_confidence_level_error,
            F64Properties{.desc = "GPS confidence level for error threshold",
                          .bounds = Bounds<double>::ClosedInterval(0.5, 1.0)}),
        Describe(
            "mag_confidence_level_warning",
            &Config::mag_confidence_level_warning,
            F64Properties{
                .desc = "Magnetometer confidence level for warning threshold",
                .bounds = Bounds<double>::ClosedInterval(0.5, 1.0)}),
        Describe(
            "mag_confidence_level_error", &Config::mag_confidence_level_error,
            F64Properties{
                .desc = "Magnetometer confidence level for error threshold",
                .bounds = Bounds<double>::ClosedInterval(0.5, 1.0)}),
        Describe(
            "max_sum_error_variance", &Config::max_sum_error_variance,
            F64Properties{.desc = "Maximum allowable sum of error variances",
                          .bounds = Bounds<double>::GreaterThan(0.0)}));
  };

  EskfEstimator(std::shared_ptr<QuadrotorModel> model,
                std::shared_ptr<Config> config,
                std::shared_ptr<spdlog::logger> logger = nullptr);

  std::string_view name() const override { return kName; }

  std::unique_ptr<EstimatorContext> createContext() const override;
  // Lifecycle
  AutopilotErrc reset(
      EstimatorContext& context, const QuadrotorState& initial_state,
      const Eigen::Ref<const Eigen::MatrixXd>& initial_cov) const override;

  bool isHealthy(const EstimatorContext& context) const override;

  // Core Async Logic (Worker Thread)
  AutopilotErrc predict(
      QuadrotorState& state, EstimatorContext& context,
      const std::shared_ptr<const InputBase>& u) const override;

  AutopilotErrc correct(
      QuadrotorState& state, EstimatorContext& context,
      const std::shared_ptr<const MeasurementBase>& z) const override;

  // Latency Compensation (Control Thread)
  QuadrotorState extrapolate(const QuadrotorState& state,
                             const EstimatorContext& context,
                             double t) const override;

  std::size_t stateDim() const override { return 16; }
  std::size_t tangentDim() const override { return kNumErrorStates; }

 private:
  template <int N>
  struct InnovStats {
    double mahalanobis_distance;
    Eigen::LLT<Eigen::Matrix<double, N, N>> llt_fac;
  };

  template <int N>
  InnovStats<N> computeMahalanobisDistance(
      const Eigen::Vector<double, N>& innovation,
      const Eigen::Matrix<double, N, N>& innov_cov) const;

  void reportObservationStats(
      const Eigen::Ref<const Eigen::MatrixXd>& innov_cov,
      const Eigen::Ref<const Eigen::VectorXd>& expectation,
      const Eigen::Ref<const Eigen::VectorXd>& observation) const;

  template <typename Fn>
  bool checkOutlier(double m2_dist, const OutlierClassifier& classifier,
                    Fn reporter) const;

  // Internal Prediction Implementations
  // Returns true if state was updated, false if skipped (e.g. dt=0)
  AutopilotErrc predictKinematics(QuadrotorState& state, const Context& context,
                                  const Eigen::Vector3d& accel,
                                  const Eigen::Vector3d& gyro, double dt) const;

  AutopilotErrc predictCovariance(const QuadrotorState& state, Context& context,
                                  const Eigen::Vector3d& accel,
                                  const Eigen::Vector3d& gyro, double dt) const;

  // Correction Implementations
  // Returns success/failure code
  AutopilotErrc correctGps(
      QuadrotorState& state, Context& context,
      const std::shared_ptr<const class LocalPositionData>& z) const;
  AutopilotErrc correctMag(QuadrotorState& state, Context& context,
                           const std::shared_ptr<const class MagData>& z) const;

  // Helpers
  void injectError(QuadrotorState& state, Context& context,
                   const ErrorState& dx) const;
  void resetCovariance(Context& context, const ErrorState& dx) const;

  AutopilotErrc setError(AutopilotErrc ec) const;

  // Configuration & State
  std::shared_ptr<Config> config_;

  mutable std::atomic_bool has_previous_error_ = false;

  OutlierClassifier gps_outlier_classifier_;
  OutlierClassifier mag_outlier_classifier_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_ESKF_HPP_
