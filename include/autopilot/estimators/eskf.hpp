#ifndef AUTOPILOT_ESKF_HPP_
#define AUTOPILOT_ESKF_HPP_

#include "autopilot/estimators/async_estimator.hpp"
#include "autopilot/simulator/sensors.hpp"

namespace autopilot {

// Key intrinsic dimensions related to state and data layouts
constexpr auto kPositionError = BlockDef<0, 3>{};
constexpr auto kRotationError = NextBlock<3>(kPositionError);
constexpr auto kVelocityError = NextBlock<3>(kRotationError);
constexpr auto kAccelBiasError = NextBlock<3>(kVelocityError);
constexpr auto kGyroBiasError = NextBlock<3>(kAccelBiasError);

constexpr auto kPoseError = BlockDef<0, 6>{};
constexpr auto kTwistError = NextBlock<6>(kPoseError);

constexpr int kNumErrorStates =
    SumSizes(kPositionError, kRotationError, kVelocityError, kAccelBiasError,
             kGyroBiasError);

using ErrorState = Eigen::Vector<double, kNumErrorStates>;
using ErrorCov = Eigen::Matrix<double, kNumErrorStates, kNumErrorStates>;

class ErrorStateKalmanFilter : public AsyncEstimator {
 public:
  static constexpr char kName[] = "ErrorStateKalmanFilter";

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

    std::string_view name() const override { return "ErrorStateKalmanFilterConfig"; }

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

  ErrorStateKalmanFilter(std::shared_ptr<QuadrotorModel> model,
                         std::shared_ptr<Config> config,
                         std::shared_ptr<spdlog::logger> logger = nullptr);

  // Lifecycle
  std::error_code reset(
      const QuadrotorState& initial_state,
      const Eigen::Ref<const Eigen::MatrixXd>& initial_cov) override;

  bool isHealthy() const override;

  Eigen::Ref<const Eigen::MatrixXd> getCovariance() const override {
    return P_;
  }

 protected:
  // Core Async Logic (Worker Thread)
  std::error_code processInput(
      const std::shared_ptr<const InputBase>& u) override;
  std::error_code processMeasurement(
      const std::shared_ptr<const MeasurementBase>& z) override;

  // Latency Compensation (Control Thread)
  QuadrotorState extrapolateState(const QuadrotorState& state,
                                  double t) const override;

 private:
  template <int N>
  struct InnovStats {
    double mahalanobis_distance;
    Eigen::LLT<Eigen::Matrix<double, N, N>> llt_fac;
  };

  template <int N>
  InnovStats<N> computeMahalanobisDistance(
      const Eigen::Vector<double, N>& innovation,
      const Eigen::Matrix<double, N, N>& innov_cov);

  void reportObservationStats(
      const Eigen::Ref<const Eigen::MatrixXd>& innov_cov,
      const Eigen::Ref<const Eigen::VectorXd>& expectation,
      const Eigen::Ref<const Eigen::VectorXd>& observation);

  template <typename Fn>
  bool checkOutlier(double m2_dist, const OutlierClassifier& classifier,
                    Fn reporter);

  // Internal Prediction Implementations
  // Returns true if state was updated, false if skipped (e.g. dt=0)
  std::error_code predictKinematics(QuadrotorState& state,
                                    const Eigen::Vector3d& accel,
                                    const Eigen::Vector3d& gyro,
                                    double dt) const;

  std::error_code predictCovariance(const Eigen::Vector3d& accel,
                                    const Eigen::Vector3d& gyro, double dt);

  // Correction Implementations
  // Returns success/failure code
  std::error_code correctGps(const std::shared_ptr<const class GpsData>& z);
  std::error_code correctMag(const std::shared_ptr<const class MagData>& z);

  // Helpers
  void injectError(const ErrorState& dx);
  void resetError(const ErrorState& dx);

  std::error_code setError(AutopilotErrc ec);

  // Configuration & State
  std::shared_ptr<Config> config_;
  QuadrotorState nominal_state_;
  ErrorCov P_ = ErrorCov::Identity();
  std::atomic_bool initialized_ = false;
  std::atomic_bool has_previous_error_ = false;

  mutable std::mutex extrapolation_mutex_;
  // Internal Biases (Not in QuadrotorState)
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();

  // Cache for extrapolation
  Eigen::Vector3d last_accel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();

  OutlierClassifier gps_outlier_classifier_;
  OutlierClassifier mag_outlier_classifier_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_ESKF_HPP_
