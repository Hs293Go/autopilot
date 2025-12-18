#ifndef AUTOPILOT_SENSORS_HPP_
#define AUTOPILOT_SENSORS_HPP_

#include <random>

#include "Eigen/Dense"
#include "autopilot/base/config_base.hpp"
#include "autopilot/core/common.hpp"
#include "autopilot/core/math.hpp"

namespace autopilot {

enum class OutlierClassification { kNormal, kWarning, kError };

class OutlierClassifier {
 public:
  OutlierClassifier(double warning_threshold, double error_threshold)
      : warning_threshold_(warning_threshold),
        error_threshold_(error_threshold) {}

  OutlierClassification classify(double mahalanobis_distance) const {
    if (mahalanobis_distance > error_threshold_) {
      return OutlierClassification::kError;
    }
    if (mahalanobis_distance > warning_threshold_) {
      return OutlierClassification::kWarning;
    }
    return OutlierClassification::kNormal;
  }

  double warning_threshold() const { return warning_threshold_; }

  double error_threshold() const { return error_threshold_; }

 private:
  double warning_threshold_ = 0.0;
  double error_threshold_ = 0.0;
};

struct ImuNoiseConfig final : public ReflectiveConfigBase<ImuNoiseConfig> {
  std::string_view name() const override { return "ImuNoiseConfig"; }
  double gyro_noise_density = deg2rad(2.0 * 35.0 / 3600.0);
  double gyro_random_walk = deg2rad(2.0 * 4.0 / 3600.0);
  double gyro_bias_correlation_time = 1.0e+3;
  double gyro_turn_on_bias_sigma = deg2rad(0.5);

  double accel_noise_density = 2.0 * 2.0e-3;
  double accel_random_walk = 2.0 * 3.0e-3;
  double accel_bias_correlation_time = 300.0;
  double accel_turn_on_bias_sigma = 20.0e-3 * 9.81;

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("gyro_noise_density", &ImuNoiseConfig::gyro_noise_density,
               F64Properties{.desc = "Gyro Noise Density (rad/s/√Hz)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("gyro_random_walk", &ImuNoiseConfig::gyro_random_walk,
               F64Properties{.desc = "Gyro Random Walk (rad/s²/√Hz)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("gyro_bias_correlation_time",
               &ImuNoiseConfig::gyro_bias_correlation_time,
               F64Properties{.desc = "Gyro Bias Correlation Time (seconds)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("gyro_turn_on_bias_sigma",
               &ImuNoiseConfig::gyro_turn_on_bias_sigma,
               F64Properties{.desc = "Gyro Turn-On Bias Sigma (radians)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("accel_noise_density", &ImuNoiseConfig::accel_noise_density,
               F64Properties{.desc = "Accelerometer Noise Density (m/s²/√Hz)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("accel_random_walk", &ImuNoiseConfig::accel_random_walk,
               F64Properties{.desc = "Accelerometer Random Walk (m/s³/√Hz)",
                             .bounds = Bounds<double>::Positive()}),
      Describe(
          "accel_bias_correlation_time",
          &ImuNoiseConfig::accel_bias_correlation_time,
          F64Properties{.desc = "Accelerometer Bias Correlation Time (seconds)",
                        .bounds = Bounds<double>::Positive()}),
      Describe("accel_turn_on_bias_sigma",
               &ImuNoiseConfig::accel_turn_on_bias_sigma,
               F64Properties{.desc = "Accelerometer Turn-On Bias Sigma (m/s²)",
                             .bounds = Bounds<double>::Positive()}));
};

struct GpsNoiseConfig final : public ReflectiveConfigBase<GpsNoiseConfig> {
  std::string_view name() const override { return "GpsNoiseConfig"; }

  double hor_pos_std_dev = 3.0;
  double ver_pos_std_dev = 6.0;
  double hor_vel_std_dev = 0.1;
  double ver_vel_std_dev = 0.1;

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("hor_pos_std_dev", &GpsNoiseConfig::hor_pos_std_dev,
               F64Properties{.desc = "Horizontal Position Std Dev (meters)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("ver_pos_std_dev", &GpsNoiseConfig::ver_pos_std_dev,
               F64Properties{.desc = "Vertical Position Std Dev (meters)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("hor_vel_std_dev", &GpsNoiseConfig::hor_vel_std_dev,
               F64Properties{.desc = "Horizontal Velocity Std Dev (m/s)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("ver_vel_std_dev", &GpsNoiseConfig::ver_vel_std_dev,
               F64Properties{.desc = "Vertical Velocity Std Dev (m/s)",
                             .bounds = Bounds<double>::Positive()}));
};

// State holder for a single noise process (e.g., Gyro X axis)
class NoiseProcess {
 public:
  NoiseProcess();

  NoiseProcess(double noise_density, double random_walk,
               double correlation_time);

  std::error_code configure(double noise_density, double random_walk,
                            double correlation_time);

  void initializeBias(double turn_on_sigma);

  // Updates bias and returns the corrupt value (true_val + bias + white_noise)
  double corrupt(double true_value, double dt);

  double bias() const { return bias_; }

 private:
  std::mt19937 rng_;
  double noise_density_ = 0.0;
  double random_walk_ = 0.0;
  double correlation_time_ = 1.0;
  double bias_ = 0.0;
};

class MultivariableNoiseProcess {
 public:
  static constexpr Eigen::Index kMaxDim = 6;

  template <Eigen::Index Dim>
    requires(Dim > 0 && Dim <= kMaxDim)
  class Dimension : public std::integral_constant<Eigen::Index, Dim> {};

  using Vector = heapless::VectorX<double, kMaxDim>;

  template <Eigen::Index Dim>
  MultivariableNoiseProcess(Dimension<Dim> /*dim*/)
      : dim_(Dim),
        rng_(std::random_device{}()),
        noise_density_(Vector::Zero(Dim)),
        random_walk_(Vector::Zero(Dim)),
        correlation_time_(Vector::Ones(Dim)),
        bias_(Vector::Zero(Dim)) {}

  std::error_code configure(
      const Eigen::Ref<const Eigen::VectorXd>& noise_density,
      const Eigen::Ref<const Eigen::VectorXd>& random_walk,
      const Eigen::Ref<const Eigen::VectorXd>& correlation_time);

  std::error_code initializeBias(
      const Eigen::Ref<const Eigen::VectorXd>& turn_on_sigma);

  // Updates bias and returns the corrupt value (true_val + bias + white_noise)
  std::error_code corrupt(Eigen::Ref<Eigen::VectorXd> value, double dt);

  Eigen::VectorXd bias() const { return bias_; }

 private:
  Eigen::Index dim_;
  std::mt19937 rng_;
  Vector noise_density_;
  Vector random_walk_;
  Vector correlation_time_;
  Vector bias_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_SENSORS_HPP_
