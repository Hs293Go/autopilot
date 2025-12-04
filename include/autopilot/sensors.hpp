#ifndef AUTOPILOT_SENSORS_HPP_
#define AUTOPILOT_SENSORS_HPP_

#include <random>
#include <span>

#include "Eigen/Dense"
#include "autopilot/common.hpp"

namespace autopilot {

struct ImuNoiseConfig {
  // Default values from ADIS16448 (nano_rotor defaults)
  double gyro_noise_density = 2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
  double gyro_random_walk = 2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
  double gyro_bias_correlation_time = 1.0e+3;
  double gyro_turn_on_bias_sigma = 0.5 / 180.0 * M_PI;

  double accel_noise_density = 2.0 * 2.0e-3;
  double accel_random_walk = 2.0 * 3.0e-3;
  double accel_bias_correlation_time = 300.0;
  double accel_turn_on_bias_sigma = 20.0e-3 * 9.81;
};

struct GpsNoiseConfig {
  double hor_pos_std_dev = 3.0;
  double ver_pos_std_dev = 6.0;
  double hor_vel_std_dev = 0.1;
  double ver_vel_std_dev = 0.1;
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
