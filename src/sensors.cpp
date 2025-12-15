#include "autopilot/simulator/sensors.hpp"

#include "autopilot/core/common.hpp"

namespace autopilot {

NoiseProcess::NoiseProcess() : rng_(std::random_device{}()) {}

NoiseProcess::NoiseProcess(double noise_density, double random_walk,
                           double correlation_time)
    : rng_(std::random_device{}()),
      noise_density_(noise_density),
      random_walk_(random_walk),
      correlation_time_(correlation_time) {}

std::error_code NoiseProcess::configure(double noise_density,
                                        double random_walk,
                                        double correlation_time) {
  if (noise_density < 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }

  if (random_walk < 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }

  if (correlation_time <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }

  noise_density_ = noise_density;
  random_walk_ = random_walk;
  correlation_time_ = correlation_time;
  return {};
}

void NoiseProcess::initializeBias(double turn_on_sigma) {
  std::normal_distribution dist(0.0, 1.0);
  bias_ = turn_on_sigma * dist(rng_);
}

double NoiseProcess::corrupt(double true_value, double dt) {
  // 1. Propagate Bias (Discrete Gauss-Markov)
  // Ref: nano_rotor/src/sensors/imu.cpp
  const double bias_decay = std::exp(-dt / correlation_time_);

  // Bias random walk stddev (discretized)
  const double bias_walk_std =
      std::sqrt(-std::pow(random_walk_, 2) * correlation_time_ / 2.0 *
                (std::exp(-2.0 * dt / correlation_time_) - 1.0));

  std::normal_distribution dist(0.0, 1.0);
  bias_ = bias_decay * bias_ + bias_walk_std * dist(rng_);

  // 2. Add White Noise
  const double white_noise_std = noise_density_ / std::sqrt(dt);
  return true_value + bias_ + white_noise_std * dist(rng_);
}

std::error_code MultivariableNoiseProcess::configure(
    const Eigen::Ref<const Eigen::VectorXd>& noise_density,
    const Eigen::Ref<const Eigen::VectorXd>& random_walk,
    const Eigen::Ref<const Eigen::VectorXd>& correlation_time) {
  if (noise_density.size() != dim_ || random_walk.size() != dim_ ||
      correlation_time.size() != dim_) {
    return make_error_code(AutopilotErrc::kInvalidDimension);
  }
  if (noise_density.minCoeff() < 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  if (random_walk.minCoeff() < 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  if ((correlation_time.array() <= 0.0).any()) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }

  noise_density_ = noise_density;
  random_walk_ = random_walk;
  correlation_time_ = correlation_time;
  return {};
}

std::error_code MultivariableNoiseProcess::initializeBias(
    const Eigen::Ref<const Eigen::VectorXd>& turn_on_sigma) {
  if (turn_on_sigma.size() != dim_) {
    return make_error_code(AutopilotErrc::kInvalidDimension);
  }

  std::normal_distribution<double> dist(0.0, 1.0);
  // Initialize bias: sigma * random_val
  bias_ = turn_on_sigma.cwiseProduct(
      Vector::NullaryExpr(dim_, [&] { return dist(rng_); }));
  return {};
}

std::error_code MultivariableNoiseProcess::corrupt(
    Eigen::Ref<Eigen::VectorXd> value, double dt) {
  using Eigen::exp;
  using Eigen::sqrt;

  if (value.size() != dim_) {
    return make_error_code(AutopilotErrc::kInvalidBufferSize);
  }

  // 1. Propagate Bias (Discrete Gauss-Markov)
  const Vector bias_decay = (-dt * correlation_time_.array().inverse()).exp();

  // FIX 1: Parenthesis correction. (exp(x) - 1.0), not exp(x - 1.0)
  // Formula: sigma_d = sqrt( -sigma^2 * tau / 2 * (e^(-2dt/tau) - 1) )
  const Vector bias_walk_std =
      sqrt(-random_walk_.array().square() * correlation_time_.array() / 2.0 *
           (exp(-2.0 * dt / correlation_time_.array()) - 1.0));

  std::normal_distribution<double> dist(0.0, 1.0);

  // Update internal bias state
  // Note: unaryExpr is efficient here
  bias_ =
      bias_decay.cwiseProduct(bias_) +
      bias_walk_std.unaryExpr([&](double sigma) { return sigma * dist(rng_); });

  // 2. Add White Noise
  const Vector white_noise_std = noise_density_ / std::sqrt(dt);

  // FIX 2: Generate independent noise for each axis
  // Previous code multiplied vector by one scalar. We need a vector of randoms.
  const Vector white_noise = white_noise_std.cwiseProduct(
      Vector::NullaryExpr(dim_, [&] { return dist(rng_); }));

  value += bias_ + white_noise;
  return {};
}

}  // namespace autopilot
