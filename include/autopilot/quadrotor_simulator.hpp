#ifndef AUTOPILOT_SIMULATOR_HPP_
#define AUTOPILOT_SIMULATOR_HPP_

#include "Eigen/Dense"
#include "autopilot/definitions.hpp"
#include "autopilot/module.hpp"
#include "autopilot/quadrotor_model.hpp"
#include "autopilot/sensor_data.hpp"
#include "autopilot/sensors.hpp"

namespace autopilot {

struct QuadrotorSimulatorConfig {
  ImuNoiseConfig imu;
  GpsNoiseConfig gps;
  unsigned int random_seed = 0;
  // Environment
  Eigen::Vector3d gravity = Eigen::Vector3d::UnitZ() * 9.81;
};

struct SensorData {
  std::shared_ptr<ImuData> imu;
  std::shared_ptr<GpsData> gps;
};

class QuadrotorSimulator : public Module {
 public:
  using Config = QuadrotorSimulatorConfig;

  QuadrotorSimulator(std::shared_ptr<QuadrotorModel> model,
                     std::shared_ptr<Config> config,
                     std::shared_ptr<spdlog::logger> logger = nullptr);

  // Initialize state
  void setState(const QuadrotorState& state);

  // Get current true state (for logging/feedback)
  const QuadrotorState& state() const { return state_; }

  // Advance simulation by dt seconds
  // input_cmd: expects motor_thrusts to be populated
  void step(const QuadrotorCommand& input_cmd, double dt);

  // New method to get noisy measurements
  SensorData getSensorMeasurements(double dt);

  std::shared_ptr<ImuData> getImuMeasurement(double dt);

  std::shared_ptr<GpsData> getGpsMeasurement();

 private:
  // Derivative function for RK4

  using SimStateVector = Eigen::Vector<double, OdometryF64::kNumParams + 4>;

  SimStateVector computeSystemDerivative(
      const SimStateVector& x, const Eigen::Vector4d& target_motor_speeds);

  std::shared_ptr<Config> config_;

  QuadrotorState state_;
  Eigen::Vector4d motor_speeds_ =
      Eigen::Vector4d::Zero();  // Internal state: rad/s

  // Noise Processes
  MultivariableNoiseProcess accel_noise_{
      MultivariableNoiseProcess::Dimension<3>()};
  MultivariableNoiseProcess gyro_noise_{
      MultivariableNoiseProcess::Dimension<3>()};

  // GPS Noise is simple white noise, so we just keep the generator
  std::mt19937 gps_rng_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_SIMULATOR_HPP_
