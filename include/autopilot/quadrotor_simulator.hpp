#ifndef AUTOPILOT_SIMULATOR_HPP_
#define AUTOPILOT_SIMULATOR_HPP_

#include "Eigen/Dense"
#include "autopilot/definitions.hpp"
#include "autopilot/module.hpp"
#include "autopilot/quadrotor_model.hpp"

namespace autopilot {

class QuadrotorSimulator : public Module {
 public:
  struct Config {
    std::uint32_t random_seed = 0;
  };
  QuadrotorSimulator(std::shared_ptr<QuadrotorModel> model,
                     std::shared_ptr<Config> cfg,
                     std::shared_ptr<spdlog::logger> logger = nullptr);

  // Initialize state
  void setState(const QuadrotorState& state);

  // Get current true state (for logging/feedback)
  const QuadrotorState& state() const { return state_; }

  // Advance simulation by dt seconds
  // input_cmd: expects motor_thrusts to be populated
  void step(const QuadrotorCommand& input_cmd, double dt);

 private:
  // Derivative function for RK4

  using SimStateVector = Eigen::Vector<double, OdometryF64::kNumParams + 4>;

  SimStateVector computeSystemDerivative(
      const SimStateVector& x, const Eigen::Vector4d& target_motor_speeds);

  std::shared_ptr<Config> cfg_;

  QuadrotorState state_;
  Eigen::Vector4d motor_speeds_ =
      Eigen::Vector4d::Zero();  // Internal state: rad/s
};

}  // namespace autopilot

#endif  // AUTOPILOT_SIMULATOR_HPP_
