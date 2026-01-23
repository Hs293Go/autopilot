#ifndef AUTOPILOT_PID_ATTITUDE_CONTROLLER_HPP_
#define AUTOPILOT_PID_ATTITUDE_CONTROLLER_HPP_

#include <array>

#include "autopilot/controllers/attitude_error.hpp"
#include "autopilot/controllers/cascade_controller.hpp"
#include "pid.hpp"  // Assumes the header from previous steps is available

namespace autopilot {

// =================================================================
// PID Attitude Controller
// Logic: Cascade Control
//        Outer: Attitude Error -> Target Rate (P control)
//        Inner: Rate Error -> Torque (PID control)
// =================================================================

using namespace autopilot::literals;

class PidAttitudeController : public AttitudeControllerBase {
 public:
  static constexpr char kName[] = "PidAttitude";

  // 1. Config Struct matching the Autopilot Reflection System
  struct Config final : ReflectiveConfigBase<Config> {
    std::string_view name() const override { return kName; }

    // Outer Loop: Attitude -> Rate
    // P-Gains to convert angle error (rad) to rate setpoint (rad/s)
    Eigen::Vector3d k_att = Eigen::Vector3d::Constant(6.0);

    PidConfig roll_pid = PidConfigData{.kp = 1.5,
                                       .ki = 0.5,
                                       .kd = 0.1,
                                       .sample_time_s = 0.001,
                                       .output_min = -10.0,
                                       .output_max = 10.0};
    PidConfig pitch_pid = PidConfigData{.kp = 1.5,
                                        .ki = 0.5,
                                        .kd = 0.1,
                                        .sample_time_s = 0.001,
                                        .output_min = -10.0,
                                        .output_max = 10.0};
    PidConfig yaw_pid = PidConfigData{.kp = 1.0,
                                      .ki = 0.3,
                                      .kd = 0.05,
                                      .sample_time_s = 0.001,
                                      .output_min = -5.0,
                                      .output_max = 5.0};

    template <std::size_t I>
    const PidConfig& getPid() const {
      return std::get<I>(std::tie(roll_pid, pitch_pid, yaw_pid));
    }

    // Limits
    Eigen::Vector3d max_integrator_torque = Eigen::Vector3d::Constant(0.5);

    // Configurable Error Metric (reuse existing logic)
    AttitudeErrorLaw error_law = AttitudeErrorLaw::kGeometricSO3;

    static constexpr auto kDescriptors = std::make_tuple(
        Describe("k_att", &Config::k_att,
                 F64Properties{.desc = "Attitude Proportional Gains (1/s)",
                               .bounds = Bounds<double>::Positive()}),
        Describe("roll_pid", &Config::roll_pid,
                 Properties{.desc = "Roll Axis PID Configuration"}),
        Describe("pitch_pid", &Config::pitch_pid,
                 Properties{.desc = "Pitch Axis PID Configuration"}),
        Describe("yaw_pid", &Config::yaw_pid,
                 Properties{.desc = "Yaw Axis PID Configuration"}),
        Describe("max_integrator_torque", &Config::max_integrator_torque,
                 F64Properties{.desc = "Integrator Saturation Limit (Nm)",
                               .bounds = Bounds<double>::Positive()}),
        Describe(
            "error_law", &Config::error_law,
            I64Properties{
                .desc = "Attitude Error Computation Method",
                .map = kEnumMapping<"QuaternionBased"_s,
                                    AttitudeErrorLaw::kQuaternionBased,  //
                                    "GeometricSO3"_s,
                                    AttitudeErrorLaw::kGeometricSO3,  //
                                    "TiltPrioritizing"_s,
                                    AttitudeErrorLaw::kTiltPrioritizing>}));
  };

  PidAttitudeController(std::shared_ptr<QuadrotorModel> model,
                        std::shared_ptr<Config> config,
                        std::shared_ptr<spdlog::logger> logger = nullptr);

  PidAttitudeController(std::shared_ptr<QuadrotorModel> model,
                        std::shared_ptr<spdlog::logger> logger = nullptr);

  // Runtime Tuning
  std::shared_ptr<const Config> config() const { return config_; }
  std::shared_ptr<Config> config() { return config_; }

  AutopilotErrc compute(const QuadrotorState& state,
                        const AttitudeReference& ref,
                        AttitudeOutput& out) const override;

  void reset() override;

 private:
  std::shared_ptr<Config> config_;

  // Mutable because PID class maintains state (integrator, last time),
  // but compute() in autopilot is const.
  // In a strict sense, the controller *state* changes, so mutable is
  // appropriate for internal logic components.
  mutable std::array<autopilot::PID, 3> pids_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_PID_ATTITUDE_CONTROLLER_HPP_
