#ifndef AUTOPILOT_CASCADE_CONTROLLER_HPP_
#define AUTOPILOT_CASCADE_CONTROLLER_HPP_

#include <utility>

#include "autopilot/base.hpp"

namespace autopilot {

// 1. Position Control
struct PositionReference {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration_ff;  // Feed-forward
};

struct PositionOutput {
  Eigen::Vector3d target_force;  // The thrust vector (mass * accel)
};

// 2. Attitude Control
struct AttitudeReference {
  Eigen::Quaterniond orientation;
  Eigen::Vector3d body_rate;      // Feed-forward
  Eigen::Vector3d angular_accel;  // Feed-forward
};

struct AttitudeOutput {
  Eigen::Vector3d body_rate;
  Eigen::Vector3d torque;
};

class PositionControllerBase : public Module {
 public:
  PositionControllerBase(const std::string& name,
                         std::shared_ptr<QuadrotorModel> model,
                         std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("PositionController.{}", name), std::move(model),
               std::move(logger)) {}

  virtual std::error_code compute(const QuadrotorState& state,
                                  const PositionReference& ref,
                                  PositionOutput& command) const = 0;

  virtual void reset() {}
};

class AttitudeControllerBase : public Module {
 public:
  AttitudeControllerBase(const std::string& name,
                         std::shared_ptr<QuadrotorModel> model,
                         std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("AttitudeController.{}", name), std::move(model),
               std::move(logger)) {}

  virtual std::error_code compute(const QuadrotorState& attitude,
                                  const AttitudeReference& ref,
                                  AttitudeOutput& command) const = 0;

  virtual void reset() {}
};

class CascadeControllerConfig {
 public:
  double posctl_dt() const { return posctl_dt_; }

  double attctl_dt() const { return attctl_dt_; }

  std::error_code setPosctlDt(double dt) {
    if (dt <= 0.0) {
      return make_error_code(AutopilotErrc::kOutOfBounds);
    }
    posctl_dt_ = dt;
    return {};
  }

  std::error_code setAttctlDt(double dt) {
    if (dt <= 0.0) {
      return make_error_code(AutopilotErrc::kOutOfBounds);
    }
    if (dt > posctl_dt_) {
      return make_error_code(AutopilotErrc::kOutOfBounds);
    }
    attctl_dt_ = dt;
    return {};
  }

 private:
  double posctl_dt_ = 0.02;
  double attctl_dt_ = 0.001;
};

class CascadeController : public ControllerBase {
 public:
  using Config = CascadeControllerConfig;

  CascadeController(const std::shared_ptr<QuadrotorModel>& model,
                    std::shared_ptr<CascadeControllerConfig> config,
                    const std::shared_ptr<spdlog::logger>& logger = nullptr);

  // The "Fast Loop" Entry Point (e.g. 500Hz)
  expected<std::size_t, std::error_code> compute(
      const QuadrotorState& state, std::span<const QuadrotorCommand> setpoints,
      std::span<QuadrotorCommand> outputs) override;

  // Safety Critical: Clear Integrators
  void reset() override;

  std::shared_ptr<PositionControllerBase> positionController() {
    return position_controller_;
  }

  std::shared_ptr<AttitudeControllerBase> attitudeController() {
    return attitude_controller_;
  }

 private:
  // Configuration
  int pos_divider_ = 10;  // 500Hz / 10 = 50Hz
  double dt_ = 0.002;     // 500Hz

  std::shared_ptr<Config> config_;
  // Composition
  std::shared_ptr<PositionControllerBase> position_controller_;
  std::shared_ptr<AttitudeControllerBase> attitude_controller_;

  // State for Multi-Rate Divider
  double last_posctl_time_ = -1.0;
  double last_attctl_time_ = -1.0;
  QuadrotorCommand out_cmd_;
  double collective_thrust_;
  AttitudeReference last_att_ref_;
};
;

}  // namespace autopilot

#endif  // AUTOPILOT_CASCADE_CONTROLLER_HPP_
