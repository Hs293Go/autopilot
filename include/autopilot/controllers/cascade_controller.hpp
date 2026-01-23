#ifndef AUTOPILOT_CASCADE_CONTROLLER_HPP_
#define AUTOPILOT_CASCADE_CONTROLLER_HPP_

#include <utility>

#include "autopilot/base/controller_base.hpp"

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
  static constexpr std::string_view kModuleRootType = "PositionController";

  PositionControllerBase(const std::string& name,
                         std::shared_ptr<QuadrotorModel> model,
                         std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("PositionController.{}", name), std::move(model),
               std::move(logger)) {}

  virtual AutopilotErrc compute(const QuadrotorState& state,
                                const PositionReference& ref,
                                PositionOutput& command) const = 0;

  virtual void reset() {}
};

struct PositionControllerConfig : ConfigBase {};

using PositionControllerFactory = GenericFactory<PositionControllerBase>;

class AttitudeControllerBase : public Module {
 public:
  static constexpr std::string_view kModuleRootType = "AttitudeController";

  AttitudeControllerBase(const std::string& name,
                         std::shared_ptr<QuadrotorModel> model,
                         std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("AttitudeController.{}", name), std::move(model),
               std::move(logger)) {}

  virtual AutopilotErrc compute(const QuadrotorState& attitude,
                                const AttitudeReference& ref,
                                AttitudeOutput& command) const = 0;

  virtual void reset() {}
};

struct AttitudeControllerConfig : ConfigBase {};

using AttitudeControllerFactory = GenericFactory<AttitudeControllerBase>;

static constexpr char const* kCascadeControllerKey = "CascadeController";

class CascadeControllerConfig
    : public ReflectiveConfigBase<CascadeControllerConfig> {
 public:
  std::string_view name() const override { return kCascadeControllerKey; }

  double posctl_dt() const { return posctl_dt_; }

  double attctl_dt() const { return attctl_dt_; }

  AutopilotErrc setPosctlDt(double dt) {
    if (dt <= 0.0) {
      return AutopilotErrc::kOutOfBounds;
    }
    posctl_dt_ = dt;
    return {};
  }

  AutopilotErrc setAttctlDt(double dt) {
    if (dt <= 0.0) {
      return AutopilotErrc::kOutOfBounds;
    }
    if (dt > posctl_dt_) {
      return AutopilotErrc::kOutOfBounds;
    }
    attctl_dt_ = dt;
    return {};
  }

  const std::string& position_controller_type() const {
    return position_controller_.type;
  }

  std::shared_ptr<ConfigBase> position_controller_cfg() const {
    return position_controller_.config;
  }

  const std::string& attitude_controller_type() const {
    return attitude_controller_.type;
  }

  std::shared_ptr<ConfigBase> attitude_controller_cfg() const {
    return attitude_controller_.config;
  }

 private:
  friend ReflectiveConfigBase<CascadeControllerConfig>;

  double posctl_dt_ = 0.02;
  double attctl_dt_ = 0.001;

  Polymorphic<PositionControllerFactory> position_controller_;
  Polymorphic<AttitudeControllerFactory> attitude_controller_;

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("position_controller_dt", &CascadeControllerConfig::posctl_dt_,
               F64Properties{.desc = "Position controller time step (s)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("attitude_controller_dt", &CascadeControllerConfig::attctl_dt_,
               F64Properties{.desc = "Attitude controller time step (s)",
                             .bounds = Bounds<double>::Positive()}),
      Describe(
          "position_controller_cfg",
          &CascadeControllerConfig::position_controller_,
          Properties{.desc = "Position controller type and configuration"}),
      Describe(
          "attitude_controller_cfg",
          &CascadeControllerConfig::attitude_controller_,
          Properties{.desc = "Attitude controller type and configuration"}));
};

class CascadeController : public ControllerBase {
 public:
  using Config = CascadeControllerConfig;

  std::string_view name() const override { return kCascadeControllerKey; }

  CascadeController(const std::shared_ptr<QuadrotorModel>& model,
                    std::shared_ptr<CascadeControllerConfig> config,
                    const std::shared_ptr<spdlog::logger>& logger = nullptr);

  // The "Fast Loop" Entry Point (e.g. 500Hz)
  std::expected<std::size_t, AutopilotErrc> compute(
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

}  // namespace autopilot

#define REGISTER_POSITION_CONTROLLER(ConcreteType)                        \
  static const autopilot::Registrar<ConcreteType,                         \
                                    autopilot::PositionControllerFactory> \
      kRegistrarFor##ConcreteType(ConcreteType::kName)

#define REGISTER_ATTITUDE_CONTROLLER(ConcreteType)                        \
  static const autopilot::Registrar<ConcreteType,                         \
                                    autopilot::AttitudeControllerFactory> \
      kRegistrarFor##ConcreteType(ConcreteType::kName)

#endif  // AUTOPILOT_CASCADE_CONTROLLER_HPP_
