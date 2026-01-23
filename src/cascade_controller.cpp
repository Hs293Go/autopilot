#include "autopilot/controllers/cascade_controller.hpp"

#include "autopilot/controllers/geometric_controller.hpp"

#if __has_include(<spdlog/fmt/ranges.h>)
#include "spdlog/fmt/ranges.h"
#else
#include "fmt/ranges.h"
#endif
namespace autopilot {

namespace {

Eigen::Quaterniond forceToAttitude(const Eigen::Vector3d& force, double yaw) {
  // 1. Collective Thrust (Project onto Body Z, or Norm)
  // Norm is safer for aggressive maneuvers (Swing-Twist decomposition)
  double collective_thrust = force.norm();

  // 2. Orientation Construction
  if (collective_thrust < 1e-3) {
    // Singularity check: Zero thrust -> keep current attitude or level
    return Eigen::Quaterniond::Identity();
  }

  Eigen::Vector3d z_b = force / collective_thrust;

  // Desired Heading Vector (in XY plane)
  const Eigen::Vector3d y_c(-std::sin(yaw), std::cos(yaw), 0.0);

  Eigen::Vector3d x_b = y_c.cross(z_b);
  if (x_b.norm() < 1e-3) {
    // Gimbal Lock Case (Force is straight up/down, can't determine heading)
    // Fallback to purely Level + Yaw
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));  // simplified
  }
  x_b.normalize();

  const Eigen::Vector3d y_b = z_b.cross(x_b);

  Eigen::Matrix3d rotation;
  rotation << x_b, y_b, z_b;
  return Eigen::Quaterniond(rotation);
}
}  // namespace

CascadeController::CascadeController(
    const std::shared_ptr<QuadrotorModel>& model,
    std::shared_ptr<Config> config,
    const std::shared_ptr<spdlog::logger>& logger)
    : ControllerBase("CascadedController", model, logger),
      config_(std::move(config)) {
  // Initialize sub-controllers (could be injected via ctor in future)
  // For now, we assume standard PID implementations
  if (config_->position_controller_cfg() &&
      config_->attitude_controller_cfg()) {
    position_controller_ = PositionControllerFactory::Create(
        config_->position_controller_cfg(), model, logger);
    attitude_controller_ = AttitudeControllerFactory::Create(
        config_->attitude_controller_cfg(), model, logger);
    return;
  }
  position_controller_ =
      std::make_shared<GeometricPositionController>(model, logger);
  attitude_controller_ =
      std::make_shared<GeometricAttitudeController>(model, logger);
  this->logger()->warn(
      "Failed to load sub-controller configs; Falling back to default "
      "Geometric controllers.");
}

std::expected<std::size_t, AutopilotErrc> CascadeController::compute(
    const QuadrotorState& state, std::span<const QuadrotorCommand> setpoints,
    std::span<QuadrotorCommand> outputs) {
  if (outputs.empty() || setpoints.empty()) {
    logger()->error(
        "Empty input or output buffers provided to CascadeController.");
    return std::unexpected(AutopilotErrc::kInvalidBufferSize);
  }

  const auto& setpoint_cmd = setpoints[0];
  // 1. Divider Logic (Run Position Controller at e.g. 50Hz)
  // --------------------------------------------------------
  const bool run_pos_loop =
      last_posctl_time_ < 0.0 ||
      (state.timestamp_secs - last_posctl_time_) >= config_->posctl_dt();

  if (run_pos_loop) {
    // A. Extract "Horizon 0" Setpoint

    // B. Populate Locked-Down Reference struct
    // This acts as the "Sanitizer" for the raw command
    const PositionReference pos_ref = {
        .position = setpoint_cmd.position(),
        .velocity = setpoint_cmd.velocity(),
        .acceleration_ff = setpoint_cmd.acceleration(),
    };

    // C. Run Position Kernel
    PositionOutput pos_out;
    if (auto ec = position_controller_->compute(state, pos_ref, pos_out);
        ec != AutopilotErrc::kNone) {
      logger()->error("Position controller failed, reason: {}", ec);
      return std::unexpected(ec);
    }

    if (auto ec = out_cmd_.setForce(pos_out.target_force);
        ec != AutopilotErrc::kNone) {
      logger()->error("Failed to set force: {}, reason: {}",
                      pos_out.target_force, ec);
      return std::unexpected(ec);
    }

    // Project desired force into body Z axis; NOT simply norm of desired force
    collective_thrust_ =
        (state.odometry.pose().rotation().inverse() * pos_out.target_force).z();
    if (auto ec = out_cmd_.setCollectiveThrust(collective_thrust_);
        ec != AutopilotErrc::kNone) {
      logger()->error("Failed to set collective thrust: {:.4f}, reason: {}",
                      collective_thrust_, ec);
      return std::unexpected(ec);
    }

    // D. Geometric Conversion (Force Vector -> Attitude Quaternion)
    // This updates the "held" reference for the fast loop
    last_att_ref_.orientation =
        forceToAttitude(pos_out.target_force, setpoint_cmd.yaw());
    last_posctl_time_ = state.timestamp_secs;
  }

  // Pass through feedthrough independent of position loop
  last_att_ref_.body_rate = setpoint_cmd.bodyRate();
  last_att_ref_.angular_accel = setpoint_cmd.angularAcceleration();

  const bool run_att_loop =
      last_attctl_time_ < 0.0 ||
      (state.timestamp_secs - last_attctl_time_) >= config_->attctl_dt();

  if (run_att_loop) {
    // 2. Run Attitude Controller (Every Tick, e.g. 500Hz)
    // ---------------------------------------------------
    AttitudeOutput att_out;
    // Always use FRESH state, but potentially STALE (held) reference
    if (auto ec = attitude_controller_->compute(state, last_att_ref_, att_out);
        ec != AutopilotErrc::kNone) {
      logger()->error("Attitude controller failed, reason: {}", ec);
      return std::unexpected(ec);
    }

    // 3. Output Generation (The Mixer)
    // ---------------------------------------------------
    // In a real system, Mixer might be its own module.
    // Here we do a simple allocation or pass torque/thrust if the drone handles
    // mixing.

    out_cmd_.setTimestamp(state.timestamp_secs + dt_);  // dt of the fast loop

    // If we have a mixer:
    Eigen::Vector4d thrust_moments;
    thrust_moments << collective_thrust_, att_out.torque;
    const Eigen::Vector4d motor_thrusts =
        model()->thrustTorqueToMotorThrusts(thrust_moments).cwiseMax(0.0);
    if (auto ec = out_cmd_.setMotorThrusts(motor_thrusts);
        ec != AutopilotErrc::kNone) {
      logger()->error("Failed to set motor thrusts: {}, reason: {}",
                      motor_thrusts, ec);
      return std::unexpected(ec);
    }

    if (auto ec = out_cmd_.setBodyRate(att_out.body_rate);
        ec != AutopilotErrc::kNone) {
      logger()->error("Failed to set body rate: {}, reason: {}",
                      att_out.body_rate, ec);
      return std::unexpected(ec);
    }

    // If we output wrench (for simulator):
    if (auto ec = out_cmd_.setTorque(att_out.torque);
        ec != AutopilotErrc::kNone) {
      return std::unexpected(ec);
    }
    last_attctl_time_ = state.timestamp_secs;
  }
  outputs[0] = out_cmd_;
  return 1U;  // We produced 1 valid step
}

void CascadeController::reset() {
  // Reset the inner controllers (assuming they expose reset)
  position_controller_->reset();
  attitude_controller_->reset();

  // Reset the "Hold" value to hover
  last_att_ref_ =
      AttitudeReference{.orientation = Eigen::Quaterniond::Identity(),
                        .body_rate = Eigen::Vector3d::Zero(),
                        .angular_accel = Eigen::Vector3d::Zero()};
}

REGISTER_CONTROLLER(CascadeController, "CascadeController");
}  // namespace autopilot
