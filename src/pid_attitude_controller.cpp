#include "autopilot/controllers/pid_attitude_controller.hpp"

#include "autopilot/core/math.hpp"

namespace autopilot {

PidAttitudeController::PidAttitudeController(
    std::shared_ptr<QuadrotorModel> model, std::shared_ptr<Config> config,
    std::shared_ptr<spdlog::logger> logger)
    : AttitudeControllerBase("PidAttitude", std::move(model),
                             std::move(logger)),
      config_(std::move(config)),
      // Initialize 3 PIDs with default configs. Actual config is applied in
      // compute/update.
      pids_{autopilot::PID(config_->roll_pid),
            autopilot::PID(config_->pitch_pid),
            autopilot::PID(config_->yaw_pid)} {}

PidAttitudeController::PidAttitudeController(
    std::shared_ptr<QuadrotorModel> model,
    std::shared_ptr<spdlog::logger> logger)
    : PidAttitudeController(std::move(model), std::make_shared<Config>(),
                            std::move(logger)) {}

void PidAttitudeController::reset() {
  for (auto& pid : pids_) {
    pid.reset();
  }
}

AutopilotErrc PidAttitudeController::compute(const QuadrotorState& state,
                                             const AttitudeReference& ref,
                                             AttitudeOutput& out) const {
  // 1. Sync Configs (Cheap copy, ensures runtime tuning works)

  // 2. Unpack State
  const Eigen::Quaterniond& q = state.odometry.pose().rotation();
  const Eigen::Vector3d& rate_fb = state.odometry.twist().angular();
  const Eigen::Quaterniond& q_des = ref.orientation;

  const Eigen::Vector3d angle_error =
      EvaluateAttitudeError(q, q_des, config_->error_law);

  // Apply P-Gain
  // We apply saturation to the rate setpoint to stay within physical limits
  Eigen::Vector3d rate_sp_feedback = -config_->k_att.cwiseProduct(angle_error)
                                          .cwiseMax(-model()->max_body_rate())
                                          .cwiseMin(model()->max_body_rate());

  // 4. Feedforward: Trajectory Rate
  // The reference body rate is usually expressed in the DESIRED body frame.
  // We must rotate it into the CURRENT body frame to track it directly.
  const Eigen::Matrix3d rot_fb = q.toRotationMatrix();
  const Eigen::Matrix3d rot_des = q_des.toRotationMatrix();
  const Eigen::Matrix3d rot_des_to_curr = rot_fb.transpose() * rot_des;

  const Eigen::Vector3d rate_ff_body = rot_des_to_curr * ref.body_rate;

  // Total Desired Rate
  Eigen::Vector3d rate_sp_total = rate_sp_feedback + rate_ff_body;

  // Write to output for logging/debugging
  out.body_rate = rate_sp_total;

  // 5. Inner Loop: Rate Error -> Torque (PID)
  double now = state.timestamp_secs;
  const auto torque_output =
      [&]<auto... Ix>(std::index_sequence<Ix...>) -> Eigen::Vector3d {
    return {pids_[Ix].compute(rate_fb[Ix], rate_sp_total[Ix], now)...};
  }(std::make_index_sequence<3>());

  // 6. Acceleration Feedforward (Inertia * Alpha_ref)
  const Eigen::Vector3d accel_ff_body = rot_des_to_curr * ref.angular_accel;
  const Eigen::Vector3d torque_ff = model()->inertia() * accel_ff_body;

  // Gyroscopic Feedforward (Optional, often helps with aggressive maneuvers)
  // Torque needed to cancel gyroscopic precession: w x (J * w)
  const Eigen::Vector3d torque_gyro =
      rate_fb.cross(model()->inertia() * rate_fb);

  out.torque = torque_output + torque_ff + torque_gyro;

  return {};
}

// Register the controller so the Factory can create it by name "PidAttitude"
REGISTER_ATTITUDE_CONTROLLER(PidAttitudeController);

}  // namespace autopilot
