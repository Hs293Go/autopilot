#include "autopilot/controllers/geometric_controller.hpp"

#include "autopilot/core/math.hpp"

namespace autopilot {
GeometricPositionController::GeometricPositionController(
    std::shared_ptr<QuadrotorModel> model, std::shared_ptr<Config> config,
    std::shared_ptr<spdlog::logger> logger)
    : PositionControllerBase("GeometricPosition", std::move(model),
                             std::move(logger)),
      config_(std::move(config)) {}

GeometricPositionController::GeometricPositionController(
    std::shared_ptr<QuadrotorModel> model,
    std::shared_ptr<spdlog::logger> logger)
    : GeometricPositionController(std::move(model), std::make_shared<Config>(),
                                  std::move(logger)) {}

GeometricAttitudeController::GeometricAttitudeController(
    std::shared_ptr<QuadrotorModel> model, std::shared_ptr<Config> config,
    std::shared_ptr<spdlog::logger> logger)
    : AttitudeControllerBase("GeometricAttitude", std::move(model),
                             std::move(logger)),
      config_(std::move(config)) {}

GeometricAttitudeController::GeometricAttitudeController(
    std::shared_ptr<QuadrotorModel> model,
    std::shared_ptr<spdlog::logger> logger)
    : GeometricAttitudeController(std::move(model), std::make_shared<Config>(),
                                  std::move(logger)) {}

std::error_code GeometricPositionController::compute(
    const QuadrotorState& state, const PositionReference& ref,
    PositionOutput& out) const {
  // --- 1. Error Calculation (Euclidean) ---
  const Eigen::Vector3d& p = state.odometry.pose().translation();
  const Eigen::Vector3d& v = state.odometry.twist().linear();

  const Eigen::Vector3d position_error = p - ref.position;
  const Eigen::Vector3d velocity_error = v - ref.velocity;

  // --- 2. Control Law (PD + FF) ---
  // F_des = -Kp*ep - Kv*ev - m*g + m*a_ref (g is gravity vector in inertial
  // frame and subtracted)
  const Eigen::Vector3d a_des = -config_->kp.cwiseProduct(position_error) -
                                config_->kv.cwiseProduct(velocity_error) +
                                ref.acceleration_ff - model()->grav_vector();

  Eigen::Vector3d f_des = model()->mass() * a_des;
  // --- 3. Safety/Sanity Limits ---
  // Simple saturation on the total acceleration magnitude
  const double collective_thrust_des = f_des.norm();
  if (collective_thrust_des > model()->max_collective_thrust()) {
    logger()->warn(
        "{}: Desired force ({:.2f} N) exceeds max ({:.2f} N). Scaling down.",
        name(), collective_thrust_des, model()->max_collective_thrust());
    f_des = f_des * model()->max_collective_thrust() / collective_thrust_des;
  }

  // --- 4. Output Generation ---
  out.target_force = f_des;

  return {};
}

std::error_code GeometricAttitudeController::compute(
    const QuadrotorState& state, const AttitudeReference& ref,
    AttitudeOutput& out) const {
  // --- 1. Unpack State ---
  const Eigen::Quaterniond& q = state.odometry.pose().rotation();
  const Eigen::Vector3d& rate_fb = state.odometry.twist().angular();
  const Eigen::Quaterniond& q_des = ref.orientation;

  // Convention check: Lee defines e_R such that positive error drives
  // negative moment e_R = 0.5 * (R_d^T * R - R^T * R_d) Using standard
  // rotation matrix math is often safer than quaternion shortcuts to match
  // the paper exactly.
  const Eigen::Matrix3d rot_fb = q.toRotationMatrix();
  const Eigen::Matrix3d rot_des = q_des.toRotationMatrix();

  Eigen::Vector3d angle_error =
      0.5 * vee(rot_des.transpose() * rot_fb - rot_fb.transpose() * rot_des);

  // May be used as a computed 'body_rate_setpoint', e.g., in
  // mavros_controllers. In this case the body_rate field in ref is ignored
  const Eigen::Vector3d rate_ref_comp = config_->kR.cwiseProduct(angle_error)
                                            .cwiseMax(-model()->max_body_rate())
                                            .cwiseMin(model()->max_body_rate());
  out.body_rate = -rate_ref_comp;

  // Precompute the transformation from desired body frame to current body frame
  const Eigen::Matrix3d rot_des_to_curr = rot_fb.transpose() * rot_des;

  // Resolve body rate reference and angular acceleration in current body frame.
  const Eigen::Vector3d rate_ref_body = rot_des_to_curr * ref.body_rate;
  const Eigen::Vector3d accel_ref_body = rot_des_to_curr * ref.angular_accel;

  Eigen::Vector3d rate_error = rate_fb - rate_ref_body;

  out.torque = -rate_ref_comp - config_->kOmega.cwiseProduct(rate_error) +
               model()->inertia() * accel_ref_body;

  // 2. Gyroscopic / Transport Toggle
  if (config_->enable_exact_linearization) {
    // Strategy A: Exact Linearization (Lee 2010)
    // Cancel natural gyro dynamics (w x Jw)
    // Handle transport theorem cross-term J(w x w_ref)

    const Eigen::Vector3d gyro_cancel =
        rate_fb.cross(model()->inertia() * rate_fb);
    const Eigen::Vector3d transport_coupling =
        model()->inertia() * rate_fb.cross(rate_ref_body);

    out.torque += gyro_cancel - transport_coupling;

  } else {
    // Strategy B: Robust Feedforward (Standard)
    // Feed-forward the expected gyroscopic torque based on trajectory
    // Ignores 'transport_coupling' as it's a second-order error term
    out.torque += rate_ref_body.cross(model()->inertia() * rate_ref_body);
  }
  return {};
}
REGISTER_POSITION_CONTROLLER(GeometricPositionController);
REGISTER_ATTITUDE_CONTROLLER(GeometricAttitudeController);
}  // namespace autopilot
