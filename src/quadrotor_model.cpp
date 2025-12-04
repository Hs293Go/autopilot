#include "autopilot/quadrotor_model.hpp"

#include "autopilot/common.hpp"
#include "autopilot/geometry.hpp"

namespace autopilot {

std::error_code QuadrotorModelCfg::setMass(double mass) {
  if (!std::isfinite(mass)) {
    return (make_error_code(AutopilotErrc::kNumericallyNonFinite));
  }
  mass_ = mass;
  return {};
}

std::error_code QuadrotorModelCfg::setInertiaElements(
    const InertiaElements& inertia_elems) {
  if (!std::isfinite(inertia_elems.ixx) || !std::isfinite(inertia_elems.iyy) ||
      !std::isfinite(inertia_elems.izz) || !std::isfinite(inertia_elems.ixy) ||
      !std::isfinite(inertia_elems.ixz) || !std::isfinite(inertia_elems.iyz)) {
    return (make_error_code(AutopilotErrc::kNumericallyNonFinite));
  }

  if (inertia_elems.ixx <= 0.0 || inertia_elems.iyy <= 0.0 ||
      inertia_elems.izz <= 0.0) {
    return (make_error_code(AutopilotErrc::kPhysicallyInvalid));
  }

  if (inertia_elems.ixx + inertia_elems.iyy < inertia_elems.izz ||
      inertia_elems.ixx + inertia_elems.izz < inertia_elems.iyy ||
      inertia_elems.iyy + inertia_elems.izz < inertia_elems.ixx) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }

  inertia_elems_ = inertia_elems;
  return {};
}

std::error_code QuadrotorModelCfg::setFrontMotorPosition(
    const Eigen::Ref<const Eigen::Vector2d>& position) {
  if (!position.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (position.minCoeff() <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  front_motor_position_ = position;
  return {};
}

std::error_code QuadrotorModelCfg::setBackMotorPosition(
    const Eigen::Ref<const Eigen::Vector2d>& position) {
  if (!position.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (position.minCoeff() <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  back_motor_position_ = position;
  return {};
}

std::error_code QuadrotorModelCfg::setTorqueConstant(double torque_constant) {
  if (!std::isfinite(torque_constant)) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (torque_constant <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  torque_constant_ = torque_constant;
  return {};
}

std::error_code QuadrotorModelCfg::setMotorTimeConstantUp(
    double time_constant_up) {
  if (!std::isfinite(time_constant_up)) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  if (time_constant_up <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  motor_time_constant_up_ = time_constant_up;
  return {};
}

std::error_code QuadrotorModelCfg::setMotorTimeConstantDown(
    double time_constant_down) {
  if (!std::isfinite(time_constant_down)) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  if (time_constant_down <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  motor_time_constant_down_ = time_constant_down;
  return {};
}

std::error_code QuadrotorModelCfg::setThrustCurveCoeff(
    double thrust_curve_coeff) {
  if (!std::isfinite(thrust_curve_coeff)) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (thrust_curve_coeff <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  thrust_curve_coeff_ = thrust_curve_coeff;
  return {};
}

std::error_code QuadrotorModelCfg::setMinCollectiveThrust(double min_thrust) {
  if (!std::isfinite(min_thrust)) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (min_thrust < 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  min_collective_thrust_ = min_thrust;
  return {};
}

std::error_code QuadrotorModelCfg::setMaxCollectiveThrust(double max_thrust) {
  if (!std::isfinite(max_thrust)) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (max_thrust <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }

  if (max_thrust <= min_collective_thrust_) {
    return make_error_code(
        AutopilotErrc::kInvalidOrdering);  // Max thrust must exceed min
  }
  max_collective_thrust_ = max_thrust;
  return {};
}

std::error_code QuadrotorModelCfg::setMaxBodyRate(
    const Eigen::Ref<const Eigen::Vector3d>& max_body_rate) {
  if (!max_body_rate.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (max_body_rate.minCoeff() <= 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }
  max_body_rate_ = max_body_rate;
  return {};
}

std::error_code QuadrotorModelCfg::setGravVector(
    const Eigen::Ref<const Eigen::Vector3d>& grav_vector) {
  if (!grav_vector.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  grav_vector_ = grav_vector;
  return {};
}

void QuadrotorModel::updateDerivedState() {
  // 1. Recompute the Cholesky decomposition
  // Accessing cfg_->inertia_ assuming you add a getter or friend access
  updateInertia();
  updateMixerMatrix();

  // 2. Update the cache version
  // cached_cfg_version_ = cfg_->getVersion();
}

void QuadrotorModel::updateInertia() {
  // Keep inertia symmetric
  Eigen::Matrix3d inertia_cand;
  const auto [ixx, iyy, izz, ixy, ixz, iyz] = cfg_->inertia_elems();
  inertia_cand << ixx, ixy, ixz,  //
      ixy, iyy, iyz,              //
      ixz, iyz, izz;
  const Eigen::LLT<Eigen::Matrix3d> llt = inertia_cand.llt();
  if (llt.info() == Eigen::Success) {
    inertia_ = inertia_cand;
    inv_inertia_ = llt.solve(Eigen::Matrix3d::Identity());
  } else {
    static constexpr double kRelThreshold = 1e-6;
    static constexpr double kMinAbsThreshold = 1e-12;
    // Off-diagonals are too large; project to nearest SPD matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(inertia_cand);

    const Eigen::Matrix3d& vm = eig.eigenvectors();
    const Eigen::Vector3d& dv = eig.eigenvalues();

    // Clamp eigenvalues to a dynamic minimum threshold
    const double threshold =
        std::max(kMinAbsThreshold, dv.maxCoeff() * kRelThreshold);
    const Eigen::Vector3d clamped_dv = dv.cwiseMax(threshold);
    inertia_ = vm * clamped_dv.asDiagonal() * vm.transpose();
    inv_inertia_ = vm * clamped_dv.cwiseInverse().asDiagonal() * vm.transpose();
  }
}

void QuadrotorModel::updateMixerMatrix() {
  const auto fx = cfg_->front_motor_position().x();
  const auto fy = cfg_->front_motor_position().y();
  const auto bx = cfg_->back_motor_position().x();
  const auto by = cfg_->back_motor_position().y();
  const auto c = cfg_->torque_constant();

  switch (cfg_->motor_layout()) {
    case MotorLayout::kPx4:
      allocation_matrix_ << Eigen::RowVector4d::Ones(),  //
          -fy, by, -fy, by,                              //
          -fx, bx, fx, -bx,                              //
          -c, -c, c, c;

      break;
    case MotorLayout::kBetaflight:
      allocation_matrix_ << Eigen::RowVector4d::Ones(),  //
          -by, -fy, by, fy,                              //
          bx, -fx, bx, -fx,                              //
          c, -c, -c, c;
      break;
  }
  inv_allocation_matrix_ = allocation_matrix_.inverse();
}

Eigen::Vector4d QuadrotorModel::thrustTorqueToMotorThrusts(
    const Eigen::Ref<const Eigen::Vector4d>& moments) const {
  return inv_allocation_matrix_ * moments;
}

Eigen::Vector4d QuadrotorModel::thrustTorqueToMotorThrusts(
    const ThrustTorque& thrust_torque) const {
  Eigen::Vector4d moments;
  moments(0) = thrust_torque.collective_thrust;
  moments.tail<3>() = thrust_torque.torque;
  return thrustTorqueToMotorThrusts(moments);
}

Eigen::Vector4d QuadrotorModel::motorThrustsToThrustTorqueVector(
    const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts) const {
  return allocation_matrix_ * motor_thrusts;
}

ThrustTorque QuadrotorModel::motorThrustsToThrustTorque(
    const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts) const {
  return {.collective_thrust = motor_thrusts.sum(),
          .torque = allocation_matrix_.bottomRows<3>() * motor_thrusts};
}

typename OdometryF64::ParamVector QuadrotorModel::rigidBodyDynamics(
    const Eigen::Ref<const typename OdometryF64::ParamVector>& state,
    const Eigen::Ref<const typename WrenchF64::ParamVector>& input) const {
  using RigidBodyState = typename OdometryF64::ParamVector;
  OdometryViewF64 odom(state.data());
  WrenchViewF64 wrench_view(input.data());

  const Eigen::Ref<const Eigen::Vector3d> velocity = odom.twist().linear();
  const Eigen::Ref<const Eigen::Vector3d> body_rate = odom.twist().angular();

  Eigen::Quaterniond qw;  // Purely imaginary quaternion for angular velocity
  qw.coeffs() << body_rate / 2.0, 0.0;

  RigidBodyState state_dot;
  state_dot << velocity,                             // position part
      (odom.pose().rotation() * qw).coeffs(),        // orientation part
      wrench_view.force() / mass() + grav_vector(),  // linear acceleration part
      invInertia() *
          (wrench_view.torque() - body_rate.cross(inertia() * body_rate));
  return state_dot;
}

OdometryF64 QuadrotorModel::rigidBodyDynamics(const OdometryF64& state,
                                              const WrenchF64& input) const {
  using RigidBodyState = typename OdometryF64::ParamVector;
  const Eigen::Ref<const Eigen::Vector3d> velocity = state.twist().linear();
  const Eigen::Ref<const Eigen::Vector3d> body_rate = state.twist().angular();

  Eigen::Quaterniond qw;  // Purely imaginary quaternion for angular velocity
  qw.coeffs() << body_rate / 2.0, 0.0;

  RigidBodyState state_dot;
  state_dot << velocity,                        // position part
      (state.pose().rotation() * qw).coeffs(),  // orientation part
      input.force() / mass() + grav_vector(),   // linear acceleration part
      invInertia() * (input.torque() - body_rate.cross(inertia() * body_rate));
  return OdometryViewF64(state_dot.data());
}

Eigen::Vector4d QuadrotorModel::motorSpeedDynamics(
    const Eigen::Ref<const Eigen::Vector4d>& motor_speeds_curr,
    const Eigen::Ref<const Eigen::Vector4d>& motor_speeds_sp) const {
  return motor_speeds_curr.binaryExpr(
      motor_speeds_sp, [this](auto speed, auto speed_sp) {
        const double change = speed_sp - speed;
        return change / (change > 0.0 ? motor_time_constant_up()
                                      : motor_time_constant_down());
      });
}

}  // namespace autopilot
