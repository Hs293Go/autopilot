#include "autopilot/quadrotor_model.hpp"

#include "autopilot/common.hpp"

namespace autopilot {

Eigen::Vector4d QuadrotorModel::momentsToMotorThrusts(
    const Eigen::Ref<const Eigen::Vector4d>& moments) const {
  return inv_allocation_matrix_ * moments;
}
Eigen::Vector4d QuadrotorModel::motorThrustsToMoments(
    const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts) const {
  return allocation_matrix_ * motor_thrusts;
}

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

  if (inertia_elems.ixx * inertia_elems.iyy < inertia_elems.izz ||
      inertia_elems.ixx * inertia_elems.izz < inertia_elems.iyy ||
      inertia_elems.iyy * inertia_elems.izz < inertia_elems.ixx) {
    return (make_error_code(AutopilotErrc::kPhysicallyInvalid));
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

}  // namespace autopilot
