#include "autopilot/definitions.hpp"

#include "autopilot/math.hpp"

namespace autopilot {

double QuadrotorCommand::yaw() const {
  const Eigen::Quaterniond& q = setpoint_.odometry.pose().rotation();
  if (hasComponent(QuadrotorStateComponent::kYawOnly)) {
    return std::atan2(q.z(), q.w()) * 2.0;
  }
  return QuaternionToRollPitchYaw(q).z();
}

std::error_code QuadrotorCommand::setPosition(
    const Eigen::Ref<const Eigen::Vector3d>& position) {
  if (!position.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  setpoint_.odometry.pose().translation() = position;
  addComponent(QuadrotorStateComponent::kPosition);
  return {};
}

std::error_code QuadrotorCommand::setVelocity(
    const Eigen::Ref<const Eigen::Vector3d>& velocity) {
  if (!velocity.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  setpoint_.odometry.twist().linear() = velocity;
  addComponent(QuadrotorStateComponent::kVelocity);
  return {};
}

std::error_code QuadrotorCommand::setOrientation(
    const Eigen::Quaterniond& orientation) {
  if (!orientation.coeffs().allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (!IsClose(orientation.norm(), 1.0)) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }

  setpoint_.odometry.pose().rotation() = orientation;
  addComponent(QuadrotorStateComponent::kOrientation);
  return {};
}

std::error_code QuadrotorCommand::setYaw(double yaw) {
  if (!std::isfinite(yaw)) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  const double half_yaw = yaw / 2.0;
  setpoint_.odometry.pose().rotation() = {std::cos(half_yaw), 0.0, 0.0,
                                          std::sin(half_yaw)};
  addComponent(QuadrotorStateComponent::kYawOnly);
  return {};
}

std::error_code QuadrotorCommand::setBodyRate(
    const Eigen::Ref<const Eigen::Vector3d>& body_rate) {
  setpoint_.odometry.twist().angular() = body_rate;
  addComponent(QuadrotorStateComponent::kAngularVelocity);
  return {};
}

std::error_code QuadrotorCommand::setAcceleration(
    const Eigen::Ref<const Eigen::Vector3d>& acceleration) {
  if (!acceleration.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  setpoint_.accel.linear() = acceleration;
  addComponent(QuadrotorStateComponent::kAcceleration);
  return {};
}

std::error_code QuadrotorCommand::setAngularAcceleration(
    const Eigen::Ref<const Eigen::Vector3d>& angular_acceleration) {
  if (!angular_acceleration.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  setpoint_.accel.angular() = angular_acceleration;
  addComponent(QuadrotorStateComponent::kAngularAcceleration);
  return {};
}

std::error_code QuadrotorCommand::setForce(
    const Eigen::Ref<const Eigen::Vector3d>& force) {
  if (!force.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  setpoint_.wrench.force() = force;
  addComponent(QuadrotorStateComponent::kForce);
  return {};
}

std::error_code QuadrotorCommand::setTorque(
    const Eigen::Ref<const Eigen::Vector3d>& torque) {
  if (!torque.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  setpoint_.wrench.torque() = torque;
  addComponent(QuadrotorStateComponent::kTorque);
  return {};
}

std::error_code QuadrotorCommand::setCollectiveThrust(
    double collective_thrust) {
  if (!std::isfinite(collective_thrust)) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  setpoint_.collective_thrust = collective_thrust;
  addComponent(QuadrotorStateComponent::kCollectiveThrust);
  return {};
}

std::error_code QuadrotorCommand::setMotorThrusts(
    const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts) {
  if (!motor_thrusts.allFinite()) {
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (motor_thrusts.minCoeff() < 0.0) {
    return make_error_code(AutopilotErrc::kPhysicallyInvalid);
  }

  setpoint_.motor_thrusts = motor_thrusts;
  addComponent(QuadrotorStateComponent::kMotorThrusts);
  return {};
}

void QuadrotorCommand::reset(double timestamp_secs) {
  set_components_ = 0;
  setpoint_ = QuadrotorState();  // Reset values to zero/identity
  setpoint_.timestamp_secs = timestamp_secs;
}

}  // namespace autopilot
