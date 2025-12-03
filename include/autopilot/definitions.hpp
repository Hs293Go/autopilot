#ifndef AUTOPILOT_DEFINITION_HPP_
#define AUTOPILOT_DEFINITION_HPP_

#include <cstdint>

#include "autopilot/geometry.hpp"

namespace autopilot {

enum class QuadrotorStateComponent : std::uint32_t {
  kPosition = 1 << 0,
  kVelocity = 1 << 1,
  kOrientation = 1 << 2,
  kYawOnly = 1 << 3,
  kAngularVelocity = 1 << 4,
  kAcceleration = 1 << 5,
  kAngularAcceleration = 1 << 6,
  kForce = 1 << 7,
  kTorque = 1 << 8,
  kCollectiveThrust = 1 << 9,
  kMotorThrusts = 1 << 10,
};

struct QuadrotorState {
  double timestamp_secs = 0.0;
  OdometryF64 odometry;
  AccelF64 accel;
  WrenchF64 wrench;
  double collective_thrust = 0.0;
  Eigen::Vector4d motor_thrusts = Eigen::Vector4d::Zero();
};

class QuadrotorCommand {
 public:
  QuadrotorCommand() = default;
  QuadrotorCommand(double timestamp_secs)
      : setpoint_{.timestamp_secs = timestamp_secs,
                  .odometry = {},
                  .accel = {},
                  .wrench = {}} {}

  const QuadrotorState& setpoint() const { return setpoint_; }

  const Eigen::Vector3d& position() const {
    return setpoint_.odometry.pose().translation();
  }

  const Eigen::Quaterniond& orientation() const {
    return setpoint_.odometry.pose().rotation();
  }

  double yaw() const;

  const Eigen::Vector3d& velocity() const {
    return setpoint_.odometry.twist().linear();
  }

  const Eigen::Vector3d& bodyRate() const {
    return setpoint_.odometry.twist().angular();
  }

  const Eigen::Vector3d& acceleration() const {
    return setpoint_.accel.linear();
  }

  const Eigen::Vector3d& angularAcceleration() const {
    return setpoint_.accel.angular();
  }

  const Eigen::Vector3d& force() const { return setpoint_.wrench.force(); }

  const Eigen::Vector3d& torque() const { return setpoint_.wrench.torque(); }

  const Eigen::Vector4d& motorThrusts() const {
    return setpoint_.motor_thrusts;
  }

  void setTimestamp(double timestamp_secs) {
    setpoint_.timestamp_secs = timestamp_secs;
  }

  std::error_code setPosition(
      const Eigen::Ref<const Eigen::Vector3d>& position);

  std::error_code setVelocity(
      const Eigen::Ref<const Eigen::Vector3d>& velocity);

  std::error_code setOrientation(const Eigen::Quaterniond& orientation);

  std::error_code setYaw(double yaw);

  std::error_code setBodyRate(
      const Eigen::Ref<const Eigen::Vector3d>& body_rate);

  std::error_code setAcceleration(
      const Eigen::Ref<const Eigen::Vector3d>& acceleration);

  std::error_code setAngularAcceleration(
      const Eigen::Ref<const Eigen::Vector3d>& angular_acceleration);

  std::error_code setForce(const Eigen::Ref<const Eigen::Vector3d>& force);

  std::error_code setTorque(const Eigen::Ref<const Eigen::Vector3d>& torque);

  std::error_code setCollectiveThrust(double collective_thrust);

  std::error_code setMotorThrusts(
      const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts);

  void reset(double timestamp_secs = 0.0);

  bool hasComponent(QuadrotorStateComponent component) const {
    return (set_components_ & to_underlying(component)) != 0;
  }

 private:
  void addComponent(QuadrotorStateComponent component) {
    set_components_ |= to_underlying(component);
  }

  QuadrotorState setpoint_;
  uint32_t set_components_ = 0;
};

}  // namespace autopilot
#endif  // AUTOPILOT_DEFINITION_HPP_
