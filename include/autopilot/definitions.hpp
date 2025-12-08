#ifndef AUTOPILOT_DEFINITION_HPP_
#define AUTOPILOT_DEFINITION_HPP_

#include <cstdint>

#include "autopilot/geometry.hpp"
#if __has_include(<spdlog/fmt/ranges.h>)
#include "spdlog/fmt/ranges.h"
#else
#if !SPDLOG_FMT_EXTERNAL
#error Including fmt/ranges.h directly is supported only on Ubuntu 22.04, whose spdlog from libspdlog-dev package uses external fmt library.
#endif
#include "fmt/ranges.h"  // IWYU pragma: keep
#endif

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

// Formatters for QuadrotorState and QuadrotorCommand.
// These are defined in terms of formatters for geometry types, defined here to
// keep the geometry.hpp clean.
namespace fmt {

struct NoSpecifierFormatter {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
};

template <std::floating_point T>
struct formatter<autopilot::Transform<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Transform<T>& tf, FormatContext& ctx) {
    return format_to(ctx.out(), "Transform(Translation: {}, Rotation: {})",
                     tf.translation(), tf.rotation().coeffs());
  }
};

template <std::floating_point T>
struct formatter<autopilot::Twist<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Twist<T>& twist, FormatContext& ctx) {
    return format_to(ctx.out(), "Twist(Linear: {}, Angular: {})",
                     twist.linear(), twist.angular());
  }
};

template <std::floating_point T>
struct formatter<autopilot::Odometry<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Odometry<T>& odom, FormatContext& ctx) {
    return format_to(ctx.out(), "Odometry(Transform: {}, Twist: {})",
                     odom.pose(), odom.twist());
  }
};

template <std::floating_point T>
struct formatter<autopilot::Accel<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Accel<T>& accel, FormatContext& ctx) {
    return format_to(ctx.out(), "Accel(Linear: {}, Angular: {})",
                     accel.linear(), accel.angular());
  }
};

template <std::floating_point T>
struct formatter<autopilot::Wrench<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Wrench<T>& wrench, FormatContext& ctx) {
    return format_to(ctx.out(), "Wrench(Force: {}, Torque: {})", wrench.force(),
                     wrench.torque());
  }
};

template <>
struct formatter<autopilot::QuadrotorState> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::QuadrotorState& comp, FormatContext& ctx) {
    return format_to(ctx.out(),
                     "QuadrotorState(Timestamp: {}, Odometry: {}, "
                     "Accelerations: {}, Wrench: {}, "
                     "Collective Thrust: {}, Motor Thrusts: {})",
                     comp.timestamp_secs, comp.odometry, comp.accel,
                     comp.wrench, comp.collective_thrust, comp.motor_thrusts);
  }
};

template <>
struct formatter<autopilot::QuadrotorCommand> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::QuadrotorCommand& cmd, FormatContext& ctx) {
    return format_to(ctx.out(), "QuadrotorCommand(Setpoint: {})",
                     cmd.setpoint());
  }
};
}  // namespace fmt

static_assert(fmt::is_formattable<autopilot::QuadrotorState>::value);
static_assert(fmt::is_formattable<autopilot::QuadrotorCommand>::value);
#endif  // AUTOPILOT_DEFINITION_HPP_
