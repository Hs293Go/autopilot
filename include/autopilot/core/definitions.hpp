#ifndef AUTOPILOT_DEFINITION_HPP_
#define AUTOPILOT_DEFINITION_HPP_

#include <cstdint>
#include <ranges>
#include <variant>

#include "autopilot/core/geometry.hpp"
#include "fmt/ranges.h"  // IWYU pragma: keep

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
  kSentinel
};

inline constexpr uint32_t kNumQuadrotorStateComponents =
    std::countr_zero(std::to_underlying(QuadrotorStateComponent::kSentinel) -
                     1) +
    1;

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

  explicit QuadrotorCommand(const QuadrotorState& setpoint)
      : setpoint_(setpoint) {}

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

  AutopilotErrc setPosition(const Eigen::Ref<const Eigen::Vector3d>& position);

  AutopilotErrc setVelocity(const Eigen::Ref<const Eigen::Vector3d>& velocity);

  AutopilotErrc setOrientation(const Eigen::Quaterniond& orientation);

  AutopilotErrc setYaw(double yaw);

  AutopilotErrc setBodyRate(const Eigen::Ref<const Eigen::Vector3d>& body_rate);

  AutopilotErrc setAcceleration(
      const Eigen::Ref<const Eigen::Vector3d>& acceleration);

  AutopilotErrc setAngularAcceleration(
      const Eigen::Ref<const Eigen::Vector3d>& angular_acceleration);

  AutopilotErrc setForce(const Eigen::Ref<const Eigen::Vector3d>& force);

  AutopilotErrc setTorque(const Eigen::Ref<const Eigen::Vector3d>& torque);

  AutopilotErrc setCollectiveThrust(double collective_thrust);

  AutopilotErrc setMotorThrusts(
      const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts);

  void reset(double timestamp_secs = 0.0);

  bool hasComponent(QuadrotorStateComponent component) const {
    return (set_components_ & std::to_underlying(component)) != 0;
  }

  uint32_t components() const { return set_components_; }

 private:
  void addComponent(QuadrotorStateComponent component) {
    set_components_ |= std::to_underlying(component);
  }

  QuadrotorState setpoint_;
  uint32_t set_components_ = 0;
};

struct FollowVelocity {};

struct Fixed {
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double yaw_acceleration = 0.0;
};

struct PointOfInterest {
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double focus_radius = 0.1;  // Below 10cm, freeze yaw to avoid singularity
};

using HeadingPolicy = std::variant<FollowVelocity, Fixed, PointOfInterest>;

struct KinematicState {
  double timestamp_secs = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  Eigen::Vector3d snap = Eigen::Vector3d::Zero();

  double yaw = 0.0;
  double yaw_rate = 0.0;
  double yaw_acceleration = 0.0;

  auto translational() const {
    return std::tie(position, velocity, acceleration, jerk, snap);
  }
  auto rotational() const { return std::tie(yaw, yaw_rate, yaw_acceleration); }

  QuadrotorState toQuadrotorState() const {
    QuadrotorState state;
    state.timestamp_secs = timestamp_secs;
    state.odometry.pose().translation() = position;
    state.odometry.twist().linear() = velocity;
    state.accel.linear() = acceleration;
    return state;
  }
};

struct TrajectoryWaypoint {
  double time_from_start_secs = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::optional<Eigen::Vector3d> velocity = std::nullopt;
  std::optional<Eigen::Vector3d> acceleration = std::nullopt;
  std::optional<double> yaw = std::nullopt;
};
}  // namespace autopilot

// Formatters for QuadrotorState and QuadrotorCommand.
// These are defined in terms of formatters for geometry types, defined here to
// keep the core/geometry.hpp clean.
namespace fmt {

struct NoSpecifierFormatter {
  constexpr auto parse(format_parse_context& ctx) const { return ctx.begin(); }
};

template <>
struct formatter<autopilot::QuadrotorStateComponent> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::QuadrotorStateComponent& comp,
              FormatContext& ctx) const {
    switch (comp) {
      case autopilot::QuadrotorStateComponent::kPosition:
        return ::fmt::format_to(ctx.out(), "Position");
      case autopilot::QuadrotorStateComponent::kVelocity:
        return ::fmt::format_to(ctx.out(), "Velocity");
      case autopilot::QuadrotorStateComponent::kOrientation:
        return ::fmt::format_to(ctx.out(), "Orientation");
      case autopilot::QuadrotorStateComponent::kYawOnly:
        return ::fmt::format_to(ctx.out(), "YawOnly");
      case autopilot::QuadrotorStateComponent::kAngularVelocity:
        return ::fmt::format_to(ctx.out(), "AngularVelocity");
      case autopilot::QuadrotorStateComponent::kAcceleration:
        return ::fmt::format_to(ctx.out(), "Acceleration");
      case autopilot::QuadrotorStateComponent::kAngularAcceleration:
        return ::fmt::format_to(ctx.out(), "AngularAcceleration");
      case autopilot::QuadrotorStateComponent::kForce:
        return ::fmt::format_to(ctx.out(), "Force");
      case autopilot::QuadrotorStateComponent::kTorque:
        return ::fmt::format_to(ctx.out(), "Torque");
      case autopilot::QuadrotorStateComponent::kCollectiveThrust:
        return ::fmt::format_to(ctx.out(), "CollectiveThrust");
      case autopilot::QuadrotorStateComponent::kMotorThrusts:
        return ::fmt::format_to(ctx.out(), "MotorThrusts");
      default:
        std::unreachable();
    }
  }
};

template <typename T>
struct formatter<autopilot::Transform<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Transform<T>& tf, FormatContext& ctx) const {
    return ::fmt::format_to(ctx.out(),
                            "Transform(Translation: {}, Rotation: {})",
                            tf.translation(), tf.rotation().coeffs());
  }
};

template <typename T>
struct formatter<autopilot::Twist<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Twist<T>& twist, FormatContext& ctx) const {
    return ::fmt::format_to(ctx.out(), "Twist(Linear: {}, Angular: {})",
                            twist.linear(), twist.angular());
  }
};

template <typename T>
struct formatter<autopilot::Odometry<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Odometry<T>& odom, FormatContext& ctx) const {
    return ::fmt::format_to(ctx.out(), "Odometry(Transform: {}, Twist: {})",
                            odom.pose(), odom.twist());
  }
};

template <typename T>
struct formatter<autopilot::Accel<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Accel<T>& accel, FormatContext& ctx) const {
    return ::fmt::format_to(ctx.out(), "Accel(Linear: {}, Angular: {})",
                            accel.linear(), accel.angular());
  }
};

template <typename T>
struct formatter<autopilot::Wrench<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::Wrench<T>& wrench, FormatContext& ctx) const {
    return ::fmt::format_to(ctx.out(), "Wrench(Force: {}, Torque: {})",
                            wrench.force(), wrench.torque());
  }
};

template <>
struct formatter<autopilot::QuadrotorState> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::QuadrotorState& comp, FormatContext& ctx) const {
    return ::fmt::format_to(ctx.out(),
                            "QuadrotorState(Timestamp: {}, {}, "
                            "{}, {}, "
                            "Collective Thrust: {}, Motor Thrusts: {})",
                            comp.timestamp_secs, comp.odometry, comp.accel,
                            comp.wrench, comp.collective_thrust,
                            comp.motor_thrusts);
  }
};

template <>
struct formatter<autopilot::QuadrotorCommand> : NoSpecifierFormatter {
  template <typename FormatContext>
  auto format(const autopilot::QuadrotorCommand& cmd,
              FormatContext& ctx) const {
    auto set_components_view =
        std::views::iota(0u, autopilot::kNumQuadrotorStateComponents) |
        std::views::filter([mask = cmd.components()](int i) {
          return (mask & (1u << i)) != 0;
        }) |
        std::views::transform([](int i) {
          return static_cast<autopilot::QuadrotorStateComponent>(1u << i);
        });
    return ::fmt::format_to(
        ctx.out(), "QuadrotorCommand(Setpoint: {}, Components Set: {})",
        cmd.setpoint(), std::move(set_components_view));
  }
};
}  // namespace fmt

static_assert(fmt::is_formattable<autopilot::QuadrotorState>::value);
static_assert(fmt::is_formattable<autopilot::QuadrotorCommand>::value);
#endif  // AUTOPILOT_DEFINITION_HPP_
