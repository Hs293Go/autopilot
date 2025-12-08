#ifndef AUTOPILOT_QUADROTOR_MODEL_HPP_
#define AUTOPILOT_QUADROTOR_MODEL_HPP_

#include <cmath>
#include <memory>

#include "Eigen/Dense"
#include "autopilot/geometry.hpp"

namespace autopilot {

static constexpr double kMiniquad250SideLength = 0.25 * std::numbers::sqrt2;
static constexpr double kHalfRotation = 2.0 * std::numbers::pi;
static constexpr double kFullRotation = 2.0 * std::numbers::pi;

static constexpr double kDefaultRollPitchInertia = 0.005;
static constexpr double kDefaultYawInertia = 0.009;

struct InertiaElements {
  double ixx = kDefaultRollPitchInertia;
  double iyy = kDefaultRollPitchInertia;
  double izz = kDefaultYawInertia;
  double ixy = 0.0;
  double ixz = 0.0;
  double iyz = 0.0;
};

enum class MotorLayout {
  kBetaflight,
  kPx4,
};

class QuadrotorModelCfg {
 public:
  QuadrotorModelCfg() = default;

  [[nodiscard]] double mass() const { return mass_; }

  [[nodiscard]] const InertiaElements& inertia_elems() const {
    return inertia_elems_;
  }

  [[nodiscard]] MotorLayout motor_layout() const { return motor_layout_; }

  [[nodiscard]] const Eigen::Vector2d& front_motor_position() const {
    return front_motor_position_;
  }

  [[nodiscard]] const Eigen::Vector2d& back_motor_position() const {
    return back_motor_position_;
  }

  [[nodiscard]] double torque_constant() const { return torque_constant_; }

  [[nodiscard]] double motor_time_constant_up() const {
    return motor_time_constant_up_;
  }
  [[nodiscard]] double motor_time_constant_down() const {
    return motor_time_constant_down_;
  }
  [[nodiscard]] double thrust_curve_coeff() const {
    return thrust_curve_coeff_;
  }

  [[nodiscard]] double min_collective_thrust() const {
    return min_collective_thrust_;
  }

  [[nodiscard]] double max_collective_thrust() const {
    return max_collective_thrust_;
  }

  [[nodiscard]] const Eigen::Vector3d& max_body_rate() const {
    return max_body_rate_;
  }

  [[nodiscard]] const Eigen::Vector3d& grav_vector() const {
    return grav_vector_;
  }

  std::error_code setMass(double mass);

  std::error_code setInertiaElements(const InertiaElements& inertia_elems);

  void setMotorLayout(MotorLayout layout) { motor_layout_ = layout; }

  std::error_code setFrontMotorPosition(double x, double y) {
    return setFrontMotorPosition(Eigen::Vector2d{x, y});
  }

  std::error_code setFrontMotorPosition(
      const Eigen::Ref<const Eigen::Vector2d>& position);

  std::error_code setBackMotorPosition(double x, double y) {
    return setBackMotorPosition(Eigen::Vector2d{x, y});
  }

  std::error_code setBackMotorPosition(
      const Eigen::Ref<const Eigen::Vector2d>& position);

  std::error_code setTorqueConstant(double torque_constant);

  std::error_code setMotorTimeConstantUp(double time_constant_up);

  std::error_code setMotorTimeConstantDown(double time_constant_down);

  std::error_code setThrustCurveCoeff(double thrust_curve_coeff);

  std::error_code setMinCollectiveThrust(double min_thrust);

  std::error_code setMaxCollectiveThrust(double max_thrust);

  std::error_code setMaxBodyRate(
      const Eigen::Ref<const Eigen::Vector3d>& max_body_rate);

  std::error_code setGravVector(
      const Eigen::Ref<const Eigen::Vector3d>& grav_vector);

  std::error_code setGravAcceleration(double grav_accel_magnitude) {
    return setGravVector(Eigen::Vector3d::UnitZ() * grav_accel_magnitude);
  }

 private:
  // Physical parameters
  double mass_ = 1.0;  // in kilograms
  InertiaElements inertia_elems_ = {0.005, 0.005, 0.009, 0.0, 0.0, 0.0};

  // Mixer parameters
  MotorLayout motor_layout_ = MotorLayout::kBetaflight;
  Eigen::Vector2d front_motor_position_ =
      Eigen::Vector2d::Constant(kMiniquad250SideLength / 2.0);
  Eigen::Vector2d back_motor_position_ =
      Eigen::Vector2d::Constant(kMiniquad250SideLength / 2.0);
  double torque_constant_ = 0.01;

  double motor_time_constant_up_ = 0.02;    // seconds
  double motor_time_constant_down_ = 0.02;  // seconds
  double thrust_curve_coeff_ = 8.54858e-6;  // N/(rad/s)^2

  double min_collective_thrust_ = 0.0;
  double max_collective_thrust_ = 15.0;

  Eigen::Vector3d max_body_rate_ = {kHalfRotation, kHalfRotation,
                                    kFullRotation};

  Eigen::Vector3d grav_vector_ = -Eigen::Vector3d::UnitZ() * 9.81;
};

struct ThrustTorque {
  double collective_thrust;
  Eigen::Vector3d torque;
};

class QuadrotorModel {
 public:
  // Constructor: Attach config and compute initial derived state
  explicit QuadrotorModel(std::shared_ptr<QuadrotorModelCfg> cfg)
      : cfg_(std::move(cfg)) {
    updateDerivedState();
  }

  [[nodiscard]] double mass() const { return cfg_->mass(); }

  [[nodiscard]] const Eigen::Matrix3d& inertia() const { return inertia_; }

  [[nodiscard]] const Eigen::Matrix3d& invInertia() const {
    return inv_inertia_;
  }

  [[nodiscard]] double motor_time_constant_up() const {
    return cfg_->motor_time_constant_up();
  }
  [[nodiscard]] double motor_time_constant_down() const {
    return cfg_->motor_time_constant_down();
  }
  [[nodiscard]] double thrust_curve_coeff() const {
    return cfg_->thrust_curve_coeff();
  }

  [[nodiscard]] double min_collective_thrust() const {
    return cfg_->min_collective_thrust();
  }

  [[nodiscard]] double max_collective_thrust() const {
    return cfg_->max_collective_thrust();
  }

  [[nodiscard]] const Eigen::Vector3d& max_body_rate() const {
    return cfg_->max_body_rate();
  }

  [[nodiscard]] const Eigen::Vector3d& grav_vector() const {
    return cfg_->grav_vector();
  }

  [[nodiscard]] Eigen::Vector4d thrustTorqueToMotorThrusts(
      const Eigen::Ref<const Eigen::Vector4d>& moments) const;

  [[nodiscard]] Eigen::Vector4d thrustTorqueToMotorThrusts(
      const ThrustTorque& thrust_torque) const;

  [[nodiscard]] Eigen::Vector4d motorThrustsToThrustTorqueVector(
      const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts) const;

  [[nodiscard]] ThrustTorque motorThrustsToThrustTorque(
      const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts) const;

  [[nodiscard]] typename OdometryF64::ParamVector rigidBodyDynamics(
      const Eigen::Ref<const typename OdometryF64::ParamVector>& state,
      const Eigen::Ref<const typename WrenchF64::ParamVector>& input) const;

  [[nodiscard]] OdometryF64 rigidBodyDynamics(const OdometryF64& state,
                                              const WrenchF64& input) const;

  [[nodiscard]] Eigen::Vector4d motorSpeedDynamics(
      const Eigen::Ref<const Eigen::Vector4d>& motor_speeds_curr,
      const Eigen::Ref<const Eigen::Vector4d>& motor_speeds_sp) const;

 private:
  void updateDerivedState();

  void updateInertia();

  void updateMixerMatrix();

  std::shared_ptr<QuadrotorModelCfg> cfg_;
  Eigen::Matrix3d inertia_;
  Eigen::Matrix3d inv_inertia_;

  Eigen::Matrix4d allocation_matrix_;
  Eigen::Matrix4d inv_allocation_matrix_;
};
}  // namespace autopilot

#endif  // AUTOPILOT_QUADROTOR_MODEL_HPP_
