#ifndef INCLUDE_AUTOPILOT_QUADROTOR_MODEL_HPP_
#define INCLUDE_AUTOPILOT_QUADROTOR_MODEL_HPP_

#include <cmath>
#include <memory>

#include "Eigen/Dense"

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

  double mass() const { return mass_; }

  const InertiaElements& inertia_elems() const { return inertia_elems_; }

  MotorLayout motor_layout() const { return motor_layout_; }

  const Eigen::Vector2d& front_motor_position() const {
    return front_motor_position_;
  }

  const Eigen::Vector2d& back_motor_position() const {
    return back_motor_position_;
  }

  double torque_constant() const { return torque_constant_; }

  double min_collective_thrust() const { return min_collective_thrust_; }

  double max_collective_thrust() const { return max_collective_thrust_; }

  const Eigen::Vector3d& max_body_rate() const { return max_body_rate_; }

  const Eigen::Vector3d& grav_vector() const { return grav_vector_; }

  std::error_code setMass(double mass);

  std::error_code setInertiaElements(const InertiaElements& inertia_elems);

  void setMotorLayout(MotorLayout layout) { motor_layout_ = layout; }

  std::error_code setFrontMotorPosition(
      const Eigen::Ref<const Eigen::Vector2d>& position);

  std::error_code setBackMotorPosition(
      const Eigen::Ref<const Eigen::Vector2d>& position);

  std::error_code setTorqueConstant(double torque_constant);

  std::error_code setMinCollectiveThrust(double min_thrust);

  std::error_code setMaxCollectiveThrust(double max_thrust);

  std::error_code setMaxBodyRate(
      const Eigen::Ref<const Eigen::Vector3d>& max_body_rate);

  std::error_code setGravVector(
      const Eigen::Ref<const Eigen::Vector3d>& grav_vector);

 private:
  // Physical parameters
  double mass_ = 1.0;  // in kilograms
  InertiaElements inertia_elems_ = {0.005, 0.005, 0.009, 0.0, 0.0, 0.0};

  // Mixer parameters
  Eigen::Vector2d front_motor_position_ =
      Eigen::Vector2d::Constant(kMiniquad250SideLength / 2.0);
  Eigen::Vector2d back_motor_position_ =
      Eigen::Vector2d::Constant(kMiniquad250SideLength / 2.0);
  double torque_constant_ = 0.01;

  MotorLayout motor_layout_ = MotorLayout::kBetaflight;
  double min_collective_thrust_ = 0.0;
  double max_collective_thrust_ = 15.0;

  Eigen::Vector3d max_body_rate_ = {kHalfRotation, kHalfRotation,
                                    kFullRotation};

  Eigen::Vector3d grav_vector_ = Eigen::Vector3d::UnitZ() * 9.81;
};

class QuadrotorModel {
 public:
  // Constructor: Attach config and compute initial derived state
  explicit QuadrotorModel(std::shared_ptr<QuadrotorModelCfg> cfg)
      : cfg_(std::move(cfg)) {
    updateDerivedState();
  }

  double mass() const { return cfg_->mass(); }

  const Eigen::Matrix3d& inertia() const { return inertia_; }

  const Eigen::Matrix3d& invInertia() const { return inv_inertia_; }

  double min_collective_thrust() const { return cfg_->min_collective_thrust(); }

  double max_collective_thrust() const { return cfg_->max_collective_thrust(); }

  const Eigen::Vector3d& grav_vector() const { return cfg_->grav_vector(); }

  [[nodiscard]] Eigen::Vector4d momentsToMotorThrusts(
      const Eigen::Ref<const Eigen::Vector4d>& moments) const;

  Eigen::Vector4d motorThrustsToMoments(
      const Eigen::Ref<const Eigen::Vector4d>& motor_thrusts) const;

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

#endif  // INCLUDE_AUTOPILOT_QUADROTOR_MODEL_HPP_
