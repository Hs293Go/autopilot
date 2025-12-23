#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "autopilot/core/definitions.hpp"
#include "autopilot/core/quadrotor_model.hpp"
#include "testing/matchers.hpp"

namespace ap = autopilot;

TEST(TestQuadrotorModelConfig, InvalidParameters) {
  // Test invalid configurations for QuadrotorModelCfg

  auto cfg = std::make_shared<ap::QuadrotorModelCfg>();

  // 1. Negative mass
  EXPECT_THAT(cfg->setMass(-1.0), IsNonEmptyErrorCode());

  // 2. Zero inertia elements
  ap::InertiaElements zero_inertia{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_THAT(cfg->setInertiaElements(zero_inertia), IsNonEmptyErrorCode());

  // 3. Invalid motor positions (e.g., negative arm length)
  EXPECT_THAT(cfg->setFrontMotorPosition(-0.1, 0.1), IsNonEmptyErrorCode());
  EXPECT_THAT(cfg->setBackMotorPosition(0.1, -0.1), IsNonEmptyErrorCode());

  // 4. Negative torque constant
  EXPECT_THAT(cfg->setTorqueConstant(-0.01), IsNonEmptyErrorCode());

  // 5. Invalid gravity vector (zero vector)
  EXPECT_THAT(cfg->setGravVector(Eigen::Vector3d::Zero()),
              IsNonEmptyErrorCode());

  // 6. Negative motor time constants
  EXPECT_THAT(cfg->setMotorTimeConstantUp(-0.05), IsNonEmptyErrorCode());
  EXPECT_THAT(cfg->setMotorTimeConstantDown(-0.05), IsNonEmptyErrorCode());

  // 7. Negative collective thrust bounds
  EXPECT_THAT(cfg->setMaxCollectiveThrust(-10.0), IsNonEmptyErrorCode());
  EXPECT_THAT(cfg->setMinCollectiveThrust(-5.0), IsNonEmptyErrorCode());

  // This is valid
  EXPECT_THAT(cfg->setMinCollectiveThrust(1.0), IsEmptyErrorCode());

  // Min thrust greater than max thrust
  EXPECT_THAT(cfg->setMaxCollectiveThrust(0.5), IsNonEmptyErrorCode());
}

class TestQuadrotorModel : public ::testing::Test {
 protected:
  void SetUp() override {
    // 1. Create a configuration with known parameters
    auto cfg = std::make_shared<ap::QuadrotorModelCfg>();

    // Physical parameters
    ASSERT_EQ(cfg->setMass(kMass), std::error_code());
    ASSERT_EQ(cfg->setInertiaElements(inertia_), std::error_code());

    // Motor layout parameters (Standard X configuration)
    // Front right (0), Back left (1), Front left (2), Back right (3) - or
    // similar depending on layout
    ASSERT_EQ(cfg->setFrontMotorPosition(kArmLength, kArmLength),
              std::error_code());
    ASSERT_EQ(cfg->setBackMotorPosition(kArmLength, kArmLength),
              std::error_code());
    ASSERT_EQ(cfg->setTorqueConstant(kTorqueConstant), std::error_code());

    // Set Gravity to standard -9.81 Z
    ASSERT_EQ(cfg->setGravVector(-Eigen::Vector3d::UnitZ() * kGravity),
              std::error_code());

    model_ = std::make_shared<ap::QuadrotorModel>(cfg);
  }

  // Constants for testing
  static constexpr double kMass = 1.5;             // kg
  static constexpr double kArmLength = 0.25;       // m
  static constexpr double kTorqueConstant = 0.02;  // Nm/N
  static constexpr double kGravity = 9.81;         // m/s^2

  // Diagonal inertia matrix
  ap::InertiaElements inertia_{0.03, 0.03, 0.05, 0.0, 0.0, 0.0};

  std::shared_ptr<ap::QuadrotorModel> model_;
};

TEST_F(TestQuadrotorModel, MixerMatrixInversion) {
  // Test 1: Verify Wrench -> Motor Thrusts -> Wrench round trip
  // This confirms the allocation matrix inversion is correct.

  // Create a random valid wrench
  // Note: Quadrotors can't generate arbitrary wrenches (e.g. lateral force),
  // but the model's mixer usually solves for [Thrust, Tx, Ty, Tz].
  // We simulate the output of a controller asking for specific moments and
  // thrust.

  struct TestCase {
    double collective_thrust;
    Eigen::Vector3d torque;
  };

  std::vector<TestCase> cases = {
      {10.0, {0.0, 0.0, 0.0}},   // Pure hover
      {20.0, {0.5, -0.5, 0.1}},  // Mixed maneuver
      {5.0, {0.0, 0.0, 1.0}},    // Pure yaw
      {15.0, {1.0, 1.0, 0.0}}    // Pure pitch/roll
  };

  for (const auto& test_case : cases) {
    Eigen::Vector4d input_moments;
    input_moments(0) = test_case.collective_thrust;
    input_moments.tail<3>() = test_case.torque;

    // 1. Inverse Mixing: Wrench -> Motor Thrusts
    Eigen::Vector4d motor_thrusts =
        model_->thrustTorqueToMotorThrusts(input_moments);

    // 2. Forward Mixing: Motor Thrusts -> Wrench
    Eigen::Vector4d recovered_moments =
        model_->motorThrustsToThrustTorqueVector(motor_thrusts);

    // 3. Verify
    EXPECT_TRUE(input_moments.isApprox(recovered_moments, 1e-9))
        << "Mixer round-trip failed.\nInput:\n"
        << input_moments.transpose() << "\nRecovered:\n"
        << recovered_moments.transpose();

    // Also verify the struct-based helper
    ap::ThrustTorque tt = model_->motorThrustsToThrustTorque(motor_thrusts);
    EXPECT_NEAR(tt.collective_thrust, test_case.collective_thrust, 1e-9);
    EXPECT_TRUE(tt.torque.isApprox(test_case.torque, 1e-9));
  }
}

TEST_F(TestQuadrotorModel, DynamicsSanityHover) {
  // Test 2: Hover Condition
  // If the drone is flat and applying force = mass * gravity UP, acceleration
  // should be zero.

  ap::OdometryF64 state;
  state.pose().translation().setZero();
  state.pose().rotation().setIdentity();  // Aligned with world frame
  state.twist().linear().setZero();
  state.twist().angular().setZero();

  // Force required to hover: F = m * g
  // In body frame (which is aligned with world), this is +Z direction to
  // counteract -Z gravity
  const double hover_thrust = kMass * kGravity;

  ap::WrenchF64 input;
  input.force() = Eigen::Vector3d(0.0, 0.0, hover_thrust);
  input.torque().setZero();

  // Compute Dynamics
  ap::OdometryF64 derivative = model_->rigidBodyDynamics(state, input);

  // Assertions
  // 1. Linear Acceleration (v_dot) should be zero
  // derivative.twist().linear() corresponds to linear acceleration in the state
  // derivative
  EXPECT_TRUE(derivative.twist().linear().isZero(1e-9))
      << "Hover acceleration is not zero: "
      << derivative.twist().linear().transpose();

  // 2. Angular Acceleration (w_dot) should be zero (no torque)
  EXPECT_TRUE(derivative.twist().angular().isZero(1e-9));

  // 3. Velocity (p_dot) should be zero (initial velocity was zero)
  EXPECT_TRUE(derivative.pose().translation().isZero(1e-9));
}

TEST_F(TestQuadrotorModel, DynamicsSanityFreeFall) {
  // Test 3: Free Fall Condition
  // If inputs are zero, the drone should accelerate downwards at exactly
  // gravity.

  ap::OdometryF64 state;
  state.pose().translation().setZero();
  state.pose().rotation().setIdentity();
  state.twist().linear().setZero();
  state.twist().angular().setZero();

  ap::WrenchF64 input;
  input.force().setZero();
  input.torque().setZero();

  // Compute Dynamics
  ap::OdometryF64 derivative = model_->rigidBodyDynamics(state, input);

  // Assertions
  // 1. Linear Acceleration should be exactly gravity vector
  const Eigen::Vector3d expected_accel = model_->grav_vector();
  EXPECT_TRUE(derivative.twist().linear().isApprox(expected_accel, 1e-9))
      << "Freefall acceleration mismatch.\nExpected: "
      << expected_accel.transpose()
      << "\nActual: " << derivative.twist().linear().transpose();
}

TEST_F(TestQuadrotorModel, DynamicsAngularMomentum) {
  // Test 4: Pure Torque application
  // Apply torque around X, check angular acceleration.
  // alpha = I_inv * (Torque - w x I w)
  // At w=0, alpha = I_inv * Torque

  ap::OdometryF64 state;
  state.pose().rotation().setIdentity();
  state.twist().angular().setZero();  // No gyroscopic terms

  ap::WrenchF64 input;
  input.force().setZero();
  input.torque() = Eigen::Vector3d(0.1, 0.0, 0.0);  // 0.1 Nm around X

  ap::OdometryF64 derivative = model_->rigidBodyDynamics(state, input);

  // Ixx = 0.03. alpha_x = 0.1 / 0.03 = 3.333...
  const double expected_alpha_x = 0.1 / inertia_.ixx;

  EXPECT_NEAR(derivative.twist().angular().x(), expected_alpha_x, 1e-9);
  EXPECT_NEAR(derivative.twist().angular().y(), 0.0, 1e-9);
  EXPECT_NEAR(derivative.twist().angular().z(), 0.0, 1e-9);
}
