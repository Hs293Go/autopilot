#include <random>
#include <ranges>

#include "autopilot/controllers/attitude_error.hpp"
#include "autopilot/core/math.hpp"
#include "gtest/gtest.h"
#include "testing/matchers.hpp"

namespace {
Eigen::Quaterniond GenerateRandomRotation(double fixed_angle_rad,
                                          Eigen::Vector3d* axis_out = nullptr) {
  // 1. Setup random number engine
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::normal_distribution<double> dist(0.0, 1.0);

  // 2. Generate a random unit axis
  // Normal distribution for each component ensures uniform distribution on the
  // sphere
  Eigen::Vector3d axis(dist(gen), dist(gen), dist(gen));
  axis.normalize();
  if (axis_out != nullptr) {
    *axis_out = axis;
  }

  // 3. Create Angle-Axis and convert to Quaternion
  return autopilot::AngleAxisToQuaternion(fixed_angle_rad * axis);
}
}  // namespace

static constexpr int kNumTrials = 1000;

static constexpr auto kAttitudeErrorLaws = {
    autopilot::AttitudeErrorLaw::kGeometricSO3,
    autopilot::AttitudeErrorLaw::kQuaternionBased,
    autopilot::AttitudeErrorLaw::kTiltPrioritizing,
};

TEST(GeneralAttitudeErrorTest, ZeroRotation) {
  Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q2 = Eigen::Quaterniond::Identity();

  for (auto error_law : kAttitudeErrorLaws) {
    Eigen::Vector3d error = autopilot::EvaluateAttitudeError(q1, q2, error_law);
    EXPECT_TRUE(error.isZero(1e-10));
  }
}

TEST(GeneralAttitudeErrorTest, GeometricSO3At90Deg) {
  for (auto error_law : kAttitudeErrorLaws | std::views::take(1)) {
    for (int i = 0; i < kNumTrials; ++i) {
      Eigen::Quaterniond q1 = Eigen::Quaterniond::UnitRandom();
      Eigen::Vector3d axis;
      Eigen::Quaterniond q2 =
          q1 * GenerateRandomRotation(std::numbers::pi / 2, &axis);

      Eigen::Vector3d error =
          autopilot::EvaluateAttitudeError(q1, q2, error_law);

      switch (error_law) {
        case autopilot::AttitudeErrorLaw::kGeometricSO3:
          ASSERT_THAT(error, AngleAxisIsClose(-axis));
          break;
        case autopilot::AttitudeErrorLaw::kQuaternionBased:
          ASSERT_THAT(error, AngleAxisIsClose(-std::numbers::sqrt2 * axis));
          break;
        case autopilot::AttitudeErrorLaw::kTiltPrioritizing:
          ASSERT_TRUE(false)
              << "Programming error: Tilt-prioritizing should not be "
                 "tested in this loop.";
      }
    }
  }
}

TEST(GeneralAttitudeErrorTest, GeometricSO3At180Deg) {
  for (auto error_law : kAttitudeErrorLaws | std::views::take(2)) {
    for (int i = 0; i < kNumTrials; ++i) {
      Eigen::Quaterniond q1 = Eigen::Quaterniond::UnitRandom();
      Eigen::Vector3d axis;
      Eigen::Quaterniond q2 =
          q1 * GenerateRandomRotation(std::numbers::pi, &axis);

      Eigen::Vector3d error =
          autopilot::EvaluateAttitudeError(q1, q2, error_law);
      switch (error_law) {
        case autopilot::AttitudeErrorLaw::kGeometricSO3:
          EXPECT_TRUE(error.isZero(1e-10));
          break;
        case autopilot::AttitudeErrorLaw::kQuaternionBased:
          ASSERT_THAT(error, IsCloseOrFlipped(-2.0 * axis, 2.0));
          break;
        case autopilot::AttitudeErrorLaw::kTiltPrioritizing:
          ASSERT_TRUE(false)
              << "Programming error: Tilt-prioritizing should not be "
                 "tested in this loop.";
      }
    }
  }
}

TEST(TestTiltPrioritizingAttitudeError, PureTiltRotations) {
  for (int i = 0; i < kNumTrials; ++i) {
    Eigen::Quaterniond q1 = Eigen::Quaterniond::UnitRandom();
    Eigen::Vector3d axis;
    axis << Eigen::Vector2d::Random().normalized(), 0.0;
    Eigen::Quaterniond q2 = q1 * autopilot::AngleAxisToQuaternion(axis);
    Eigen::Vector3d error = autopilot::EvaluateAttitudeError(
        q1, q2, autopilot::AttitudeErrorLaw::kTiltPrioritizing);
    ASSERT_NEAR(error.z(), 0.0, 1e-8);
  }
}

TEST(TestTiltPrioritizingAttitudeError, RegularRotations) {
  for (int i = 0; i < kNumTrials; ++i) {
    Eigen::Quaterniond q1 = Eigen::Quaterniond::UnitRandom();
    Eigen::Quaterniond q2 =
        q1 * Eigen::Quaterniond::UnitRandom();  // Full random rotation
    Eigen::Vector3d error = autopilot::EvaluateAttitudeError(
        q1, q2, autopilot::AttitudeErrorLaw::kTiltPrioritizing);

    Eigen::Quaterniond rot_diff = q1.inverse() * q2;
    ASSERT_THAT(rot_diff, QuaternionIsClose(
                              autopilot::AngleAxisToQuaternion(Eigen::Vector3d(
                                  -error.x(), -error.y(), 0.0)) *
                              autopilot::AngleAxisToQuaternion(
                                  Eigen::Vector3d::UnitZ() * -error.z())));
  }
}
