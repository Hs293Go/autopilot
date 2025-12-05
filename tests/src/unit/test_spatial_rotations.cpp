
#include <limits>
#include <random>

#include "Eigen/Dense"
#include "autopilot/math.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace ap = autopilot;

namespace test_utils {
class FixtureWithRNG : public ::testing::Test {
 public:
  using ::testing::Test::Test;

  std::mt19937& rng() { return rng_; }

  double random() { return dist_(rng_); }

  double random(double low, double high) {
    using ParamType = std::uniform_real_distribution<double>::param_type;
    return dist_(rng_, ParamType{low, high});
  }

  Eigen::Vector3d randomVector() {
    return Eigen::Vector3d::NullaryExpr([this] { return random(); });
  }

  Eigen::Vector3d randomUnitVector() { return randomVector().normalized(); }

 private:
  std::random_device dev_;
  std::mt19937 rng_{dev_()};
  std::uniform_real_distribution<> dist_{-1.0, 1.0};
};

}  // namespace test_utils

inline constexpr double kSmallAngle = 1e-2;
static inline const double kTinyAngle =
    std::pow(std::numeric_limits<double>::min(), 0.75);

static constexpr auto kNumTrials = 1000;

using testing::DoubleNear;
using testing::Pointwise;

MATCHER_P(QuaternionIsClose, expected, ::testing::PrintToString(expected)) {
  return ap::IsClose(arg.angularDistance(expected), 0.0);
}

MATCHER_P(AllClose, expected, "") {
  // reshaped() (ravel) arguments so that Eigen iterate through them
  // element-wise
  return ExplainMatchResult(Pointwise(DoubleNear(1e-6), expected.reshaped()),
                            arg.reshaped(), result_listener);
}

MATCHER_P(AngleAxisIsClose, expected, "") {
  return ExplainMatchResult(
      testing::Conditional(
          ap::IsClose(arg.norm(), std::numbers::pi),
          testing::AnyOf(AllClose(-expected), AllClose(expected)),
          AllClose(expected)),
      arg, result_listener);
}

MATCHER(IsOrthogonal,
        "Matrix must be orthogonal, i.e. itself multiplied by its transpose is "
        "equal to the identity matrix") {
  return ((arg * arg.transpose()).isIdentity(1e-8) &&
          (arg.transpose() * arg).isIdentity(1e-8));
}

MATCHER(IsNormalized, "Expected unit norm") {
  return ap::IsClose(arg.norm(), 1.0);
}

class TestRotation : public test_utils::FixtureWithRNG {
 public:
  using FixtureWithRNG::FixtureWithRNG;
};

TEST_F(TestRotation, ZeroAngleAxisToQuaternion) {
  const Eigen::Vector3d axis_angle = Eigen::Vector3d::Zero();

  const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(axis_angle);
  const Eigen::Quaterniond expected = Eigen::Quaterniond::Identity();
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expected));
}

TEST_F(TestRotation, TinyAngleAxisToQuaternion) {
  // Very small value that could potentially cause underflow.
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Vector3d axis_angle = kTinyAngle * axis;

  const Eigen::Quaterniond expected(Eigen::AngleAxisd(kTinyAngle, axis));
  const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(axis_angle);
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expected));
}

TEST_F(TestRotation, SmallAngleAxisToQuaternion) {
  // Small, finite value to test.
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Vector3d axis_angle = kSmallAngle * axis;

  const Eigen::Quaterniond expected(Eigen::AngleAxisd(kSmallAngle, axis));
  const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(axis_angle);
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expected));
}

TEST_F(TestRotation, AngleAxisToQuaternion) {
  for (int i = 0; i < 3; ++i) {
    auto axis = Eigen::Vector3d::Unit(i);
    auto axis_angle = std::numbers::pi * axis / 2.0;

    Eigen::Quaterniond expected;
    expected.w() = std::numbers::sqrt2 / 2.0;
    expected.vec() = std::numbers::sqrt2 / 2.0 * axis;
    const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(axis_angle);
    ASSERT_THAT(result, IsNormalized());
    ASSERT_THAT(result, QuaternionIsClose(expected));
  }
}

// Transforms a unit quaternion to an axis angle.
TEST_F(TestRotation, UnitQuaternionToAngleAxis) {
  const Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();

  const Eigen::Vector3d expected = Eigen::Vector3d::Zero();
  const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AllClose(expected));
}

TEST_F(TestRotation, TinyQuaternionToAngleAxis) {
  // Very small value that could potentially cause underflow.
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(kTinyAngle, axis));

  const Eigen::Vector3d expected = kTinyAngle * axis;
  const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AngleAxisIsClose(expected));
}

TEST_F(TestRotation, SmallQuaternionToAngleAxis) {
  // Small, finite value to test.
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(kSmallAngle, axis));

  const Eigen::Vector3d expected = kSmallAngle * axis;
  const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AngleAxisIsClose(expected));
}

TEST_F(TestRotation, QuaternionToAngleAxis) {
  Eigen::Quaterniond quaternion;

  for (int i = 0; i < 3; ++i) {
    quaternion.w() = std::sqrt(3) / 2;
    quaternion.vec() = Eigen::Vector3d::Unit(i) * 0.5;

    const Eigen::Vector3d expected =
        Eigen::Vector3d::Unit(i) * std::numbers::pi / 3;
    const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }

  for (int i = 0; i < 3; ++i) {
    quaternion.w() = 0.0;
    quaternion.vec() = Eigen::Vector3d::Unit(i);

    const Eigen::Vector3d expected =
        Eigen::Vector3d::Unit(i) * std::numbers::pi;
    const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, ZeroAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis_angle = Eigen::Vector3d::Zero();

  const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expected));
}

TEST_F(TestRotation, TinyAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis_angle(1e-24, 2e-24, 3e-24);

  const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expected));
}

TEST_F(TestRotation, SmallAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Vector3d axis_angle = kTinyAngle * axis;

  const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(kTinyAngle, axis).toRotationMatrix();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expected));
}

// Transforms a rotation by pi/2 around X to a rotation matrix and back.
TEST_F(TestRotation, AngleAxisToRotationMatrix) {
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d angle_axis =
        std::numbers::pi / 2 * Eigen::Vector3d::Unit(i);
    const int j = (i + 1) % 3;
    const int k = (j + 1) % 3;
    Eigen::Matrix3d expected;
    expected(i, i) = 1.0;
    expected(i, j) = expected(i, k) = expected(j, i) = expected(k, i) = 0.0;
    expected(j, j) = expected(k, k) = 0.0;
    expected(j, k) = -1.0;
    expected(k, j) = 1.0;
    const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(angle_axis);
    ASSERT_THAT(result, IsOrthogonal());
    ASSERT_THAT(result, AllClose(expected));
  }

  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d angle_axis =
        std::numbers::pi * Eigen::Vector3d::Unit(i);
    const int j = (i + 1) % 3;
    const int k = (j + 1) % 3;
    Eigen::Matrix3d expected;
    expected(i, i) = 1.0;
    expected(i, j) = expected(i, k) = expected(j, i) = expected(k, i) = 0.0;
    expected(j, k) = expected(k, j) = 0.0;
    expected(j, j) = -1.0;
    expected(k, k) = -1.0;
    const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(angle_axis);
    ASSERT_THAT(result, IsOrthogonal());
    ASSERT_THAT(result, AllClose(expected));
  }
}

TEST_F(TestRotation, QuaternionToAngleAxisAngleIsLessThanPi) {
  const double half_theta = 0.75 * std::numbers::pi;
  Eigen::Quaterniond quaternion;
  quaternion.x() = cos(half_theta);
  quaternion.y() = sin(half_theta);
  quaternion.z() = 0.0;
  quaternion.w() = 0.0;
  auto angle_axis = ap::QuaternionToAngleAxis(quaternion);
  const double angle = angle_axis.norm();
  ASSERT_LE(angle, std::numbers::pi);
}

TEST_F(TestRotation, AngleAxisToQuaternionAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    const Eigen::Vector3d expected =
        std::numbers::pi * random() * randomUnitVector();
    const Eigen::Quaterniond intermediate = ap::AngleAxisToQuaternion(expected);
    ASSERT_THAT(intermediate, IsNormalized());
    const Eigen::Vector3d result = ap::QuaternionToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, QuaternionToAngleAxisAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    const Eigen::Quaterniond expected = Eigen::Quaterniond::UnitRandom();
    const Eigen::Vector3d intermediate = ap::QuaternionToAngleAxis(expected);
    const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(intermediate);
    ASSERT_THAT(result, IsNormalized());
    ASSERT_THAT(result, QuaternionIsClose(expected));
  }
}

TEST_F(TestRotation, AngleAxisToRotationMatrixAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    auto angle = std::numbers::pi * random();
    const Eigen::Vector3d expected = angle * randomUnitVector();
    const Eigen::Matrix3d intermediate =
        ap::AngleAxisToRotationMatrix(expected);
    ASSERT_THAT(intermediate, IsOrthogonal());
    const Eigen::Vector3d result = ap::RotationMatrixToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, RotationMatrixToAngleAxisAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    const Eigen::Quaterniond expected_quaternion =
        Eigen::Quaterniond::UnitRandom();
    const Eigen::Matrix3d expected = expected_quaternion.toRotationMatrix();
    const Eigen::Vector3d intermediate =
        ap::RotationMatrixToAngleAxis(expected);
    const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(intermediate);
    ASSERT_THAT(result, AllClose(expected));
  }
}

// Takes a bunch of random axis/angle values, converts them to quaternions,
// and back again.
TEST_F(TestRotation, NearPiAngleAxisToQuaternionAndBack) {
  Eigen::Quaterniond intermediate;
  constexpr double kMaxSmallAngle = 1e-8;
  for (int i = 0; i < kNumTrials; i++) {
    const double theta = std::numbers::pi - kMaxSmallAngle * random();
    const Eigen::Vector3d expected = theta * randomUnitVector();
    intermediate = ap::AngleAxisToQuaternion(expected);
    ASSERT_THAT(intermediate, IsNormalized());
    const Eigen::Vector3d result = ap::QuaternionToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, NearPiAngleAxisToRotationMatrixAndBack) {
  constexpr double kMaxSmallAngle = 1e-8;
  for (int i = 0; i < kNumTrials; i++) {
    const double theta = std::numbers::pi - kMaxSmallAngle * random();
    const Eigen::Vector3d expected = theta * randomUnitVector();

    const Eigen::Matrix3d intermediate =
        ap::AngleAxisToRotationMatrix(expected);
    const Eigen::Vector3d result = ap::RotationMatrixToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, NearPiRotationMatrixToAngleAxisAndBack) {
  constexpr double kMaxSmallAngle = 1e-8;
  for (int a = 0; a < int(kNumTrials / 3); ++a) {
    for (int i = 0; i < 3; ++i) {
      const double theta = std::numbers::pi - kMaxSmallAngle * random();
      const double st = sin(theta);
      const double ct = cos(theta);
      const int j = (i + 1) % 3;
      const int k = (j + 1) % 3;
      Eigen::Matrix3d expected;
      expected(i, i) = 1.0;
      expected(i, j) = expected(i, k) = expected(j, i) = expected(k, i) = 0.0;
      expected(j, k) = st;
      expected(k, j) = -st;
      expected(j, j) = expected(k, k) = -ct;
      const Eigen::Vector3d intermediate =
          ap::RotationMatrixToAngleAxis(expected);
      const Eigen::Matrix3d result =
          ap::AngleAxisToRotationMatrix(intermediate);
      ASSERT_THAT(result, AllClose(expected));
    }
  }
}
