
#include <limits>
#include <random>

#include "Eigen/Dense"
#include "autopilot/core/math.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "testing/matchers.hpp"
#include "unsupported/Eigen/AutoDiff"

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

MATCHER_P(QuaternionIsClose, expectation,
          ::testing::PrintToString(expectation)) {
  return ap::IsClose(arg.angularDistance(expectation), 0.0);
}

MATCHER_P(AngleAxisIsClose, expectation, "") {
  return ExplainMatchResult(
      testing::Conditional(
          ap::IsClose(arg.norm(), std::numbers::pi),
          testing::AnyOf(AllClose(-expectation), AllClose(expectation)),
          AllClose(expectation)),
      arg, result_listener);
}

MATCHER(IsOrthogonal,
        "Matrix must be orthogonal, i.e. itself multiplied by its transpose is "
        "equal to the identity matrix") {
  return ((arg * arg.transpose()).isIdentity(1e-8) &&
          (arg.transpose() * arg).isIdentity(1e-8));
}

MATCHER(IsNormalized, "expectation unit norm") {
  return ap::IsClose(arg.norm(), 1.0);
}

class TestRotation : public test_utils::FixtureWithRNG {
 public:
  using FixtureWithRNG::FixtureWithRNG;
};

TEST_F(TestRotation, ZeroAngleAxisToQuaternion) {
  const Eigen::Vector3d axis_angle = Eigen::Vector3d::Zero();

  const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(axis_angle);
  const Eigen::Quaterniond expectation = Eigen::Quaterniond::Identity();
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expectation));
}

TEST_F(TestRotation, TinyAngleAxisToQuaternion) {
  // Very small value that could potentially cause underflow.
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Vector3d axis_angle = kTinyAngle * axis;

  const Eigen::Quaterniond expectation(Eigen::AngleAxisd(kTinyAngle, axis));
  const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(axis_angle);
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expectation));
}

TEST_F(TestRotation, SmallAngleAxisToQuaternion) {
  // Small, finite value to test.
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Vector3d axis_angle = kSmallAngle * axis;

  const Eigen::Quaterniond expectation(Eigen::AngleAxisd(kSmallAngle, axis));
  const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(axis_angle);
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expectation));
}

TEST_F(TestRotation, AngleAxisToQuaternion) {
  for (int i = 0; i < 3; ++i) {
    auto axis = Eigen::Vector3d::Unit(i);
    auto axis_angle = std::numbers::pi * axis / 2.0;

    Eigen::Quaterniond expectation;
    expectation.w() = std::numbers::sqrt2 / 2.0;
    expectation.vec() = std::numbers::sqrt2 / 2.0 * axis;
    const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(axis_angle);
    ASSERT_THAT(result, IsNormalized());
    ASSERT_THAT(result, QuaternionIsClose(expectation));
  }
}

// Transforms a unit quaternion to an axis angle.
TEST_F(TestRotation, UnitQuaternionToAngleAxis) {
  const Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();

  const Eigen::Vector3d expectation = Eigen::Vector3d::Zero();
  const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AllClose(expectation));
}

TEST_F(TestRotation, TinyQuaternionToAngleAxis) {
  // Very small value that could potentially cause underflow.
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(kTinyAngle, axis));

  const Eigen::Vector3d expectation = kTinyAngle * axis;
  const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AngleAxisIsClose(expectation));
}

TEST_F(TestRotation, SmallQuaternionToAngleAxis) {
  // Small, finite value to test.
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(kSmallAngle, axis));

  const Eigen::Vector3d expectation = kSmallAngle * axis;
  const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AngleAxisIsClose(expectation));
}

TEST_F(TestRotation, QuaternionToAngleAxis) {
  Eigen::Quaterniond quaternion;

  for (int i = 0; i < 3; ++i) {
    quaternion.w() = std::sqrt(3) / 2;
    quaternion.vec() = Eigen::Vector3d::Unit(i) * 0.5;

    const Eigen::Vector3d expectation =
        Eigen::Vector3d::Unit(i) * std::numbers::pi / 3;
    const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
    ASSERT_THAT(result, AngleAxisIsClose(expectation));
  }

  for (int i = 0; i < 3; ++i) {
    quaternion.w() = 0.0;
    quaternion.vec() = Eigen::Vector3d::Unit(i);

    const Eigen::Vector3d expectation =
        Eigen::Vector3d::Unit(i) * std::numbers::pi;
    const Eigen::Vector3d result = ap::QuaternionToAngleAxis(quaternion);
    ASSERT_THAT(result, AngleAxisIsClose(expectation));
  }
}

TEST_F(TestRotation, ZeroAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis_angle = Eigen::Vector3d::Zero();

  const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expectation = Eigen::Matrix3d::Identity();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expectation));
}

TEST_F(TestRotation, TinyAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis_angle(1e-24, 2e-24, 3e-24);

  const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expectation = Eigen::Matrix3d::Identity();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expectation));
}

TEST_F(TestRotation, SmallAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis = randomUnitVector();
  const Eigen::Vector3d axis_angle = kTinyAngle * axis;

  const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expectation =
      Eigen::AngleAxisd(kTinyAngle, axis).toRotationMatrix();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expectation));
}

// Transforms a rotation by pi/2 around X to a rotation matrix and back.
TEST_F(TestRotation, AngleAxisToRotationMatrix) {
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d angle_axis =
        std::numbers::pi / 2 * Eigen::Vector3d::Unit(i);
    const int j = (i + 1) % 3;
    const int k = (j + 1) % 3;
    Eigen::Matrix3d expectation;
    expectation(i, i) = 1.0;
    expectation(i, j) = expectation(i, k) = expectation(j, i) =
        expectation(k, i) = 0.0;
    expectation(j, j) = expectation(k, k) = 0.0;
    expectation(j, k) = -1.0;
    expectation(k, j) = 1.0;
    const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(angle_axis);
    ASSERT_THAT(result, IsOrthogonal());
    ASSERT_THAT(result, AllClose(expectation));
  }

  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d angle_axis =
        std::numbers::pi * Eigen::Vector3d::Unit(i);
    const int j = (i + 1) % 3;
    const int k = (j + 1) % 3;
    Eigen::Matrix3d expectation;
    expectation(i, i) = 1.0;
    expectation(i, j) = expectation(i, k) = expectation(j, i) =
        expectation(k, i) = 0.0;
    expectation(j, k) = expectation(k, j) = 0.0;
    expectation(j, j) = -1.0;
    expectation(k, k) = -1.0;
    const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(angle_axis);
    ASSERT_THAT(result, IsOrthogonal());
    ASSERT_THAT(result, AllClose(expectation));
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
    const Eigen::Vector3d expectation =
        std::numbers::pi * random() * randomUnitVector();
    const Eigen::Quaterniond intermediate =
        ap::AngleAxisToQuaternion(expectation);
    ASSERT_THAT(intermediate, IsNormalized());
    const Eigen::Vector3d result = ap::QuaternionToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expectation));
  }
}

TEST_F(TestRotation, QuaternionToAngleAxisAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    const Eigen::Quaterniond expectation = Eigen::Quaterniond::UnitRandom();
    const Eigen::Vector3d intermediate = ap::QuaternionToAngleAxis(expectation);
    const Eigen::Quaterniond result = ap::AngleAxisToQuaternion(intermediate);
    ASSERT_THAT(result, IsNormalized());
    ASSERT_THAT(result, QuaternionIsClose(expectation));
  }
}

TEST_F(TestRotation, AngleAxisToRotationMatrixAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    auto angle = std::numbers::pi * random();
    const Eigen::Vector3d expectation = angle * randomUnitVector();
    const Eigen::Matrix3d intermediate =
        ap::AngleAxisToRotationMatrix(expectation);
    ASSERT_THAT(intermediate, IsOrthogonal());
    const Eigen::Vector3d result = ap::RotationMatrixToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expectation));
  }
}

TEST_F(TestRotation, RotationMatrixToAngleAxisAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    const Eigen::Quaterniond expectation_quaternion =
        Eigen::Quaterniond::UnitRandom();
    const Eigen::Matrix3d expectation =
        expectation_quaternion.toRotationMatrix();
    const Eigen::Vector3d intermediate =
        ap::RotationMatrixToAngleAxis(expectation);
    const Eigen::Matrix3d result = ap::AngleAxisToRotationMatrix(intermediate);
    ASSERT_THAT(result, AllClose(expectation));
  }
}

// Takes a bunch of random axis/angle values, converts them to quaternions,
// and back again.
TEST_F(TestRotation, NearPiAngleAxisToQuaternionAndBack) {
  Eigen::Quaterniond intermediate;
  constexpr double kMaxSmallAngle = 1e-8;
  for (int i = 0; i < kNumTrials; i++) {
    const double theta = std::numbers::pi - kMaxSmallAngle * random();
    const Eigen::Vector3d expectation = theta * randomUnitVector();
    intermediate = ap::AngleAxisToQuaternion(expectation);
    ASSERT_THAT(intermediate, IsNormalized());
    const Eigen::Vector3d result = ap::QuaternionToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expectation));
  }
}

TEST_F(TestRotation, NearPiAngleAxisToRotationMatrixAndBack) {
  constexpr double kMaxSmallAngle = 1e-8;
  for (int i = 0; i < kNumTrials; i++) {
    const double theta = std::numbers::pi - kMaxSmallAngle * random();
    const Eigen::Vector3d expectation = theta * randomUnitVector();

    const Eigen::Matrix3d intermediate =
        ap::AngleAxisToRotationMatrix(expectation);
    const Eigen::Vector3d result = ap::RotationMatrixToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expectation));
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
      Eigen::Matrix3d expectation;
      expectation(i, i) = 1.0;
      expectation(i, j) = expectation(i, k) = expectation(j, i) =
          expectation(k, i) = 0.0;
      expectation(j, k) = st;
      expectation(k, j) = -st;
      expectation(j, j) = expectation(k, k) = -ct;
      const Eigen::Vector3d intermediate =
          ap::RotationMatrixToAngleAxis(expectation);
      const Eigen::Matrix3d result =
          ap::AngleAxisToRotationMatrix(intermediate);
      ASSERT_THAT(result, AllClose(expectation));
    }
  }
}

TEST_F(TestRotation, LeftRightQuaternionMatrixVsQuaternionProduct) {
  for (int i = 0; i < kNumTrials; i++) {
    const Eigen::Quaterniond q1 = Eigen::Quaterniond::UnitRandom();
    const Eigen::Quaterniond q2 = Eigen::Quaterniond::UnitRandom();

    const Eigen::Matrix4d left_matrix = ap::LeftQuaternionMatrix(q1);
    const Eigen::Matrix4d right_matrix = ap::RightQuaternionMatrix(q2);

    const Eigen::Quaterniond product = q1 * q2;
    const Eigen::Quaterniond left_result(left_matrix * q2.coeffs());
    const Eigen::Quaterniond right_result(right_matrix * q1.coeffs());

    ASSERT_THAT(left_result, QuaternionIsClose(product));
    ASSERT_THAT(right_result, QuaternionIsClose(product));
  }
}

TEST_F(TestRotation, RotatePointByQuaternion) {
  using ADScalar = Eigen::AutoDiffScalar<Eigen::Vector4d>;

  for (int i = 0; i < kNumTrials; i++) {
    Eigen::Vector3d point = Eigen::Vector3d::UnitX();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::UnitRandom();

    const Eigen::Matrix<double, 3, 4> result =
        autopilot::jacobians::RotatePointByQuaternion(rotation, point);

    // Initialize AD components
    Eigen::Quaternion<ADScalar> q_ad = rotation.cast<ADScalar>();

    // Seed derivatives: index 0=w, 1=x, 2=y, 3=z
    for (Eigen::Index i = 0; i < 4; ++i) {
      q_ad.coeffs()[i].derivatives() = Eigen::Vector4d::Unit(i);
    }
    // Sandwich product
    Eigen::Quaternion<ADScalar> p_ad(0, point.x(), point.y(), point.z());
    Eigen::Quaternion<ADScalar> res_quat_ad = q_ad * p_ad * q_ad.conjugate();

    Eigen::Matrix<double, 3, 4> expected_jacobian;
    expected_jacobian << res_quat_ad.x().derivatives().transpose(),
        res_quat_ad.y().derivatives().transpose(),
        res_quat_ad.z().derivatives().transpose();

    ASSERT_THAT(result, AllClose(expected_jacobian));
  }
}
