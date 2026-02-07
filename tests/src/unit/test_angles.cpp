#include "autopilot/core/math.hpp"
#include "gtest/gtest.h"

template <typename T>
class Angles : public ::testing::Test {};

using Floats = ::testing::Types<float, double>;

TYPED_TEST_SUITE(Angles, Floats);

TYPED_TEST(Angles, TestWrapToPi) {
  using autopilot::wrapToPi;
  constexpr auto kPi = std::numbers::pi_v<TypeParam>;
  constexpr TypeParam kZero(0);
  constexpr TypeParam kEpsilon =
      std::is_same_v<TypeParam, float> ? TypeParam(1e-5) : TypeParam(1e-12);
  EXPECT_NEAR(kZero, wrapToPi(kZero), kEpsilon);
  EXPECT_NEAR(-kPi, wrapToPi(kPi), kEpsilon);  // was kPi
  EXPECT_NEAR(kZero, wrapToPi(2 * kPi), kEpsilon);
  EXPECT_NEAR(-kPi, wrapToPi(3 * kPi), kEpsilon);  // was kPi
  EXPECT_NEAR(kZero, wrapToPi(4 * kPi), kEpsilon);

  EXPECT_NEAR(kZero, wrapToPi(-kZero), kEpsilon);
  EXPECT_NEAR(-kPi, wrapToPi(-kPi), kEpsilon);  // was kPi
  EXPECT_NEAR(kZero, wrapToPi(-2 * kPi), kEpsilon);
  EXPECT_NEAR(-kPi, wrapToPi(-3 * kPi), kEpsilon);  // was kPi
  EXPECT_NEAR(kZero, wrapToPi(-4 * kPi), kEpsilon);

  EXPECT_NEAR(kZero, wrapToPi(-kZero), kEpsilon);
  EXPECT_NEAR(-kPi / 2, wrapToPi(-kPi / 2), kEpsilon);
  EXPECT_NEAR(-kPi, wrapToPi(-kPi), kEpsilon);  // was kPi
  EXPECT_NEAR(kPi / 2, wrapToPi(-3 * kPi / 2), kEpsilon);
  EXPECT_NEAR(kZero, wrapToPi(-4 * kPi / 2), kEpsilon);

  EXPECT_NEAR(kZero, wrapToPi(kZero), kEpsilon);
  EXPECT_NEAR(kPi / 2, wrapToPi(kPi / 2), kEpsilon);
  EXPECT_NEAR(kPi / 2, wrapToPi(5 * kPi / 2), kEpsilon);
  EXPECT_NEAR(kPi / 2, wrapToPi(9 * kPi / 2), kEpsilon);
  EXPECT_NEAR(kPi / 2, wrapToPi(-3 * kPi / 2), kEpsilon);
}
