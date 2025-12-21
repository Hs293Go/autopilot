#include "autopilot/planning/polynomial.hpp"
#include "gtest/gtest.h"

TEST(PolynomialTest, ConstructionFromCoefficients) {
  Eigen::VectorXd coeffs(4);
  coeffs << 1.0, -2.0, 3.0, -4.0;  // Represents -4t^3 + 3t^2 - 2t + 1
  autopilot::Polynomial<double> p(coeffs);

  EXPECT_EQ(p.degree(), 3);

  autopilot::Polynomial p_expected(1.0, -2.0, 3.0, -4.0);
  EXPECT_EQ(p, p_expected);
}

TEST(PolynomialTest, DerivativeConsistency) {
  Eigen::VectorXd random_coeffs = Eigen::VectorXd::Random(8);
  autopilot::Polynomial<double> p(random_coeffs);

  auto p_prime = p.deriv(1);
  double t = 2.5;

  // Test that the derivative polynomial evaluates to the same value as the
  // analytic method
  EXPECT_NEAR(p_prime(t), p.derivVal(t, 1), 1e-12);
}

TEST(PolynomialTest, MonomialRule) {
  // p(t) = t^3 -> coeffs = [0, 0, 0, 1]
  autopilot::Polynomial p(0.0, 0.0, 0.0, 1.0);

  // p'(t) = 3t^2. At t=2, p'(2) = 12.
  EXPECT_NEAR(p.derivVal(2.0, 1), 12.0, 1e-12);
}

TEST(PolynomialTest, ZeroPolynomial) {
  Eigen::VectorXd zero_coeffs = Eigen::VectorXd::Zero(5);
  autopilot::Polynomial p(zero_coeffs);

  // The zero polynomial and its derivatives should evaluate to zero
  for (int order = 0; order <= 3; ++order) {
    EXPECT_NEAR(p.derivVal(1.0, order), 0.0, 1e-12);
    EXPECT_NEAR(p.derivVal(-3.5, order), 0.0, 1e-12);
  }
}

TEST(PolynomialTest, HighOrderDerivative) {
  // p(t) = 2t^5 + 3t^4 + t^3 + 4t^2 + 5t + 6
  Eigen::VectorXd coeffs(6);
  coeffs << 6, 5, 4, 1, 3, 2;
  autopilot::Polynomial p(coeffs);

  // The 5th derivative should be constant: p^(5)(t) = 240
  for (double t : {0.0, 1.0, -1.0, 2.5}) {
    EXPECT_NEAR(p.derivVal(t, 5), 240.0, 1e-12);
  }

  // The 6th derivative should be zero
  for (double t : {0.0, 1.0, -1.0, 2.5}) {
    EXPECT_NEAR(p.derivVal(t, 6), 0.0, 1e-12);
  }
}

TEST(PolynomialTest, ConstantPolynomial) {
  Eigen::VectorXd const_coeffs(1);
  const_coeffs << 7.5;
  autopilot::Polynomial p(const_coeffs);

  // The constant polynomial's derivatives should be zero
  for (int order = 1; order <= 3; ++order) {
    EXPECT_NEAR(p.derivVal(10.0, order), 0.0, 1e-12);
    EXPECT_NEAR(p.derivVal(-5.0, order), 0.0, 1e-12);
  }

  // The polynomial itself should evaluate to the constant value
  EXPECT_NEAR(p.derivVal(10.0, 0), 7.5, 1e-12);
  EXPECT_NEAR(p.derivVal(-5.0, 0), 7.5, 1e-12);
}

TEST(PolynomialTest, LinearPolynomial) {
  // p(t) = 3t + 2
  Eigen::Vector2d coeffs;
  coeffs << 2, 3;
  autopilot::Polynomial<double> p(coeffs);

  // The first derivative should be constant: p'(t) = 3
  for (double t : {0.0, 1.0, -1.0, 2.5}) {
    EXPECT_NEAR(p.derivVal(t, 1), 3.0, 1e-12);
  }

  // The second derivative should be zero
  for (double t : {0.0, 1.0, -1.0, 2.5}) {
    EXPECT_NEAR(p.derivVal(t, 2), 0.0, 1e-12);
  }
}

TEST(PolynomialTest, QuadraticPolynomial) {
  // p(t) = t^2 + 2t + 1
  Eigen::Vector3d coeffs;
  coeffs << 1, 2, 1;
  autopilot::Polynomial p(coeffs);

  // The first derivative: p'(t) = 2t + 2
  EXPECT_NEAR(p.derivVal(1.0, 1), 4.0, 1e-12);
  EXPECT_NEAR(p.derivVal(-1.0, 1), 0.0, 1e-12);

  // The second derivative: p''(t) = 2
  for (double t : {0.0, 1.0, -1.0, 2.5}) {
    EXPECT_NEAR(p.derivVal(t, 2), 2.0, 1e-12);
  }

  // The third derivative should be zero
  for (double t : {0.0, 1.0, -1.0, 2.5}) {
    EXPECT_NEAR(p.derivVal(t, 3), 0.0, 1e-12);
  }
}

TEST(PolynomialTest, NegativeTimeEvaluation) {
  // p(t) = 2t^3 - 4t + 1
  Eigen::Vector4d coeffs;
  coeffs << 1, -4, 0, 2;
  autopilot::Polynomial p(coeffs);

  // Evaluate at negative time
  EXPECT_NEAR(p.derivVal(-2.0, 0),
              2 * (-2.0) * (-2.0) * (-2.0) - 4 * (-2.0) + 1, 1e-12);
  EXPECT_NEAR(p.derivVal(-2.0, 1), 6 * (-2.0) * (-2.0) - 4, 1e-12);
}

TEST(PolynomialTest, LargeCoefficients) {
  // p(t) = 1e6 * t^2 + 1e5 * t + 1e4
  Eigen::Vector3d coeffs;
  coeffs << 1e4, 1e5, 1e6;
  autopilot::Polynomial p(coeffs);

  // Evaluate polynomial and its derivatives at t=1.0
  EXPECT_NEAR(p.derivVal(1.0, 0), 1e6 * 1.0 * 1.0 + 1e5 * 1.0 + 1e4, 1e-6);
  EXPECT_NEAR(p.derivVal(1.0, 1), 2e6 * 1.0 + 1e5, 1e-6);
  EXPECT_NEAR(p.derivVal(1.0, 2), 2e6, 1e-6);
}
