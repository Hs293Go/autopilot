#include "autopilot/core/math.hpp"
#include "gmock/gmock-matchers.h"

MATCHER(IsEmptyErrorCode, "") {
  if (arg == std::error_code()) {
    return true;
  }
  *result_listener << "which is error code: " << arg.message();
  return false;
}

MATCHER(IsNonEmptyErrorCode, "") {
  if (arg != std::error_code()) {
    return true;
  }
  *result_listener << "which is an empty error code";
  return false;
}

MATCHER_P(AllClose, expectation, "") {
  // reshaped() (ravel) arguments so that Eigen iterate through them
  // element-wise
  return ExplainMatchResult(
      testing::Pointwise(testing::DoubleNear(1e-6), expectation.reshaped()),
      arg.reshaped(), result_listener);
}

MATCHER_P(QuaternionIsClose, expectation,
          ::testing::PrintToString(expectation)) {
  return autopilot::IsClose(arg.angularDistance(expectation), 0.0);
}

MATCHER_P2(IsCloseOrFlipped, expectation, norm, "") {
  return ExplainMatchResult(
      testing::Conditional(
          autopilot::IsClose(arg.norm(), norm),
          testing::AnyOf(AllClose(-expectation), AllClose(expectation)),
          AllClose(expectation)),
      arg, result_listener);
}

MATCHER_P(AngleAxisIsClose, expectation, "") {
  return ExplainMatchResult(IsCloseOrFlipped(expectation, std::numbers::pi),
                            arg, result_listener);
}

MATCHER(IsOrthogonal,
        "Matrix must be orthogonal, i.e. itself multiplied by its transpose is "
        "equal to the identity matrix") {
  return ((arg * arg.transpose()).isIdentity(1e-8) &&
          (arg.transpose() * arg).isIdentity(1e-8));
}
