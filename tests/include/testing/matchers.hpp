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
