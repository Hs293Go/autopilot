#include <ranges>

#include "autopilot/core/butterworth_filter.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "testing/matchers.hpp"

TEST(ButterworthFilterTest, ImpulseResponse) {
  // Example test case for Butterworth filter response
  autopilot::ButterworthFilter<float, 1> bw;
  ASSERT_THAT(bw.initialize(40.0f, 100.0f), IsEmptyErrorCode());

  const float expected_impulse_response[] = {
      0.6389455251590224F,   0.5475887728761645F,    -0.2506954995302367F,
      0.06049454749474992F,  0.03434341454513083F,   -0.06422609909766833F,
      0.05923216261452053F,  -0.041188570644668146F, 0.02262660178837562F,
      -0.008859056897431019F};

  const float step[] = {1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                        0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

  float result[10];
  for (int i = 0; i < 10; ++i) {
    result[i] = bw.compute(Eigen::Matrix<float, 1, 1>(step[i])).value();
  }

  ASSERT_THAT(result, testing::Pointwise(testing::FloatNear(1e-6F),
                                         expected_impulse_response));
}
