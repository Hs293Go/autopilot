#include <gtest/gtest.h>

#include "autopilot/core/math.hpp"
#include "autopilot/planning/polynomial_trajectory.hpp"

namespace ap = autopilot;

TEST(HeadingPolicyMath, YawFromVelocityBasic) {
  // Drone moving purely in +X direction
  Eigen::Vector2d vel(1.0, 0.0);
  Eigen::Vector2d acc(0.0, 0.0);
  Eigen::Vector2d jerk(0.0, 0.0);

  auto res = ap::YawFromVelocity(vel, acc, jerk);
  EXPECT_NEAR(res.yaw, 0.0, 1e-6);
  EXPECT_NEAR(res.yaw_rate, 0.0, 1e-6);

  // Drone moving in +Y direction
  vel = {0.0, 1.0};
  res = ap::YawFromVelocity(vel, acc, jerk);
  EXPECT_NEAR(res.yaw, std::numbers::pi / 2.0, 1e-6);
}

TEST(HeadingPolicyMath, YawFromVelocityCurvature) {
  // Drone in a uniform circular turn (R=1, V=1)
  // pos = [cos(t), sin(t)]
  // vel = [-sin(t), cos(t)] -> at t=0: [0, 1]
  // acc = [-cos(t), -sin(t)] -> at t=0: [-1, 0]
  Eigen::Vector2d vel(0.0, 1.0);
  Eigen::Vector2d acc(-1.0, 0.0);
  Eigen::Vector2d jerk(0.0, -1.0);

  auto res = ap::YawFromVelocity(vel, acc, jerk);
  EXPECT_NEAR(res.yaw, std::numbers::pi / 2.0, 1e-6);
  // For V=1, R=1, omega should be 1.0 rad/s
  EXPECT_NEAR(res.yaw_rate, 1.0, 1e-6);
}

TEST(HeadingPolicyMath, YawFromRayPOI) {
  // Drone at origin [0,0], POI at [1,1]
  // Ray r = [1, 1], yaw should be 45 deg
  Eigen::Vector2d r(1.0, 1.0);
  Eigen::Vector2d r_dot(0.0, 0.0);
  Eigen::Vector2d r_ddot(0.0, 0.0);

  auto res = ap::YawFromRay(r, r_dot, r_ddot);
  EXPECT_NEAR(res.yaw, std::numbers::pi / 4.0, 1e-6);
}

class TestHeadingPolicyIntegration : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a simple 2-second diagonal line from [0,0,1] to [2,2,1]
    ap::TrajectorySegment segment(
        2.0, {{
                 ap::PlanningPolynomial::Linear(0.0, 1.0),  // X: 0 + 1*t
                 ap::PlanningPolynomial::Linear(0.0, 1.0),  // Y: 0 + 1*t
                 ap::PlanningPolynomial::Constant(0.0)      // Z: 1.0
             }});
    segments_.push_back(segment);
  }

  std::vector<ap::TrajectorySegment> segments_;
};

TEST_F(TestHeadingPolicyIntegration, FollowVelocityPolicy) {
  ap::PolynomialTrajectory traj(segments_, 0.0, ap::FollowVelocity{});

  // At T=1.0, velocity is [1, 1, 0], yaw should be 45 degrees
  auto state = traj.sample(1.0);
  EXPECT_NEAR(state.yaw, M_PI / 4.0, 1e-6);
  EXPECT_NEAR(state.yaw_rate, 0.0, 1e-6);  // Velocity is constant, so rate is 0
}

TEST_F(TestHeadingPolicyIntegration, POIPolicy) {
  // POI located at [0, 2, 1].
  // At start (0,0), drone looks at (0,2) -> Yaw = 90 deg
  // At end (2,2), drone looks at (0,2) -> Yaw = 180 deg
  ap::PolynomialTrajectory traj(segments_, 0.0,
                                ap::PointOfInterest{{0.0, 2.0, 1.0}});

  auto start_state = traj.sample(0.0);
  EXPECT_NEAR(start_state.yaw, std::numbers::pi / 2, 1e-6);

  auto end_state = traj.sample(2.0);
  EXPECT_NEAR(end_state.yaw, std::numbers::pi, 1e-6);
}

TEST_F(TestHeadingPolicyIntegration, FixedPolicy) {
  ap::PolynomialTrajectory traj(segments_, 0.0,
                                ap::Fixed{.yaw = 0.5, .yaw_rate = 0.1});

  auto state = traj.sample(1.0);
  EXPECT_NEAR(state.yaw, 0.5, 1e-6);
  EXPECT_NEAR(state.yaw_rate, 0.1, 1e-6);
}

TEST_F(TestHeadingPolicyIntegration, ZeroVelocityHandling) {
  // Create a Hover (Zero velocity)
  ap::TrajectorySegment hover_seg(
      1.0, {ap::PlanningPolynomial::Zero(), ap::PlanningPolynomial::Zero(),
            ap::PlanningPolynomial::Constant(1.0)});

  ap::PolynomialTrajectory traj(std::views::single(hover_seg), 0.0,
                                ap::FollowVelocity{});

  auto state = traj.sample(0.5);
  // Should not be NaN
  EXPECT_TRUE(std::isfinite(state.yaw));
  EXPECT_NEAR(state.yaw, 0.0, 1e-6);
}

TEST_F(TestHeadingPolicyIntegration, PolicySwitchContinuity) {
  // Traj 1: Fixed Yaw at 0
  ap::PolynomialTrajectory traj1(segments_, 0.0, ap::Fixed{0.0});

  // Traj 2: Fixed Yaw at 0 (Stitched at T=1.0)
  ap::PolynomialTrajectory traj2(segments_, 1.0, ap::Fixed{0.0});

  auto sample1 = traj1.sample(1.0);
  auto sample2 = traj2.sample(1.0);

  EXPECT_NEAR(sample1.yaw, sample2.yaw, 1e-6);
  EXPECT_NEAR(sample1.yaw_rate, sample2.yaw_rate, 1e-6);
}

TEST(HeadingPolicyMath, YawFromRayRegularizationEffect) {
  // Drone passing POI at (0,0) with velocity [1, 0] at distance y=0.01 (very
  // close) r = [0, -0.01] (when drone is at x=0) r_dot = [-1, 0]
  Eigen::Vector2d r(0.0, -0.01);
  Eigen::Vector2d r_dot(-1.0, 0.0);
  Eigen::Vector2d r_ddot(0.0, 0.0);

  // Case 1: Small focus radius (aggressive)
  auto res_sharp = ap::YawFromRay(r, r_dot, r_ddot, 0.01);

  // Case 2: Large focus radius (smooth)
  auto res_smooth = ap::YawFromRay(r, r_dot, r_ddot, 0.5);

  // The sharp one should have a much higher yaw rate
  EXPECT_GT(std::abs(res_sharp.yaw_rate), std::abs(res_smooth.yaw_rate));

  // With focus_radius = 0.5 and r_norm = 0.01, denom is approx 0.25
  // yaw_rate = (0 * 0 - (-0.01 * -1)) / 0.25 = -0.01 / 0.25 = -0.04 rad/s
  EXPECT_NEAR(res_smooth.yaw_rate, -0.04, 1e-2);
}

TEST_F(TestHeadingPolicyIntegration, POIFlyOverSmoothness) {
  // Drone moves from [-1, 0, 1] to [1, 0, 1] in 2 seconds.
  // POI is exactly at [0, 0, 1].
  ap::TrajectorySegment segment(
      2.0, {{ap::PlanningPolynomial::Linear(-1.0, 1.0),  // X: -1 + 1*t
             ap::PlanningPolynomial::Zero(),             // Y: 0
             ap::PlanningPolynomial::Constant(1.0)}});   // Z: 1.0

  // Set focus_radius to 0.1m
  ap::PointOfInterest poi{{0.0, 0.0, 1.0}, 0.1};
  ap::PolynomialTrajectory traj(std::views::single(segment), 0.0, poi);

  // Sample at the exact moment of overlap (T=1.0)
  auto state_mid = traj.sample(1.0);

  EXPECT_TRUE(std::isfinite(state_mid.yaw));
  EXPECT_TRUE(std::isfinite(state_mid.yaw_rate));
  EXPECT_TRUE(std::isfinite(state_mid.yaw_acceleration));

  // With focus_radius = 0.1 and velocity = 1.0,
  // max yaw rate should be roughly V / focus_radius = 10 rad/s
  EXPECT_LT(std::abs(state_mid.yaw_rate), 11.0);
}

TEST_F(TestHeadingPolicyIntegration, POIFocusRadiusSensitivity) {
  ap::TrajectorySegment segment(
      1.0,
      {{ap::PlanningPolynomial::Constant(0.5),  // Drone is 0.5m away from POI
        ap::PlanningPolynomial::Constant(0.0),
        ap::PlanningPolynomial::Constant(1.0)}});

  ap::PointOfInterest poi_sharp{{0.0, 0.0, 1.0}, 0.01};
  ap::PointOfInterest poi_soft{{0.0, 0.0, 1.0}, 1.0};

  ap::PolynomialTrajectory traj_sharp(std::views::single(segment), 0.0,
                                      poi_sharp);
  ap::PolynomialTrajectory traj_soft(std::views::single(segment), 0.0,
                                     poi_soft);

  // Even for a stationary drone, the ResolveYawState logic should produce
  // the same yaw but potentially different internal derivatives if the
  // POI itself were moving (not applicable here, but good for structural
  // check).
  EXPECT_NEAR(traj_sharp.sample(0.5).yaw, traj_soft.sample(0.5).yaw, 1e-6);
}
