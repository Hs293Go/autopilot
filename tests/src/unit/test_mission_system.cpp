#include "autopilot/planning/mission.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace ap = autopilot;
class TestMissionSystem : public ::testing::Test {
 protected:
  ap::QuadrotorState MakeState(double t,
                               const Eigen::Vector3d& pos = {0, 0, 1}) {
    ap::QuadrotorState s;
    s.timestamp_secs = t;
    s.odometry.pose().translation() = pos;
    return s;
  }
};

TEST_F(TestMissionSystem, LineToBrakingBehavior) {
  ap::MinimumSnapSolver solver;
  ap::Mission stopping_mission(0.0);

  stopping_mission.lineTo({5, 0, 1}, 5.0, solver, false, ap::Fixed{0.0}, true);

  EXPECT_EQ(stopping_mission.size(), 2);

  ap::Mission nonstop_mission(0.0);
  nonstop_mission.lineTo({5, 0, 1}, 5.0, solver, false, ap::Fixed{0.0}, false);
  EXPECT_EQ(nonstop_mission.size(), 1);
}

TEST_F(TestMissionSystem, SequentialLineTo) {
  ap::MinimumSnapSolver solver;
  const Eigen::Vector3d corners[] = {
      {0, 0, 1}, {5, 0, 1}, {5, 5, 1}, {0, 5, 1}, {0, 0, 1}};

  ap::Mission stopping_mission(0.0);

  for (const auto& pos : corners) {
    // Every corner is a "hard stop" requirement
    stopping_mission.lineTo(pos, 5.0, solver, false, ap::Fixed{0.0},
                            true /* stop_at_end */);
  }

  EXPECT_EQ(stopping_mission.size(), std::ssize(corners) * 2);

  ap::Mission nonstop_mission(0.0);

  for (const auto& pos : corners) {
    // Every corner is a "hard stop" requirement
    nonstop_mission.lineTo(pos, 5.0, solver, false, ap::Fixed{0.0}, false);
  }

  EXPECT_EQ(nonstop_mission.size(), std::ssize(corners));
}

// 1. Verify "Append" (Sequencing)
TEST_F(TestMissionSystem, SequentialExecution) {
  ap::Mission mission(0.0);
  ap::MinimumSnapSolver solver;

  mission.lineTo({5, 0, 1}, 5.0, solver);  // 0-5s
  mission.lineTo({5, 5, 1}, 5.0, solver);  // 5-10s

  EXPECT_EQ(mission.size(), 2);
  EXPECT_THAT(mission.getTailTime(), testing::DoubleEq(10.0));
  EXPECT_THAT(mission[1]->startTime(), testing::DoubleEq(5.0));
}

// 2. Verify "Interrupt" (Splicing)
TEST_F(TestMissionSystem, MidFlightInterrupt) {
  ap::Mission mission(0.0);
  ap::MinimumSnapSolver solver;

  mission.lineTo({10, 0, 1}, 10.0, solver);  // Original intent: 0-10s

  // Advance to T=4.0
  std::ignore = mission.getUpdatedTrajectory(MakeState(4.0), false);

  // Interrupt! New destination.
  mission.lineTo({0, 10, 1}, 2.0, solver, true /* interrupt */);

  // Verification
  ASSERT_EQ(mission.size(), 2);
  EXPECT_THAT(mission[0]->endTime(),
              testing::DoubleEq(4.0));  // Old leg cut at 4s
  EXPECT_THAT(mission[1]->startTime(),
              testing::DoubleEq(4.0));  // New leg starts at 4s
}

// 3. Verify Equilibrium Gating (The "Wait" State)
TEST_F(TestMissionSystem, EquilibriumGating) {
  ap::Mission mission(0.0);
  ap::MinimumSnapSolver solver;

  // Gated Move
  mission.lineTo({1, 0, 1}, 1.0, solver, false, ap::Fixed{0},
                 true /* stop_at_end */);
  mission.lineTo({2, 0, 1}, 1.0, solver);

  // At T=1.5, drone is at [0,0,1] (HAS NOT ARRIVED AT [1,0,1])
  auto state_bad = MakeState(1.5, {0, 0, 1});
  std::ignore = mission.getUpdatedTrajectory(state_bad, true);
  auto horizon = mission.getUpdatedTrajectory(state_bad, true);

  // The mission should NOT advance because equilibrium failed.
  // It should return ONLY the first segment (clamped).
  EXPECT_EQ(mission.size(), 2);
  EXPECT_EQ(horizon.size(), 1);
  EXPECT_EQ(horizon[0], mission[0]);
}

TEST_F(TestMissionSystem, ReplaceMissionExactTime) {
  ap::MinimumSnapSolver solver;
  ap::Mission mission(0.0);

  // Create a 5s segment [0, 5]
  mission.lineTo({5, 0, 1}, 5.0, solver);

  // Interrupt EXACTLY at T=0.0
  // If the logic is '>=', the mission becomes empty here.
  bool ok = mission.lineTo({0, 5, 1}, 5.0, solver, true /* interrupt */);

  ASSERT_TRUE(ok);
  EXPECT_GT(mission.size(), 0)
      << "Mission was wiped by inclusive boundary pruning";
  EXPECT_THAT(mission[0]->startTime(), testing::DoubleEq(0));
}

TEST_F(TestMissionSystem, MissionDelayedByLateEquilibrium) {
  ap::MinimumSnapSolver solver;
  ap::Mission mission(0.0);

  // Seg A: [0, 5] (Stop at end), Seg B: [5, 10]
  mission.lineTo({5, 0, 1}, 5.0, solver, false, ap::Fixed{}, true);
  mission.lineTo({5, 5, 1}, 5.0, solver);

  // Simulate drone arriving at T=5.0 but NOT settled until T=7.0
  // We feed a 'not settled' state at T=5.0
  std::ignore = mission.getUpdatedTrajectory(MakeState(5.0, {4.5, 0, 1}), true);

  // Now we settle at T=7.0
  std::ignore = mission.getUpdatedTrajectory(MakeState(7.0, {5.0, 0, 1}), true);

  // ASSERT: The next segment (which was [5, 10]) must now be [7, 12]
  ASSERT_EQ(mission.size(), 1);
  EXPECT_THAT(mission[0]->startTime(), testing::DoubleEq(7.0));
  EXPECT_THAT(mission[0]->endTime(), testing::DoubleEq(12.0));
}

TEST_F(TestMissionSystem, StopAtEndHandoverContinuity) {
  ap::MinimumSnapSolver solver;
  ap::Mission mission(0.0);

  mission.lineTo({5, 0, 1}, 5.0, solver, false, ap::Fixed{}, true);

  // Segment 0 is Polynomial [0, 5], Segment 1 is Hover [5, inf]
  auto poly_end = mission[0]->sample(5.0);
  auto hover_start = mission[1]->sample(5.0);

  EXPECT_THAT(poly_end.position.x(),
              testing::DoubleEq(hover_start.position.x()));
  EXPECT_THAT(poly_end.position.y(),
              testing::DoubleEq(hover_start.position.y()));
  // Verify the 'Brake' is actually stationary
  EXPECT_THAT(hover_start.velocity.norm(), testing::DoubleEq(0));
}
