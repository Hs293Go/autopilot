
#include "autopilot/planning/minimum_snap_solver.hpp"
#include "gtest/gtest.h"

static constexpr double kPosEps = 1e-8;
static constexpr double kDerEps = 1e-6;

// You need these adapters (implement based on your trajectory API)
Eigen::Vector3d EvalPos(const autopilot::PolynomialTrajectory& traj, double t) {
  auto state = traj.sample(t);
  return state.position;
}
Eigen::Vector3d EvalDer(const autopilot::PolynomialTrajectory& traj, double t,
                        int order) {
  autopilot::KinematicState state = traj.sample(t);
  switch (order) {
    case 0:
      return state.position;
    case 1:
      return state.velocity;
    case 2:
      return state.acceleration;
    case 3:
      return state.jerk;
    case 4:
      return state.snap;
    default:
      return Eigen::Vector3d::Zero();
  }
}

TEST(MinSnap, CatchesTooFewWaypoints) {
  autopilot::MinimumSnapSolver solver;

  std::vector<autopilot::TrajectoryWaypoint> wps;  // Empty waypoints

  auto res = solver.solve(wps);
  ASSERT_FALSE(res.has_value());

  wps.push_back({0.0, {0, 0, 0}});  // Single waypoint
  res = solver.solve(wps);
  ASSERT_FALSE(res.has_value());

  wps.push_back({1.0, {1, 1, 0}});  // Now two waypoints
  res = solver.solve(wps);
  ASSERT_TRUE(res.has_value()) << res.error().message();
}

TEST(MinSnap, InterpolatesPositionsAndContinuity) {
  autopilot::MinimumSnapSolver solver;

  std::vector<autopilot::TrajectoryWaypoint> wps;
  wps.push_back({0.0, {0, 0, 0}});
  wps.push_back({1.0, {1, 2, 0}});
  wps.push_back({2.0, {2, 0, 1}});

  auto res = solver.solve(wps);
  ASSERT_TRUE(res.has_value());
  const auto& traj = res.value();

  // Waypoint positions
  for (size_t i = 0; i < wps.size(); ++i) {
    auto p = EvalPos(traj, wps[i].time_from_start_secs);
    EXPECT_NEAR(p.x(), wps[i].position.x(), kPosEps);
    EXPECT_NEAR(p.y(), wps[i].position.y(), kPosEps);
    EXPECT_NEAR(p.z(), wps[i].position.z(), kPosEps);
  }

  // Continuity at internal knot(s): C^3 for degree 7 => orders 0..3
  for (size_t i = 1; i + 1 < wps.size(); ++i) {
    double t = wps[i].time_from_start_secs;
    for (int order = 0; order <= 3; ++order) {
      auto d_l =
          EvalDer(traj, t - 1e-9, order);  // or evaluate by segment boundary
      auto d_r = EvalDer(traj, t + 1e-9, order);
      EXPECT_NEAR((d_l - d_r).norm(), 0.0, kDerEps);
    }
  }
}

TEST(MinSnap, RespectsEndpointVelocityAcceleration) {
  autopilot::MinimumSnapSolver solver;

  autopilot::TrajectoryWaypoint w0;
  w0.time_from_start_secs = 0.0;
  w0.position = {0, 0, 0};
  w0.velocity = Eigen::Vector3d(1.0, -2.0, 0.5);

  autopilot::TrajectoryWaypoint w1;
  w1.time_from_start_secs = 2.0;
  w1.position = {2, 0, 1};
  w1.acceleration = Eigen::Vector3d(0.2, 0.0, -0.1);

  std::vector<autopilot::TrajectoryWaypoint> wps = {w0, w1};

  auto res = solver.solve(wps);
  ASSERT_TRUE(res.has_value());
  const auto& traj = res.value();

  auto v0 = EvalDer(traj, 0.0, 1);
  EXPECT_NEAR((v0 - *w0.velocity).norm(), 0.0, kDerEps);

  auto a_t = EvalDer(traj, 2.0, 2);
  EXPECT_NEAR((a_t - *w1.acceleration).norm(), 0.0, kDerEps);
}
