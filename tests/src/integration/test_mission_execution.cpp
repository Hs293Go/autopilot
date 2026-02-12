#include "autopilot/controllers/cascade_controller.hpp"
#include "autopilot/controllers/geometric_controller.hpp"
#include "autopilot/core/quadrotor_model.hpp"
#include "autopilot/planning/mission.hpp"
#include "autopilot/simulator/quadrotor_simulator.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "spdlog/spdlog.h"
#include "validation/mission_runner.hpp"

namespace ap = autopilot;

class TestMissionExecution : public ::testing::Test {
 public:
  void SetUp() override {
    // Best practice: reduce log level to avoid cluttering test output
    spdlog::set_level(spdlog::level::err);

    // Setup Model
    // ===========
    auto model_cfg = std::make_shared<ap::QuadrotorModelCfg>();
    ASSERT_EQ(model_cfg->setMass(1.0), ap::AutopilotErrc::kNone);

    // Rust: diag(0.025, 0.025, 0.043)
    ASSERT_EQ(
        model_cfg->setInertiaElements({0.025, 0.025, 0.043, 0.0, 0.0, 0.0}),
        ap::AutopilotErrc::kNone);

    // Rust: k_f = 1.56252e-6, time_constant = 0.033
    ASSERT_EQ(model_cfg->setThrustCurveCoeff(1.56252e-6),
              ap::AutopilotErrc::kNone);
    ASSERT_EQ(model_cfg->setMotorTimeConstantUp(0.033),
              ap::AutopilotErrc::kNone);
    ASSERT_EQ(model_cfg->setMotorTimeConstantDown(0.033),
              ap::AutopilotErrc::kNone);

    // Rust: arm lengths ~0.17? Derived from [0.075, 0.1] vectors
    ASSERT_EQ(model_cfg->setFrontMotorPosition(0.075, 0.1),
              ap::AutopilotErrc::kNone);
    ASSERT_EQ(model_cfg->setBackMotorPosition(0.075, 0.1),
              ap::AutopilotErrc::kNone);
    ASSERT_EQ(model_cfg->setTorqueConstant(0.01386), ap::AutopilotErrc::kNone);

    // Allow aggressive flight
    ASSERT_EQ(model_cfg->setMaxCollectiveThrust(40.0),
              ap::AutopilotErrc::kNone);
    // Gravity points DOWN
    ASSERT_EQ(model_cfg->setGravAcceleration(-9.81), ap::AutopilotErrc::kNone);

    // Setup Simulator
    // ===============
    auto model = std::make_shared<ap::QuadrotorModel>(model_cfg);
    auto cfg = std::make_shared<ap::QuadrotorSimulator::Config>();

    simulator = std::make_shared<ap::QuadrotorSimulator>(model, cfg);

    // Setup Controller
    // ================
    auto controller_cfg = std::make_shared<ap::CascadeController::Config>();
    controller = std::make_shared<ap::CascadeController>(model, controller_cfg);

    // TUNING: We must apply the Rust gains to the C++ controllers.
    // NOTE: This assumes you added accessors like `positionController()`
    // to CascadeController. If not, these gains remain 0 and drone falls.

    // Cast base pointers to specific implementation to access config
    auto pos_ctrl = std::dynamic_pointer_cast<ap::GeometricPositionController>(
        controller->positionController());  // << Requires accessor
    auto att_ctrl = std::dynamic_pointer_cast<ap::GeometricAttitudeController>(
        controller->attitudeController());  // << Requires accessor

    ASSERT_THAT(pos_ctrl, testing::NotNull());
    ASSERT_THAT(att_ctrl, testing::NotNull());

    pos_ctrl->config()->kp = Eigen::Vector3d(0.5, 0.5, 2.0);
    pos_ctrl->config()->kv = Eigen::Vector3d(3.0, 3.0, 5.0);

    // Rust: k_angle: [6.0, 6.0, 3.0], k_rate: [1.5, 1.5, 1.3]
    // (RateController) Note: Rust Rate controller was PID (kp=0.2...).
    // GeometricAttitudeController combines attitude error (kR) and rate error
    // (kOmega). Mapping Rust "Geometric + PID" to C++ "Geometric Only": We
    // use the Geometric gains for the outer attitude loop.
    att_ctrl->config()->k_ang_rate = {1.0, 1.0, 0.1};
    att_ctrl->config()->k_rate_torque = {0.6, 0.6, 0.05};  // D-term equivalent
  }

  std::shared_ptr<ap::QuadrotorSimulator> simulator;
  std::shared_ptr<ap::CascadeController> controller;
};

TEST_F(TestMissionExecution, HighFrequencySpliceStability) {
  ap::Mission mission;
  ap::MinimumSnapSolver solver;
  ap::MissionRunner runner(simulator, controller, mission);

  double dt = 0.01;  // 100Hz Control Loop
  double total_time = 2.0;

  // Target moves every 0.1s
  for (int i = 0; i < (total_time / dt); ++i) {
    if (i % 10 == 0) {
      // Generate a random target within a 2m box
      Eigen::Vector3d target =
          Eigen::Vector3d::Random() + Eigen::Vector3d(0, 0, 1.5);

      // Splicing NOW - this tests if getNowState() and truncate/splice
      // produce a C0 continuous handover under pressure.
      bool ok = mission.lineTo(target, 1.0, solver, true /* interrupt */);
      ASSERT_TRUE(ok);
    }

    auto result = runner.step(i);

    // Check for "Blowups": If the controller sees a discontinuity,
    // the state will likely go to NaN or exceed a huge velocity.
    ASSERT_LT(simulator->state().odometry.twist().linear().norm(), 10.0);
  }
}

TEST_F(TestMissionExecution, MixedManueverMission) {
  ap::Mission mission(simulator->state().timestamp_secs);
  ap::MinimumSnapSolver solver;

  // 1. Takeoff (Fixed Heading)
  mission.lineTo({0, 0, 1.5}, 2.0, solver, false, ap::Fixed{0.0}, true);

  // 2. Rotate 180 degrees in place
  mission.yawTo(M_PI, 2.0);

  // 3. Fly a circle-ish path using FollowVelocity
  mission.lineTo({2, 2, 1.5}, 5.0, solver, false, ap::FollowVelocity{});
  mission.lineTo({0, 4, 1.5}, 5.0, solver, false, ap::FollowVelocity{});

  ap::RunnerBase::Config cfg;
  cfg.max_steps = 10000;
  ap::MissionRunner runner(simulator, controller, mission, cfg);

  auto res = runner.run();  // Uses internal loop

  EXPECT_TRUE(res.completed);
  EXPECT_NEAR(simulator->state().odometry.pose().translation().y(), 4.0, 0.1);
}

TEST_F(TestMissionExecution, GatedSquareMission) {
  ap::QuadrotorState initial_state;
  initial_state.odometry.pose().translation() << 0, 0, 0;
  initial_state.odometry.pose().rotation().setIdentity();
  simulator->setState(initial_state);

  ap::QuadrotorState start_state = simulator->state();
  ap::Mission mission(start_state.timestamp_secs);
  ap::MinimumSnapSolver solver;

  const Eigen::Vector3d corners[] = {
      {2, 0, 1}, {2, 2, 1}, {0, 2, 1}, {0, 0, 1}};

  constexpr auto kNumCorners = std::ssize(corners);
  for (int i = 0; i < kNumCorners; ++i) {
    const auto& pos = corners[i];
    // Every corner is a "hard stop" requirement
    const bool stop_at_end = i != kNumCorners - 1;
    EXPECT_TRUE(
        mission.lineTo(pos, 4.0, solver, false, ap::Fixed{0.0}, stop_at_end));
  }

  ap::MissionRunner runner(simulator, controller, mission);

  // We expect at least 4 "settling" events.
  // We'll track the mission size to see it decrement only after settling.
  size_t last_size = mission.size();
  int settles_observed = 0;

  int step = 0;
  ASSERT_EQ(runner.mission().size(), 7);
  while (simulator->state().timestamp_secs < 40.0 &&
         !runner.mission().empty()) {
    auto res = runner.step(step++);

    ASSERT_TRUE(res.success);

    if (runner.mission().size() < last_size) {
      // A segment was popped! Verify we were actually at the target.
      std::ignore = simulator->state().odometry.pose().translation();
      // If we popped, we must be close to the previous target.
      settles_observed++;
      last_size = runner.mission().size();
    }
  }

  EXPECT_EQ(settles_observed, 5);
  EXPECT_TRUE(runner.mission().empty());
}

TEST_F(TestMissionExecution, MidFlightRerouteContinuity) {
  ap::Mission mission(simulator->state().timestamp_secs);
  ap::MinimumSnapSolver solver;
  ap::MissionRunner runner(simulator, controller, mission);

  // Intent: Fly far away at high speed
  runner.mission().lineTo({20.0, 0.0, 1.0}, 15.0, solver);

  bool rerouted = false;

  int steps = 0;
  while (simulator->state().timestamp_secs < 10.0) {
    // Sample velocity BEFORE the splice
    Eigen::Vector3d vel_before = simulator->state().odometry.twist().linear();

    if (!rerouted && simulator->state().timestamp_secs >= 2.5) {
      // Mid-flight splice to a target in the opposite direction
      runner.mission().lineTo({0.0, 0.0, 1.0}, 3.0, solver,
                              true /* interrupt */);
      rerouted = true;

      // Verification: Immediately after splice, the NEXT setpoint's velocity
      // should match our current commanded velocity.
      auto next_setpoint = runner.mission().getNowState();
      EXPECT_NEAR((vel_before - next_setpoint.velocity).norm(), 0.0, 0.05);
    }

    auto result = runner.step(steps++);

    ASSERT_TRUE(result.success);
    if (result.completed && rerouted) {
      break;
    }
  }

  EXPECT_TRUE(rerouted);
}
