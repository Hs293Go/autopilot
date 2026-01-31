#include <vector>

#include "autopilot/controllers/cascade_controller.hpp"
#include "autopilot/controllers/geometric_controller.hpp"
#include "autopilot/core/quadrotor_model.hpp"
#include "autopilot/planning/minimum_snap_solver.hpp"
#include "autopilot/planning/preset_trajectories.hpp"
#include "autopilot/simulator/quadrotor_simulator.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "testing/matchers.hpp"
#include "validation/mission_runner.hpp"

namespace ap = autopilot;
class TestTrajectoryExecution : public ::testing::Test {
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

    pos_ctrl->config()->kp = Eigen::Vector3d(0.8, 0.8, 2.0);
    pos_ctrl->config()->kv = Eigen::Vector3d(1.5, 1.5, 3.0);

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

TEST_F(TestTrajectoryExecution, MinsnapTrajectoryTest) {
  // Waypoints
  // =========
  std::vector<ap::TrajectoryWaypoint> waypoints = {
      {0, {0.0, 0.0, 1.0}},
      {2, {5.0, 0.0, 1.0}},
      {4, {5.0, 5.0, 1.0}},
      {6, {0.0, 5.0, 1.0}},
      {8, {0.0, 0.0, 1.0}}
      // Spiraling back to origin
  };

  ap::MinimumSnapSolver traj_solver;
  auto trajectory_res = traj_solver.solve(waypoints, ap::Fixed{});
  ASSERT_TRUE(trajectory_res.has_value());

  ap::Mission mission;
  ap::TrajectoryRunner runner(
      simulator, controller,
      std::make_unique<ap::PolynomialTrajectory>(trajectory_res.value()));

  // Initial State: Hover at [0,0,1]
  ap::QuadrotorState initial_state;
  initial_state.odometry.pose().translation() = waypoints.front().position;
  initial_state.odometry.pose().rotation().setIdentity();
  simulator->setState(initial_state);

  // ---------------------------------------------------------------------------
  // 4. Simulation Loop
  // ---------------------------------------------------------------------------
  auto result = runner.run();
  ASSERT_TRUE(result.completed);
}

struct InitialStateAndTrajectory {
  ap::QuadrotorState initial_state;
  std::shared_ptr<ap::TrajectoryBase> trajectory;
};

class TestPresetTrajectoryExecution
    : public TestTrajectoryExecution,
      public testing::WithParamInterface<InitialStateAndTrajectory> {};

static const ap::QuadrotorState kTestInitialPosition = ap::QuadrotorState{
    0.0, ap::OdometryF64(ap::TransformF64::PureTranslation(0.0, 0.0, 1.0), {})};

INSTANTIATE_TEST_SUITE_P(
    PresetTrajectories, TestPresetTrajectoryExecution,
    testing::Values(
        InitialStateAndTrajectory{
            .initial_state = kTestInitialPosition,
            .trajectory = std::make_shared<ap::Hover>(
                Eigen::Vector3d::UnitZ() * 1.0, 0.0, 0.0)},
        InitialStateAndTrajectory{
            .initial_state = kTestInitialPosition,
            .trajectory = std::make_shared<ap::HeadingChange>(
                kTestInitialPosition.odometry.pose().translation(), 0.0,
                std::numbers::pi / 2, 0.0, 2.0)}));

TEST_P(TestPresetTrajectoryExecution, SettlingTest) {
  const auto& [initial_state, trajectory] = GetParam();

  simulator->setState(initial_state);

  // 2. Setup Mission with a single Hover segment
  ASSERT_TRUE(trajectory != nullptr);
  ap::RunnerBase::Config cfg;
  ap::TrajectoryRunner runner(simulator, controller, trajectory, cfg);

  auto result = runner.run();
  ASSERT_TRUE(result.completed);

  auto final_state = simulator->state();
  auto equilibrium = trajectory->checkEquilibrium(final_state, {0.1, 0.1});

  EXPECT_EQ(equilibrium.state, ap::EquilibriumState::kStatic);
  EXPECT_LT(equilibrium.position_error_mag, 0.1);
  EXPECT_LT(equilibrium.velocity_error_mag, 0.1);
  EXPECT_LT(equilibrium.angle_error_mag, 0.1);
}
