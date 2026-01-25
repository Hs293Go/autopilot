#include <vector>

#include "autopilot/controllers/cascade_controller.hpp"
#include "autopilot/controllers/geometric_controller.hpp"
#include "autopilot/core/quadrotor_model.hpp"
#include "autopilot/planning/minimum_snap_solver.hpp"
#include "autopilot/simulator/quadrotor_simulator.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "testing/matchers.hpp"
#include "validation/mission_runner.hpp"

namespace ap = autopilot;
class TestIntegrationCascadeControllerSim : public ::testing::Test {
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
    auto controller =
        std::make_shared<ap::CascadeController>(model, controller_cfg);

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

    // Waypoints
    // =========
    waypoints = {
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

    runner = std::make_shared<ap::MissionRunner>(
        simulator, controller,
        std::make_shared<ap::PolynomialTrajectory>(trajectory_res.value()));
    ASSERT_THAT(runner, testing::NotNull());
  }

  std::shared_ptr<ap::QuadrotorSimulator> simulator;
  std::shared_ptr<ap::MissionRunner> runner;
  std::vector<ap::TrajectoryWaypoint> waypoints;
};

TEST_F(TestIntegrationCascadeControllerSim, RunMission) {
  // Initial State: Hover at [0,0,1]
  ap::QuadrotorState initial_state;
  initial_state.odometry.pose().translation() = waypoints.front().position;
  initial_state.odometry.pose().rotation().setIdentity();
  simulator->setState(initial_state);

  // ---------------------------------------------------------------------------
  // 4. Simulation Loop
  // ---------------------------------------------------------------------------
  auto result = runner->run();
  ASSERT_TRUE(result.completed);
}
