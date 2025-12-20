#include <vector>

#include "autopilot/controllers/cascade_controller.hpp"
#include "autopilot/controllers/geometric_controller.hpp"
#include "autopilot/core/quadrotor_model.hpp"
#include "autopilot/simulator/quadrotor_simulator.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "validation/mission_runner.hpp"

MATCHER(IsEmptyErrorCode, "") {
  if (arg == std::error_code()) {
    return true;
  }
  *result_listener << "which is error code: " << arg.message();
  return false;
}

namespace ap = autopilot;
class TestIntegrationCascadeControllerSim : public ::testing::Test {
 public:
  void SetUp() override {
    // Best practice: reduce log level to avoid cluttering test output
    spdlog::set_level(spdlog::level::err);

    // Setup Model
    // ===========
    auto model_cfg = std::make_shared<ap::QuadrotorModelCfg>();
    ASSERT_THAT(model_cfg->setMass(1.0), IsEmptyErrorCode());

    // Rust: diag(0.025, 0.025, 0.043)
    ASSERT_EQ(
        model_cfg->setInertiaElements({0.025, 0.025, 0.043, 0.0, 0.0, 0.0}),
        std::error_code());

    // Rust: k_f = 1.56252e-6, time_constant = 0.033
    ASSERT_THAT(model_cfg->setThrustCurveCoeff(1.56252e-6), IsEmptyErrorCode());
    ASSERT_THAT(model_cfg->setMotorTimeConstantUp(0.033), IsEmptyErrorCode());
    ASSERT_THAT(model_cfg->setMotorTimeConstantDown(0.033), IsEmptyErrorCode());

    // Rust: arm lengths ~0.17? Derived from [0.075, 0.1] vectors
    ASSERT_THAT(model_cfg->setFrontMotorPosition(0.075, 0.1),
                IsEmptyErrorCode());
    ASSERT_THAT(model_cfg->setBackMotorPosition(0.075, 0.1),
                IsEmptyErrorCode());
    ASSERT_THAT(model_cfg->setTorqueConstant(0.01386), IsEmptyErrorCode());

    // Allow aggressive flight
    ASSERT_THAT(model_cfg->setMaxCollectiveThrust(40.0), IsEmptyErrorCode());
    // Gravity points DOWN
    ASSERT_THAT(model_cfg->setGravAcceleration(-9.81), IsEmptyErrorCode());

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
    att_ctrl->config()->kR = {1.0, 1.0, 0.1};
    att_ctrl->config()->kOmega = {0.6, 0.6, 0.05};  // D-term equivalent

    // Waypoints
    // =========
    waypoints = {
        {{5.0, 0.0, 1.0}, 0.0},
        {{5.0, 5.0, 1.0}, std::numbers::pi / 2},
        {{0.0, 5.0, 1.0}, std::numbers::pi},
        {{0.0, 0.0, 1.0}, 3.0 * std::numbers::pi / 2}
        // Spiraling back to origin
    };

    runner =
        std::make_shared<ap::MissionRunner>(simulator, controller, waypoints);
    ASSERT_THAT(runner, testing::NotNull());
  }

  std::shared_ptr<ap::QuadrotorSimulator> simulator;
  std::shared_ptr<ap::MissionRunner> runner;
  std::vector<ap::MissionWaypoint> waypoints;
};

TEST_F(TestIntegrationCascadeControllerSim, RunMission) {
  // Initial State: Hover at [0,0,1]
  ap::QuadrotorState initial_state;
  initial_state.odometry.pose().translation() = {0.0, 0.0, 1.0};
  initial_state.odometry.pose().rotation().setIdentity();
  simulator->setState(initial_state);

  // ---------------------------------------------------------------------------
  // 4. Simulation Loop
  // ---------------------------------------------------------------------------
  auto result = runner->run();
  ASSERT_TRUE(result.completed);
}
