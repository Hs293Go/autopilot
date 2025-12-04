#include <vector>

#include "autopilot/cascade_controller.hpp"
#include "autopilot/geometric_controller.hpp"
#include "autopilot/quadrotor_model.hpp"
#include "autopilot/quadrotor_simulator.hpp"
#include "gtest/gtest.h"
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
    ASSERT_EQ(model_cfg->setMass(1.0), std::error_code());

    // Rust: diag(0.025, 0.025, 0.043)
    ASSERT_EQ(
        model_cfg->setInertiaElements({0.025, 0.025, 0.043, 0.0, 0.0, 0.0}),
        std::error_code());

    // Rust: k_f = 1.56252e-6, time_constant = 0.033
    ASSERT_EQ(model_cfg->setThrustCurveCoeff(1.56252e-6), std::error_code());
    ASSERT_EQ(model_cfg->setMotorTimeConstantUp(0.033), std::error_code());
    ASSERT_EQ(model_cfg->setMotorTimeConstantDown(0.033), std::error_code());

    // Rust: arm lengths ~0.17? Derived from [0.075, 0.1] vectors
    ASSERT_EQ(model_cfg->setFrontMotorPosition(0.075, 0.1), std::error_code());
    ASSERT_EQ(model_cfg->setBackMotorPosition(0.075, 0.1), std::error_code());
    ASSERT_EQ(model_cfg->setTorqueConstant(0.01386), std::error_code());

    // Allow aggressive flight
    ASSERT_EQ(model_cfg->setMaxCollectiveThrust(40.0), std::error_code());
    // Gravity points DOWN
    ASSERT_EQ(model_cfg->setGravAcceleration(-9.81), std::error_code());

    // Setup Simulator
    // ===============
    auto model = std::make_shared<ap::QuadrotorModel>(model_cfg);
    auto cfg = std::make_shared<ap::QuadrotorSimulator::Config>();

    simulator = std::make_shared<ap::QuadrotorSimulator>(model, cfg);

    // Setup Controller
    // ================
    auto controller = std::make_shared<ap::CascadeController>(model);

    // TUNING: We must apply the Rust gains to the C++ controllers.
    // NOTE: This assumes you added accessors like `positionController()`
    // to CascadeController. If not, these gains remain 0 and drone falls.

    // Cast base pointers to specific implementation to access config
    auto pos_ctrl = std::dynamic_pointer_cast<ap::GeometricPositionController>(
        controller->positionController());  // << Requires accessor
    auto att_ctrl = std::dynamic_pointer_cast<ap::GeometricAttitudeController>(
        controller->attitudeController());  // << Requires accessor

    ASSERT_NE(pos_ctrl, nullptr);
    ASSERT_NE(att_ctrl, nullptr);

    // Rust: k_position: [1.0, 1.0, 4.0], k_velocity: [1.8, 1.8, 8.0]
    pos_ctrl->config()->kp = Eigen::Vector3d(1.0, 1.0, 3.0);
    pos_ctrl->config()->kv = Eigen::Vector3d(1.8, 1.8, 6.0);

    // Rust: k_angle: [6.0, 6.0, 3.0], k_rate: [1.5, 1.5, 1.3]
    // (RateController) Note: Rust Rate controller was PID (kp=0.2...).
    // GeometricAttitudeController combines attitude error (kR) and rate error
    // (kOmega). Mapping Rust "Geometric + PID" to C++ "Geometric Only": We
    // use the Geometric gains for the outer attitude loop.
    att_ctrl->config()->kR = {3.0, 3.0, 0.1};
    att_ctrl->config()->kOmega = {1.0, 1.0, 0.01};  // D-term equivalent

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
    ASSERT_NE(runner, nullptr);
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
