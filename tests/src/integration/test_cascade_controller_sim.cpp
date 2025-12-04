#include <spdlog/sinks/stdout_color_sinks.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "autopilot/cascade_controller.hpp"
#include "autopilot/geometric_controller.hpp"
#include "autopilot/quadrotor_model.hpp"
#include "autopilot/quadrotor_simulator.hpp"

using namespace autopilot;

// =============================================================================
// Constants & Configuration (Matching Rust quadrotor_control.rs)
// =============================================================================
static constexpr double kControlDt = 0.01;  // 100 Hz Control Loop
static constexpr int kSimSubsteps = 10;     // Physics runs at 1000 Hz
static constexpr double kSimDt = kControlDt / kSimSubsteps;
static constexpr int kMaxSimSteps = 5000;  // 50 seconds total
static constexpr double kRadiusAcceptance = 0.1;

// Waypoints from Rust example
struct Waypoint {
  Eigen::Vector3d position;
  double yaw;
};

const std::vector<Waypoint> kWaypoints = {
    {{5.0, 0.0, 1.0}, 0.0},
    {{5.0, 5.0, 1.0}, M_PI_2},
    {{0.0, 5.0, 1.0}, M_PI},
    {{0.0, 0.0, 1.0}, 3.0 * M_PI_2}  // Spiraling back to origin
};

// =============================================================================
// CSV Logger
// =============================================================================
void write_csv(const std::string& filename,
               const std::vector<double>& time_hist,
               const std::vector<QuadrotorState>& state_hist,
               const std::vector<QuadrotorCommand>& cmd_hist,
               const std::vector<Waypoint>& targets) {
  std::filesystem::path path(filename);
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "Failed to open " << filename << '\n';
    return;
  }

  // Header matching Rust output for easy comparison
  file << "type,time,x,y,z,pd,qd,rd,p,q,r\n";

  // 1. Log Targets (Static markers)
  for (const auto& wp : targets) {
    file << fmt::format("target,{},{},{},{},NaN,NaN,NaN,NaN,NaN,NaN\n", -1.0,
                        wp.position.x(), wp.position.y(), wp.position.z());
  }

  // 2. Log Trajectory
  for (size_t i = 0; i < state_hist.size(); ++i) {
    const auto& s = state_hist[i];
    const auto& c = cmd_hist[i];

    // Extract desired body rates if available
    Eigen::Vector3d rate_sp =
        c.hasComponent(QuadrotorStateComponent::kAngularVelocity)
            ? c.bodyRate()
            : Eigen::Vector3d::Zero();

    const auto& pos = s.odometry.pose().translation();
    const auto& rate = s.odometry.twist().angular();

    file << fmt::format("output,{},{},{},{},{},{},{},{},{},{}\n", time_hist[i],
                        pos.x(), pos.y(), pos.z(), rate_sp.x(), rate_sp.y(),
                        rate_sp.z(), rate.x(), rate.y(), rate.z());
  }
  std::cout << "Simulation results written to " << filename << '\n';
}

#define CHECK(expr)                                           \
  do {                                                        \
    if (auto error_code = (expr)) {                           \
      spdlog::error("CHECK failed: {} with reason {}", #expr, \
                    error_code.message());                    \
      return EXIT_FAILURE;                                    \
    }                                                         \
  } while (0)

// =============================================================================
// Main Driver
// =============================================================================
int main() {
  auto logger = spdlog::stderr_color_mt("console");
  logger->set_level(spdlog::level::info);

  // ---------------------------------------------------------------------------
  // 1. Setup Model (Parameters from Rust example)
  // ---------------------------------------------------------------------------
  auto model_cfg = std::make_shared<QuadrotorModelCfg>();

  // Rust: mass = 1.0
  CHECK(model_cfg->setMass(1.0));

  // Rust: diag(0.025, 0.025, 0.043)
  CHECK(model_cfg->setInertiaElements({0.025, 0.025, 0.043, 0.0, 0.0, 0.0}));

  // Rust: k_f = 1.56252e-6, time_constant = 0.033
  CHECK(model_cfg->setThrustCurveCoeff(1.56252e-6));
  CHECK(model_cfg->setMotorTimeConstantUp(0.033));
  CHECK(model_cfg->setMotorTimeConstantDown(0.033));

  // Rust: arm lengths ~0.17? Derived from [0.075, 0.1] vectors
  CHECK(model_cfg->setFrontMotorPosition(0.075, 0.1));
  CHECK(model_cfg->setBackMotorPosition(0.075, 0.1));
  CHECK(model_cfg->setTorqueConstant(0.01386));

  CHECK(model_cfg->setMaxCollectiveThrust(40.0));  // Allow aggressive flight
  CHECK(model_cfg->setGravAcceleration(
      -9.81));  // Gravity points DOWN (+Z) in NED?
  // CHECK: Rust uses -9.81 in Z for dynamics equation, implying Z is UP.
  // "collective_thrust * Z_b - Z_w * 9.81"
  // Our C++ model adds gravity vector. If Z is UP, g = [0,0,-9.81].

  auto model = std::make_shared<QuadrotorModel>(model_cfg);

  // ---------------------------------------------------------------------------
  // 2. Setup Simulator
  // ---------------------------------------------------------------------------
  auto cfg = std::make_shared<QuadrotorSimulator::Config>();
  auto simulator = std::make_shared<QuadrotorSimulator>(model, cfg, logger);

  // Initial State: Hover at [0,0,1]
  QuadrotorState initial_state;
  initial_state.odometry.pose().translation() = {0.0, 0.0, 1.0};
  initial_state.odometry.pose().rotation().setIdentity();
  simulator->setState(initial_state);

  // ---------------------------------------------------------------------------
  // 3. Setup Controller
  // ---------------------------------------------------------------------------
  auto controller = std::make_shared<CascadeController>(model, logger);

  // TUNING: We must apply the Rust gains to the C++ controllers.
  // NOTE: This assumes you added accessors like `positionController()`
  // to CascadeController. If not, these gains remain 0 and drone falls.

  // Cast base pointers to specific implementation to access config
  auto pos_ctrl = std::dynamic_pointer_cast<GeometricPositionController>(
      controller->positionController());  // << Requires accessor
  auto att_ctrl = std::dynamic_pointer_cast<GeometricAttitudeController>(
      controller->attitudeController());  // << Requires accessor

  if (pos_ctrl && att_ctrl) {
    // Rust: k_position: [1.0, 1.0, 4.0], k_velocity: [1.8, 1.8, 8.0]
    pos_ctrl->config()->kp = {1.0, 1.0, 4.0};
    pos_ctrl->config()->kv = {1.8, 1.8, 8.0};

    // Rust: k_angle: [6.0, 6.0, 3.0], k_rate: [1.5, 1.5, 1.3] (RateController)
    // Note: Rust Rate controller was PID (kp=0.2...).
    // GeometricAttitudeController combines attitude error (kR) and rate error
    // (kOmega). Mapping Rust "Geometric + PID" to C++ "Geometric Only": We use
    // the Geometric gains for the outer attitude loop.
    att_ctrl->config()->kR = {6.0, 6.0, 3.0};
    att_ctrl->config()->kOmega = {1.5, 1.5, 1.3};  // D-term equivalent
  } else {
    logger->warn(
        "Could not configure gains! Ensure CascadeController exposes "
        "sub-controllers.");
  }

  // ---------------------------------------------------------------------------
  // 4. Simulation Loop
  // ---------------------------------------------------------------------------
  std::vector<double> t_hist;
  std::vector<QuadrotorState> x_hist;
  std::vector<QuadrotorCommand> u_hist;

  size_t wp_idx = 0;
  QuadrotorCommand cmd_sp(0.0);
  std::vector<QuadrotorCommand> sp_buf(1);
  std::vector<QuadrotorCommand> out_buf(1);

  logger->info("Starting simulation...");

  for (int step = 0; step < kMaxSimSteps; ++step) {
    const auto& state_est = simulator->state();

    // A. Waypoint Logic
    double dist =
        (state_est.odometry.pose().translation() - kWaypoints[wp_idx].position)
            .norm();
    if (dist < kRadiusAcceptance) {
      logger->info("Waypoint {} reached at t={:.2f}s", wp_idx,
                   state_est.timestamp_secs);
      wp_idx++;
      if (wp_idx >= kWaypoints.size()) {
        logger->info("Mission Complete.");
        break;
      }
    }

    // B. Update Setpoint
    cmd_sp.reset(state_est.timestamp_secs);
    CHECK(cmd_sp.setPosition(kWaypoints[wp_idx].position));
    CHECK(cmd_sp.setYaw(kWaypoints[wp_idx].yaw));
    // Feed-forwards are zero for static waypoints
    CHECK(cmd_sp.setVelocity(Eigen::Vector3d::Zero()));
    CHECK(cmd_sp.setAcceleration(Eigen::Vector3d::Zero()));

    // C. Run Controller (100Hz)
    sp_buf[0] = cmd_sp;
    if (auto res = controller->compute(state_est, sp_buf, out_buf); !res) {
      logger->error("Controller failed: {}", res.error().message());
      break;
    }
    const auto& control_out = out_buf[0];

    // D. Run Simulator (Physics Sub-stepping 1000Hz)
    // The controller output is held constant for kSimSubsteps
    for (int s = 0; s < kSimSubsteps; ++s) {
      simulator->step(control_out, kSimDt);
    }

    // E. Logging
    t_hist.push_back(state_est.timestamp_secs);
    x_hist.push_back(state_est);
    u_hist.push_back(control_out);
  }

  write_csv("output/quadrotor_trajectory.csv", t_hist, x_hist, u_hist,
            kWaypoints);
  return 0;
}
