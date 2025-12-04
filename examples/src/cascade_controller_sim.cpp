#include <iostream>
#include <ranges>
#include <rerun.hpp>

#include "autopilot/cascade_controller.hpp"
#include "autopilot/geometric_controller.hpp"
#include "autopilot/quadrotor_model.hpp"
#include "validation/mission_runner.hpp"

// Use a distinct namespace or alias for clarity
namespace rr = rerun;
namespace rrc = rerun::components;
namespace ap = autopilot;

#define CHECK(expr)                                                        \
  do {                                                                     \
    auto ec = (expr);                                                      \
    if (ec) {                                                              \
      spdlog::error("Error at {}:{}. Expression `{}` failed with code {}", \
                    __FILE__, __LINE__, #expr, ec.message());              \
      return -1;                                                           \
    }                                                                      \
  } while (0)

static const rrc::Color kRed(255, 0, 0);
static const rrc::Color kGreen(0, 255, 0);
static const rrc::Color kBlue(0, 0, 255);

int main() {
  // 1. Setup Rerun
  auto rec = rr::RecordingStream("autopilot_mission");
  rec.spawn().exit_on_failure();

  // Setup Model
  // ===========
  auto model_cfg = std::make_shared<ap::QuadrotorModelCfg>();
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

  // Allow aggressive flight
  CHECK(model_cfg->setMaxCollectiveThrust(40.0));
  // Gravity points DOWN
  CHECK(model_cfg->setGravAcceleration(-9.81));

  // Setup Simulator
  // ===============
  auto model = std::make_shared<ap::QuadrotorModel>(model_cfg);
  auto cfg = std::make_shared<ap::QuadrotorSimulator::Config>();

  auto sim = std::make_shared<ap::QuadrotorSimulator>(model, cfg);

  // Setup Controller
  // ================
  auto ctrl = std::make_shared<ap::CascadeController>(model);
  auto pos_ctrl = std::dynamic_pointer_cast<ap::GeometricPositionController>(
      ctrl->positionController());
  auto att_ctrl = std::dynamic_pointer_cast<ap::GeometricAttitudeController>(
      ctrl->attitudeController());

  if (!pos_ctrl || !att_ctrl) {
    spdlog::error("Controller casting failed.");
    return -1;
  }

  pos_ctrl->config()->kp = Eigen::Vector3d(1.0, 1.0, 3.0) / 2.0;
  pos_ctrl->config()->kv = Eigen::Vector3d(1.8, 1.8, 6.0);

  att_ctrl->config()->kR = {3.0, 3.0, 0.1};
  att_ctrl->config()->kOmega = {1.0, 1.0, 0.01};  // D-term equivalent

  // 3. Define Mission
  std::vector<ap::MissionWaypoint> mission = {
      {{0.0, 0.0, 1.0}, 0.0},
      {{5.0, 0.0, 1.0}, 0.0},
      {{5.0, 5.0, 1.0}, std::numbers::pi / 2},
      {{0.0, 5.0, 1.0}, std::numbers::pi},
      {{0.0, 0.0, 1.0}, 3.0 * std::numbers::pi / 2}};

  ap::MissionRunner::Config mission_cfg;
  mission_cfg.max_steps = 8000;
  ap::MissionRunner runner(sim, ctrl, mission, mission_cfg);

  // 4. EXECUTE (Fast!)
  spdlog::info("Running Simulation...");
  auto result = runner.run();
  spdlog::info("Simulation Complete. Steps: {}", result.time_history.size());

  struct RerunPose {
    rrc::Vector3D position;
    rr::Quaternion orientation;
  };

  std::vector<std::tuple<rrc::Translation3D, rr::Quaternion>> traj;
  std::ranges::transform(
      result.state_history, std::back_inserter(traj),
      [](const ap::QuadrotorState& s) {
        const Eigen::Vector3f p = s.odometry.pose().translation().cast<float>();
        const Eigen::Quaternionf q = s.odometry.pose().rotation().cast<float>();
        return std::tuple(
            rrc::Translation3D(p.x(), p.y(), p.z()),
            rr::Quaternion::from_xyzw(q.x(), q.y(), q.z(), q.w()));
      });

  const auto frame =
      rr::Arrows3D()
          .with_origins({rrc::Vector3D(0.0F, 0.0F, 0.0F)})
          .with_vectors(
              {{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}})
          .with_colors({kRed, kGreen, kBlue});
  rec.log("world/drone", frame);
  // 5. LOG (Post-Process)
  // We explicitly associate simulation time with the data here.
  for (size_t i = 0; i < result.time_history.size(); ++i) {
    double t = result.time_history[i];
    const auto& state = result.state_history[i];
    const auto& cmd = result.command_history[i];

    rec.set_time_duration_secs("sim_time", t);

    // -- Log Ground Truth --

    std::vector<rrc::Vector3D> positions(i + 1);
    std::ranges::copy(traj | std::views::take(i + 1) | std::views::elements<0>,
                      positions.begin());
    rec.log("world/trajectory", rr::LineStrips3D()
                                    .with_colors({0xFFFFFFFF})
                                    .with_radii({0.02f})
                                    .with_strips(rrc::LineStrip3D(positions)));

    // Rerun expects Quaternion as (x, y, z, w) or (w, x, y, z) depending on
    // constructor. Rerun C++ Quaternion::from_xyzw(x, y, z, w) matches Eigen's
    // internal storage order usually, but check Eigen::Quaternion coeffs()
    // order (x, y, z, w).
    auto tform = rr::Transform3D()
                     .with_translation(std::get<0>(traj[i]))
                     .with_quaternion(std::get<1>(traj[i]));
    rec.log("world/drone", tform);

    // -- Log Setpoint (Ghost Drone / Marker) --
    if (cmd.hasComponent(ap::QuadrotorStateComponent::kPosition)) {
      const Eigen::Vector3f setpoint = cmd.position().cast<float>();
      rec.log("world/setpoint",
              rr::Points3D({{setpoint.x(), setpoint.y(), setpoint.z()}})
                  .with_colors({0xFF0000FF})  // Red
                  .with_radii({0.05f}));
    } else {
      spdlog::warn("Command at t={} missing position component.", t);
    }

    // -- Log Motor Thrusts (as a bar chart or scalar series) --
    // We can log these as scalars to visualize saturation
    rec.log("sensors/motors/1", rr::Scalars(state.motor_thrusts[0]));
    rec.log("sensors/motors/2", rr::Scalars(state.motor_thrusts[1]));
    rec.log("sensors/motors/3", rr::Scalars(state.motor_thrusts[2]));
    rec.log("sensors/motors/4", rr::Scalars(state.motor_thrusts[3]));
  }

  spdlog::info("Done.");
  return 0;
}
