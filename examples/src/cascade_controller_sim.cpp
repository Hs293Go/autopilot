#include <iostream>
#include <ranges>
#include <rerun.hpp>

#include "autopilot/controllers/cascade_controller.hpp"
#include "autopilot/core/quadrotor_model.hpp"
#include "autopilot/estimators/async_estimator.hpp"
#include "autopilot/estimators/eskf.hpp"
#include "autopilot/extensions/json_loader.hpp"
#include "autopilot/extensions/pretty_printer.hpp"
#include "validation/mission_runner.hpp"

// Use a distinct namespace or alias for clarity
namespace rr = rerun;
namespace rrc = rerun::components;
namespace ap = autopilot;

static const rrc::Color kRed(255, 0, 0);
static const rrc::Color kGreen(0, 255, 0);
static const rrc::Color kBlue(0, 0, 255);

struct MainConfig : public ap::ReflectiveConfigBase<MainConfig> {
  std::string_view name() const override { return "SimConfig"; }
  std::shared_ptr<ap::QuadrotorModelCfg> quadrotor_model =
      std::make_shared<ap::QuadrotorModelCfg>();
  ap::Polymorphic<ap::ControllerFactory> controller;

  std::shared_ptr<ap::EskfEstimator::Config> eskf =
      std::make_shared<ap::EskfEstimator::Config>();

  std::shared_ptr<ap::QuadrotorSimulator::Config> simulator =
      std::make_shared<ap::QuadrotorSimulator::Config>();

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("quadrotor_model", &MainConfig::quadrotor_model,
               ap::Properties{.desc = "Quadrotor Model Configuration",
                              .prefer_user_provided = true}),
      Describe("controller", &MainConfig::controller,
               ap::Properties{.desc = "Controller Configuration",
                              .prefer_user_provided = true}),
      Describe("eskf", &MainConfig::eskf,
               ap::Properties{.desc = "Error State Kalman Filter Configuration",
                              .prefer_user_provided = true}),
      Describe("simulator", &MainConfig::simulator,
               ap::Properties{.desc = "Quadrotor Simulator Configuration"}));
};

int main() {
  // 1. Setup Rerun
  auto rec = rr::RecordingStream("autopilot_mission");
  rec.spawn().exit_on_failure();

  // Setup Model
  // ===========
  auto loader = autopilot::JsonLoader::FromFile(CONFIG_FILE);
  if (!loader) {
    spdlog::error("Failed to load JSON config from file: {}", CONFIG_FILE);
    return -1;
  }

  MainConfig cfg;
  auto res = cfg.accept(*loader);
  if (res.ec) {
    spdlog::error("Failed to load SimConfig from JSON: {} for {}",
                  res.ec.message(), res.key);
    return -1;
  }

  auto model = std::make_shared<ap::QuadrotorModel>(cfg.quadrotor_model);

  auto ctrl = ap::ControllerFactory::Create(cfg.controller.config, model);

  auto est_alg = std::make_shared<ap::EskfEstimator>(model, cfg.eskf);
  auto est = std::make_shared<ap::AsyncEstimator>(est_alg);

  auto sim = std::make_shared<ap::QuadrotorSimulator>(model, cfg.simulator);

  if (auto ec = est->reset(sim->state(), Eigen::MatrixXd::Identity(15, 15))) {
    spdlog::error("Estimator reset failed: {}", ec.message());
    return -1;
  }

  auto printer = autopilot::PrettyPrinter(std::cout);
  std::ignore = cfg.accept(printer);

  // 3. Define Mission
  std::vector<ap::MissionWaypoint> mission = {
      {{0.0, 0.0, 1.0}, 0.0},
      {{5.0, 0.0, 1.0}, 0.0},
      {{5.0, 5.0, 1.0}, std::numbers::pi / 2},
      {{0.0, 5.0, 1.0}, std::numbers::pi},
      {{0.0, 0.0, 1.0}, 3.0 * std::numbers::pi / 2}};

  ap::MissionRunner::Config mission_cfg;
  mission_cfg.dt_control = 0.005;
  mission_cfg.max_steps = 12000;
  // ap::MissionRunner runner(sim, ctrl, mission, mission_cfg);
  ap::MissionRunner runner(sim, ctrl, est, mission, mission_cfg);

  // 4. EXECUTE (Fast!)
  spdlog::info("Running Simulation...");
  auto result = runner.run();
  spdlog::info("Simulation Complete. Steps: {}", result.time.size());

  struct RerunHistory {
    rrc::Translation3D real_position;
    rr::Quaternion real_orientation;
    rrc::Translation3D est_position;
    rr::Quaternion est_orientation;
    float motor_thrusts[4];
  };

  std::vector<RerunHistory> hist;
  std::ranges::transform(
      result.hist, std::back_inserter(hist), [](const autopilot::History& it) {
        const auto& [real_state, est_state, cmd] = it;
        const Eigen::Vector3f p =
            real_state.odometry.pose().translation().cast<float>();
        const Eigen::Quaternionf q =
            real_state.odometry.pose().rotation().cast<float>();
        const Eigen::Vector3f p_est =
            est_state.odometry.pose().translation().cast<float>();
        const Eigen::Quaternionf q_est =
            est_state.odometry.pose().rotation().cast<float>();
        return RerunHistory{
            .real_position = rrc::Translation3D(p.x(), p.y(), p.z()),
            .real_orientation =
                rr::Quaternion::from_xyzw(q.x(), q.y(), q.z(), q.w()),
            .est_position = rrc::Translation3D(p_est.x(), p_est.y(), p_est.z()),
            .est_orientation = rr::Quaternion::from_xyzw(q_est.x(), q_est.y(),
                                                         q_est.z(), q_est.w()),
            .motor_thrusts = {static_cast<float>(real_state.motor_thrusts[0]),
                              static_cast<float>(real_state.motor_thrusts[1]),
                              static_cast<float>(real_state.motor_thrusts[2]),
                              static_cast<float>(real_state.motor_thrusts[3])}};
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
  for (size_t i = 0; i < result.time.size(); ++i) {
    double t = result.time[i];

    rec.set_time_duration_secs("sim_time", t);

    // -- Log Ground Truth --

    std::vector<rrc::Vector3D> real_positions(i + 1);
    auto get_position = [](const RerunHistory& h) { return h.real_position; };
    std::ranges::copy(
        hist | std::views::take(i + 1) | std::views::transform(get_position),
        real_positions.begin());
    rec.log("world/trajectory",
            rr::LineStrips3D()
                .with_colors({0xFFFFFFFF})
                .with_radii({0.02f})
                .with_strips(rrc::LineStrip3D(real_positions)));
    std::vector<rrc::Vector3D> est_positions(i + 1);
    std::ranges::copy(
        hist | std::views::take(i + 1) | std::views::transform(get_position),
        est_positions.begin());
    rec.log("world/est_trajectory",
            rr::LineStrips3D()
                .with_colors({0xFFFF00FF})
                .with_radii({0.04f})
                .with_strips(rrc::LineStrip3D(est_positions)));

    // Rerun expects Quaternion as (x, y, z, w) or (w, x, y, z) depending on
    // constructor. Rerun C++ Quaternion::from_xyzw(x, y, z, w) matches Eigen's
    // internal storage order usually, but check Eigen::Quaternion coeffs()
    // order (x, y, z, w).
    auto tform = rr::Transform3D()
                     .with_translation(hist[i].real_position)
                     .with_quaternion(hist[i].real_orientation);
    rec.log("world/drone", tform);

    auto cmd = result.hist[i].command;
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
    rec.log("actuators/motors/1", rr::Scalars(hist[i].motor_thrusts[0]));
    rec.log("actuators/motors/2", rr::Scalars(hist[i].motor_thrusts[1]));
    rec.log("actuators/motors/3", rr::Scalars(hist[i].motor_thrusts[2]));
    rec.log("actuators/motors/4", rr::Scalars(hist[i].motor_thrusts[3]));

    rec.log("commands/force/x",
            rr::Scalars(result.hist[i].real_state.wrench.force().x()));
    rec.log("commands/force/y",
            rr::Scalars(result.hist[i].real_state.wrench.force().y()));
    rec.log("commands/force/z",
            rr::Scalars(result.hist[i].real_state.wrench.force().z()));

    rec.log("sensors/body_rate/x",
            rr::Scalars(
                result.hist[i].estimated_state.odometry.twist().angular().x()));
    rec.log("sensors/body_rate/y",
            rr::Scalars(
                result.hist[i].estimated_state.odometry.twist().angular().y()));
    rec.log("sensors/body_rate/z",
            rr::Scalars(
                result.hist[i].estimated_state.odometry.twist().angular().z()));
  }

  spdlog::info("Done.");
  return 0;
}
