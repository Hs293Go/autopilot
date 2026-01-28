#include <ranges>

#include "autopilot/core/quadrotor_model.hpp"
#include "autopilot/estimators/async_estimator.hpp"
#include "autopilot/extensions/json_loader.hpp"
#include "autopilot/extensions/pretty_printer.hpp"
#include "autopilot/planning/minimum_snap_solver.hpp"
#include "examples/visualization.hpp"
#include "rerun/recording_stream.hpp"
#include "validation/mission_runner.hpp"

// Use a distinct namespace or alias for clarity
namespace rr = rerun;
namespace rrc = rerun::components;
namespace ap = autopilot;

struct MainConfig : public ap::ReflectiveConfigBase<MainConfig> {
  std::string_view name() const override { return "SimConfig"; }
  std::shared_ptr<ap::QuadrotorModelCfg> quadrotor_model =
      std::make_shared<ap::QuadrotorModelCfg>();
  ap::Polymorphic<ap::ControllerFactory> controller;

  ap::Polymorphic<ap::EstimatorFactory> estimator;

  std::shared_ptr<ap::QuadrotorSimulator::Config> simulator =
      std::make_shared<ap::QuadrotorSimulator::Config>();

  ap::MissionRunner::Config mission;

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("quadrotor_model", &MainConfig::quadrotor_model,
               ap::Properties{.desc = "Quadrotor Model Configuration",
                              .prefer_user_provided = true}),
      Describe("controller", &MainConfig::controller,
               ap::Properties{.desc = "Controller Configuration",
                              .prefer_user_provided = true}),
      Describe("estimator", &MainConfig::estimator,
               ap::Properties{.desc = "Estimator Configuration",
                              .prefer_user_provided = true}),
      Describe("simulator", &MainConfig::simulator,
               ap::Properties{.desc = "Quadrotor Simulator Configuration"}),
      Describe("mission", &MainConfig::mission,
               ap::Properties{.desc = "Mission Runner Configuration"}));
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

  rec.set_time_duration_secs("sim_time", 0.0);
  ap::examples::QuadrotorVisualizer viz(
      rec, "world", static_cast<std::size_t>(cfg.mission.max_steps));
  ap::examples::MultiaxisPlotter motor_plotter(
      rec, {.axis_count = 4,
            .prefix = "actuators/motors",
            .suffix_style = ap::examples::SuffixStyle::kIndex,
            .line_specs = {}});
  ap::examples::MultiaxisPlotter force_plotter(
      rec, {.axis_count = 3,
            .prefix = "commands/force",
            .suffix_style = ap::examples::SuffixStyle::kComponents,
            .line_specs = {
                {0xFFFFFF00, 2.0f}, {0xFF00FF00, 2.0f}, {0xFF0000FF, 2.0f}}});

  ap::examples::Plotter est_err_plotter(
      rec, "estimation/position_error_magnitude",
      autopilot::examples::LineSpec{.color = 0xFFFFFF00, .linewidth = 2.0f});

  ap::examples::Plotter est_var_plotter(
      rec, "estimation/position_3sigma_magnitude",
      autopilot::examples::LineSpec{.color = 0xFF00FFFF, .linewidth = 2.0f});

  auto model = std::make_shared<ap::QuadrotorModel>(cfg.quadrotor_model);

  auto ctrl = ap::ControllerFactory::Create(cfg.controller.config, model);

  auto est_alg = ap::EstimatorFactory::Create(cfg.estimator.config, model);
  auto est = std::make_shared<ap::AsyncEstimator>(est_alg);

  auto sim = std::make_shared<ap::QuadrotorSimulator>(model, cfg.simulator);

  if (auto ec = est->resetState(sim->state());
      ec != autopilot::AutopilotErrc::kNone) {
    spdlog::error("Estimator reset failed: {}", ec);
    return -1;
  }

  spdlog::info("Simulation Configuration:\n{}", cfg);

  // 3. Define Mission
  const Eigen::Vector3d wps[] = {{0.0, 0.0, 1.0},
                                 {5.0, 0.0, 1.0},
                                 {5.0, 5.0, 1.0},
                                 {0.0, 5.0, 1.0},
                                 {0.0, 0.0, 1.0}};

  ap::MinimumSnapSolver traj_solver;

  ap::Mission mission;
  for (int i = 0; i < std::ssize(wps); ++i) {
    mission.lineTo(wps[i], 5.0, traj_solver);
    if (i > 0) {
      mission.yawBy(std::numbers::pi / 2.0, 3.0);
    }
  }

  // ap::MissionRunner runner(sim, ctrl, mission, mission_cfg);
  ap::MissionRunner runner(sim, ctrl, est, mission, cfg.mission);

  // 4. EXECUTE (Fast!)
  spdlog::info("Running Simulation...");
  auto result = runner.run();
  spdlog::info("Simulation Complete. Steps: {}", result.time.size());

  // 5. LOG (Post-Process)
  // We explicitly associate simulation time with the data here.
  for (size_t i = 0; i < result.time.size(); ++i) {
    double t = result.time[i];

    rec.set_time_duration_secs("sim_time", t);

    // -- Log Ground Truth --

    viz.log(result.hist[i].real_state, result.hist[i].command);

    // -- Log Motor Thrusts (as a bar chart or scalar series) --
    // We can log these as scalars to visualize saturation
    motor_plotter.log(result.hist[i].real_state.motor_thrusts);
    force_plotter.log(result.hist[i].real_state.wrench.force());

    const double position_est_error_norm =
        (result.hist[i].real_state.odometry.pose().translation() -
         result.hist[i].estimated_state.odometry.pose().translation())
            .norm();

    const double position_est_3sigma =
        3.0 * std::sqrt(result.hist[i].est_variance(Eigen::seqN(0, 3)).sum());
    est_err_plotter.log(position_est_error_norm);
    est_var_plotter.log(position_est_3sigma);
  }

  spdlog::info("Done.");
  return 0;
}
