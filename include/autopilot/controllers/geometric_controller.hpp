#ifndef AUTOPILOT_LEE_POSITION_CONTROLLER_HPP_
#define AUTOPILOT_LEE_POSITION_CONTROLLER_HPP_

#include "autopilot/controllers/cascade_controller.hpp"
namespace autopilot {
// =================================================================
// Geometric Position Controller
// Logic: PD Control on Euclidean Error + FeedForward (Sreenath/Lee)
// =================================================================

class GeometricPositionController : public PositionControllerBase {
 public:
  static constexpr char kName[] = "GeometricPosition";

  // 1. Matched Config Struct (Distinct)
  struct Config final : ReflectiveConfigBase<Config> {
    std::string name() const override { return kName; }

    // Gains (PD style, as this acts on R^3 errors)
    Eigen::Vector3d kp = Eigen::Vector3d::Constant(1.0);
    Eigen::Vector3d kv = Eigen::Vector3d::Constant(0.1);

    static constexpr auto kDescriptors = std::make_tuple(
        Describe("kp", &Config::kp,
                 F64Properties{.desc = "Position Proportional Gains",
                               .bounds = Bounds<double>::Positive()}),
        Describe("kv", &Config::kv,
                 F64Properties{.desc = "Position Derivative Gains",
                               .bounds = Bounds<double>::Positive()}));
  };

  GeometricPositionController(std::shared_ptr<QuadrotorModel> model,
                              std::shared_ptr<Config> config,
                              std::shared_ptr<spdlog::logger> logger = nullptr);

  GeometricPositionController(std::shared_ptr<QuadrotorModel> model,
                              std::shared_ptr<spdlog::logger> logger = nullptr);

  // Runtime Tuning
  std::shared_ptr<const Config> config() const { return config_; }

  std::shared_ptr<Config> config() { return config_; }

  std::error_code compute(const QuadrotorState& state,
                          const PositionReference& ref,
                          PositionOutput& out) const override;

 private:
  std::shared_ptr<Config> config_;
};

// =================================================================
// Geometric Attitude Controller
// Logic: Nonlinear Control on SO(3) Manifold (Sreenath/Lee)
// =================================================================

class GeometricAttitudeController : public AttitudeControllerBase {
 public:
  static constexpr char kName[] = "GeometricAttitude";
  // 1. Matched Config Struct (Distinct)
  struct Config final : ReflectiveConfigBase<Config> {
    std::string name() const override { return kName; }

    // Gains (Geometric style, acting on SO(3) metric)
    Eigen::Vector3d kR = Eigen::Vector3d::Constant(1.0);
    Eigen::Vector3d kOmega = Eigen::Vector3d::Constant(0.1);

    // Toggle Strategy
    // true = Use current state to cancel nonlinearities (Lee 2010). Sensitive
    // to noise. false = Use reference state for feedforward. Robust to noise.
    bool enable_exact_linearization = false;

    static constexpr auto kDescriptors = std::make_tuple(
        Describe("kR", &Config::kR,
                 F64Properties{.desc = "Attitude Error Gains",
                               .bounds = Bounds<double>::Positive()}),
        Describe("kOmega", &Config::kOmega,
                 F64Properties{.desc = "Angular Velocity Error Gains",
                               .bounds = Bounds<double>::Positive()}),
        Describe("enable_exact_linearization",
                 &Config::enable_exact_linearization,
                 Properties{.desc = "Enable Exact Linearization (Lee 2010)"}));
  };

  GeometricAttitudeController(std::shared_ptr<QuadrotorModel> model,
                              std::shared_ptr<Config> config,
                              std::shared_ptr<spdlog::logger> logger = nullptr);

  GeometricAttitudeController(std::shared_ptr<QuadrotorModel> model,
                              std::shared_ptr<spdlog::logger> logger = nullptr);

  std::shared_ptr<const Config> config() const { return config_; }

  std::shared_ptr<Config> config() { return config_; }

  std::error_code compute(const QuadrotorState& state,
                          const AttitudeReference& ref,
                          AttitudeOutput& out) const override;

 private:
  std::shared_ptr<Config> config_;
};
}  // namespace autopilot

#endif  // AUTOPILOT_LEE_POSITION_CONTROLLER_HPP_
