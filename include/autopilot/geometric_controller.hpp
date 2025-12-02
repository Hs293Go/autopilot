#ifndef INCLUDE_AUTOPILOT_LEE_POSITION_CONTROLLER_HPP_
#define INCLUDE_AUTOPILOT_LEE_POSITION_CONTROLLER_HPP_

#include <utility>

#include "autopilot/cascade_controller.hpp"
namespace autopilot {
// =================================================================
// Geometric Position Controller
// Logic: PD Control on Euclidean Error + FeedForward (Sreenath/Lee)
// =================================================================

class GeometricPositionController : public PositionControllerBase {
 public:
  // 1. Matched Config Struct (Distinct)
  struct Config {
    // Gains (PD style, as this acts on R^3 errors)
    Eigen::Vector3d kp = Eigen::Vector3d::Zero();
    Eigen::Vector3d kv = Eigen::Vector3d::Zero();
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
  // 1. Matched Config Struct (Distinct)
  struct Config {
    // Gains (Geometric style, acting on SO(3) metric)
    Eigen::Vector3d kR = Eigen::Vector3d::Zero();
    Eigen::Vector3d kOmega = Eigen::Vector3d::Zero();

    // Toggle Strategy
    // true = Use current state to cancel nonlinearities (Lee 2010). Sensitive
    // to noise. false = Use reference state for feedforward. Robust to noise.
    bool enable_exact_linearization = false;
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

#endif  // INCLUDE_AUTOPILOT_LEE_POSITION_CONTROLLER_HPP_
