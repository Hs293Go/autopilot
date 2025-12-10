#ifndef AUTOPILOT_CONTROLLER_BASE_HPP_
#define AUTOPILOT_CONTROLLER_BASE_HPP_

#include "autopilot/definitions.hpp"
#include "autopilot/expected.hpp"
#include "autopilot/factory.hpp"
#include "autopilot/module.hpp"
#include "autopilot/quadrotor_model.hpp"

namespace autopilot {

class ControllerBase : public Module {
 public:
  ControllerBase(const std::string& name, std::shared_ptr<QuadrotorModel> model,
                 std::shared_ptr<spdlog::logger> logger = nullptr)
      : Module(fmt::format("Controller.{}", name), std::move(model),
               std::move(logger)) {}

  virtual expected<std::size_t, std::error_code> compute(
      const QuadrotorState& state, std::span<const QuadrotorCommand> setpoints,
      std::span<QuadrotorCommand> outputs) = 0;

  virtual void reset() = 0;
};

using ControllerFactory = GenericFactory<ControllerBase>;

}  // namespace autopilot

#define REGISTER_CONTROLLER(ConcreteType, KeyName)                \
  static const autopilot::Registrar<ConcreteType,                 \
                                    autopilot::ControllerFactory> \
  kRegistrarFor##ConcreteType(KeyName)

#endif  // AUTOPILOT_CONTROLLER_BASE_HPP_
