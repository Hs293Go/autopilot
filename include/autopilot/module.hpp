#ifndef INCLUDE_AUTOPILOT_MODULE_HPP_
#define INCLUDE_AUTOPILOT_MODULE_HPP_

#include <memory>
#include <string>

#include "spdlog/spdlog.h"

namespace autopilot {

class QuadrotorModel;

class Module {
 public:
  Module(const std::string& name, std::shared_ptr<QuadrotorModel> model,
         std::shared_ptr<spdlog::logger> logger = nullptr)
      : model_(std::move(model)),
        logger_(InitLogger(name, std::move(logger))) {}

  virtual ~Module() = default;

  std::string name() const { return logger_->name(); }

  std::shared_ptr<const QuadrotorModel> model() const { return model_; }

  std::shared_ptr<QuadrotorModel> model() { return model_; }

  std::shared_ptr<spdlog::logger> logger() const { return logger_; }

 private:
  static std::shared_ptr<spdlog::logger> InitLogger(
      const std::string& name, std::shared_ptr<spdlog::logger> logger) {
    if (logger != nullptr) {
      return logger;
    }

    if (auto existing_logger = spdlog::get(name)) {
      return existing_logger;
    }

    // Mirror the sinks of the default logger
    const auto& sinks = spdlog::default_logger()->sinks();
    auto new_logger =
        std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    spdlog::register_logger(new_logger);
    return new_logger;
  }

  std::shared_ptr<QuadrotorModel> model_;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace autopilot

#endif  // INCLUDE_AUTOPILOT_MODULE_HPP_
