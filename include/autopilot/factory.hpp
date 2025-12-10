#ifndef AUTOPILOT_CORE_GENERIC_FACTORY_HPP_
#define AUTOPILOT_CORE_GENERIC_FACTORY_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "autopilot/base.hpp"
#include "autopilot/quadrotor_model.hpp"
#include "spdlog/logger.h"

namespace autopilot {

template <typename T>
concept Configurable = requires { typename T::Config; } &&
                       std::is_base_of_v<ConfigBase, typename T::Config>;

template <typename ProductBase>
class GenericFactory {
 public:
  using ModelPtr = std::shared_ptr<QuadrotorModel>;
  using ProductPtr = std::shared_ptr<ProductBase>;
  using ConfigPtr = std::shared_ptr<ConfigBase>;
  using LoggerPtr = std::shared_ptr<spdlog::logger>;

  // The internal signature for creating a product from a generic base config
  using ProductCreator =
      std::function<ProductPtr(ModelPtr, ConfigPtr, LoggerPtr)>;

  using ConfigCreator = std::function<ConfigPtr()>;

  GenericFactory() = delete;

  // =========================================================================
  // Registration
  // =========================================================================

  // We register a concrete type T. T must define:
  // 1. T::Config (which inherits ConfigBase)
  // 2. A constructor T(model, shared_ptr<T::Config>, logger)
  template <Configurable T>
  static bool Register(const std::string& key) {
    if (creators().find(key) != creators().end()) {
      return false;
    }

    // 1. Register the Config Creator
    config_creators().emplace(key, [] -> ConfigPtr {
      return std::make_shared<typename T::Config>();
    });

    // 2. Register the Product Creator (Solves the Matching Problem)
    creators().emplace(
        key,
        [](ModelPtr model, ConfigPtr cfg_base, LoggerPtr logger) -> ProductPtr {
          if (auto typed_cfg =
                  std::dynamic_pointer_cast<typename T::Config>(cfg_base)) {
            return std::make_shared<T>(std::move(model), std::move(typed_cfg),
                                       std::move(logger));
          }

          // Fallback: If someone passed a wrong config type manually
          // You might want to log an error here or throw
          return nullptr;
        });

    return true;
  }

  // =========================================================================
  // Creation
  // =========================================================================

  // 1. Create a Default Configuration
  static ConfigPtr CreateConfig(const std::string& key) {
    auto it = config_creators().find(key);
    if (it == config_creators().end()) {
      return nullptr;
    }
    return (it->second)();
  }

  template <std::derived_from<ConfigBase> T>
  static std::shared_ptr<T> CreateConfig(const char* key) {
    if (auto res =
            std::dynamic_pointer_cast<T>(CreateConfig(std::string(key)))) {
      return res;
    }
    return nullptr;
  }

  // 2. Create Product from existing Config (The "Smart" Create)
  static ProductPtr Create(ConfigPtr config, ModelPtr model,
                           LoggerPtr logger = nullptr) {
    if (!config) {
      return nullptr;
    }

    // Use the config's self-reported name to find the right factory
    std::string key = config->name();

    auto it = creators().find(key);
    if (it == creators().end()) {
      return nullptr;
    }

    return (it->second)(std::move(model), std::move(config),
                        logger ? std::move(logger) : nullptr);
  }

  template <std::derived_from<ProductBase> T>
  static std::shared_ptr<T> Create(std::shared_ptr<typename T::Config> config,
                                   ModelPtr model, LoggerPtr logger = nullptr) {
    if (auto res = std::dynamic_pointer_cast<T>(
            Create(std::move(config), std::move(model), std::move(logger)))) {
      return res;
    }
    return nullptr;
  }

  // 3. Create Default Product (Helper: creates default config internally)
  static ProductPtr Create(const std::string& key, ModelPtr model,
                           LoggerPtr logger = nullptr) {
    auto config = CreateConfig(key);
    if (!config) {
      return nullptr;
    }

    // We already have the key, so we could optimize, but reusing Create(config)
    // is safer
    return Create(config, std::move(model),
                  logger ? std::move(logger) : nullptr);
  }

 private:
  static std::map<std::string, ProductCreator>& creators() {
    static std::map<std::string, ProductCreator> impl;
    return impl;
  }

  static std::map<std::string, ConfigCreator>& config_creators() {
    static std::map<std::string, ConfigCreator> impl;
    return impl;
  }
};

template <typename T, typename Factory>
struct Registrar {
  explicit Registrar(std::string name) {
    // The new Register function is a template that generates
    // the creators internally, so we just pass the type T and the name.
    Factory::template Register<T>(std::move(name));
  }
};

}  // namespace autopilot

#endif  // AUTOPILOT_CORE_GENERIC_FACTORY_HPP_
