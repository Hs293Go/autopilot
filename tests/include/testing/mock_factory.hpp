#include "autopilot/base/factory.hpp"
#include "autopilot/base/module.hpp"

namespace testing {

namespace ap = autopilot;

class MockComponentBase : public ap::Module {
 public:
  static constexpr std::string_view kModuleRootType = "MockComponent";

  using ap::Module::Module;
  std::string_view name() const override { return "MockComponent"; }
};

using MockComponentFactory = ap::GenericFactory<MockComponentBase>;

class ConcreteMockComponent final : public testing::MockComponentBase {
 public:
  static constexpr char kName[] = "ConcreteMockComponent";

  struct Config : public ap::ReflectiveConfigBase<Config> {
    std::string_view name() const override { return kName; }

    std::int64_t some_parameter = 42;

    static constexpr auto kDescriptors = std::make_tuple(
        ap::Describe("some_parameter", &Config::some_parameter,
                     ap::I64Properties{.desc = "An example parameter"}));
  };

  ConcreteMockComponent(std::shared_ptr<ap::QuadrotorModel> model,
                        std::shared_ptr<Config> config,
                        std::shared_ptr<spdlog::logger> logger)
      : testing::MockComponentBase(kName, std::move(model), std::move(logger)),
        config_(std::move(config)) {}

  std::string_view name() const override { return kName; }

 private:
  std::shared_ptr<Config> config_;
};

}  // namespace testing
