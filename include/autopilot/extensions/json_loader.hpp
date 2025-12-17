#ifndef INCLUDE_EXT_JSON_LOADER_HPP_
#define INCLUDE_EXT_JSON_LOADER_HPP_

#include <filesystem>
#include <stack>

#include "autopilot/base/config_base.hpp"
#include "nlohmann/json_fwd.hpp"

namespace autopilot {

class JsonLoader final : public LoaderVisitor {
 public:
  JsonLoader() = delete;

  // 1. Parse on Construction
  static std::shared_ptr<JsonLoader> FromFile(
      const std::filesystem::path& path);
  static std::shared_ptr<JsonLoader> FromString(std::string_view str);

  VisitResult safeVisit(std::string_view key, double& value,
                        const F64Properties& props) override;

  VisitResult safeVisit(std::string_view key, std::int64_t& value,
                        const I64Properties& props) override;

  VisitResult safeVisit(std::string_view key, bool& value,
                        const Properties& props) override;

  VisitResult safeVisit(std::string_view key, std::string& value,
                        const StrProperties& props) override;

  VisitResult safeVisit(std::string_view key, std::span<double> value,
                        const F64Properties& props) override;

  VisitResult safeVisit(std::string_view key, std::span<std::int64_t> value,
                        const I64Properties& props) override;

  VisitResult safeVisit(std::string_view key, ConfigBase& config,
                        const Properties& props) override;

  VisitResult safeVisit(std::string_view key,
                        std::shared_ptr<ConfigBase> config,
                        const Properties& props) override;

 private:
  explicit JsonLoader(std::shared_ptr<nlohmann::json> content);

  // We need pointer stability for the root object
  std::shared_ptr<nlohmann::json> root_;
  std::stack<nlohmann::json*> node_stack_;
};
}  // namespace autopilot

#endif  // INCLUDE_EXT_JSON_LOADER_HPP_
