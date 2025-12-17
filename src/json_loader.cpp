#include "autopilot/extensions/json_loader.hpp"

#include <fstream>
#include <utility>

#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

namespace autopilot {

using nlohmann::json;

namespace {

VisitResult MaybePreferUserProvided(std::string_view key,
                                    bool prefer_user_provided) {
  if (prefer_user_provided) {
    return {make_error_code(AutopilotErrc::kConfigKeyMissing), key};
  }
  return {};
}

template <typename T>
VisitResult ProcessValue(std::string_view key, const json& node, T& value) {
  if (const auto* cand = node.get_ptr<const T*>()) {
    value = *cand;
    return {};
  }

  return {make_error_code(AutopilotErrc::kConfigTypeMismatch), key};
}

VisitResult ValidateArray(std::string_view key, const json& arr,
                          std::size_t expected_size) {
  if (!arr.is_array()) {
    return {make_error_code(AutopilotErrc::kConfigTypeMismatch), key};
  }

  if (arr.size() != expected_size) {
    return {make_error_code(AutopilotErrc::kConfigSizeMismatch), key};
  }
  return {};
}

}  // namespace

JsonLoader::JsonLoader(std::shared_ptr<nlohmann::json> content)
    : root_(std::move(content)) {
  node_stack_.push(root_.get());
}

VisitResult JsonLoader::safeVisit(std::string_view key, double& value,
                                  const F64Properties& props) {
  const auto* node = node_stack_.top();
  if (!node->contains(key)) {
    return MaybePreferUserProvided(key, props.prefer_user_provided);
  }

  return ProcessValue(key, node->at(std::string(key)), value);
}

VisitResult JsonLoader::safeVisit(std::string_view key, std::int64_t& value,
                                  const I64Properties& props) {
  const auto* node = node_stack_.top();
  if (!node->contains(key)) {
    return MaybePreferUserProvided(key, props.prefer_user_provided);
  }

  return ProcessValue(key, node->at(std::string(key)), value);
}

VisitResult JsonLoader::safeVisit(std::string_view key, bool& value,
                                  const Properties& props) {
  const auto* node = node_stack_.top();
  if (!node->contains(key)) {
    return MaybePreferUserProvided(key, props.prefer_user_provided);
  }
  return ProcessValue(key, node->at(std::string(key)), value);
}

VisitResult JsonLoader::safeVisit(std::string_view key, std::string& value,
                                  const StrProperties& props) {
  const auto* node = node_stack_.top();
  if (!node->contains(key)) {
    return MaybePreferUserProvided(key, props.prefer_user_provided);
  }
  return ProcessValue(key, node->at(std::string(key)), value);
}

VisitResult JsonLoader::safeVisit(std::string_view key, std::span<double> value,
                                  const F64Properties& props) {
  const auto* node = node_stack_.top();
  if (!node->contains(key)) {
    return MaybePreferUserProvided(key, props.prefer_user_provided);
  }

  const auto& arr = node->at(std::string(key));
  if (auto result = ValidateArray(key, arr, value.size()); result.ec) {
    return result;
  }

  for (std::size_t i = 0; i < arr.size(); ++i) {
    if (auto result = ProcessValue(key, arr[i], value[i]); result.ec) {
      return result;
    }
  }
  return {};
}

VisitResult JsonLoader::safeVisit(std::string_view key,
                                  std::span<std::int64_t> value,
                                  const I64Properties& props) {
  const auto* node = node_stack_.top();
  if (!node->contains(key)) {
    return MaybePreferUserProvided(key, props.prefer_user_provided);
  }

  const auto& arr = node->at(std::string(key));
  if (auto result = ValidateArray(key, arr, value.size()); result.ec) {
    return result;
  }
  for (std::size_t i = 0; i < arr.size(); ++i) {
    if (auto result = ProcessValue(key, arr[i], value[i]); result.ec) {
      return result;
    }
  }
  return {};
}

VisitResult JsonLoader::safeVisit(std::string_view key, ConfigBase& config,
                                  const Properties& props) {
  auto* node = node_stack_.top();
  if (!node->contains(key)) {
    return MaybePreferUserProvided(key, props.prefer_user_provided);
  }

  auto& child_node = node->at(std::string(key));
  if (!child_node.is_object()) {
    return {make_error_code(AutopilotErrc::kConfigTypeMismatch), key};
  }

  // PUSH: Enter the subsection
  node_stack_.push(&child_node);

  // RECURSE: The child config now safeVisits its fields against 'child_node'
  VisitResult result;
  result = config.accept(*this);
  node_stack_.pop();

  return result;
}

VisitResult JsonLoader::safeVisit(std::string_view key,
                                  std::shared_ptr<ConfigBase> config,
                                  const Properties& props) {
  auto* node = node_stack_.top();
  if (!node->contains(key)) {
    return MaybePreferUserProvided(key, props.prefer_user_provided);
  }

  auto& child_node = node->at(std::string(key));
  if (!child_node.is_object()) {
    return {make_error_code(AutopilotErrc::kConfigTypeMismatch), key};
  }

  // PUSH: Enter the subsection
  node_stack_.push(&child_node);

  // RECURSE: The child config now safeVisits its fields against 'child_node'
  VisitResult result;
  if (config) {
    result = config->accept(*this);
  }  // POP: Return to parent
  node_stack_.pop();

  return result;
}

std::shared_ptr<JsonLoader> JsonLoader::FromString(std::string_view str) {
  auto content = std::make_shared<json>(json::parse(str, nullptr, false));
  if (content->is_discarded()) {
    return nullptr;  // Invalid JSON
  }

  std::shared_ptr<JsonLoader> loader(new JsonLoader(std::move(content)));
  return loader;
}

std::shared_ptr<JsonLoader> JsonLoader::FromFile(
    const std::filesystem::path& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return nullptr;  // Failed to open file
  }
  auto content = std::make_shared<json>(json::parse(ifs, nullptr, false));
  if (content->is_discarded()) {
    return nullptr;  // Invalid JSON
  }

  std::shared_ptr<JsonLoader> loader(new JsonLoader(std::move(content)));
  return loader;
}
}  // namespace autopilot
