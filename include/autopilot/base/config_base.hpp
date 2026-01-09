#ifndef AUTOPILOT_BASE_HPP_
#define AUTOPILOT_BASE_HPP_

#include <memory>
#include <variant>

#include "autopilot/core/definitions.hpp"

namespace autopilot {

template <typename T>
struct Inclusive {
  T value;
};

template <typename T>
struct Exclusive {
  T value;
};

template <typename T>
struct Bounds {
  std::variant<std::monostate, Inclusive<T>, Exclusive<T>> lower =
      std::monostate();
  std::variant<std::monostate, Inclusive<T>, Exclusive<T>> upper =
      std::monostate();

  /// Check if a value is within the bounds
  bool check(T val) const {
    return std::visit(
               Overload{
                   [](auto) { return true; },
                   [val](const Inclusive<T>& lb) { return val >= lb.value; },
                   [val](const Exclusive<T>& lb) { return val > lb.value; },
               },
               lower) &&
           std::visit(
               Overload{
                   [](auto) { return true; },
                   [val](const Inclusive<T>& ub) { return val <= ub.value; },
                   [val](const Exclusive<T>& ub) { return val < ub.value; },
               },
               upper);
  }

  template <std::ranges::input_range R>
  bool checkRange(R&& range) const {
    return std::ranges::all_of(std::forward<R>(range),
                               std::bind_front(&Bounds<T>::check, this));
  }

  static constexpr Bounds GreaterThan(T val) {
    return {.lower = Exclusive(val)};
  }

  static constexpr Bounds AtLeast(T val) { return {.lower = Inclusive(val)}; }

  static constexpr Bounds LessThan(T val) { return {.upper = Exclusive(val)}; }

  static constexpr Bounds AtMost(T val) { return {.upper = Inclusive(val)}; }

  static constexpr Bounds Positive() { return GreaterThan(0); }

  static constexpr Bounds NonNegative() { return AtLeast(0); }

  static constexpr Bounds OpenInterval(T lower, T upper) {
    return {.lower = Exclusive(lower), .upper = Exclusive(upper)};
  }

  static constexpr Bounds ClosedInterval(T lower, T upper) {
    return {.lower = Inclusive(lower), .upper = Inclusive(upper)};
  }
};

struct Properties {
  /// Human-readable description of the parameter
  std::string_view desc;

  /// Whether to warn if the parameter is not set by the user
  bool prefer_user_provided = false;
};

struct F64Properties {
  /// Human-readable description of the parameter
  std::string_view desc;

  /// Whether to warn if the parameter is not set by the user
  bool prefer_user_provided = false;

  /// Valid range for the parameter
  Bounds<double> bounds = {};
};

struct EnumItem {
  std::string_view name;
  std::int64_t underlying;
};

/// Integer properties are special because integral values may be mapped to
/// strings
struct I64Properties {
  /// Human-readable description of the parameter
  std::string_view desc;

  /// Whether to warn if the parameter is not set by the user
  bool prefer_user_provided = false;

  /// Valid range for the parameter
  Bounds<std::int64_t> bounds = {};

  static constexpr auto kNoMapping = std::span<const EnumItem>{};
  // A view into a static array of mappings
  std::span<const EnumItem> map = kNoMapping;
};

struct StrProperties {
  /// Human-readable description of the parameter
  std::string_view desc;

  /// Whether to warn if the parameter is not set by the user
  bool prefer_user_provided = false;

  bool non_empty = false;
};

template <auto... Args>
struct EnumMapping {
  static_assert(sizeof...(Args) % 2 == 0, "Must be key-value pairs");
  static constexpr size_t kSize = sizeof...(Args) / 2;

  static constexpr std::array<EnumItem, kSize> value =
      []<size_t... Is>(std::index_sequence<Is...> /*unused*/) {
        return std::to_array({EnumItem{
            .name = nth_value<Is * 2>(Args...),
            .underlying =
                static_cast<std::int64_t>(nth_value<Is * 2 + 1>(Args...)),
        }...});
      }(std::make_index_sequence<kSize>());
};

template <auto... A>
constexpr auto kEnumMapping = EnumMapping<A...>::value;

struct ConfigBase;

struct VisitResult {
  std::error_code ec;
  std::string_view key;
  std::string_view message = "none";
};
/// Visitor Pattern for Configurations
struct ConfigVisitor {
  virtual ~ConfigVisitor() = default;

  /// Visit a double value
  virtual VisitResult visit(std::string_view key, double& value,
                            const F64Properties& props) = 0;

  /// Visit an integer value
  virtual VisitResult visit(std::string_view key, std::int64_t& value,
                            const I64Properties& props) = 0;

  /// Visit a boolean value
  virtual VisitResult visit(std::string_view key, bool& value,
                            const Properties& props) = 0;

  /// Visit a string value
  virtual VisitResult visit(std::string_view key, std::string& value,
                            const StrProperties& props) = 0;

  /// Visit a contiguous range of doubles
  virtual VisitResult visit(std::string_view key, std::span<double> /*value*/,
                            const F64Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

  /// Visit a contiguous range of integers
  virtual VisitResult visit(std::string_view key,
                            std::span<std::int64_t> /*value*/,
                            const I64Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

  virtual VisitResult visit(std::string_view key,
                            ConfigBase& /*config*/,  // NOLINT
                            const Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

  /// Visit a nested configuration
  virtual VisitResult visit(
      std::string_view key,
      std::shared_ptr<ConfigBase> /*config NOLINT(performance-*)*/,
      const Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }
};
struct ConstConfigVisitor {
  virtual ~ConstConfigVisitor() = default;

  virtual VisitResult visit(std::string_view key, double value,
                            const F64Properties& props) = 0;

  virtual VisitResult visit(std::string_view key, std::int64_t value,
                            const I64Properties& props) = 0;

  virtual VisitResult visit(std::string_view key, bool value,
                            const Properties& props) = 0;

  virtual VisitResult visit(std::string_view key, const std::string& value,
                            const StrProperties& props) = 0;

  virtual VisitResult visit(std::string_view key,
                            std::span<const double> /*value*/,
                            const F64Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

  virtual VisitResult visit(std::string_view key,
                            std::span<const std::int64_t> /*value*/,
                            const I64Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }
  virtual VisitResult visit(std::string_view key, const ConfigBase& /*config*/,
                            const Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

  // Supports nullable dynamic configs (e.g. inside Polymorphic)
  virtual VisitResult visit(std::string_view key,
                            const std::shared_ptr<const ConfigBase>& /*config*/,
                            const Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }
};

struct LoaderVisitor : ConfigVisitor {
  /// Visit a double value, validating bounds
  VisitResult visit(std::string_view key, double& value,
                    const F64Properties& props) final {
    return visitNumeric(key, value, props);
  }

  /// Visit an integer value, validating bounds
  VisitResult visit(std::string_view key, std::int64_t& value,
                    const I64Properties& props) final {
    return visitNumeric(key, value, props);
  }

  VisitResult visit(std::string_view key, bool& value,
                    const Properties& props) final {
    return safeVisit(key, value, props);
  }

  /// Visit a string value, validating non-empty if required
  VisitResult visit(std::string_view key, std::string& value,
                    const StrProperties& props) final {
    std::string cand = value;
    if (auto result = safeVisit(key, cand, props); result.ec) {
      return result;
    }
    if (props.non_empty && cand.empty()) {
      return {make_error_code(AutopilotErrc::kEmptyValueNotAllowed), key,
              "Empty string not allowed"};
    }
    value = cand;
    return {};
  }

  /// Visit a contiguous range of doubles, validating bounds for each element
  VisitResult visit(std::string_view key, std::span<double> value,
                    const F64Properties& props) final {
    std::vector cand(value.begin(), value.end());
    const auto result = safeVisit(key, cand, props);
    if (!props.bounds.checkRange(cand)) {
      return {make_error_code(AutopilotErrc::kOutOfBounds), key,
              "One or more values out of bounds"};
    }
    std::ranges::copy(cand, value.begin());
    return result;
  }

  VisitResult visit(std::string_view key, std::span<std::int64_t> value,
                    const I64Properties& props) final {
    std::vector cand(value.begin(), value.end());
    const auto result = safeVisit(key, cand, props);
    if (!props.bounds.checkRange(cand)) {
      return {make_error_code(AutopilotErrc::kOutOfBounds), key,
              "One or more values out of bounds"};
    }
    std::ranges::copy(cand, value.begin());
    return result;
  }

  VisitResult visit(std::string_view key,
                    ConfigBase& config,  // NOLINT
                    const Properties& props) final {
    return safeVisit(key, config, props);
  }

  VisitResult visit(std::string_view key,
                    std::shared_ptr<ConfigBase> config,  // NOLINT
                    const Properties& props) final {
    if (!config) {
      return {make_error_code(AutopilotErrc::kConfigValueUninitialized), key};
    }
    return safeVisit(key, config, props);
  }

 protected:
  /// Visit a double value
  virtual VisitResult safeVisit(std::string_view key, double& value,
                                const F64Properties& props) = 0;

  /// Visit an integer value
  virtual VisitResult safeVisit(std::string_view key, std::int64_t& value,
                                const I64Properties& props) = 0;

  /// Visit a boolean value
  virtual VisitResult safeVisit(std::string_view key, bool& value,
                                const Properties& props) = 0;

  /// Visit a string value
  virtual VisitResult safeVisit(std::string_view key, std::string& value,
                                const StrProperties& props) = 0;

  /// Visit a contiguous range of doubles
  virtual VisitResult safeVisit(std::string_view key,
                                std::span<double> /*value*/,
                                const F64Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

  /// Visit a contiguous range of integers
  virtual VisitResult safeVisit(std::string_view key,
                                std::span<std::int64_t> /*value*/,
                                const I64Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }
  virtual VisitResult safeVisit(std::string_view key,
                                ConfigBase& /*config NOLINT(performance-*)*/,
                                const Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

  /// Visit a nested configuration
  virtual VisitResult safeVisit(
      std::string_view key,
      std::shared_ptr<ConfigBase> /*config NOLINT(performance-*)*/,
      const Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

 private:
  template <typename T, typename Props>
  VisitResult visitNumeric(std::string_view key, T& value, const Props& props) {
    T cand = value;
    if (auto result = safeVisit(key, cand, props); result.ec) {
      return result;
    }
    if (!props.bounds.check(cand)) {
      return {make_error_code(AutopilotErrc::kOutOfBounds), key,
              "Value out of bounds"};
    }
    value = cand;
    return {};
  }
};

struct ConfigBase {
  virtual ~ConfigBase() = default;

  // Returns the unique key (e.g., "GeometricPosition") associated with this
  // config
  [[nodiscard]] virtual std::string_view name() const = 0;

  virtual VisitResult accept(ConfigVisitor& visitor) = 0;

  [[nodiscard]] virtual VisitResult accept(
      ConstConfigVisitor& visitor) const = 0;
};

template <typename Class, typename T, typename Props>
struct ParamDescriptor {
  std::string_view name;
  T Class::* member;
  Props props;
};

template <typename Class, typename T, typename Props>
  requires(!std::is_member_function_pointer_v<T Class::*>)
constexpr auto Describe(std::string_view name, T Class::* member, Props props) {
  return ParamDescriptor<Class, T, Props>{name, member, props};
}

template <typename Derived>
struct ReflectiveConfigBase : ConfigBase {
  VisitResult accept(ConfigVisitor& visitor) override {
    VisitResult res;
    std::apply(
        [this, &visitor, &res](const auto&... it) {
          ((res = res.ec
                      ? res
                      : this->visitImpl(it.name, it.member, it.props, visitor)),
           ...);
        },
        Derived::kDescriptors);
    return res;
  }

  VisitResult accept(ConstConfigVisitor& visitor) const override {
    VisitResult res;
    std::apply(
        [this, &visitor, &res](const auto&... it) {
          ((res = res.ec
                      ? res
                      : this->visitImpl(it.name, it.member, it.props, visitor)),
           ...);
        },
        Derived::kDescriptors);
    return res;
  }

 private:
  friend Derived;
  ReflectiveConfigBase() = default;

  template <typename Class, typename T, typename Props>
  VisitResult visitImpl(std::string_view key, T Class::* member,
                        const Props& props, ConfigVisitor& visitor) {
    if constexpr (std::is_enum_v<T>) {
      static_assert(std::is_same_v<Props, I64Properties>,
                    "Enum properties must be of type I64Properties");
      return visitEnum(key, member, props, visitor);
    } else {
      return visitor.visit(key, adaptSpan(derived().*member), props);
    }
  }

  template <typename Class, typename T>
    requires std::is_enum_v<T>
  VisitResult visitEnum(std::string_view key, T Class::* member,
                        const I64Properties& props, ConfigVisitor& visitor) {
    using std::ranges::find;
    std::string enum_str;
    if (auto it = find(props.map, static_cast<std::int64_t>(derived().*member),
                       &EnumItem::underlying);
        it != props.map.end()) {
      // Optionally, prepopulate the string mapping for the current enum
      // value. If the value is not found, inability to pre-populate the
      // buffer with a valid enum string is NOT a hard error
      enum_str = std::string(it->name);
    }
    //
    // Init enum name string to the key we found
    auto res =
        visitor.visit(key, enum_str,
                      {.desc = props.desc,
                       .prefer_user_provided = props.prefer_user_provided,
                       .non_empty = true});

    if (res.ec) {
      return res;
    }

    auto it = find(props.map, enum_str, &EnumItem::name);
    if (it == props.map.end()) {
      return {make_error_code(AutopilotErrc::kInvalidStringOption), key,
              "String not in enum mapping"};
    }
    derived().*member = static_cast<T>(it->underlying);
    return {};
  }

  template <typename Class, typename T, typename Props>
  VisitResult visitImpl(std::string_view key, T Class::* member,
                        const Props& props, ConstConfigVisitor& visitor) const {
    if constexpr (std::is_enum_v<T>) {
      static_assert(std::is_same_v<Props, I64Properties>,
                    "Enum properties must be of type I64Properties");
      return visitEnum(key, member, props, visitor);
    } else {
      return visitor.visit(key, adaptSpan(derived().*member), props);
    }
  }

  template <typename Class, typename T>
    requires std::is_enum_v<T>
  VisitResult visitEnum(std::string_view key, T Class::* member,
                        const I64Properties& props,
                        ConstConfigVisitor& visitor) const {
    using std::ranges::find;
    // Find the string mapping for the current enum value. Presenting an
    // invalid enum to a reader is a hard error.
    auto it = find(props.map, static_cast<std::int64_t>(derived().*member),
                   &EnumItem::underlying);
    if (it == props.map.end()) {
      return {make_error_code(AutopilotErrc::kInvalidEnumMapping), key,
              "Enum value not in mapping"};
    }

    // For the const visitor, we don't need to feed back to the original enum,
    // just run the visitor and return; We also don't need persistent string
    // storage
    return visitor.visit(key, std::string(it->name),
                         {.desc = props.desc,
                          .prefer_user_provided = props.prefer_user_provided,
                          .non_empty = true});
  }

  template <typename T>
  static constexpr decltype(auto) adaptSpan(T&& mem) {
    using Plain = std::remove_cvref_t<T>;
    if constexpr (std::derived_from<Plain, Eigen::MatrixBase<Plain>>) {
      static_assert(
          Plain::IsVectorAtCompileTime && Plain::InnerStrideAtCompileTime == 1,
          "Only contiguous Eigen vectors can be treated as a span");
      return std::span(mem.data(), static_cast<std::size_t>(mem.size()));
    } else {
      return mem;
    }
  }

  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }
};

template <typename F>
concept ConfigFactory = requires(const std::string& type) {
  { F::CreateConfig(type) } -> std::same_as<std::shared_ptr<ConfigBase>>;
};

template <typename Factory>
  requires ConfigFactory<Factory>
struct Polymorphic final : ConfigBase {
  std::string_view name() const override { return "Polymorphic"; }

  std::string type;
  std::shared_ptr<ConfigBase> config;
  static constexpr StrProperties kTypePRops = {
      .desc = "Polymorphic Object Type",
      .prefer_user_provided = true,
      .non_empty = true};

  VisitResult accept(ConfigVisitor& visitor) override {
    auto res = visitor.visit("type", type, kTypePRops);
    if (res.ec) {
      return res;
    }
    if (!config || config->name() != type) {
      auto new_config = Factory::CreateConfig(type);
      if (!new_config) {
        return {make_error_code(AutopilotErrc::kCreationFailure), "type"};
      }
      config = std::move(new_config);
    }

    return visitor.visit("config", config,
                         {.desc = "Polymorphic Object Config"});
  }

  VisitResult accept(ConstConfigVisitor& visitor) const override {
    // 1. Visit the type string
    auto res = visitor.visit("type", type, kTypePRops);
    if (res.ec) {
      return res;
    }

    // We pass the shared_ptr directly so the visitor can decide how to handle
    // nulls (e.g. PrettyPrinter prints "null", Saver skips it)
    return visitor.visit("config",
                         std::const_pointer_cast<const ConfigBase>(config),
                         {.desc = "Polymorphic Object Config"});
  }
};

}  // namespace autopilot

#endif  // AUTOPILOT_BASE_HPP_
