#ifndef AUTOPILOT_BASE_HPP_
#define AUTOPILOT_BASE_HPP_

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

  static constexpr Bounds GreaterThan(T val) {
    return {.lower = Exclusive(val)};
  }

  static constexpr Bounds AtLeast(T val) { return {.lower = Inclusive(val)}; }

  static constexpr Bounds LessThan(T val) { return {.upper = Exclusive(val)}; }

  static constexpr Bounds AtMost(T val) { return {.upper = Inclusive(val)}; }

  static constexpr Bounds Positive() { return GreaterThan(0); }

  static constexpr Bounds NonNegative() { return AtLeast(0); }
};

struct Properties {
  /// Human-readable description of the parameter
  std::string_view desc;

  /// Whether to warn if the parameter is not set by the user
  bool prefer_user_provided = false;
};

template <typename T>
struct NumericProperties {
  /// Human-readable description of the parameter
  std::string_view desc;

  /// Whether to warn if the parameter is not set by the user
  bool prefer_user_provided = false;

  /// Valid range for the parameter
  Bounds<T> bounds;
};

struct StrProperties {
  /// Human-readable description of the parameter
  std::string_view desc;

  /// Whether to warn if the parameter is not set by the user
  bool prefer_user_provided = false;

  bool non_empty = false;
};

using F64Properties = NumericProperties<double>;
using I64Properties = NumericProperties<std::int64_t>;

/// Visitor Pattern for Configurations
struct ConfigVisitor {
  struct VisitResult {
    std::error_code ec;
    std::string_view key;
    std::string_view message = "none";
  };

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

  /// Visit a nested configuration
  virtual VisitResult visit(
      std::string_view key,
      std::shared_ptr<struct ConfigBase> /*config NOLINT(performance-*)*/,
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
    std::vector<double> cand(value.size());
    auto result = safeVisit(key, cand, props);
    if (!std::ranges::all_of(
            cand, std::bind_front(&Bounds<double>::check, props.bounds))) {
      return {make_error_code(AutopilotErrc::kOutOfBounds), key,
              "One or more values out of bounds"};
    }
    std::copy(cand.begin(), cand.end(), value.begin());
    return result;
  }

  VisitResult visit(std::string_view key, std::span<std::int64_t> value,
                    const I64Properties& props) final {
    std::vector<std::int64_t> cand(value.size());
    auto result = safeVisit(key, cand, props);
    if (!std::ranges::all_of(cand, std::bind_front(&Bounds<std::int64_t>::check,
                                                   props.bounds))) {
      return {make_error_code(AutopilotErrc::kOutOfBounds), key,
              "One or more values out of bounds"};
    }
    std::copy(cand.begin(), cand.end(), value.begin());
    return result;
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

  /// Visit a nested configuration
  virtual VisitResult safeVisit(
      std::string_view key,
      std::shared_ptr<struct ConfigBase> /*config NOLINT(performance-*)*/,
      const Properties& /*props*/) {
    return {.ec = make_error_code(std::errc::not_supported), .key = key};
  }

 private:
  template <typename T>
  VisitResult visitNumeric(std::string_view key, T& value,
                           const NumericProperties<T>& props) {
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
  [[nodiscard]] virtual std::string name() const = 0;

  [[nodiscard]] virtual ConfigVisitor::VisitResult accept(
      ConfigVisitor& visitor) = 0;
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
  ConfigVisitor::VisitResult accept(ConfigVisitor& visitor) override {
    ConfigVisitor::VisitResult res;
    std::apply(
        [this, &visitor, &res](const auto&... it) {
          ((res = res.ec ? res
                         : visitor.visit(it.name,
                                         adaptSpan(derived().*(it.member)),
                                         it.props)),
           ...);
        },
        Derived::kDescriptors);
    return res;
  }

 private:
  friend Derived;

  template <typename T>
  static constexpr decltype(auto) adaptSpan(T& mem) {
    if constexpr (std::derived_from<T, Eigen::MatrixBase<T>>) {
      static_assert(
          T::IsVectorAtCompileTime && T::InnerStrideAtCompileTime == 1,
          "Only contiguous Eigen types are supported");
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
  // Polymorphic objects are likely held via shared pointers, so they can be
  // directly visited by our visitor API
  using SharedPtr = std::shared_ptr<Polymorphic<Factory>>;

  /// Create a new Polymorphic Config instance
  static SharedPtr Make() { return std::make_shared<Polymorphic<Factory>>(); }
  std::string name() const override { return "Polymorphic"; }

  std::string type;
  std::shared_ptr<ConfigBase> config;

  ConfigVisitor::VisitResult accept(ConfigVisitor& visitor) override {
    auto res = visitor.visit(
        "type", type, {.desc = "Polymorphic Object Type", .non_empty = true});
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
};

}  // namespace autopilot

#endif  // AUTOPILOT_BASE_HPP_
