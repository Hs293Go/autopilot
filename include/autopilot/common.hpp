#ifndef AUTOPILOT_CONCEPTS_HPP_
#define AUTOPILOT_CONCEPTS_HPP_

#include <utility>

#include "Eigen/Core"  // IWYU pragma: keep

namespace autopilot {

#if defined(__cpp_lib_unreachable) && __cpp_lib_unreachable >= 202202L
using std::unreachable;
#else
[[noreturn]] inline void unreachable() {
#if defined(__GNUC__) || defined(__clang__)
  __builtin_unreachable();
#elif defined(_MSC_VER)
  __assume(false);
#else
  throw std::logic_error("Reached unreachable code");
#endif
}
#endif

#if defined(__cpp_lib_to_underlying) && __cpp_lib_to_underlying >= 202102L
using std::to_underlying;
#else
template <typename Enum>
constexpr auto to_underlying(Enum e) noexcept {
  return static_cast<std::underlying_type_t<Enum>>(e);
}
#endif

template <typename Derived>
concept Vector2Like = static_cast<bool>(Derived::IsVectorAtCompileTime) &&
                      Derived::RowsAtCompileTime == 2;

template <typename Derived>
concept Vector3Like = static_cast<bool>(Derived::IsVectorAtCompileTime) &&
                      Derived::RowsAtCompileTime == 3;

template <typename Derived>
concept Matrix3Like = static_cast<bool>(Derived::RowsAtCompileTime == 3 &&
                                        Derived::ColsAtCompileTime == 3);

template <typename Derived>
concept Matrix4Like = static_cast<bool>(Derived::RowsAtCompileTime == 4 &&
                                        Derived::ColsAtCompileTime == 4);

namespace heapless {
template <typename Scalar, Eigen::Index MaxSize>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, MaxSize, 1>;
}

enum class AutopilotErrc {
  kInvalidDimension,
  kInvalidBufferSize,
  kInvalidOrdering,
  kNumericallyNonFinite,
  kPhysicallyInvalid,
  kOutOfBounds,
};

namespace detail {
class AutopilotErrcCategory : public std::error_category {
 public:
  [[nodiscard]] const char* name() const noexcept override {
    return "autopilot_error";
  }

  [[nodiscard]] std::string message(int ev) const override {
    switch (static_cast<AutopilotErrc>(ev)) {
      case AutopilotErrc::kInvalidDimension:
        return "Invalid dimension";
      case AutopilotErrc::kInvalidBufferSize:
        return "Invalid buffer size";
      case AutopilotErrc::kInvalidOrdering:
        return "Invalid ordering of parameters";
      case AutopilotErrc::kNumericallyNonFinite:
        return "Numerically non-finite value";
      case AutopilotErrc::kPhysicallyInvalid:
        return "Physically invalid value";
      case AutopilotErrc::kOutOfBounds:
        return "Value out of bounds";
      default:
        return "Unknown error";
    }
    unreachable();
  }

  [[nodiscard]] std::error_condition default_error_condition(
      int ev) const noexcept override {
    switch (static_cast<AutopilotErrc>(ev)) {
      case AutopilotErrc::kOutOfBounds:
        return std::make_error_condition(std::errc::result_out_of_range);
      default:
        return std::error_condition(ev, *this);
    }
    unreachable();
  }
};
}  // namespace detail
}  // namespace autopilot

inline const autopilot::detail::AutopilotErrcCategory& AutopilotErrcCategory() {
  static autopilot::detail::AutopilotErrcCategory category;
  return category;
}

inline std::error_code make_error_code(autopilot::AutopilotErrc e) {
  return {static_cast<int>(e), AutopilotErrcCategory()};
}

namespace std {
template <>
struct is_error_code_enum<autopilot::AutopilotErrc> : true_type {};
}  // namespace std

static_assert(std::is_error_code_enum_v<autopilot::AutopilotErrc>,
              "AutopilotErrc must be registered as error code enum");

static_assert(
    requires(autopilot::AutopilotErrc ec) {
      { make_error_code(ec) } -> std::same_as<std::error_code>;
    }, "Cannot convert AutopilotErrc to error_code");

#endif  // AUTOPILOT_CONCEPTS_HPP_
