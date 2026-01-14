#ifndef AUTOPILOT_CONCEPTS_HPP_
#define AUTOPILOT_CONCEPTS_HPP_

#include <utility>

#include "Eigen/Core"  // IWYU pragma: keep

namespace autopilot {

template <typename... Ts>
struct Overload : Ts... {
  using Ts::operator()...;
};
template <typename... Ts>
Overload(Ts...) -> Overload<Ts...>;

template <size_t N>
struct fixed_string {  // NOLINT(readability-identifier-naming)
  char buf[N];

  consteval fixed_string(const char (&str)[N]) { std::copy_n(str, N, buf); }

  // Auto-convert to string_view
  constexpr operator std::string_view() const { return {buf, N - 1}; }
};

namespace literals {
template <fixed_string S>
consteval auto operator""_s() {
  return S;
}

}  // namespace literals

namespace internal {
template <std::size_t Index, class P0, class... Pack>
struct nth_type_t {
  using type = typename nth_type_t<Index - 1, Pack...>::type;
};
template <class P0, class... Pack>
struct nth_type_t<0, P0, Pack...> {
  using type = P0;
};
}  // namespace internal

template <class T>
constexpr std::underlying_type_t<T> to_underlying(T value) noexcept {
  return static_cast<std::underlying_type_t<T>>(value);
}
template <std::size_t Index, class P0, class... Pack>
using nth_type = typename internal::nth_type_t<Index, P0, Pack...>::type;

template <std::size_t Index, class T0, class... Types>
constexpr decltype(auto) nth_value(T0&& p0, Types&&... pack) noexcept {
  if constexpr (0 == Index) {
    return std::forward<T0>(p0);
  } else {
    return nth_value<Index - 1>(std::forward<Types>(pack)...);
  }
}

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

template <typename Scalar, Eigen::Index MaxRows, Eigen::Index MaxCols>
using MatrixX =
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MaxRows, MaxCols>;

}  // namespace heapless

namespace ix = Eigen::indexing;

template <int Start, int N>
struct BlockDef {
  static_assert(Start >= 0, "Starting index must be greater or equal to 0");
  static_assert(N > 0, "Size must be greater than 0");

  static constexpr int kStart = Start;
  static constexpr int kEnd = Start + N;
  static constexpr int kN = N;

  [[nodiscard]] constexpr std::size_t size() const { return N; }
  /**
   * @brief Creates a sequence object for slicing Eigen Vectors, i.e.
   *
   * @return auto
   */
  auto operator()() const { return ix::seqN(ix::fix<kStart>, ix::fix<kN>); }
};

template <int N, typename PrevBlock>
constexpr auto NextBlock(PrevBlock /* prev_block */) {
  return BlockDef<PrevBlock::kEnd, N>{};
}

template <typename... Ts>
constexpr auto SumSizes(Ts... xs) {
  return (std::size(xs) + ...);
}

enum class AutopilotErrc {
  kInvalidDimension,
  kInvalidBufferSize,
  kInvalidOrdering,
  kNumericallyNonFinite,
  kPhysicallyInvalid,
  kOutOfBounds,
  kTimestampOutOfOrder,
  kUnknownSensorType,
  kNumericalInstability,
  kNumericalOutlier,
  kLinalgError,
  kEstimationContextMismatch,
  kEstimatorUninitialized,
  kConfigKeyMissing,
  kConfigValueUninitialized,
  kConfigTypeMismatch,
  kConfigSizeMismatch,
  kEmptyValueNotAllowed,
  kCreationFailure,
  kInvalidEnumMapping,
  kInvalidStringOption
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
      case AutopilotErrc::kTimestampOutOfOrder:
        return "Timestamp out of order";
      case AutopilotErrc::kUnknownSensorType:
        return "Unknown sensor type";
      case AutopilotErrc::kNumericalInstability:
        return "Numerical instability encountered";
      case AutopilotErrc::kNumericalOutlier:
        return "Numerical outlier detected";
      case AutopilotErrc::kLinalgError:
        return "Linear algebra error";
      case AutopilotErrc::kEstimatorUninitialized:
        return "Estimator not initialized";
      case AutopilotErrc::kConfigKeyMissing:
        return "Configuration key missing";
      case AutopilotErrc::kConfigValueUninitialized:
        return "Configuration value uninitialized";
      case AutopilotErrc::kConfigTypeMismatch:
        return "Configuration type mismatch";
      case AutopilotErrc::kConfigSizeMismatch:
        return "Configuration size mismatch";
      case AutopilotErrc::kEmptyValueNotAllowed:
        return "Empty value not allowed";
      case AutopilotErrc::kCreationFailure:
        return "Object creation failure";
      case AutopilotErrc::kInvalidStringOption:
        return "Invalid string option";
      case AutopilotErrc::kInvalidEnumMapping:
        return "Invalid enum mapping";
      default:
        return "Unknown error";
    }
    std::unreachable();
  }

  [[nodiscard]] std::error_condition default_error_condition(
      int ev) const noexcept override {
    switch (static_cast<AutopilotErrc>(ev)) {
      case AutopilotErrc::kOutOfBounds:
        return std::make_error_condition(std::errc::result_out_of_range);
      default:
        return std::error_condition(ev, *this);
    }
    std::unreachable();
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
