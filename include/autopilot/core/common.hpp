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
  kEstimatorUninitialized,
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
