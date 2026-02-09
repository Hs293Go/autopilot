#ifndef AUTOPILOT_PLANNING_POLYNOMIAL_HPP_
#define AUTOPILOT_PLANNING_POLYNOMIAL_HPP_

#include "Eigen/Dense"

namespace autopilot {

// Simple polynomial class mirroring numpy.polynomial.Polynomial
template <typename T, Eigen::Index Degree = Eigen::Dynamic>
class Polynomial {
 public:
  using Coefficients =
      std::conditional_t<Degree == Eigen::Dynamic, Eigen::VectorX<T>,
                         Eigen::Vector<T, Degree + 1>>;

  Polynomial() = default;
  // Initialize directly from the solver's output
  template <typename Derived>
  explicit Polynomial(const Eigen::MatrixBase<Derived>& coeffs)
      : coeffs_(coeffs) {}

  template <typename... Ts>
    requires(sizeof...(Ts) > 0 &&  // Prevent ambiguity with default ctor
             (Degree == sizeof...(Ts) || Degree == Eigen::Dynamic))
  explicit Polynomial(T t1, Ts... terms) {
    if constexpr (Degree == Eigen::Dynamic) {
      coeffs_.resize(sizeof...(Ts) + 1);
    }
    // Use << once to materialize the CommaInitializer, then fold over the
    // Eigen-specific comma operator
    ((coeffs_ << t1), ..., terms);
  }

  Eigen::Index degree() const { return coeffs_.size() - 1; }

  [[nodiscard]] T operator()(T t) const {
    T result = 0;
    for (auto i = coeffs_.size() - 1; i >= 0; --i) {
      result = result * t + coeffs_[i];
    }
    return result;
  }

  [[nodiscard]] Polynomial deriv(Eigen::Index m = 1) const {
    using std::pow;

    if (m == 0) {
      return *this;
    }

    if (m >= coeffs_.size()) {
      return Polynomial(Eigen::Vector<T, 1>::Zero());
    }

    Eigen::VectorX<T> deriv_coeffs(coeffs_.size() - m);  // New coeff vector
    for (auto i = coeffs_.size() - 1; i >= m; --i) {
      // Multiply by powers: n * (n-1) * ...
      const T c = coeffs_[i] * Eigen::VectorX<T>::NullaryExpr(m, [i](auto k) {
                                 return static_cast<T>(i - k);
                               }).prod();

      deriv_coeffs[i - m] = c;
    }
    return Polynomial(deriv_coeffs);
  }

  // Closed-form derivative
  [[nodiscard]] T derivVal(T t, Eigen::Index derivative_order) const {
    using std::pow;

    if (derivative_order == 0) {
      return (*this)(t);
    }

    if (derivative_order >= coeffs_.size()) {
      return T(0);
    }

    T result = 0.0;
    // Optimization: Compute the derivative coefficients once or on-the-fly
    // For trajectory gen, on-the-fly is usually fast enough
    for (auto i = coeffs_.size() - 1; i >= derivative_order; --i) {
      // Multiply by powers: n * (n-1) * ...
      const T c = coeffs_[i] *
                  Eigen::VectorX<T>::NullaryExpr(derivative_order, [i](auto k) {
                    return static_cast<T>(i - k);
                  }).prod();

      // Accumulate: c * t^(i-r) Note: Horner's scheme adapted for derivatives
      // is slightly more complex, standard power evaluation is often fine for
      // low orders.
      result += c * pow(t, static_cast<T>(i - derivative_order));
    }
    return result;
  }

  bool isApprox(const Polynomial<T>& other,
                T tol = static_cast<T>(1e-9)) const {
    return coeffs_.isApprox(other.coeffs_, tol);
  }

  bool operator==(const Polynomial<T>& other) const {
    return coeffs_ == other.coeffs_;
  }

  static Polynomial Linear(T intercept, T slope)
    requires(Degree != Eigen::Dynamic)
  {
    Coefficients coeffs = Coefficients::Zero();
    coeffs[0] = intercept;
    coeffs[1] = slope;
    return Polynomial(coeffs);
  }

  static Polynomial Linear(Eigen::Index degree, T intercept, T slope)
    requires(Degree == Eigen::Dynamic)
  {
    Coefficients coeffs = Coefficients::Zero(degree + 1);
    coeffs[0] = intercept;
    coeffs[1] = slope;
    return Polynomial(coeffs);
  }

  static Polynomial Constant(T value)
    requires(Degree != Eigen::Dynamic)
  {
    return Polynomial(Coefficients::UnitX() * value);
  }

  static Polynomial Constant(Eigen::Index degree, T value)
    requires(Degree == Eigen::Dynamic)
  {
    Coefficients coeffs = Coefficients::Zero(degree + 1);
    coeffs[0] = value;
    return Polynomial(coeffs);
  }

  static Polynomial Zero()
    requires(Degree != Eigen::Dynamic)
  {
    return Polynomial(Coefficients::Zero());
  }

  static Polynomial Zero(Eigen::Index degree)
    requires(Degree == Eigen::Dynamic)
  {
    return Polynomial(Coefficients::Zero(degree + 1));
  }

 private:
  Coefficients coeffs_;
};

template <typename Derived>
Polynomial(const Eigen::MatrixBase<Derived>&)
    -> Polynomial<typename Derived::Scalar,
                  Derived::RowsAtCompileTime == Eigen::Dynamic
                      ? Eigen::Dynamic
                      : Derived::RowsAtCompileTime - 1>;

using PolynomialF32 = Polynomial<float>;
using PolynomialF64 = Polynomial<double>;

}  // namespace autopilot

#endif  // AUTOPILOT_PLANNING_POLYNOMIAL_HPP_
