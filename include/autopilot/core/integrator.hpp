#ifndef AUTOPILOT_CORE_INTEGRATOR_HPP_
#define AUTOPILOT_CORE_INTEGRATOR_HPP_

#include <utility>

#include "Eigen/Core"

namespace autopilot {

enum class Methods {
  kEuler,
  kRk4,
};

struct IntegratorSpecs {
  Methods method = Methods::kRk4;
  Eigen::Index state_dim;
  Eigen::Index input_dim;
};

template <typename T, IntegratorSpecs Specs>
class Integrator {
 public:
  static constexpr Eigen::Index kStateDim = Specs.state_dim;
  static constexpr Eigen::Index kInputDim = Specs.input_dim;
  static constexpr Methods kMethod = Specs.method;

  using XVector = Eigen::Vector<T, kStateDim>;
  using UVector = Eigen::Vector<T, kInputDim>;

  struct StateAndDerivative {
    XVector state;
    XVector derivative;
  };

  using Dynamics = std::function<XVector(const Eigen::Ref<const XVector>&,
                                         const Eigen::Ref<const UVector>&)>;

  template <typename F>
  Integrator(F dynamics) : dynamics_(std::move(dynamics)) {}

  template <typename XDerived, typename UDerived>
  XVector operator()(const Eigen::MatrixBase<XDerived>& x_op,
                     const Eigen::MatrixBase<UDerived>& u, T stepsize) {
    if constexpr (UDerived::ColsAtCompileTime == 1) {
      return step(x_op, u, stepsize);
    } else {
      XVector x = x_op;
      for (Eigen::Index i = 0; i < u.cols(); ++i) {
        x = step(x, u(Eigen::all, i), stepsize);
      }
      return x;
    }
  }

  template <typename XDerived, typename UDerived>
  StateAndDerivative stepWithDerivatives(
      const Eigen::MatrixBase<XDerived>& x_op,
      const Eigen::MatrixBase<UDerived>& u, T stepsize) {
    const XVector k1 = dynamics_(x_op, u);
    if constexpr (kMethod == Methods::kEuler) {
      return {.state = x_op + stepsize * k1, .derivative = k1};
    } else if constexpr (kMethod == Methods::kRk4) {
      T half_stepsize = stepsize / T(2);
      const XVector k2 = dynamics_(x_op + k1 * half_stepsize, u);
      const XVector k3 = dynamics_(x_op + k2 * half_stepsize, u);
      const XVector k4 = dynamics_(x_op + k3 * stepsize, u);
      const XVector avg_derivative = (k1 + T(2) * k2 + T(2) * k3 + k4) / T(6);
      return {
          .state = x_op + stepsize * avg_derivative,
          .derivative = avg_derivative,
      };
    }
  }

  template <typename XDerived, typename UDerived>
  Eigen::Vector<T, XDerived::SizeAtCompileTime> step(
      const Eigen::MatrixBase<XDerived>& x_op,
      const Eigen::MatrixBase<UDerived>& u, T stepsize) {
    return stepWithDerivatives(x_op, u, stepsize).state;
  }

 private:
  Dynamics dynamics_;
};

}  // namespace autopilot

#endif  // AUTOPILOT_CORE_INTEGRATOR_HPP_
