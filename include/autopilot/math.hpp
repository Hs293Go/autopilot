#ifndef AUTOPILOT_ROTATION_HPP_
#define AUTOPILOT_ROTATION_HPP_

#include "Eigen/Dense"
#include "autopilot/common.hpp"

namespace autopilot {

template <std::floating_point T>
struct Tolerances {
  static constexpr T kDefaultAbsTol = static_cast<T>(1e-5);
  static constexpr T kDefaultRelTol = static_cast<T>(1e-8);

  T abs = kDefaultAbsTol;
  T rel = kDefaultRelTol;
};

template <std::floating_point T>
bool IsClose(T a, T b, const Tolerances<T>& tols = {}) {
  using std::abs;
  using std::max;
  using std::min;
  if (a == b) {
    return true;
  }

  const T diff = abs(a - b);
  const T norm = min((abs(a) + abs(b)), std::numeric_limits<T>::max());
  // or even faster: std::min(std::abs(a + b),
  // std::numeric_limits<float>::max()); keeping this commented out until I
  // update figures below
  return diff < max(tols.abs, tols.rel * norm);
}

template <Vector3Like Derived>
Eigen::Matrix3<typename Derived::Scalar> hat(
    const Eigen::MatrixBase<Derived>& v) {
  using Scalar = typename Derived::Scalar;
  Eigen::Matrix3<Scalar> v_hat;
  v_hat << Scalar(0), -v.z(), v.y(),  //
      v.z(), Scalar(0), -v.x(),       //
      -v.y(), v.x(), Scalar(0);
  return v_hat;
}

template <Matrix3Like Derived>
Eigen::Vector3<typename Derived::Scalar> vee(
    const Eigen::MatrixBase<Derived>& m) {
  return {m(2, 1), m(0, 2), m(1, 0)};
}

template <typename Derived>
Eigen::Vector3<typename Derived::Scalar> QuaternionToRollPitchYaw(
    const Eigen::QuaternionBase<Derived>& q) {
  using std::atan2;
  using std::sqrt;
  using Scalar = typename Derived::Scalar;
  Eigen::Vector3<Scalar> angles;

  // roll (x-axis rotation)
  const Scalar sinr_cosp = Scalar(2) * (q.w() * q.x() + q.y() * q.z());
  const Scalar cosr_cosp =
      Scalar(1) - Scalar(2) * (q.x() * q.x() + q.y() * q.y());
  angles.x() = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  const Scalar sinp =
      sqrt(Scalar(1) + Scalar(2) * (q.w() * q.y() - q.x() * q.z()));
  const Scalar cosp =
      sqrt(Scalar(1) - Scalar(2) * (q.w() * q.y() - q.x() * q.z()));
  angles.y() = Scalar(2) * atan2(sinp, cosp) - M_PI / Scalar(2);

  // yaw (z-axis rotation)
  const Scalar siny_cosp = Scalar(2) * (q.w() * q.z() + q.x() * q.y());
  const Scalar cosy_cosp =
      Scalar(1) - Scalar(2) * (q.y() * q.y() + q.z() * q.z());
  angles.z() = atan2(siny_cosp, cosy_cosp);

  return angles;
}
}  // namespace autopilot

#endif  // AUTOPILOT_ROTATION_HPP_
