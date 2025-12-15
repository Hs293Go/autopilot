#ifndef AUTOPILOT_ROTATION_HPP_
#define AUTOPILOT_ROTATION_HPP_

#include "Eigen/Dense"
#include "autopilot/core/common.hpp"

namespace autopilot {

template <int N, typename T>
  requires(std::is_arithmetic_v<T> && (N >= 0 || std::is_floating_point_v<T>))
constexpr T pown(T base) {
  if constexpr (N == 0) {
    return static_cast<T>(1);
  } else if constexpr (N < 0) {
    return static_cast<T>(1) / pown<-N>(base);
  } else if constexpr (N % 2 == 0) {  // even exponent
    const T half = pown<N / 2>(base);
    return half * half;
  } else {  // odd exponent
    return base * pown<N - 1>(base);
  }
}

static_assert(pown<0>(2.0) == 1.0);
static_assert(pown<0>(std::numeric_limits<double>::max()) == 1.0);
static_assert((pown<3>(2.0) == 8.0));
static_assert(pown<-2>(2.0) == 0.25);
static_assert(pown<4>(3) == 81);
static_assert(pown<1>(5) == 5);

template <std::floating_point T>
constexpr T deg2rad(T degrees) {
  return degrees * (std::numbers::pi_v<T> / static_cast<T>(180));
}

template <std::floating_point T>
constexpr T rad2deg(T radians) {
  return radians * (static_cast<T>(180) / std::numbers::pi_v<T>);
}

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

template <Vector3Like Derived>
Eigen::Quaternion<typename Derived::Scalar> AngleAxisToQuaternion(
    const Eigen::MatrixBase<Derived>& angle_axis) {
  using Scalar = typename Derived::Scalar;
  using std::abs;
  using std::cos;
  using std::sin;
  using std::sqrt;

  const Scalar theta_sq = angle_axis.squaredNorm();

  Scalar imag_factor;
  Scalar real_factor;

  if (IsClose(theta_sq, Scalar(0))) {
    const Scalar theta_po4 = theta_sq * theta_sq;
    imag_factor = Scalar(0.5) - Scalar(1.0 / 48.0) * theta_sq +
                  Scalar(1.0 / 3840.0) * theta_po4;
    real_factor = Scalar(1) - Scalar(1.0 / 8.0) * theta_sq +
                  Scalar(1.0 / 384.0) * theta_po4;
  } else {
    const Scalar theta = sqrt(theta_sq);
    const Scalar half_theta = Scalar(0.5) * theta;
    const Scalar sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / theta;
    real_factor = cos(half_theta);
  }

  Eigen::Quaternion<Scalar> quaternion;
  quaternion.w() = real_factor;
  quaternion.vec() = imag_factor * angle_axis;
  return quaternion;
}

template <Vector3Like Derived>
Eigen::Matrix3<typename Derived::Scalar> AngleAxisToRotationMatrix(
    const Eigen::MatrixBase<Derived>& angle_axis) {
  using std::cos;
  using std::sin;
  using std::sqrt;
  using Scalar = typename Derived::Scalar;

  const Scalar theta_sq = angle_axis.squaredNorm();
  Eigen::Matrix3<Scalar> rotation_matrix = Eigen::Matrix3<Scalar>::Identity();
  const Eigen::Matrix3<Scalar> hat_phi = hat(angle_axis);
  const Eigen::Matrix3<Scalar> hat_phi_sq = hat_phi * hat_phi;

  if (IsClose(theta_sq, Scalar(0))) {
    rotation_matrix += hat_phi + hat_phi_sq / Scalar(2);
  } else {
    const Scalar theta = sqrt(theta_sq);
    const Scalar cos_theta = cos(theta);
    const Scalar sin_theta = sin(theta);

    rotation_matrix += (Scalar(1) - cos_theta) / theta_sq * hat_phi_sq +
                       sin_theta / theta * hat_phi;
  }

  return rotation_matrix;
}

template <typename Derived>
Eigen::Vector3<typename Derived::Scalar> QuaternionToAngleAxis(
    const Eigen::QuaternionBase<Derived>& quaternion) {
  using Scalar = typename Derived::Scalar;

  using std::atan2;
  using std::sqrt;
  const Scalar squared_n = quaternion.vec().squaredNorm();
  const Scalar& w = quaternion.w();

  Scalar two_atan_nbyw_by_n;

  if (IsClose(squared_n, Scalar(0))) {
    // If quaternion is normalized and n=0, then w should be 1;
    // w=0 should never happen here!
    const Scalar squared_w = w * w;
    two_atan_nbyw_by_n =
        Scalar(2.0) / w - Scalar(2.0 / 3.0) * (squared_n) / (w * squared_w);
  } else {
    const Scalar n = sqrt(squared_n);

    // w < 0 ==> cos(theta/2) < 0 ==> theta > pi
    //
    // By convention, the condition |theta| < pi is imposed by wrapping theta
    // to pi; The wrap operation can be folded inside evaluation of atan2
    //
    // theta - pi = atan(sin(theta - pi), cos(theta - pi))
    //            = atan(-sin(theta), -cos(theta))
    //
    const Scalar atan_nbyw = (w < Scalar(0)) ? atan2(-n, -w) : atan2(n, w);
    two_atan_nbyw_by_n = Scalar(2) * atan_nbyw / n;
  }

  return two_atan_nbyw_by_n * quaternion.vec();
}

template <Matrix3Like Derived>
Eigen::Vector3<typename Derived::Scalar> RotationMatrixToAngleAxis(
    const Eigen::MatrixBase<Derived>& R) {
  return QuaternionToAngleAxis(Eigen::Quaternion<typename Derived::Scalar>(R));
}

}  // namespace autopilot

#endif  // AUTOPILOT_ROTATION_HPP_
