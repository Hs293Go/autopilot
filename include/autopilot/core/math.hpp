#ifndef AUTOPILOT_ROTATION_HPP_
#define AUTOPILOT_ROTATION_HPP_

#include "Eigen/Dense"
#include "autopilot/core/common.hpp"
#include "autopilot/planning/trajectory.hpp"

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

template <Vector2Like Derived1, Vector2Like Derived2>
  requires(std::same_as<typename Derived1::Scalar, typename Derived2::Scalar>)
typename Derived1::Scalar cross2d(const Eigen::MatrixBase<Derived1>& a,
                                  const Eigen::MatrixBase<Derived2>& b) {
  return a.x() * b.y() - a.y() * b.x();
}

template <typename T>
struct YawKinematics {
  T yaw{0};
  T yaw_rate{0};
  T yaw_acceleration{0};
};

template <Vector2Like Derived1, Vector2Like Derived2, Vector2Like Derived3>
YawKinematics<typename Derived1::Scalar> YawFromRay(
    const Eigen::MatrixBase<Derived1>& r,
    const Eigen::MatrixBase<Derived2>& r_dot,
    const Eigen::MatrixBase<Derived3>& r_ddot) {
  using std::atan2;
  using Scalar = typename Derived1::Scalar;
  constexpr Scalar kSqNormTol = 1e-6;
  const double r2 = r.squaredNorm();
  if (r2 < kSqNormTol) {
    return {};
  }
  const Scalar yaw = atan2(r.y(), r.x());

  const Scalar n = cross2d(r, r_dot);
  const Scalar yaw_rate = n / r2;

  const Scalar ndot = cross2d(r, r_ddot);
  const Scalar ddot = Scalar(2) * r.dot(r_dot);
  const Scalar yaw_acceleration = (ndot * r2 - n * ddot) / (r2 * r2);
  return {
      .yaw = yaw, .yaw_rate = yaw_rate, .yaw_acceleration = yaw_acceleration};
}

template <Vector2Like Derived1, Vector2Like Derived2, Vector2Like Derived3>
YawKinematics<typename Derived1::Scalar> YawFromVelocity(
    const Eigen::MatrixBase<Derived1>& vel2d,
    const Eigen::MatrixBase<Derived2>& acc2d,
    const Eigen::MatrixBase<Derived3>& jerk2d) {
  using std::atan2;
  using Scalar = typename Derived1::Scalar;
  constexpr Scalar kSqNormTol = 1e-6;
  const double vdv = vel2d.squaredNorm();
  if (vdv < kSqNormTol) {
    return {};
  }
  const Scalar yaw = atan2(vel2d.y(), vel2d.x());
  const Scalar vxa = cross2d(vel2d, acc2d);
  const Scalar yaw_rate = vxa / vdv;
  const Scalar tvda = Scalar(2) * vel2d.dot(acc2d);
  const Scalar vxj = cross2d(vel2d, jerk2d);
  const Scalar yaw_acceleration = (vxj * vdv - vxa * tvda) / (vdv * vdv);
  return {
      .yaw = yaw, .yaw_rate = yaw_rate, .yaw_acceleration = yaw_acceleration};
}

inline KinematicState ResolveYawState(const KinematicState& state,
                                      const HeadingPolicy& policy) {
  auto sample = state;
  const auto [yaw, yaw_rate, yaw_accel] = std::visit(
      Overload{[&sample](const FollowVelocity&) {
                 // Yaw based on velocity direction
                 const Eigen::Vector2d v = sample.velocity.head<2>();
                 const Eigen::Vector2d a = sample.acceleration.head<2>();
                 const Eigen::Vector2d j = sample.jerk.head<2>();
                 return YawFromVelocity(v, a, j);
               },

               [](const Fixed& fixed) {
                 return YawKinematics{
                     .yaw = fixed.yaw,
                     .yaw_rate = fixed.yaw_rate,
                     .yaw_acceleration = fixed.yaw_acceleration};
               },
               [&sample](const PointOfInterest& poi) {
                 const Eigen::Vector2d r =
                     (poi.point - sample.position).head<2>();  // poi - pos
                 const Eigen::Vector2d r_dot = -sample.velocity.head<2>();
                 const Eigen::Vector2d r_ddot = -sample.acceleration.head<2>();
                 return YawFromRay(r, r_dot, r_ddot);
               }},
      policy);
  sample.yaw = yaw;
  sample.yaw_rate = yaw_rate;
  sample.yaw_acceleration = yaw_accel;
  return sample;
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

namespace details {

template <auto BinaryOp, typename Derived>
Eigen::Matrix4<typename Derived::Scalar> QuaternionMatrix(
    const Eigen::QuaternionBase<Derived>& q) {
  using Scalar = typename Derived::Scalar;
  Eigen::Matrix4<Scalar> mat;
  mat << BinaryOp(q.w() * Eigen::Matrix3d::Identity(), hat(q.vec())), q.vec(),
      -q.vec().transpose(), q.w();
  return mat;
}
}  // namespace details

template <typename Derived>
Eigen::Matrix4<typename Derived::Scalar> LeftQuaternionMatrix(
    const Eigen::QuaternionBase<Derived>& q) {
  return details::QuaternionMatrix<std::plus{}>(q);
}

template <typename Derived>
Eigen::Matrix4<typename Derived::Scalar> RightQuaternionMatrix(
    const Eigen::QuaternionBase<Derived>& q) {
  return details::QuaternionMatrix<std::minus{}>(q);
}

namespace jacobians {

template <typename QDerived, Vector3Like PDerived>
Eigen::Matrix<typename QDerived::Scalar, 3, 4> RotatePointByQuaternion(
    const Eigen::QuaternionBase<QDerived>& q,
    const Eigen::MatrixBase<PDerived>& p) {
  using Scalar = typename QDerived::Scalar;
  Eigen::Vector3<Scalar> tqv = Scalar(2) * (q.vec().cross(p) + q.w() * p);
  Eigen::Matrix<Scalar, 3, 4> jac;
  jac << Scalar(2) * q.vec().dot(p) * Eigen::Matrix3<Scalar>::Identity() -
             hat(tqv),
      tqv;
  return jac;
}

}  // namespace jacobians

}  // namespace autopilot

#endif  // AUTOPILOT_ROTATION_HPP_
        // [&sample](const Lookahead& lookahead) {
        //   // 2D kinematics (use 3D vectors but only xy matters for yaw)
        //   const Eigen::Vector2d v = sample.velocity.head<2>();
        //   const Eigen::Vector2d a = sample.acceleration.head<2>();
        //   const Eigen::Vector2d j = sample.jerk.head<2>();
        //
        //   const double v2_sq = v.squaredNorm();
        //
        //   // Choose lookahead time t:
        //   //  - If Lookahead provides a time horizon, use it.
        //   //  - Else if it provides a distance, convert to time using
        //   //  speed.
        //   //  - Else default to 0.
        //   double t = std::max(lookahead.look_ahead_time_secs, 0.0);
        //
        //   // Predicted relative displacement to lookahead point:
        //   // r(t) = v t + 1/2 a t^2 + 1/6 j t^3
        //   const double t2 = t * t;
        //   const double t3 = t2 * t;
        //
        //   const Eigen::Vector3d r3 =
        //       v * t + 0.5 * a * t2 + (1.0 / 6.0) * j * t3;
        //
        //   // Time derivatives:
        //   // r_dot(t)  = v + a t + 1/2 j t^2
        //   // r_ddot(t) = a + j t
        //   const Eigen::Vector3d r_dot3 = v + a * t + 0.5 * j * t2;
        //   const Eigen::Vector3d r_ddot3 = a + j * t;
        //
        //   const Eigen::Vector2d r = r3.head<2>();
        //   const Eigen::Vector2d r_dot = r_dot3.head<2>();
        //   const Eigen::Vector2d r_ddot = r_ddot3.head<2>();
        //
        //   // If lookahead displacement is degenerate, fall back to
        //   // velocity heading
        //   if (r.squaredNorm() <= kSqNormTol) {
        //     if (v2_sq < kSqNormTol) {
        //       return YawKinematics{};
        //     }
        //     return resolveYawFromVelocity(v, a, j);
        //   }
        //
        //   return resolveYawFromRay(r, r_dot, r_ddot);
        // },
