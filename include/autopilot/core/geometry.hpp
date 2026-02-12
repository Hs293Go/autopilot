#ifndef ROBOBUFFERS_GEOM_HPP_
#define ROBOBUFFERS_GEOM_HPP_

#include "Eigen/Dense"
#include "autopilot/core/common.hpp"

namespace autopilot {

// Transform (translation and rotation)
// ====================================

template <typename Derived>
struct TransformTraits;

template <std::floating_point T>
class Transform;

template <std::floating_point T>
class TransformView;

template <typename Derived>
class TransformBase {
 public:
  using Scalar = typename TransformTraits<Derived>::Scalar;
  using DerivedVector = typename TransformTraits<Derived>::Vector;
  using DerivedQuaternion = typename TransformTraits<Derived>::Quaternion;
  using Vector = Eigen::Vector3<Scalar>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using RotationMatrix = Eigen::Matrix3<Scalar>;
  using TransformValue = Transform<Scalar>;
  using TransformMatrix = Eigen::Matrix4<Scalar>;

  static constexpr int kNumParams =
      +Vector::SizeAtCompileTime + Quaternion::Coefficients::SizeAtCompileTime;
  using ParamVector = Eigen::Vector<Scalar, kNumParams>;

  template <typename OD>
  Derived& operator=(const TransformBase<OD>& t) {
    translation() = t.translation();
    rotation() = t.rotation();
    return derived();
  }

  Derived& operator=(const TransformBase& t) {
    translation() = t.translation();
    rotation() = t.rotation();
    return derived();
  }

  template <typename OD>
  Derived& operator=(TransformBase<OD>&& t) {
    translation() = t.translation();
    rotation() = t.rotation();
    return derived();
  }

  Derived& operator=(TransformBase&& t) {
    translation() = t.translation();
    rotation() = t.rotation();
    return derived();
  }

  template <typename OD>
  Derived& operator*=(const TransformBase<OD>& t) {
    translation() += rotation() * t.translation();
    rotation() *= t.rotation();
    return derived();
  }

  template <typename OD>
  TransformValue operator*(const TransformBase<OD>& t) const {
    Transform<Scalar> res = derived();
    res *= t;
    return res;
  }

  template <Vector3Like OD>
  Eigen::Vector<Scalar, 3> operator*(const Eigen::MatrixBase<OD>& v) const {
    return rotation() * v + translation();
  }

  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  const DerivedVector& translation() const { return derived().translation(); }
  const DerivedQuaternion& rotation() const { return derived().rotation(); }

  DerivedVector& translation() { return derived().translation(); }
  DerivedQuaternion& rotation() { return derived().rotation(); }

  TransformValue inverse() const {
    const Quaternion r_inv = rotation().conjugate();
    const Vector t_inv = -(r_inv * translation());
    return {t_inv, r_inv};
  }

  TransformMatrix matrix() const {
    TransformMatrix res = TransformMatrix::Identity();
    res.template topLeftCorner<3, 3>() = toRotationMatrix();
    res.template topRightCorner<3, 1>() = translation();
    return res;
  }

  RotationMatrix toRotationMatrix() const {
    return rotation().toRotationMatrix();
  }

  bool allFinite() const {
    return translation().allFinite() && rotation().coeffs().allFinite();
  }

  ParamVector params() const {
    ParamVector params;
    params.template head<3>() = translation();
    params.template segment<4>(3) = rotation().coeffs();
    return params;
  }

 private:
  TransformBase() = default;
  TransformBase(const TransformBase&) = default;
  TransformBase(TransformBase&&) = default;
  friend Derived;
};

template <std::floating_point T>
struct TransformTraits<Transform<T>> {
  using Scalar = T;
  using Vector = Eigen::Vector3<T>;
  using Quaternion = Eigen::Quaternion<T>;
};

template <typename Derived>
class TwistBase;

template <std::floating_point T>
class Transform : public TransformBase<Transform<T>> {
 public:
  using Base = TransformBase<Transform<T>>;
  using Base::kNumParams;
  Transform() = default;

  template <Vector3Like TDerived, typename RDerived>
  Transform(const Eigen::MatrixBase<TDerived>& t,
            const Eigen::QuaternionBase<RDerived>& r)
      : translation_(t), rotation_(r) {}

  template <Matrix4Like Derived>
  explicit Transform(const Eigen::MatrixBase<Derived>& mat)
      : translation_(mat.template topRightCorner<3, 1>()),
        rotation_(mat.template topLeftCorner<3, 3>()) {}

  template <typename OD>
  Transform(const TransformBase<OD>& other) {
    Base::operator=(other);
  }

  const Eigen::Vector3<T>& translation() const { return translation_; }
  const Eigen::Quaternion<T>& rotation() const { return rotation_; }

  Eigen::Vector3<T>& translation() { return translation_; }
  Eigen::Quaternion<T>& rotation() { return rotation_; }

  template <typename Derived>
  static Transform exp(const TwistBase<Derived>& twist);

  static Transform identity() {
    return {Eigen::Vector3<T>::Zero(), Eigen::Quaternion<T>::Identity()};
  }

  template <Vector3Like Derived>
  static Transform PureTranslation(const Eigen::MatrixBase<Derived>& t) {
    return {t, Eigen::Quaternion<T>::Identity()};
  }

  static Transform PureTranslation(T x, T y, T z) {
    return {Eigen::Vector3<T>(x, y, z), Eigen::Quaternion<T>::Identity()};
  }

  template <typename RDerived>
  static Transform PureRotation(const Eigen::QuaternionBase<RDerived>& r) {
    return {Eigen::Vector3<T>::Zero(), r};
  }

 private:
  Eigen::Vector3<T> translation_ = Eigen::Vector3<T>::Zero();
  Eigen::Quaternion<T> rotation_ = Eigen::Quaternion<T>::Identity();
};

template <std::floating_point T>
struct TransformTraits<TransformView<T>> {
  using Scalar = T;
  using Vector = Eigen::Map<const Eigen::Vector3<T>>;
  using Quaternion = Eigen::Map<const Eigen::Quaternion<T>>;
};

template <std::floating_point T>
class TransformView : public TransformBase<TransformView<T>> {
 public:
  using Base = TransformBase<TransformView<T>>;
  using Base::kNumParams;
  TransformView() = default;

  TransformView(const T* data) : translation_(data), rotation_(data + 3) {}

  const Eigen::Map<const Eigen::Vector3<T>>& translation() const {
    return translation_;
  }
  const Eigen::Map<const Eigen::Quaternion<T>>& rotation() const {
    return rotation_;
  }

 private:
  Eigen::Map<const Eigen::Vector3<T>> translation_;
  Eigen::Map<const Eigen::Quaternion<T>> rotation_;
};

// Twist (linear and angular velocity)
// ===================================

template <typename Derived>
struct TwistTraits;

template <std::floating_point T>
class Twist;

template <std::floating_point T>
class TwistView;

template <typename Derived>
class TwistBase {
 public:
  using Scalar = typename TwistTraits<Derived>::Scalar;
  using DerivedVector = typename TwistTraits<Derived>::Vector;
  using Vector = Eigen::Vector3<Scalar>;
  using TwistValue = Twist<Scalar>;
  static constexpr int kNumParams = Vector::SizeAtCompileTime * 2;
  using ParamVector = Eigen::Vector<Scalar, kNumParams>;

  template <typename OD>
  Derived& operator=(const TwistBase<OD>& other) {
    linear() = other.linear();
    angular() = other.angular();
    return derived();
  }

  Derived& operator=(const TwistBase& other) {
    linear() = other.linear();
    angular() = other.angular();
    return derived();
  }

  template <typename OD>
  Derived& operator=(TwistBase<OD>&& other) {
    linear() = other.linear();
    angular() = other.angular();
    return derived();
  }

  Derived& operator=(TwistBase&& other) {
    linear() = other.linear();
    angular() = other.angular();
    return derived();
  }

  template <typename OD>
  Derived& operator+=(const TwistBase<OD>& other) {
    linear() += other.linear();
    angular() += other.angular();
    return derived();
  }

  template <typename OD>
  Derived& operator-=(const TwistBase<OD>& other) {
    linear() -= other.linear();
    angular() -= other.angular();
    return derived();
  }

  Derived& operator*=(const Scalar s) {
    linear() *= s;
    angular() *= s;
    return derived();
  }

  Derived& operator/=(const Scalar s) {
    linear() /= s;
    angular() /= s;
    return derived();
  }

  template <typename OD>
  TwistValue operator+(const TwistBase<OD>& other) const {
    TwistValue res = derived();
    res += other;
    return res;
  }

  template <typename OD>
  TwistValue operator-(const TwistBase<OD>& other) const {
    TwistValue res = derived();
    res -= other;
    return res;
  }

  TwistValue operator*(const Scalar s) const {
    TwistValue res = derived();
    res *= s;
    return res;
  }

  TwistValue operator/(const Scalar s) const {
    TwistValue res = derived();
    res /= s;
    return res;
  }

  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  const DerivedVector& linear() const { return derived().linear(); }
  const DerivedVector& angular() const { return derived().angular(); }

  DerivedVector& linear() { return derived().linear(); }
  DerivedVector& angular() { return derived().angular(); }

  bool allFinite() const {
    return linear().allFinite() && angular().allFinite();
  }

  ParamVector params() const {
    ParamVector params;
    params.template head<3>() = linear();
    params.template tail<3>() = angular();
    return params;
  }

 private:
  TwistBase() = default;
  TwistBase(const TwistBase&) = default;
  TwistBase(TwistBase&&) = default;
  friend Derived;
};

template <std::floating_point T>
struct TwistTraits<Twist<T>> {
  using Scalar = T;
  using Vector = Eigen::Vector3<T>;
};

template <std::floating_point T>
class Twist : public TwistBase<Twist<T>> {
 public:
  using Base = TwistBase<Twist<T>>;
  using Base::kNumParams;
  Twist() = default;

  template <Vector3Like LDerived, Vector3Like ADerived>
  Twist(const Eigen::MatrixBase<LDerived>& v,
        const Eigen::MatrixBase<ADerived>& w)
      : linear_(v), angular_(w) {}

  Twist(const Twist&) = default;
  Twist& operator=(const Twist&) = default;

  Twist(Twist&&) = default;
  Twist& operator=(Twist&&) = default;

  template <typename OD>
  Twist(const TwistBase<OD>& other) {
    this->TwistBase<Twist>::operator=(other);
  }

  template <typename OD>
  Twist& operator=(const TwistBase<OD>& other) {
    this->TwistBase<Twist>::operator=(other);
    return *this;
  }

  const Eigen::Vector3<T>& linear() const { return linear_; }
  const Eigen::Vector3<T>& angular() const { return angular_; }

  Eigen::Vector3<T>& linear() { return linear_; }
  Eigen::Vector3<T>& angular() { return angular_; }

  static Twist zero() {
    return {Eigen::Vector3<T>::Zero(), Eigen::Vector3<T>::Zero()};
  }

 private:
  Eigen::Vector3<T> linear_ = Eigen::Vector3<T>::Zero();
  Eigen::Vector3<T> angular_ = Eigen::Vector3<T>::Zero();
};

template <std::floating_point T>
struct TwistTraits<TwistView<T>> {
  using Scalar = T;
  using Vector = Eigen::Map<const Eigen::Vector3<T>>;
};

template <std::floating_point T>
class TwistView : public TwistBase<TwistView<T>> {
 public:
  using Base = TwistBase<TwistView<T>>;
  using Base::kNumParams;

  TwistView(T const* data) : linear_(data), angular_(data + 3) {}
  const Eigen::Map<const Eigen::Vector3<T>>& linear() const { return linear_; }
  const Eigen::Map<const Eigen::Vector3<T>>& angular() const {
    return angular_;
  }

 private:
  Eigen::Map<const Eigen::Vector3<T>> linear_;
  Eigen::Map<const Eigen::Vector3<T>> angular_;
};

// Accel (linear and angular acceleration)
// =======================================

template <typename Derived>
struct AccelTraits;

template <std::floating_point T>
class Accel;

template <std::floating_point T>
class AccelView;

template <typename Derived>
class AccelBase {
 public:
  using Scalar = typename AccelTraits<Derived>::Scalar;
  using DerivedVector = typename AccelTraits<Derived>::Vector;
  using Vector = Eigen::Vector3<Scalar>;
  using AccelValue = Accel<Scalar>;

  static constexpr int kNumParams = Vector::SizeAtCompileTime * 2;
  using ParamVector = Eigen::Vector<Scalar, 6>;

  template <typename OD>
  Derived& operator=(const AccelBase<OD>& other) {
    linear() = other.linear();
    angular() = other.angular();
    return derived();
  }

  Derived& operator=(const AccelBase<Derived>& other) {
    linear() = other.linear();
    angular() = other.angular();
    return derived();
  }

  template <typename OD>
  Derived& operator+=(const TwistBase<OD>& other) {
    linear() += other.linear();
    angular() += other.angular();
    return derived();
  }

  template <typename OD>
  Derived& operator-=(const TwistBase<OD>& other) {
    linear() -= other.linear();
    angular() -= other.angular();
    return derived();
  }

  Derived& operator*=(const Scalar s) {
    linear() *= s;
    angular() *= s;
    return derived();
  }

  Derived& operator/=(const Scalar s) {
    linear() /= s;
    angular() /= s;
    return derived();
  }

  template <typename OD>
  AccelValue operator+(const AccelBase<OD>& other) const {
    AccelValue res = derived();
    res += other;
    return res;
  }

  template <typename OD>
  AccelValue operator-(const AccelBase<OD>& other) const {
    AccelValue res = derived();
    res -= other;
    return res;
  }

  AccelValue operator*(const Scalar s) const {
    AccelValue res = derived();
    res *= s;
    return res;
  }

  AccelValue operator/(const Scalar s) const {
    AccelValue res = derived();
    res /= s;
    return res;
  }

  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }
  const DerivedVector& linear() const { return derived().linear(); }
  const DerivedVector& angular() const { return derived().angular(); }
  DerivedVector& linear() { return derived().linear(); }
  DerivedVector& angular() { return derived().angular(); }

  bool allFinite() const {
    return linear().allFinite() && angular().allFinite();
  }

  ParamVector params() const {
    ParamVector params;
    params.template head<3>() = linear();
    params.template tail<3>() = angular();
    return params;
  }

 private:
  AccelBase() = default;
  AccelBase(const AccelBase&) = default;
  AccelBase(AccelBase&&) = default;
  friend Derived;
};

template <std::floating_point T>
class Accel : public AccelBase<Accel<T>> {
 public:
  using Base = AccelBase<Accel<T>>;
  using Base::kNumParams;
  Accel() = default;

  template <Vector3Like LDerived, Vector3Like ADerived>
  Accel(const Eigen::MatrixBase<LDerived>& v,
        const Eigen::MatrixBase<ADerived>& w)
      : linear_(v), angular_(w) {}

  Accel(const Accel&) = default;
  Accel& operator=(const Accel&) = default;
  Accel(Accel&&) = default;
  Accel& operator=(Accel&&) = default;

  template <typename OD>
  Accel(const AccelBase<OD>& other) {
    this->AccelBase<Accel>::operator=(other);
  }

  template <typename OD>
  Accel& operator=(const AccelBase<OD>& other) {
    this->AccelBase<Accel>::operator=(other);
    return *this;
  }

  const Eigen::Vector3<T>& linear() const { return linear_; }
  const Eigen::Vector3<T>& angular() const { return angular_; }

  Eigen::Vector3<T>& linear() { return linear_; }
  Eigen::Vector3<T>& angular() { return angular_; }

  static Accel Zero() {
    return {Eigen::Vector3<T>::Zero(), Eigen::Vector3<T>::Zero()};
  }

 private:
  Eigen::Vector3<T> linear_ = Eigen::Vector3<T>::Zero();
  Eigen::Vector3<T> angular_ = Eigen::Vector3<T>::Zero();
};

template <std::floating_point T>
struct AccelTraits<Accel<T>> {
  using Scalar = T;
  using Vector = Eigen::Vector3<T>;
};

template <std::floating_point T>
class AccelView : public AccelBase<AccelView<T>> {
 public:
  using Base = AccelBase<AccelView<T>>;
  using Base::kNumParams;
  AccelView(T const* data) : linear_(data), angular_(data + 3) {}
  const Eigen::Map<const Eigen::Vector3<T>>& linear() const { return linear_; }
  const Eigen::Map<const Eigen::Vector3<T>>& angular() const {
    return angular_;
  }

 private:
  Eigen::Map<const Eigen::Vector3<T>> linear_;
  Eigen::Map<const Eigen::Vector3<T>> angular_;
};

template <std::floating_point T>
struct AccelTraits<AccelView<T>> {
  using Scalar = T;
  using Vector = Eigen::Map<const Eigen::Vector3<T>>;
};

// Wrench (force and torque)
// =========================

template <typename Derived>
struct WrenchTraits;

template <std::floating_point T>
class Wrench;

template <std::floating_point T>
class WrenchView;

template <typename Derived>
class WrenchBase {
 public:
  using Scalar = typename WrenchTraits<Derived>::Scalar;
  using DerivedVector = typename WrenchTraits<Derived>::Vector;
  using Vector = Eigen::Vector<Scalar, 3>;
  using WrenchValue = Wrench<Scalar>;
  static constexpr int kNumParams = Vector::SizeAtCompileTime * 2;
  using ParamVector = Eigen::Vector<Scalar, kNumParams>;

  template <typename OD>
  Derived& operator=(const WrenchBase<OD>& other) {
    force() = other.force();
    torque() = other.torque();
    return derived();
  }

  Derived& operator=(const WrenchBase& other) {
    force() = other.force();
    torque() = other.torque();
    return derived();
  }

  template <typename OD>
  Derived& operator+=(const WrenchBase<OD>& other) {
    force() += other.force();
    torque() += other.torque();
    return derived();
  }

  template <typename OD>
  Derived& operator-=(const WrenchBase<OD>& other) {
    force() -= other.force();
    torque() -= other.torque();
    return derived();
  }

  Derived& operator*=(const Scalar s) {
    force() *= s;
    torque() *= s;
    return derived();
  }

  Derived& operator/=(const Scalar s) {
    force() /= s;
    torque() /= s;
    return derived();
  }

  template <typename OD>
  WrenchValue operator+(const WrenchBase<OD>& other) const {
    WrenchValue res = derived();
    res += other;
    return res;
  }

  template <typename OD>
  WrenchValue operator-(const WrenchBase<OD>& other) const {
    WrenchValue res = derived();
    res -= other;
    return res;
  }

  WrenchValue operator*(const Scalar s) const {
    WrenchValue res = derived();
    res *= s;
    return res;
  }

  WrenchValue operator/(const Scalar s) const {
    WrenchValue res = derived();
    res /= s;
    return res;
  }

  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  const DerivedVector& force() const { return derived().force(); }
  const DerivedVector& torque() const { return derived().torque(); }
  DerivedVector& force() { return derived().force(); }
  DerivedVector& torque() { return derived().torque(); }

  ParamVector params() const {
    ParamVector params;
    params.template head<3>() = force();
    params.template tail<3>() = torque();
    return params;
  }

  bool allFinite() const { return force().allFinite() && torque().allFinite(); }

 private:
  friend Derived;

  WrenchBase() = default;
  WrenchBase(const WrenchBase&) = default;
  WrenchBase(WrenchBase&&) = default;
};

template <std::floating_point T>
struct WrenchTraits<Wrench<T>> {
  using Scalar = T;
  using Vector = Eigen::Vector3<T>;
};

template <std::floating_point T>
class Wrench : public WrenchBase<Wrench<T>> {
 public:
  using Base = WrenchBase<Wrench<T>>;
  using Base::kNumParams;

  Wrench() = default;
  Wrench(const Eigen::Vector3<T>& f, const Eigen::Vector3<T>& t)
      : force_(f), torque_(t) {}

  Wrench(const Wrench&) = default;
  Wrench& operator=(const Wrench&) = default;

  Wrench(Wrench&&) = default;
  Wrench& operator=(Wrench&&) = default;

  template <typename OD>
  Wrench(const WrenchBase<OD>& other) {
    this->WrenchBase<Wrench>::operator=(other);
  }

  template <typename OD>
  Wrench& operator=(const WrenchBase<OD>& other) {
    this->WrenchBase<Wrench>::operator=(other);
    return *this;
  }

  const Eigen::Vector3<T>& force() const { return force_; }
  const Eigen::Vector3<T>& torque() const { return torque_; }
  Eigen::Vector3<T>& force() { return force_; }
  Eigen::Vector3<T>& torque() { return torque_; }
  static Wrench<T> Zero() {
    return {Eigen::Vector3<T>::Zero(), Eigen::Vector3<T>::Zero()};
  }

 private:
  Eigen::Vector3<T> force_ = Eigen::Vector3<T>::Zero();
  Eigen::Vector3<T> torque_ = Eigen::Vector3<T>::Zero();
};

template <std::floating_point T>
struct WrenchTraits<WrenchView<T>> {
  using Scalar = T;
  using Vector = Eigen::Map<const Eigen::Vector3<T>>;
};

template <std::floating_point T>
class WrenchView : public WrenchBase<WrenchView<T>> {
 public:
  using Base = WrenchBase<WrenchView<T>>;
  using Base::kNumParams;

  WrenchView(T const* data) : force_(data), torque_(data + 3) {}
  const Eigen::Map<const Eigen::Vector3<T>>& force() const { return force_; }
  const Eigen::Map<const Eigen::Vector3<T>>& torque() const { return torque_; }

 private:
  Eigen::Map<const Eigen::Vector3<T>> force_;
  Eigen::Map<const Eigen::Vector3<T>> torque_;
};

// Odometry (pose and twist)
// =========================

template <typename Derived>
struct OdometryTraits;

template <typename Derived>
class OdometryBase {
 public:
  using Scalar = typename OdometryTraits<Derived>::Scalar;
  using TransformType = typename OdometryTraits<Derived>::TransformType;
  using TwistType = typename OdometryTraits<Derived>::TwistType;
  static constexpr int kNumParams =
      TransformType::kNumParams + TwistType::kNumParams;
  using ParamVector = Eigen::Vector<Scalar, kNumParams>;

  Derived& operator=(const OdometryBase<Derived>& other) {
    pose() = other.pose();
    twist() = other.twist();
    return derived();
  }

  template <typename OD>
  Derived& operator=(const OdometryBase<OD>& other) {
    pose() = other.pose();
    twist() = other.twist();
    return derived();
  }

  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  const TransformType& pose() const { return derived().pose(); }
  const TwistType& twist() const { return derived().twist(); }

  TransformType& pose() { return derived().pose(); }
  TwistType& twist() { return derived().twist(); }

  bool allFinite() const { return pose().allFinite() && twist().allFinite(); }

  ParamVector params() const {
    ParamVector params;
    params.template head<7>() = pose().params();
    params.template tail<6>() = twist().params();
    return params;
  }

  friend Derived;

 private:
  OdometryBase() = default;

  OdometryBase(const OdometryBase&) = default;

  OdometryBase(OdometryBase&&) = default;
};

template <std::floating_point T>
class Odometry;

template <std::floating_point T>
class OdometryView;

template <std::floating_point T>
struct OdometryTraits<Odometry<T>> {
  using Scalar = T;
  using TransformType = Transform<T>;
  using TwistType = Twist<T>;
};

template <std::floating_point T>
class Odometry : public OdometryBase<Odometry<T>> {
 public:
  using Base = OdometryBase<Odometry<T>>;
  using Base::kNumParams;
  Odometry() = default;
  Odometry(const Transform<T>& p, const Twist<T>& t) : pose_(p), twist_(t) {}

  Odometry(const Odometry&) = default;
  Odometry& operator=(const Odometry&) = default;

  Odometry(Odometry&&) = default;
  Odometry& operator=(Odometry&&) = default;

  template <typename OD>
  Odometry(const OdometryBase<OD>& other) {
    this->Base::operator=(other);
  }

  const Transform<T>& pose() const { return pose_; }
  const Twist<T>& twist() const { return twist_; }
  Transform<T>& pose() { return pose_; }
  Twist<T>& twist() { return twist_; }

 private:
  Transform<T> pose_;
  Twist<T> twist_;
};

template <std::floating_point T>
struct OdometryTraits<OdometryView<T>> {
  using Scalar = T;
  using TransformType = TransformView<T>;
  using TwistType = TwistView<T>;
};

template <std::floating_point T>
class OdometryView : public OdometryBase<OdometryView<T>> {
 public:
  using Base = OdometryBase<OdometryView<T>>;
  using Base::kNumParams;

  OdometryView() = default;
  OdometryView(const T* data) : pose_(data), twist_(data + 7) {}
  const TransformView<T>& pose() const { return pose_; }
  const TwistView<T>& twist() const { return twist_; }

 private:
  TransformView<T> pose_;
  TwistView<T> twist_;
};

using TransformF32 = Transform<float>;
using TransformViewF32 = TransformView<float>;
using TransformF64 = Transform<double>;
using TransformViewF64 = TransformView<double>;

using TwistF32 = Twist<float>;
using TwistViewF32 = TwistView<float>;
using TwistF64 = Twist<double>;
using TwistViewF64 = TwistView<double>;

using OdometryF32 = Odometry<float>;
using OdometryViewF32 = OdometryView<float>;
using OdometryF64 = Odometry<double>;
using OdometryViewF64 = OdometryView<double>;

using AccelF32 = Accel<float>;
using AccelViewF32 = AccelView<float>;
using AccelF64 = Accel<double>;
using AccelViewF64 = AccelView<double>;

using WrenchF32 = Wrench<float>;
using WrenchViewF32 = WrenchView<float>;
using WrenchF64 = Wrench<double>;
using WrenchViewF64 = WrenchView<double>;

static_assert(requires {
  { TransformF64::kNumParams } -> std::convertible_to<int>;
  { TransformViewF64::kNumParams } -> std::convertible_to<int>;
  { TwistF64::kNumParams } -> std::convertible_to<int>;
  { TwistViewF64::kNumParams } -> std::convertible_to<int>;
  { OdometryF64::kNumParams } -> std::convertible_to<int>;
  { OdometryViewF64::kNumParams } -> std::convertible_to<int>;
});

}  // namespace autopilot

#endif  // ROBOBUFFERS_GEOM_HPP_
