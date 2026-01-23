#include "Eigen/Dense"
#include "autopilot/core/definitions.hpp"
#include "autopilot/core/math.hpp"

namespace autopilot::ocp {

static constexpr Eigen::Index kAmbientDim = OdometryF64::kNumParams;
static constexpr Eigen::Index kTangentDim = 12;
static constexpr Eigen::Index kControlDim = 4;

struct StateCost {
  double qval;
  Eigen::Vector<double, kTangentDim> qjac;
  Eigen::Vector<double, kControlDim> rjac;
  Eigen::Matrix<double, kTangentDim, kTangentDim> qhes;
  Eigen::Matrix<double, kControlDim, kControlDim> rhes;
#ifdef AUTOPILOT_ILQC_ENABLE_CROSS_TERMS
  Eigen::Matrix<double, kControlDim, kTangentDim> phes;
#endif
};

struct TerminalCost {
  double qval;
  Eigen::Vector<double, kTangentDim> qjac;
  Eigen::Matrix<double, kTangentDim, kTangentDim> qhes;
};

struct State {
  double timestamp_secs;
  OdometryF64 odom;  // p, q, v, w
};

struct Control {
  Eigen::Vector4d motor_speeds;  // thrust, body rates
};

struct AngleErrorJacobianResult {
  Eigen::Vector3d angle_error;
  Eigen::Quaterniond attitude_error;
  Eigen::Matrix3d jacobian;
};

static AngleErrorJacobianResult AngleErrorAndJacobian(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& q_des) {
  //  // Simple quaternion error cost
  const Eigen::Quaterniond qe = q.inverse() * q_des;

  // Swing-twist decomposition based angle error
  const Eigen::Vector3d angle_error(qe.x() * qe.z() + qe.y() * qe.w(),
                                    qe.y() * qe.z() - qe.x() * qe.w(), qe.z());

  // Derive jacobian of angle error w.r.t. quaternion components
  constexpr auto kRegularizer = 1e-3;

  // Common scaling term in the swing-twist error
  const double scl_sq =
      1.0 / (qe.w() * qe.w() + qe.z() * qe.z() + kRegularizer);
  const double scl = std::sqrt(scl_sq);
  const double scl_cb = scl_sq * scl;

  const double k1 = angle_error.x() / scl_cb;
  const double k2 = angle_error.y() / scl_cb;
  const Eigen::Matrix<double, 3, 4> d_err_by_qe{
      {qe.w() * scl, -qe.z() * scl, -qe.w() * k1, qe.z() * k1},
      {qe.z() / scl, qe.w() * scl, qe.w() * k2, qe.y() * k2},
      {0.0, 0.0, qe.w() * qe.w() * scl_cb, -qe.z() * qe.w() * scl_cb}};

  // qe = R(q_des) * q.inverse()
  const Eigen::Matrix4d d_qe_by_q =
      RightQuaternionMatrix(q_des) * Diag(-1.0, -1.0, -1.0, 1.0);

  // q is actually q_nom * I(delta phi)
  // dq/dphi = L(q_nom)[:, 0:3]
  const Eigen::Matrix<double, 4, 3> d_q_by_dphi =
      0.5 * LeftQuaternionMatrix(q).leftCols<3>();
  return {.angle_error = angle_error,
          .attitude_error = qe,
          .jacobian = d_err_by_qe * d_qe_by_q * d_q_by_dphi};
}

// NOTE: Validate outside of cost functions. These must be plain math
struct Costs {
  StateCost StageCost(const State& state, const State& state_ref,
                      const Control& cmd, const Control& cmd_ref) const {
    const auto& q = state.odom.pose().rotation();
    const auto& q_des = state_ref.odom.pose().rotation();
    const auto& [angle_error, qe, ae_by_phi] = AngleErrorAndJacobian(q, q_des);
    Eigen::Matrix<double, kTangentDim, kTangentDim> d_err_by_x;
    d_err_by_x.setZero();
    d_err_by_x.block<3, 3>(0, 0).setIdentity();
    d_err_by_x.block<3, 3>(3, 3) = ae_by_phi;
    d_err_by_x.block<3, 3>(6, 6).setIdentity();
    d_err_by_x.block<3, 3>(9, 9).setIdentity();

    Eigen::Vector<double, kTangentDim> xerr;
    xerr << state.odom.pose().translation() -
                state_ref.odom.pose().translation(),                    //
        angle_error,                                                    //
        state.odom.twist().linear() - state_ref.odom.twist().linear(),  //
        state.odom.twist().angular() - state_ref.odom.twist().angular();
    const Eigen::Vector4d uerr = cmd.motor_speeds - cmd_ref.motor_speeds;

    const Eigen::Matrix<double, kTangentDim, kTangentDim> scl_d_err_by_x =
        d_err_by_x.transpose() * qwgt;

    const Eigen::Vector<double, kTangentDim> qjac = 2.0 * scl_d_err_by_x * xerr;
    const Eigen::Matrix<double, kTangentDim, kTangentDim> qhes =
        2.0 * scl_d_err_by_x * d_err_by_x;

    return StateCost{
        .qval = xerr.dot(qwgt * xerr) + uerr.dot(rwgt * uerr),
        .qjac = qjac,
        .rjac = 2.0 * rwgt * uerr,
        .qhes = qhes,
        .rhes = 2.0 * rwgt,
    };
  }

  TerminalCost TerminalCost(const State& state) const;

  Eigen::Matrix<double, kTangentDim, kTangentDim> qwgt;
  Eigen::Matrix<double, kControlDim, kControlDim> rwgt;
};

}  // namespace autopilot::ocp
