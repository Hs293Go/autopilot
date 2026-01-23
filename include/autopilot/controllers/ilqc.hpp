#ifndef AUTOPILOT_CONTROLLER_ILQC_HPP_
#define AUTOPILOT_CONTROLLER_ILQC_HPP_

#ifndef AUTOPILOT_ILQC_HORIZON_STEPS
#define AUTOPILOT_ILQC_HORIZON_STEPS 20
#endif

#include <utility>

#include "Eigen/Dense"
#include "autopilot/core/definitions.hpp"
#include "autopilot/core/math.hpp"

namespace autopilot::ocp {

inline constexpr Eigen::Index kAmbientDim = OdometryF64::kNumParams;
inline constexpr Eigen::Index kTangentDim = 12;
inline constexpr Eigen::Index kControlDim = 4;
inline constexpr Eigen::Index kHorizonSteps = AUTOPILOT_ILQC_HORIZON_STEPS;

struct StageCost {
  double val;
  Eigen::Vector<double, kTangentDim> lx;
  Eigen::Vector<double, kControlDim> lu;
  Eigen::Matrix<double, kTangentDim, kTangentDim> lxx;
  Eigen::Matrix<double, kControlDim, kControlDim> lux;
#ifdef AUTOPILOT_ILQC_ENABLE_CROSS_TERMS
  Eigen::Matrix<double, kControlDim, kTangentDim> phes;
#endif
};

struct TerminalCost {
  double val;
  Eigen::Vector<double, kTangentDim> vx;
  Eigen::Matrix<double, kTangentDim, kTangentDim> vxx;
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
  StageCost stage(const State& state, const State& state_ref,
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

    return {
        .val = xerr.dot(qwgt * xerr) + uerr.dot(rwgt * uerr),
        .lx = qjac,
        .lu = 2.0 * rwgt * uerr,
        .lxx = qhes,
        .lux = 2.0 * rwgt,
    };
  }

  TerminalCost terminal(const State& state, const State& state_ref) const {
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

    const Eigen::Matrix<double, kTangentDim, kTangentDim> scl_d_err_by_x =
        d_err_by_x.transpose() * qwgt;

    const Eigen::Vector<double, kTangentDim> qjac = 2.0 * scl_d_err_by_x * xerr;
    const Eigen::Matrix<double, kTangentDim, kTangentDim> qhes =
        2.0 * scl_d_err_by_x * d_err_by_x;

    return {
        .val = xerr.dot(qwgt * xerr),
        .vx = qjac,
        .vxx = qhes,
    };
  }

  Eigen::Matrix<double, kTangentDim, kTangentDim> qwgt;
  Eigen::Matrix<double, kControlDim, kControlDim> rwgt;
};

struct Trajectory {
  Eigen::Matrix<double, kAmbientDim, kHorizonSteps + 1> xs;
  Eigen::Matrix<double, kControlDim, kHorizonSteps> us;
};

struct Linearization {
  std::array<Eigen::Matrix<double, kTangentDim, kTangentDim>, kHorizonSteps>
      fjac;
  std::array<Eigen::Matrix<double, kTangentDim, kControlDim>, kHorizonSteps>
      gjac;
};

struct Policy {
  Eigen::Matrix<double, kControlDim, kHorizonSteps> u_ff;
  std::array<Eigen::Matrix<double, kControlDim, kTangentDim>, kHorizonSteps>
      k_fb;
};

struct ILQCPlannerCfg {
  Eigen::Index max_iter = 50;
  double reg = 1e-6;  // Levenberg reg on Quu
  double tol_cost = 1e-4;
  double tol_step = 1e-4;
};

class ILQCPlanner {
 public:
  ILQCPlanner(Costs costs, ILQCPlannerCfg cfg)
      : costs_(std::move(costs)), cfg_(cfg) {}

  void solve(const State& x0, std::span<const State> x_ref,
             std::span<const Control> u_ref, Policy* policy_out) {
    Policy policy;
    // Initialization
    for (std::size_t i = 0; i < kHorizonSteps; ++i) {
      policy.u_ff(ix::all, i) = u_ref[i].motor_speeds;
      policy.k_fb[i].setZero();
    }

    Trajectory traj;
    double last_cost = std::numeric_limits<double>::infinity();

    for (Eigen::Index iter = 0; iter < cfg_.max_iter; ++iter) {
      // 1. Rollout: Forward simulate the current policy
      traj = rollout(x0, policy);

      // 2. Evaluate Cost
      double current_total_cost = 0;
      std::vector<StageCost> stage_costs;
      stage_costs.reserve(kHorizonSteps);

      for (std::size_t k = 0; k < kHorizonSteps; ++k) {
        Control u_k{.motor_speeds = traj.us(ix::all, k)};
        State x_k = ambient_to_state(traj.xs(ix::all, k));
        stage_costs.push_back(costs_.stage(x_k, x_ref[k], u_k, u_ref[k]));
        current_total_cost += stage_costs.back().val;
      }

      State x_N = ambient_to_state(traj.xs(ix::all, kHorizonSteps));
      TerminalCost term_cost = costs_.terminal(x_N, x_ref[kHorizonSteps]);
      current_total_cost += term_cost.val;

      // Convergence Check (Cost)
      if (std::abs(last_cost - current_total_cost) / current_total_cost <
          cfg_.tol_cost) {
        break;
      }
      last_cost = current_total_cost;

      // 3. Linearize Dynamics along the trajectory
      Linearization lin = linearize(traj);

      // 4. Backward Pass & Update Policy
      double step_norm = 0;
      policy = backward_pass(traj, lin, stage_costs, term_cost, &step_norm);

      // Convergence Check (Step)
      if (step_norm < cfg_.tol_step) {
        break;
      }
    }

    if (policy_out) {
      *policy_out = policy;
    }
  }

 private:
  Costs costs_;
  ILQCPlannerCfg cfg_;

  Trajectory rollout(const State& x0, const Policy& policy) {
    Trajectory traj;
    traj.xs.col(0) =
        x0.odom.params();  // Assuming params() returns ambient state vector

    for (std::size_t k = 0; k < kHorizonSteps; ++k) {
      // Compute error in tangent space for feedback (x - x_nom)
      // Since we don't have a nominal traj yet in the first iter,
      // we usually linearize around the rollout.

      // In iLQR, u = u_ff + K * dx. Here dx is effectively 0 during the nominal
      // rollout because u_ff is updated to include the previous feedback.
      traj.us(ix::all, k) = policy.u_ff(ix::all, k);

      // Update dynamics: x_{k+1} = f(x_k, u_k)
      // Note: You need a Dynamics model instance here, similar to Python's
      // self._mdl traj.xs.col(k+1) = model_.update(traj.xs.col(k),
      // traj.us.col(k));
    }
    return traj;
  }

  Linearization linearize(const Trajectory& traj) {
    Linearization lin;
    for (std::size_t k = 0; k < kHorizonSteps; ++k) {
      // Compute Jacobians A (fjac) and B (gjac) at each step
      // model_.df(traj.xs.col(k), traj.us.col(k), &lin.fjac[k], &lin.gjac[k]);
    }
    return lin;
  }

  Policy backward_pass(const Trajectory& traj, const Linearization& lin,
                       const std::vector<StageCost>& stage_costs,
                       const TerminalCost& term_cost, double* step_norm) {
    Policy new_policy;

    // Boundary conditions at N
    Eigen::Vector<double, kTangentDim> v_x = term_cost.vx;
    Eigen::Matrix<double, kTangentDim, kTangentDim> v_xx = term_cost.vxx;

    Eigen::Matrix<double, kHorizonSteps, kControlDim> du_ff;

    for (std::size_t k = kHorizonSteps - 1; std::cmp_greater_equal(k, 0); --k) {
      const auto& A = lin.fjac[k];
      const auto& B = lin.gjac[k];
      const auto& sc = stage_costs[k];

      // Q-function expansion
      Eigen::Vector<double, kControlDim> Q_u = sc.lu + B.transpose() * v_x;
      Eigen::Matrix<double, kControlDim, kTangentDim> Q_ux =
          sc.lux + B.transpose() * v_xx * A;
      Eigen::Matrix<double, kControlDim, kControlDim> Q_uu =
          sc.lux.leftCols(kControlDim) /* Simplified */ +
          B.transpose() * v_xx * B;

      // Regularization
      Eigen::Matrix<double, kControlDim, kControlDim> Q_uu_reg =
          Q_uu +
          cfg_.reg *
              Eigen::Matrix<double, kControlDim, kControlDim>::Identity();

      // Solve for gains
      Eigen::LLT<Eigen::MatrixXd> llt(Q_uu_reg);
      Eigen::Vector<double, kControlDim> k_ff = llt.solve(-Q_u);
      Eigen::Matrix<double, kControlDim, kTangentDim> K_fb = llt.solve(-Q_ux);

      du_ff(ix::all, k) = k_ff;
      new_policy.k_fb[k] = K_fb;

      // Update Value Function approximation for step k-1
      v_x = sc.lx + A.transpose() * v_x + K_fb.transpose() * Q_uu * k_ff +
            K_fb.transpose() * Q_u + Q_ux.transpose() * k_ff;
      v_xx = sc.lxx + A.transpose() * v_xx * A +
             K_fb.transpose() * Q_uu * K_fb + K_fb.transpose() * Q_ux +
             Q_ux.transpose() * K_fb;
    }

    // Update feedforward: u_new = u_old + du_ff
    new_policy.u_ff = traj.us.transpose() + du_ff;
    *step_norm = du_ff.norm();

    return new_policy;
  }

  // Helper to convert internal Eigen matrices back to State structs
  State ambient_to_state(const Eigen::Vector<double, kAmbientDim>& x_vec) {
    State s;
    s.odom = OdometryView(x_vec.data());
    return s;
  }
};

}  // namespace autopilot::ocp

#endif  // AUTOPILOT_CONTROLLER_ILQC_HPP_
