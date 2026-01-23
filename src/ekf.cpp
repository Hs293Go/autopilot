#include "autopilot/estimators/ekf.hpp"

#include <utility>

#include "autopilot/core/geometry.hpp"
#include "autopilot/core/math.hpp"
#include "autopilot/estimators/sensor_data.hpp"

namespace autopilot {

EkfEstimator::EkfEstimator(std::shared_ptr<QuadrotorModel> model,
                           std::shared_ptr<Config> config,
                           std::shared_ptr<spdlog::logger> logger)
    : EstimatorBase(kName, std::move(model), std::move(logger)),
      cfg_(std::move(config)) {}

std::unique_ptr<EstimatorContext> EkfEstimator::createContext() const {
  return std::make_unique<Context>();
}

AutopilotErrc EkfEstimator::reset(
    EstimatorContext& context, const QuadrotorState& state,
    const Eigen::Ref<const Eigen::MatrixXd>& cov) const {
  auto& ctx = static_cast<Context&>(context);

  if (cov.rows() == kNumStates && cov.cols() == kNumStates) {
    ctx.P = cov;
  } else {
    ctx.P.setIdentity();  // Fallback
  }

  ctx.accel_bias.setZero();
  ctx.gyro_bias.setZero();
  ctx.last_accel =
      -(state.odometry.pose().rotation().inverse() * model()->grav_vector());
  ctx.last_gyro.setZero();
  ctx.initialized = true;
  return AutopilotErrc::kNone;
}

AutopilotErrc EkfEstimator::predict(
    QuadrotorState& state, EstimatorContext& context,
    const std::shared_ptr<const InputBase>& u) const {
  auto& ctx = static_cast<Context&>(context);
  const auto imu = std::dynamic_pointer_cast<const ImuData>(u);
  if (!imu) {
    return AutopilotErrc::kUnknownSensorType;
  }

  double dt = imu->timestamp_secs() - state.timestamp_secs;
  if (dt <= 0) {
    return {};
  }

  const Eigen::Quaterniond q = state.odometry.pose().rotation();
  const Eigen::Vector3d omega = imu->gyro - ctx.gyro_bias;
  const Eigen::Vector3d accel = imu->accel - ctx.accel_bias;

  state.odometry.pose().translation() += dt * state.odometry.twist().linear();
  state.odometry.twist().linear() += dt * (model()->grav_vector() + q * accel);

  // Eigen::Quaterniond integration: q_dot = 0.5 * q * omega
  const Eigen::Quaterniond dq(0.0, 0.5 * omega.x() * dt, 0.5 * omega.y() * dt,
                              0.5 * omega.z() * dt);
  state.odometry.pose().rotation().coeffs() += (q * dq).coeffs();
  state.odometry.pose().rotation().normalize();
  state.odometry.twist().angular() = omega;

  const Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
  const Eigen::Matrix4d left_quaternion_matrix = LeftQuaternionMatrix(q);
  // 2. Covariance Propagation (Jacobians fjac and gjac from agi)
  StateMatrix fjac = StateMatrix::Zero();
  Eigen::Matrix<double, kNumStates, 6> gjac =
      Eigen::Matrix<double, kNumStates, 6>::Zero();

  fjac(kPosition(), kVelocity()) = Eigen::Matrix3d::Identity();
  const Eigen::Quaterniond q_omega(0.0, omega.x(), omega.y(), omega.z());
  // d/dq q_dot
  fjac(kOrientation(), kOrientation()) = 0.5 * RightQuaternionMatrix(q_omega);

  // d/dbw q_dot
  fjac(kOrientation(), kGyroBias()) = -0.5 * left_quaternion_matrix.leftCols(3);

  // d/dw q_dot
  gjac(kOrientation(), kGyro()) = 0.5 * left_quaternion_matrix.leftCols(3);

  // d/dq v_dot
  const Eigen::Vector4d qaccvec(accel.x(), accel.y(), accel.z(), 0.0);
  const Eigen::Quaterniond qacc{qaccvec};

  fjac(kVelocity(), kOrientation()) =
      jacobians::RotatePointByQuaternion(q, accel);
  // d/dba v_dot
  fjac(kVelocity(), kAccelBias()) = -rotation_matrix;

  // d/da v_dot
  gjac(kVelocity(), kAccel()) = rotation_matrix;

  const StateMatrix disc_fjac = StateMatrix::Identity() + dt * fjac;
  StateMatrix proc_cov = StateMatrix::Zero();

  proc_cov.diagonal() << cfg_->pos_proc_var, cfg_->att_proc_var,
      cfg_->vel_proc_var, cfg_->gyro_bias_proc_var, cfg_->accel_bias_proc_var;

  Eigen::Vector<double, 6> imu_var;
  imu_var << Eigen::Vector3d::Constant(cfg_->accel_noise_density),
      Eigen::Vector3d::Constant(cfg_->gyro_noise_density);
  StateMatrix cand_cov =
      disc_fjac * ctx.P * disc_fjac.transpose() + proc_cov * dt;

  // Add Noise
  // Standard white noise on inputs (dt^2)
  cand_cov.noalias() += dt * (gjac * imu_var.asDiagonal() * gjac.transpose());
  // Random walks (dt)
  ctx.P = 0.5 * (cand_cov + cand_cov.transpose());

  ctx.last_accel = imu->accel;
  ctx.last_gyro = imu->gyro;
  state.timestamp_secs = imu->timestamp_secs();

  return {};
}

AutopilotErrc EkfEstimator::correct(
    QuadrotorState& state, EstimatorContext& context,
    const std::shared_ptr<const MeasurementBase>& z) const {
  auto& ctx = static_cast<Context&>(context);
  const auto gps = std::dynamic_pointer_cast<const LocalPositionData>(z);
  if (!gps) {
    return AutopilotErrc::kUnknownSensorType;  // This "dumb" version only
                                               // handles position
  }

  // H maps 16D state to 3D position
  Eigen::Matrix<double, 3, kNumStates> hjac =
      Eigen::Matrix<double, 3, kNumStates>::Zero();
  hjac(kPosition(), kPosition()) = Eigen::Matrix3d::Identity();

  const Eigen::Vector3d y =
      gps->position_enu - state.odometry.pose().translation();
  const Eigen::Matrix3d innov_cov =
      hjac * ctx.P * hjac.transpose() + gps->covariance();
  const Eigen::Matrix<double, kNumStates, 3> kalman_gain =
      ctx.P * hjac.transpose() * innov_cov.inverse();

  // Update
  Eigen::VectorXd dx = kalman_gain * y;
  state.odometry.pose().translation() += dx(kPosition());
  state.odometry.pose().rotation().coeffs() += dx(kOrientation());
  state.odometry.pose().rotation().normalize();
  state.odometry.twist().linear() += dx(kVelocity());

  ctx.gyro_bias += dx(kGyroBias());
  ctx.accel_bias += dx(kAccelBias());

  StateMatrix error_cov_cand =
      (StateMatrix::Identity() - kalman_gain * hjac) * ctx.P;

  ctx.P = 0.5 * (error_cov_cand + error_cov_cand.transpose());

  // spdlog::info("{}", ctx.P.diagonal()(kOrientation()));

  return {};
}

QuadrotorState EkfEstimator::extrapolate(const QuadrotorState& state,
                                         const EstimatorContext& context,
                                         double t) const {
  const auto& ctx = static_cast<const Context&>(context);
  QuadrotorState pred = state;
  double dt = t - state.timestamp_secs;

  if (dt > 0) {
    const Eigen::Quaterniond q = pred.odometry.pose().rotation();
    const Eigen::Vector3d omega = ctx.last_gyro - ctx.gyro_bias;
    const Eigen::Vector3d accel = ctx.last_accel - ctx.accel_bias;

    pred.odometry.pose().translation() += dt * pred.odometry.twist().linear();
    pred.odometry.twist().linear() += dt * (model()->grav_vector() + q * accel);

    const Eigen::Quaterniond dq(0.0, 0.5 * omega.x() * dt, 0.5 * omega.y() * dt,
                                0.5 * omega.z() * dt);
    pred.odometry.pose().rotation().coeffs() += (q * dq).coeffs();
    pred.odometry.pose().rotation().normalize();
    pred.odometry.twist().angular() = omega;
    pred.timestamp_secs = t;
  }
  return pred;
}

bool EkfEstimator::isHealthy(const EstimatorContext& context) const {
  const auto& ctx = static_cast<const Context&>(context);
  return ctx.initialized && ctx.P.allFinite() &&
         ctx.P.diagonal().minCoeff() >= 0;
}

REGISTER_ESTIMATOR(EkfEstimator);

}  // namespace autopilot
