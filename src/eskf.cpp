
#include "autopilot/estimators/eskf.hpp"

#include <utility>

#include "autopilot/base/estimator_base.hpp"
#include "autopilot/estimators/sensor_data.hpp"
// Assuming math headers are merged as discussed
#include "autopilot/core/geometry.hpp"
#include "autopilot/core/math.hpp"
#include "boost/math/distributions/chi_squared.hpp"
#if __has_include(<spdlog/fmt/ranges.h>)
#include "spdlog/fmt/ranges.h"
#else
#include "fmt/ranges.h"
#endif

namespace autopilot {

// F = df(x, u, w)/dx
using SystemJacobian = Eigen::Matrix<double, EskfEstimator::kNumErrorStates,
                                     EskfEstimator::kNumErrorStates>;
using ProcNoiseCov = Eigen::Matrix<double, EskfEstimator::kNumErrorStates,
                                   EskfEstimator::kNumErrorStates>;

using EnuPositionJacobian =
    Eigen::Matrix<double, 3, EskfEstimator::kNumErrorStates>;
using EnuPositionKalmanGain =
    Eigen::Matrix<double, EskfEstimator::kNumErrorStates, 3>;

using MagJacobian = Eigen::Matrix<double, 3, EskfEstimator::kNumErrorStates>;
using MagKalmanGain = Eigen::Matrix<double, EskfEstimator::kNumErrorStates, 3>;

static const boost::math::chi_squared kDist(3);

EskfEstimator::EskfEstimator(std::shared_ptr<QuadrotorModel> model,
                             std::shared_ptr<Config> config,
                             std::shared_ptr<spdlog::logger> logger)
    : EstimatorBase(kName, std::move(model), std::move(logger)),
      config_(std::move(config)),
      gps_outlier_classifier_(
          boost::math::quantile(kDist, config_->gps_confidence_level_warning),

          boost::math::quantile(kDist, config_->gps_confidence_level_error)),
      mag_outlier_classifier_(
          boost::math::quantile(kDist, config_->mag_confidence_level_warning),
          boost::math::quantile(kDist, config_->mag_confidence_level_error)) {}

std::unique_ptr<EstimatorContext> EskfEstimator::createContext() const {
  return std::make_unique<Context>();
}

AutopilotErrc EskfEstimator::reset(
    EstimatorContext& context, const QuadrotorState& initial_state,
    const Eigen::Ref<const Eigen::MatrixXd>& initial_cov) const {
  auto& ctx = static_cast<Context&>(context);
  // Lock handled by AsyncEstimator base if needed, or we assume this
  // is called before start().
  if (initial_cov.rows() != kNumErrorStates ||
      initial_cov.cols() != kNumErrorStates) {
    return setError(AutopilotErrc::kInvalidDimension);
  }

  ctx.P = initial_cov;
  ctx.accel_bias.setZero();
  ctx.gyro_bias.setZero();
  const auto& q = initial_state.odometry.pose().rotation();
  ctx.last_accel = -(q.inverse() * model()->grav_vector());
  ctx.last_gyro.setZero();

  ctx.is_initialized = true;
  has_previous_error_ = false;
  return {};
}

bool EskfEstimator::isHealthy(const EstimatorContext& context) const {
  const auto& ctx = static_cast<const Context&>(context);
  // 2. Check Initialization (Frankenstate Guard)
  if (!context.isInitialized()) {
    return false;
  }

  if (has_previous_error_) {
    return false;
  }

  // 3. Check Mathematical Stability
  // Access context.P directly (Thread-safe? context.P is modified on worker,
  // read here on control thread?) Ideally, we should check a
  // 'committed_health_' flag updated by the worker, or grab the state_mutex_ if
  // context.P is protected. Assuming context.P is effectively atomic or we
  // accept a tearing read for a health check:

  if (!ctx.P.allFinite()) {
    return false;
  }

  // Check diagonal positivity (Standard Variance constraint)
  if (ctx.P.diagonal().minCoeff() < 0) {
    return false;
  }

  // Optional: Check trace magnitude (Is uncertainty exploding?)
  if (ctx.P.trace() > config_->max_sum_error_variance) {
    return false;
  }

  return true;
}

void reportPosteriorStats(const std::shared_ptr<spdlog::logger>& logger,
                          const Eigen::Ref<const Eigen::MatrixXd>& posterior) {
  logger->error("Posterior covariance is not finite");

  const auto min_val = posterior.minCoeff();
  const auto max_val = posterior.maxCoeff();
  logger->error("Covariance range: min={:.3e} max={:.3e}", min_val, max_val);

  auto num_non_finite = (!posterior.array().isFinite()).count();
  logger->debug(
      "Posterior covariance matrix has {} non-finite entries "
      "out of {}",
      num_non_finite, posterior.size());

  logger->trace("Posterior covariance matrix: {}", posterior.rowwise());
}

// -----------------------------------------------------------------------------
// WORKER THREAD: Input Handling
// -----------------------------------------------------------------------------
AutopilotErrc EskfEstimator::predict(
    QuadrotorState& state, EstimatorContext& context,
    const std::shared_ptr<const InputBase>& u) const {
  if (!context.isInitialized()) {
    logger()->warn("ESKF: Ignoring input before initialization");
    return setError(AutopilotErrc::kEstimatorUninitialized);
  }
  auto& ctx = static_cast<Context&>(context);

  auto cand = state;

  // Dispatch
  if (const auto imu = std::dynamic_pointer_cast<const ImuData>(u)) {
    // 1. Compute dt
    // In a robust system, we track last_imu_time. For now, trust the sequence.
    // If this is the first packet, we can't integrate.
    double dt = imu->timestamp - cand.timestamp_secs;
    if (dt <= 0) {
      return setError(AutopilotErrc::kTimestampOutOfOrder);
    }  // Out of order or duplicate

    // 2. Predict Error Covariance
    if (auto ec = predictCovariance(cand, ctx, imu->accel, imu->gyro, dt);
        ec != AutopilotErrc::kNone) {
      return ec;
    }

    // 3. Predict Nominal cand (Kinematics)
    if (auto ec = predictKinematics(cand, ctx, imu->accel, imu->gyro, dt);
        ec != AutopilotErrc::kNone) {
      return ec;
    }

    // Cache for extrapolation
    ctx.last_accel = imu->accel;
    ctx.last_gyro = imu->gyro;

    // 4. Commit
    state = cand;
    return {};
  }

  logger()->warn("ESKF: Unknown input type");
  return setError(AutopilotErrc::kUnknownSensorType);
}

// -----------------------------------------------------------------------------
// WORKER THREAD: Measurement Handling
// -----------------------------------------------------------------------------
AutopilotErrc EskfEstimator::correct(
    QuadrotorState& state, EstimatorContext& context,
    const std::shared_ptr<const MeasurementBase>& z) const {
  AutopilotErrc ec;

  auto& ctx = static_cast<Context&>(context);

  if (const auto gps = std::dynamic_pointer_cast<const LocalPositionData>(z)) {
    ec = correctGps(state, ctx, gps);
  } else if (const auto mag = std::dynamic_pointer_cast<const MagData>(z)) {
    ec = correctMag(state, ctx, mag);
  } else {
    logger()->warn("ESKF: Unknown measurement type");
    return setError(AutopilotErrc::kUnknownSensorType);
  }

  if (ec != AutopilotErrc::kNone) {
    logger()->warn("ESKF Update Failed: {}", ec);
    // Robustness: Maybe inflate P if update fails?
  }
  return ec;
}

// -----------------------------------------------------------------------------
// CONTROL THREAD: Extrapolation
// -----------------------------------------------------------------------------
QuadrotorState EskfEstimator::extrapolate(const QuadrotorState& state,
                                          const EstimatorContext& context,
                                          double t) const {
  const auto& ctx = static_cast<const Context&>(context);
  QuadrotorState pred_state = state;
  double dt = t - state.timestamp_secs;

  // Use last known inputs to predict forward.
  // Biases are applied inside predictKinematics using internal members.
  if (dt > 0) {
    if (auto ec = predictKinematics(pred_state, ctx, ctx.last_accel,
                                    ctx.last_gyro, dt);
        ec != AutopilotErrc::kNone) {
      logger()->warn("ESKF Extrapolation Failed at t={:.3} (dt={:.3}): {}", t,
                     dt, ec);
      return state;  // Return unmodified state on failure
    }
  }
  return pred_state;
}

template <int N>
EskfEstimator::InnovStats<N> EskfEstimator::computeMahalanobisDistance(
    const Eigen::Vector<double, N>& innovation,
    const Eigen::Matrix<double, N, N>& innov_cov) const {
  auto llt_fac = innov_cov.llt();
  // Compute squared Mahalanobis distance for chi-squared distance check in
  // every case, distinguishing between the two cases of Cholesky
  // factorization success or failure.

  if (llt_fac.info() == Eigen::Success) {
    // Cholesky factorization succeeded
    return {innovation.dot(llt_fac.solve(innovation)), llt_fac};
  }

  logger()->error(
      "Cholesky factorization failed, using SVD for Mahalanobis distance");

  const auto svdfac =
      innov_cov.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  const double min_sv = svdfac.singularValues()[N - 1];
  const double max_sv = svdfac.singularValues()[0];
  logger()->debug("Singular values range: min={:.3}, max={:.3}", min_sv,
                  max_sv);

  logger()->trace("Innovation covariance matrix: {}", innov_cov.rowwise());

  // Solve and ignore errors since we know the singular vectors have been
  // computed
  return {innovation.dot(svdfac.solve(innovation)), llt_fac};
}

template <typename Fn>
bool EskfEstimator::checkOutlier(double m2_dist,
                                 const OutlierClassifier& classifier,
                                 Fn reporter) const {
  switch (classifier.classify(m2_dist)) {
    case OutlierClassification::kNormal:
      break;
    case OutlierClassification::kWarning:
      reporter();
      logger()->warn(
          "Measurement Warning: Mahalanobis distance = {:.3f} vs Chi-squared "
          "distance = {:.3f}",
          m2_dist, classifier.warning_threshold());
      break;
    case OutlierClassification::kError:
      reporter();
      logger()->error(
          "Measurement Rejected: Mahalanobis distance = {:.3f} vs Chi-squared "
          "distance = {:.3f}",
          m2_dist, classifier.error_threshold());
      return false;
  }
  return true;
}

void EskfEstimator::reportObservationStats(
    const Eigen::Ref<const Eigen::MatrixXd>& innov_cov,
    const Eigen::Ref<const Eigen::VectorXd>& expectation,
    const Eigen::Ref<const Eigen::VectorXd>& observation) const {
  const auto max_val = innov_cov.diagonal().maxCoeff();
  const auto min_val = innov_cov.diagonal().minCoeff();
  logger()->debug("Innov diag: min={:.3f}, max={:.3f}", min_val, max_val);
  logger()->trace("Predicted observation: {}", expectation);
  logger()->trace("Actual observation: {}", observation);
}

// -----------------------------------------------------------------------------
// Math: Kinematics (Shared by Predict and Extrapolate)
// -----------------------------------------------------------------------------
AutopilotErrc EskfEstimator::predictKinematics(QuadrotorState& state,
                                               const Context& context,
                                               const Eigen::Vector3d& accel,
                                               const Eigen::Vector3d& gyro,
                                               double dt) const {
  // 1. Bias Correction
  Eigen::Vector3d acc_unbiased = accel - context.accel_bias;
  Eigen::Vector3d gyr_unbiased = gyro - context.gyro_bias;

  // 2. Integration
  OdometryF64 cand = state.odometry;
  const auto& p = state.odometry.pose().translation();
  const auto& q = state.odometry.pose().rotation();
  const auto& v = state.odometry.twist().linear();

  // a_world = R * a_body + g
  // Assumption: model()->grav_vector() is [0, 0, -9.81]
  const Eigen::Vector3d acc_world = q * acc_unbiased + model()->grav_vector();

  const Eigen::Vector3d delta_velocity = acc_world * dt;
  const Eigen::Vector3d delta_angle = gyr_unbiased * dt;

  cand.pose().translation() = p + v * dt + 0.5 * delta_velocity * dt;
  cand.pose().rotation() = q * AngleAxisToQuaternion(delta_angle);
  cand.twist().linear() = v + delta_velocity;
  cand.twist().angular() = gyr_unbiased;

  if (!p.allFinite() || !q.coeffs().allFinite() || !v.allFinite()) {
    logger()->warn("ESKF: Non-finite state in kinematics prediction");
    logger()->trace("Position: {}, Orientation: {}, Velocity: {}", p,
                    q.coeffs(), v);
    // This is a const member function (functionally pure), which doesn't leave
    // the estimator in a bad state. No need to set has_previous_error_.
    return AutopilotErrc::kNumericallyNonFinite;
  }
  state.odometry = cand;
  state.timestamp_secs += dt;
  return {};
}

// -----------------------------------------------------------------------------
// Math: Covariance Prediction
// -----------------------------------------------------------------------------
AutopilotErrc EskfEstimator::predictCovariance(const QuadrotorState& state,
                                               Context& context,
                                               const Eigen::Vector3d& accel,
                                               const Eigen::Vector3d& gyro,
                                               double dt) const {
  // Jacobian F

  const auto& q = state.odometry.pose().rotation();
  const Eigen::Matrix3d rotmat = q.toRotationMatrix();
  const Eigen::Vector3d acc_unbiased = accel - context.accel_bias;
  const Eigen::Vector3d gyr_unbiased = gyro - context.gyro_bias;
  const Eigen::Vector3d delta_angle = gyr_unbiased * dt;

  // Pos -> Vel

  SystemJacobian fjac = SystemJacobian::Zero();
  const Eigen::Matrix3d dt_eye = Eigen::Matrix3d::Identity() * dt;
  const Eigen::Matrix3d dt_rotmat = rotmat * dt;

  // Position
  fjac(kPositionError(), kPositionError()).setIdentity();
  fjac(kPositionError(), kVelocityError()) = dt_eye;

  // Rotation
  fjac(kRotationError(), kRotationError()) =
      AngleAxisToRotationMatrix(-delta_angle);
  fjac(kRotationError(), kGyroBiasError()) = -dt_eye;

  // Velocity
  fjac(kVelocityError(), kVelocityError()).setIdentity();
  fjac(kVelocityError(), kRotationError()) = -dt_rotmat * hat(acc_unbiased);
  fjac(kVelocityError(), kAccelBiasError()) = -dt_rotmat;

  // Accel bias
  fjac(kAccelBiasError(), kAccelBiasError()).setIdentity();

  // Gyro bias
  fjac(kGyroBiasError(), kGyroBiasError()).setIdentity();

  if (!fjac.allFinite()) {
    logger()->warn("ESKF: Non-finite system jacobian in covariance prediction");
    return setError(AutopilotErrc::kNumericallyNonFinite);
  }

  // Process Noise Q (Discrete approx)
  ProcNoiseCov proc_cov = ProcNoiseCov::Zero();

  proc_cov(kVelocityError(), kVelocityError()) =
      dt * dt_eye * config_->accel_noise_density;
  proc_cov(kRotationError(), kRotationError()) =
      dt * dt_eye * config_->gyro_noise_density;
  proc_cov(kAccelBiasError(), kAccelBiasError()) =
      dt_eye * config_->accel_bias_random_walk;
  proc_cov(kGyroBiasError(), kGyroBiasError()) =
      dt_eye * config_->gyro_bias_random_walk;

  context.P = fjac * context.P * fjac.transpose() + proc_cov;

  if (!context.P.allFinite()) {
    logger()->warn("ESKF: Non-finite error covariance after prediction step");
    return setError(AutopilotErrc::kNumericallyNonFinite);
  }
  return {};
}

// -----------------------------------------------------------------------------
// Math: Corrections
// -----------------------------------------------------------------------------
AutopilotErrc EskfEstimator::correctGps(
    QuadrotorState& state, Context& context,
    const std::shared_ptr<const LocalPositionData>& z) const {
  // 1. Residual
  Eigen::Vector3d y = z->position_enu - state.odometry.pose().translation();

  // 2. Jacobian H
  EnuPositionJacobian hjac = EnuPositionJacobian::Zero();
  hjac(ix::all, kPositionError()).setIdentity();

  // 3. Kalman Update
  // S = HPH' + R
  const Eigen::Matrix3d innov_cov =
      hjac * context.P * hjac.transpose() + z->covariance();

  // Invert S using LLT
  const auto [m2_dist, llt_fac] = computeMahalanobisDistance(y, innov_cov);
  auto reporter =
      std::bind_front(&EskfEstimator::reportObservationStats, this, innov_cov,
                      state.odometry.pose().translation(), z->position_enu);
  if (!checkOutlier(m2_dist, gps_outlier_classifier_, std::move(reporter))) {
    return setError(AutopilotErrc::kNumericalOutlier);
  }

  if (llt_fac.info() != Eigen::Success) {
    logger()->warn(
        "ESKF: Numerical instability in GPS correction (innov_cov inversion)");
    return setError(AutopilotErrc::kNumericallyNonFinite);
  }

  // K = PH' S^-1 is equivalent to K.' = S^-1 HP'
  const EnuPositionKalmanGain kalman_gain =
      llt_fac.solve(hjac * context.P).transpose();

  // Update P = (I - KH)P(I - KH)' + KRK'
  const ErrorCov cov_update = ErrorCov::Identity() - kalman_gain * hjac;
  const ErrorCov cand_posterior =
      cov_update * context.P * cov_update.transpose() +
      kalman_gain * z->covariance() * kalman_gain.transpose();
  if (!cand_posterior.allFinite()) {
    reportPosteriorStats(logger(), cand_posterior);
    return setError(AutopilotErrc::kNumericallyNonFinite);
  }

  if (cand_posterior.diagonal().minCoeff() <= 0) {
    logger()->warn(
        "ESKF: Non-positive definite posterior covariance in GPS correction");
    return setError(AutopilotErrc::kLinalgError);
  }

  context.P = cand_posterior;

  // Update State
  ErrorState dx = kalman_gain * y;
  injectError(state, context, dx);
  resetCovariance(context, dx);

  return {};
}

AutopilotErrc EskfEstimator::correctMag(
    QuadrotorState& state, Context& context,
    const std::shared_ptr<const MagData>& z) const {
  // Predict: b_body = R^T * b_ref
  const Eigen::Vector3d b_pred =
      state.odometry.pose().rotation().inverse() * z->ref_field_enu;

  Eigen::Vector3d y = z->field_body - b_pred;

  // H w.r.t Rot Error = [b_pred]_x
  MagJacobian hjac = MagJacobian::Zero();
  hjac(ix::all, kRotationError()) = hat(b_pred);

  // Standard Update (Same structure as GPS, simplified here)
  const Eigen::Matrix3d innov_cov =
      hjac * context.P * hjac.transpose() + z->covariance();
  const auto [mahalanobis_distance, llt_fac] =
      computeMahalanobisDistance(y, innov_cov);

  auto reporter = std::bind_front(&EskfEstimator::reportObservationStats, this,
                                  innov_cov, b_pred, z->field_body);
  if (!checkOutlier(mahalanobis_distance, mag_outlier_classifier_,
                    std::move(reporter))) {
    return setError(AutopilotErrc::kNumericalOutlier);
  }

  if (llt_fac.info() != Eigen::Success) {
    logger()->warn(
        "ESKF: Numerical instability in Mag correction (innov_cov inversion)");
    return setError(AutopilotErrc::kNumericalInstability);
  }

  const MagKalmanGain kalman_gain = llt_fac.solve(hjac * context.P).transpose();

  const ErrorCov cov_update = ErrorCov::Identity() - kalman_gain * hjac;
  const ErrorCov cand_posterior =
      cov_update * context.P * cov_update.transpose() +
      kalman_gain * z->covariance() * kalman_gain.transpose();
  if (!cand_posterior.allFinite()) {
    reportPosteriorStats(logger(), cand_posterior);
    return setError(AutopilotErrc::kNumericallyNonFinite);
  }

  if (cand_posterior.diagonal().minCoeff() <= 0) {
    logger()->warn(
        "ESKF: Non-positive definite posterior covariance in Mag correction");
    return setError(AutopilotErrc::kLinalgError);
  }
  context.P = cand_posterior;

  const ErrorState dx = kalman_gain * y;
  injectError(state, context, dx);
  resetCovariance(context, dx);

  return {};
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
void EskfEstimator::injectError(QuadrotorState& state, Context& context,
                                const ErrorState& dx) const {
  state.odometry.pose().translation() += dx(kPositionError());

  state.odometry.pose().rotation() =
      (state.odometry.pose().rotation() *
       AngleAxisToQuaternion(dx(kRotationError())))
          .normalized();

  state.odometry.twist().linear() += dx(kVelocityError());
  context.accel_bias += dx(kAccelBiasError());
  context.gyro_bias += dx(kGyroBiasError());
}

void EskfEstimator::resetCovariance(Context& context,
                                    const ErrorState& dx) const {
  // P = G P G' with G = I - 0.5[dtheta]x
  ErrorCov rot_reset = ErrorCov::Identity();
  rot_reset(kRotationError(), kRotationError()) -=
      0.5 * hat(dx(kRotationError()));
  context.P = rot_reset * context.P * rot_reset.transpose();
}

AutopilotErrc EskfEstimator::setError(AutopilotErrc ec) const {
  has_previous_error_ = true;
  return ec;
}

REGISTER_ESTIMATOR(EskfEstimator);

}  // namespace autopilot
