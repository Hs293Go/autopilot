
#include "autopilot/eskf.hpp"

#include <utility>

#include "autopilot/sensor_data.hpp"
// Assuming math headers are merged as discussed
#include "autopilot/geometry.hpp"
#include "autopilot/math.hpp"
#include "boost/math/distributions/inverse_chi_squared.hpp"
#if __has_include(<spdlog/fmt/ranges.h>)
#include "spdlog/fmt/ranges.h"
#else
#include "fmt/ranges.h"
#endif

namespace autopilot {

using ErrorCovUpdate = ErrorCov;

// F = df(x, u, w)/dx
using SystemJacobian = Eigen::Matrix<double, kNumErrorStates, kNumErrorStates>;
using ProcNoiseCov = Eigen::Matrix<double, kNumErrorStates, kNumErrorStates>;

using EnuPositionJacobian = Eigen::Matrix<double, 3, kNumErrorStates>;
using EnuPositionKalmanGain = Eigen::Matrix<double, kNumErrorStates, 3>;

using MagJacobian = Eigen::Matrix<double, 3, kNumErrorStates>;
using MagKalmanGain = Eigen::Matrix<double, kNumErrorStates, 3>;

// Covariance of the attitude reset operation,
// i.e. the G in P(4:7, 4:7) = G * P(4:7, 4:7) * G.'
using RotationErrorResetCov =
    Eigen::Matrix<double, kRotationError.size(), kRotationError.size()>;

static const boost::math::inverse_chi_squared kDist(3);
ErrorStateKalmanFilter::ErrorStateKalmanFilter(
    std::shared_ptr<QuadrotorModel> model, std::shared_ptr<Config> config,
    std::shared_ptr<spdlog::logger> logger)
    : AsyncEstimator("ESKF", std::move(model), std::move(logger)),
      config_(std::move(config)),
      gps_outlier_classifier_(
          boost::math::quantile(kDist, config_->gps_confidence_level_warning),

          boost::math::quantile(kDist, config_->gps_confidence_level_error)),
      mag_outlier_classifier_(
          boost::math::quantile(kDist, config_->mag_confidence_level_warning),
          boost::math::quantile(kDist, config_->mag_confidence_level_error)) {}

void ErrorStateKalmanFilter::reset(const QuadrotorState& initial_state) {
  // Lock handled by AsyncEstimator base if needed, or we assume this
  // is called before start().
  nominal_state_ = initial_state;
  updateCommittedState(initial_state);
  P_.setIdentity();
  accel_bias_.setZero();
  gyro_bias_.setZero();
  last_accel_.setZero();
  last_gyro_.setZero();
}

void reportObservationStats(
    const std::shared_ptr<spdlog::logger>& logger,
    const Eigen::Ref<const Eigen::MatrixXd>& innov_cov,
    const Eigen::Ref<const Eigen::VectorXd>& expected,
    const Eigen::Ref<const Eigen::VectorXd>& observation) {
  const auto max_val = innov_cov.diagonal().maxCoeff();
  const auto min_val = innov_cov.diagonal().minCoeff();
  logger->debug("Innov diag: min={:.3f}, max={:.3f}", min_val, max_val);

  logger->trace("Predicted observation: {}", expected);
  logger->trace("Actual observation: {}", observation);
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
std::error_code ErrorStateKalmanFilter::processInput(
    const std::shared_ptr<const InputBase>& u) {
  // Dispatch
  if (const auto imu = std::dynamic_pointer_cast<const ImuData>(u)) {
    // 1. Compute dt
    // In a robust system, we track last_imu_time. For now, trust the sequence.
    // If this is the first packet, we can't integrate.
    double dt = imu->timestamp - nominal_state_.timestamp_secs;
    if (dt <= 0) {
      return make_error_code(AutopilotErrc::kTimestampOutOfOrder);
    }  // Out of order or duplicate

    // 2. Predict Nominal State (Kinematics)
    QuadrotorState next_state = nominal_state_;
    if (auto ec = predictKinematics(next_state, imu->accel, imu->gyro, dt)) {
      return ec;
    }

    // 3. Predict Error Covariance
    if (auto ec = predictCovariance(imu->accel, imu->gyro, dt); ec) {
      return ec;
    }

    // 4. Commit
    updateCommittedState(next_state);

    // Cache for extrapolation
    last_accel_ = imu->accel;
    last_gyro_ = imu->gyro;
    return {};
  }

  logger()->warn("ESKF: Unknown input type");
  return make_error_code(AutopilotErrc::kUnknownSensorType);
}

// -----------------------------------------------------------------------------
// WORKER THREAD: Measurement Handling
// -----------------------------------------------------------------------------
std::error_code ErrorStateKalmanFilter::processMeasurement(
    const std::shared_ptr<const MeasurementBase>& z) {
  std::error_code ec;

  if (const auto gps = std::dynamic_pointer_cast<const GpsData>(z)) {
    ec = correctGps(gps);
  } else if (const auto mag = std::dynamic_pointer_cast<const MagData>(z)) {
    ec = correctMag(mag);
  } else {
    logger()->warn("ESKF: Unknown measurement type");
    return make_error_code(AutopilotErrc::kUnknownSensorType);
  }

  if (ec) {
    logger()->warn("ESKF Update Failed: {}", ec.message());
    // Robustness: Maybe inflate P if update fails?
  }
  return ec;
}

// -----------------------------------------------------------------------------
// CONTROL THREAD: Extrapolation
// -----------------------------------------------------------------------------
QuadrotorState ErrorStateKalmanFilter::extrapolateState(
    const QuadrotorState& state, double t) const {
  QuadrotorState pred_state = state;
  double dt = t - state.timestamp_secs;

  // Use last known inputs to predict forward.
  // Biases are applied inside predictKinematics using internal members.
  if (dt > 0) {
    if (auto ec = predictKinematics(pred_state, last_accel_, last_gyro_, dt)) {
      logger()->warn("ESKF Extrapolation Failed at t={:.3} (dt={:.3}): {}", t,
                     dt, ec.message());
      return state;  // Return unmodified state on failure
    }
  }
  return pred_state;
}

template <int N>
ErrorStateKalmanFilter::InnovStats<N>
ErrorStateKalmanFilter::computeMahalanobisDistance(
    const Eigen::Vector<double, N>& innovation,
    const Eigen::Matrix<double, N, N>& innov_cov) {
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

// -----------------------------------------------------------------------------
// Math: Kinematics (Shared by Predict and Extrapolate)
// -----------------------------------------------------------------------------
std::error_code ErrorStateKalmanFilter::predictKinematics(
    QuadrotorState& state, const Eigen::Vector3d& accel,
    const Eigen::Vector3d& gyro, double dt) const {
  // 1. Bias Correction
  Eigen::Vector3d acc_unbiased = accel - accel_bias_;
  Eigen::Vector3d gyr_unbiased = gyro - gyro_bias_;

  // 2. Integration
  OdometryF64 cand = state.odometry;
  const auto& p = state.odometry.pose().translation();
  const auto& q = state.odometry.pose().rotation();
  const auto& v = state.odometry.twist().linear();

  // a_world = R * a_body + g
  // Assumption: model()->grav_vector() is [0, 0, -9.81]
  const Eigen::Vector3d acc_world = q * acc_unbiased + model()->grav_vector();

  cand.pose().translation() = p + v * dt + 0.5 * acc_world * dt * dt;
  cand.pose().rotation() = q * AngleAxisToQuaternion(gyr_unbiased * dt);
  cand.twist().linear() = v + acc_world * dt;

  if (!p.allFinite() || !q.coeffs().allFinite() || !v.allFinite()) {
    logger()->warn("ESKF: Non-finite state in kinematics prediction");
    logger()->trace("Position: {}, Orientation: {}, Velocity: {}", p,
                    q.coeffs(), v);
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  state.odometry = cand;
  state.timestamp_secs += dt;
  return {};
}

// -----------------------------------------------------------------------------
// Math: Covariance Prediction
// -----------------------------------------------------------------------------
std::error_code ErrorStateKalmanFilter::predictCovariance(
    const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt) {
  // Jacobian F

  const auto& q = nominal_state_.odometry.pose().rotation();
  const Eigen::Matrix3d rotmat = q.toRotationMatrix();
  const Eigen::Vector3d acc_unbiased = accel - accel_bias_;
  const Eigen::Vector3d gyr_unbiased = gyro - gyro_bias_;
  const Eigen::Vector3d delta_angle = gyr_unbiased * dt;

  // Pos -> Vel

  SystemJacobian fjac = SystemJacobian::Zero();
  const Eigen::Matrix3d dt_eye = Eigen::Matrix3d::Identity() * dt;

  // Position
  fjac(kPositionError(), kPositionError()).setIdentity();
  fjac(kPositionError(), kVelocityError()) = dt_eye;

  // Rotation
  fjac(kRotationError(), kRotationError()) =
      AngleAxisToRotationMatrix(-delta_angle);
  fjac(kRotationError(), kGyroBiasError()) = -dt_eye;

  // Velocity
  fjac(kVelocityError(), kVelocityError()).setIdentity();
  fjac(kVelocityError(), kRotationError()) = -rotmat * hat(acc_unbiased) * dt;
  fjac(kVelocityError(), kAccelBiasError()) = -rotmat * dt;

  // Accel bias
  fjac(kAccelBiasError(), kAccelBiasError()).setIdentity();

  // Gyro bias
  fjac(kGyroBiasError(), kGyroBiasError()).setIdentity();

  if (!fjac.allFinite()) {
    logger()->warn("ESKF: Non-finite system jacobian in covariance prediction");
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
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

  P_ = fjac * P_ * fjac.transpose() + proc_cov;

  if (!P_.allFinite()) {
    logger()->warn("ESKF: Non-finite error covariance after prediction step");
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }
  return {};
}

// -----------------------------------------------------------------------------
// Math: Corrections
// -----------------------------------------------------------------------------
std::error_code ErrorStateKalmanFilter::correctGps(
    const std::shared_ptr<const GpsData>& z) {
  // 1. Residual
  Eigen::Vector3d y =
      z->position_enu - nominal_state_.odometry.pose().translation();

  // 2. Jacobian H
  EnuPositionJacobian hjac = EnuPositionJacobian::Zero();
  hjac(ix::all, kPositionError()).setIdentity();

  // 3. Kalman Update
  // S = HPH' + R
  const Eigen::Matrix3d innov_cov =
      hjac * P_ * hjac.transpose() + z->covariance();

  // Invert S using LLT
  const auto [m2_dist, llt_fac] = computeMahalanobisDistance(y, innov_cov);
  auto report_details = std::bind_front(
      reportObservationStats, logger(), innov_cov,
      nominal_state_.odometry.pose().translation(), z->position_enu);
  switch (gps_outlier_classifier_.classify(m2_dist)) {
    case OutlierClassification::kNormal:
      break;
    case OutlierClassification::kWarning:
      report_details();
      logger()->warn(
          "ESKF GPS Measurement Warning:"
          "Mahalanobis distance = {:.3f} vs Chi-squared distance = {:.3f}",
          m2_dist, gps_outlier_classifier_.warning_threshold());
      break;
    case OutlierClassification::kError:
      report_details();
      logger()->error(
          "ESKF GPS Measurement Rejected:"
          "Mahalanobis distance = {:.3f} vs Chi-squared distance = {:.3f}",
          m2_dist, gps_outlier_classifier_.error_threshold());
      return make_error_code(AutopilotErrc::kNumericalOutlier);
  }

  if (llt_fac.info() != Eigen::Success) {
    logger()->warn(
        "ESKF: Numerical instability in GPS correction (innov_cov inversion)");
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  // K = PH' S^-1 is equivalent to K.' = S^-1 HP'
  const EnuPositionKalmanGain kalman_gain =
      llt_fac.solve(hjac * P_).transpose();

  // Update P = (I - KH)P(I - KH)' + KRK'
  const ErrorCov cov_update = ErrorCov::Identity() - kalman_gain * hjac;
  const ErrorCov cand_posterior =
      cov_update * P_ * cov_update.transpose() +
      kalman_gain * z->covariance() * kalman_gain.transpose();
  if (!cand_posterior.allFinite()) {
    reportPosteriorStats(logger(), cand_posterior);
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (cand_posterior.diagonal().minCoeff() <= 0) {
    logger()->warn(
        "ESKF: Non-positive definite posterior covariance in GPS correction");
    return make_error_code(AutopilotErrc::kLinalgError);
  }

  P_ = cand_posterior;

  // Update State
  ErrorState dx = kalman_gain * y;
  injectError(dx);
  resetError(dx);

  return {};
}

std::error_code ErrorStateKalmanFilter::correctMag(
    const std::shared_ptr<const MagData>& z) {
  // Predict: b_body = R^T * b_ref
  const Eigen::Vector3d b_pred =
      nominal_state_.odometry.pose().rotation().inverse() * z->ref_field_enu;

  Eigen::Vector3d y = z->field_body - b_pred;

  // H w.r.t Rot Error = [b_pred]_x
  MagJacobian hjac = MagJacobian::Zero();
  hjac(ix::all, kRotationError()) = hat(b_pred);

  // Standard Update (Same structure as GPS, simplified here)
  const Eigen::Matrix3d innov_cov =
      hjac * P_ * hjac.transpose() + z->covariance();
  const auto [mahalanobis_distance, llt_fac] =
      computeMahalanobisDistance(y, innov_cov);
  auto report_details = std::bind_front(reportObservationStats, logger(),
                                        innov_cov, b_pred, z->field_body);
  switch (mag_outlier_classifier_.classify(mahalanobis_distance)) {
    case OutlierClassification::kNormal:
      break;
    case OutlierClassification::kWarning:
      report_details();
      logger()->warn(
          "ESKF Mag Measurement Warning:"
          "Mahalanobis distance = {:.3f} vs Chi-squared distance = {:.3f}",
          mahalanobis_distance, mag_outlier_classifier_.warning_threshold());
      break;
    case OutlierClassification::kError:
      report_details();
      logger()->error(
          "ESKF Mag Measurement Rejected:"
          "Mahalanobis distance = {:.3f} vs Chi-squared distance = {:.3f}",
          mahalanobis_distance, mag_outlier_classifier_.error_threshold());
      return make_error_code(AutopilotErrc::kNumericalOutlier);
  }
  if (llt_fac.info() != Eigen::Success) {
    logger()->warn(
        "ESKF: Numerical instability in Mag correction (innov_cov inversion)");
    return make_error_code(AutopilotErrc::kNumericalInstability);
  }

  const MagKalmanGain kalman_gain = llt_fac.solve(hjac * P_).transpose();

  const ErrorCov cov_update = ErrorCov::Identity() - kalman_gain * hjac;
  const ErrorCov cand_posterior =
      cov_update * P_ * cov_update.transpose() +
      kalman_gain * z->covariance() * kalman_gain.transpose();
  if (!cand_posterior.allFinite()) {
    reportPosteriorStats(logger(), cand_posterior);
    return make_error_code(AutopilotErrc::kNumericallyNonFinite);
  }

  if (cand_posterior.diagonal().minCoeff() <= 0) {
    logger()->warn(
        "ESKF: Non-positive definite posterior covariance in Mag correction");
    return make_error_code(AutopilotErrc::kLinalgError);
  }
  P_ = cand_posterior;

  const ErrorState dx = kalman_gain * y;
  injectError(dx);
  resetError(dx);

  return {};
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
void ErrorStateKalmanFilter::injectError(const ErrorState& dx) {
  auto working_state = nominal_state_;

  working_state.odometry.pose().translation() += dx(kPositionError());

  Eigen::Vector3d dtheta = dx(kRotationError());
  Eigen::Quaterniond dq(1, 0.5 * dtheta.x(), 0.5 * dtheta.y(),
                        0.5 * dtheta.z());
  working_state.odometry.pose().rotation() =
      (working_state.odometry.pose().rotation() * dq).normalized();

  working_state.odometry.twist().linear() += dx(kVelocityError());
  accel_bias_ += dx(kAccelBiasError());
  gyro_bias_ += dx(kGyroBiasError());
  updateCommittedState(working_state);
}

void ErrorStateKalmanFilter::resetError(const ErrorState& dx) {
  // P = G P G' with G = I - 0.5[dtheta]x
  ErrorCovUpdate rot_reset = ErrorCovUpdate::Identity();
  rot_reset(kRotationError(), kRotationError()) -=
      0.5 * hat(dx(kRotationError()));
  P_ = rot_reset * P_ * rot_reset.transpose();
}

}  // namespace autopilot
