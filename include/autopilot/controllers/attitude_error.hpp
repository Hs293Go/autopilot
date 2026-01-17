#ifndef AUTOPILOT_CONTROLLER_ATTITUDE_ERROR_LAW_HPP_
#define AUTOPILOT_CONTROLLER_ATTITUDE_ERROR_LAW_HPP_

#include "Eigen/Dense"

namespace autopilot {
enum class AttitudeErrorLaw {
  kGeometricSO3,
  kQuaternionBased,
  kTiltPrioritizing
};

Eigen::Vector3d EvaluateAttitudeError(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& q_des,
    AttitudeErrorLaw error_law = AttitudeErrorLaw::kGeometricSO3);
}  // namespace autopilot

#endif  // AUTOPILOT_CONTROLLER_ATTITUDE_ERROR_LAW_HPP_
