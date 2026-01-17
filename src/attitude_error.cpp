#include "autopilot/controllers/attitude_error.hpp"

#include "autopilot/core/math.hpp"

namespace autopilot {
Eigen::Vector3d EvaluateAttitudeError(const Eigen::Quaterniond& q,
                                      const Eigen::Quaterniond& q_des,
                                      AttitudeErrorLaw error_law) {
  const Eigen::Quaterniond qe = q.inverse() * q_des;
  // Warning! Lee's convention for Geometric tracking Control sets the rate
  // setpoint to the NEGATIVE of the attitude error, while the
  // quaternion-based/tilt prioritizing control methods more often write the
  // rate setpoint equals a certain expression (not clearly labeled as an
  // attitude error). Therefore, the quaternion derived expressions are
  // explicitly negated to match Lee's convention.
  switch (error_law) {
    case AttitudeErrorLaw::kGeometricSO3: {
      // Geometric Tracking Control of a Quadrotor UAV on SE(3), Lee, Leok,
      // and McClamroch
      const Eigen::Matrix3d rot_fb = q.toRotationMatrix();
      const Eigen::Matrix3d rot_des = q_des.toRotationMatrix();

      return 0.5 *
             vee(rot_des.transpose() * rot_fb - rot_fb.transpose() * rot_des);
    }
    case AttitudeErrorLaw::kQuaternionBased:
      // Nonlinear Quadrocopter Attitude Control Technical Report,
      // Brescianini, Hehn, D'Andrea

      // -2.0 is explicitly negated
      return -std::copysign(2.0, qe.w()) * qe.vec();
      break;

    case AttitudeErrorLaw::kTiltPrioritizing: {
      // Tilt-Prioritized Quadrocopter Attitude Control, Brescianini, D'Andrea
      const double qt_w_sq = pown<2>(qe.w()) + pown<2>(qe.z());
      Eigen::Vector3d angle_error;
      if (IsClose(qt_w_sq, 0.0)) {
        // When w_sq = cos(thrust_error_angle / 2) == 0, thrust_error_angle =
        // 180 degrees, aka thrust_vec_body and thrust_vec_target are directly
        // opposite. Just rotating by att_target_to_body corrects the thrust
        angle_error = -QuaternionToAngleAxis(
            Eigen::Quaterniond(0.0, qe.x(), qe.y(), 0.0));
        angle_error.z() = 0.0;
      } else {
        const double qt_w = sqrt(qt_w_sq);
        const double i_qt_w = 1.0 / qt_w;
        const double qe_w_by_qt_w = i_qt_w * qe.w();
        const double qe_z_by_qt_w = i_qt_w * qe.z();

        // We deviate from Brescianini's original law:
        //
        // kp_xy * vec(qe) + kp_z *sgn(qe_0) * vec(qe_yaw)
        //
        // By using the full quaternionic log map to extract the rotation
        // error vector, which is, correctly, the magnitude of the angle error
        // about the axis. Instead of taking the vector part and scaling the
        // error by sine of the angle error
        angle_error = -QuaternionToAngleAxis(Eigen::Quaterniond(
            qt_w, qe_w_by_qt_w * qe.x() - qe_z_by_qt_w * qe.y(),
            qe_w_by_qt_w * qe.y() + qe_z_by_qt_w * qe.x(), 0.0));
        angle_error.z() = -2.0 * (qe.w() < 0.0 ? atan2(-qe.z(), -qe.w())
                                               : atan2(qe.z(), qe.w()));
      }
      return angle_error;
    }
  }
  std::unreachable();
}
}  // namespace autopilot
