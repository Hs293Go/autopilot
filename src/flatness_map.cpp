#include "autopilot/planning/flatness_map.hpp"

#include "autopilot/core/math.hpp"

#define TRY(expr)                                                    \
  do {                                                               \
    if (AutopilotErrc _tmp = (expr); _tmp != AutopilotErrc::kNone) { \
      return std::unexpected(_tmp);                                  \
    }                                                                \
  } while (0)

namespace autopilot {

std::expected<QuadrotorCommand, AutopilotErrc> FlatnessMap::compute(
    const KinematicState& kinematics, double yaw, double yaw_rate,
    double yaw_accel) const {
  const auto& [t, pos, vel, acc, jerk, snap] = kinematics;
  QuadrotorCommand cmd(t);

  const double m = model_->mass();

  // Specific acceleration
  const Eigen::Vector3d sa = acc - model_->grav_vector();
  const double n_sq = sa.squaredNorm();

  if (n_sq < 1e-6) {
    return std::unexpected(AutopilotErrc::kSingularConfiguration);
  }
  TRY(cmd.setPosition(pos));
  TRY(cmd.setVelocity(vel));
  TRY(cmd.setAcceleration(acc));

  const double n = std::sqrt(n_sq);
  const Eigen::Vector3d z = sa / n;
  const Eigen::Vector3d force = m * sa;
  TRY(cmd.setForce(force));

  const double thrust = z.dot(force);
  TRY(cmd.setCollectiveThrust(thrust));

  const double tilt_den = std::sqrt(2 * (1 + z.z()));

  // Pure tilt quaternion
  const Eigen::Quaterniond qz(0.5 * tilt_den, -z.y() / tilt_den,
                              z.x() / tilt_den, 0.0);

  const double c_half_psi = std::cos(yaw / 2.0);
  const double s_half_psi = std::sin(yaw / 2.0);
  const Eigen::Quaterniond q_yaw(
      qz.w() * c_half_psi, qz.x() * c_half_psi + qz.y() * s_half_psi,
      qz.y() * c_half_psi - qz.x() * s_half_psi, qz.w() * s_half_psi);

  TRY(cmd.setOrientation(q_yaw));

  const double d_acc_norm = z.dot(jerk);
  const Eigen::Vector3d d_z = (jerk - d_acc_norm * z) / n;
  const double c_psi = 2.0 * pown<2>(c_half_psi) - 1.0;
  const double s_psi = 2.0 * c_half_psi * s_half_psi;
  const double omega_den = z.z() + 1.0;
  const double d_z_den = d_z.z() / omega_den;
  const Eigen::Vector3d body_rate(
      (d_z.x() * s_psi - d_z.y() * c_psi -
       (z.x() * s_psi - z.y() * c_psi) * d_z_den),
      (d_z.x() * c_psi + d_z.y() * s_psi -
       (z.x() * c_psi + z.y() * s_psi) * d_z_den),
      (z.y() * d_z.x() - z.x() * d_z.y()) / omega_den + yaw_rate);

  TRY(cmd.setBodyRate(body_rate));

  const double d2_accel_nrm = n * d_z.squaredNorm() + z.dot(snap);

  const Eigen::Vector3d d2_z =
      (snap - d2_accel_nrm * z - 2.0 * d_acc_norm * d_z) / n;

  const double d2_z_den = d2_z.z() / omega_den;
  const double h_x = (d2_z.x() * c_psi + d2_z.y() * s_psi -
                      (z.x() * c_psi + z.y() * s_psi) * d2_z_den);
  const double h_y = (d2_z.x() * s_psi - d2_z.y() * c_psi -
                      (z.x() * s_psi - z.y() * c_psi) * d2_z_den);
  const Eigen::Vector3d angular_accel(
      -h_y + body_rate.y() * body_rate.z(), h_x - body_rate.x() * body_rate.z(),
      (z.y() * d2_z.x() - z.x() * d2_z.y()) / omega_den -
          (body_rate.z() - yaw_rate) * d_z_den + yaw_accel);
  TRY(cmd.setAngularAcceleration(angular_accel));

  TRY(cmd.setTorque(model_->inertia() * angular_accel +
                    body_rate.cross(model_->inertia() * body_rate)));
  return cmd;
}

}  // namespace autopilot
