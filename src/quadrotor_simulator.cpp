#include "autopilot/quadrotor_simulator.hpp"

#include <utility>

namespace autopilot {

template <typename T, int N>
struct StateAndDerivative {
  Eigen::Vector<T, N> state;
  Eigen::Vector<T, N> derivative;
};

template <typename Fcn, typename Derived>
  requires(std::invocable<Fcn, const Eigen::MatrixBase<Derived>&> &&
           bool(Eigen::MatrixBase<Derived>::IsVectorAtCompileTime))
StateAndDerivative<typename Derived::Scalar, Derived::SizeAtCompileTime>
RK4Step(Fcn f, const Eigen::MatrixBase<Derived>& x,
        typename Derived::Scalar dt) {
  using Scalar = typename Derived::Scalar;
  using VectorType = Eigen::Vector<Scalar, Derived::SizeAtCompileTime>;
  const VectorType k1 = f(x);
  const VectorType k2 = f(x + k1 * (dt / Scalar(2)));
  const VectorType k3 = f(x + k2 * (dt / Scalar(2)));
  const VectorType k4 = f(x + k3 * dt);
  const VectorType avg_derivative =
      (k1 + Scalar(2) * k2 + Scalar(2) * k3 + k4) / Scalar(6);
  return {.state = x + avg_derivative * dt, .derivative = avg_derivative};
}

QuadrotorSimulator::SimStateVector QuadrotorSimulator::computeSystemDerivative(
    const SimStateVector& x, const Eigen::Vector4d& target_motor_speeds) {
  // Unpack state vector into 13x1 odometry and 4x1 motor speeds
  const OdometryViewF64 odom_state(x.data());
  Eigen::Ref<const Eigen::Vector4d> motor_speeds_curr = x.tail<4>();

  // Resolve motor speeds to motor thrusts
  // F = k_f * w^2
  const double k_f = model()->thrust_curve_coeff();
  const Eigen::Vector4d thrusts_curr = k_f * motor_speeds_curr.cwiseAbs2();

  // Allocation [Collective Thrust, Tx, Ty, Tz]
  const auto& [collective_thrust, torque_body] =
      model()->motorThrustsToThrustTorque(thrusts_curr);

  // Get current orientation from state vector
  const Eigen::Quaterniond& q_odom = odom_state.pose().rotation();

  // Calculate Thrust Force: F_body = [0, 0, T_coll]
  const Eigen::Vector3d thrust_body =
      Eigen::Vector3d::UnitZ() * collective_thrust;
  // Rotate F_body to World Frame: F_world = R * F_body. This is required to
  // match the rigidBodyDynamics formulation (F_World/m - g_vector).
  const Eigen::Vector3d thrust_world = q_odom * thrust_body;

  // Pack Wrench. This internally creates a 6x1 vector, accessible via params()
  const WrenchF64 wrench(thrust_world, torque_body);

  // 3. Rigid Body Dynamics (13-DoF ODE)
  // This uses the function provided by the user
  SimStateVector d;
  d << model()->rigidBodyDynamics(odom_state.params(), wrench.params()),
      model()->motorSpeedDynamics(motor_speeds_curr, target_motor_speeds);

  return d;
}

// ====================================================================
// Simulator Implementation
// ====================================================================

QuadrotorSimulator::QuadrotorSimulator(std::shared_ptr<QuadrotorModel> model,
                                       std::shared_ptr<Config> config,
                                       std::shared_ptr<spdlog::logger> logger)
    : Module("QuadrotorSimulator", std::move(model), std::move(logger)),
      config_(std::move(config)) {
  // Config parameters are now assumed to be in model_->cfg()
  state_.timestamp_secs = 0.0;
  state_.odometry.pose().rotation().setIdentity();
  std::ignore = accel_noise_.configure(
      Eigen::Vector3d::Constant(config_->imu.accel_noise_density),
      Eigen::Vector3d::Constant(config_->imu.accel_random_walk),
      Eigen::Vector3d::Constant(config_->imu.accel_bias_correlation_time));
  std::ignore = gyro_noise_.configure(
      Eigen::Vector3d::Constant(config_->imu.gyro_noise_density),
      Eigen::Vector3d::Constant(config_->imu.gyro_random_walk),
      Eigen::Vector3d::Constant(config_->imu.gyro_bias_correlation_time));
}

void QuadrotorSimulator::setState(const QuadrotorState& state) {
  state_ = state;
  // Initialize motor speeds based on current thrusts
  // assuming w = sqrt(F / kf)
  state_.motor_thrusts = state.motor_thrusts.cwiseMax(0.0);
  // Assuming k_f is available from the model now
  const double k_f = model()->thrust_curve_coeff();
  motor_speeds_ = (state_.motor_thrusts / k_f).cwiseSqrt();
}

void QuadrotorSimulator::step(const QuadrotorCommand& input_cmd, double dt) {
  // 1. Convert Input Thrusts to Target Motor Speeds
  // F = k_f * w^2  => w_ref = sqrt(F_cmd / k_f)
  // Assuming k_f is available from the model now
  const double k_f = model()->thrust_curve_coeff();

  Eigen::Vector4d thrust_cmd = input_cmd.motorThrusts().cwiseMax(0.0);
  Eigen::Vector4d target_motor_speeds = (thrust_cmd / k_f).cwiseSqrt();

  // 2. Pack Odometry State into Flat Vector
  // Order: Pos(3), Quat(4), LinVel(3), AngVel(3) - 13 elements
  // (OdometryF64::ParamVector)
  SimStateVector x;
  x << state_.odometry.params(), motor_speeds_;
  // Apply updates
  const auto& [x_new, dx] = RK4Step(
      [this, &target_motor_speeds](auto&& state_vec) {
        return computeSystemDerivative(
            std::forward<decltype(state_vec)>(state_vec), target_motor_speeds);
      },
      x, dt);

  // 4. Unpack State
  state_.timestamp_secs += dt;
  state_.odometry = OdometryViewF64(x_new.data());
  motor_speeds_ = x_new.tail<4>().cwiseMax(0.0);

  // Enforce constraints on state
  state_.odometry.pose().rotation().normalize();  // Normalize quaternion
  // Update physical outputs (thrusts/wrench) derived from the new motor speeds
  state_.motor_thrusts = k_f * motor_speeds_.array().square().matrix();

  // Recalculate wrench applied to body for logging purposes
  const auto& [collective_thrust, torque] =
      model()->motorThrustsToThrustTorque(state_.motor_thrusts);
  state_.collective_thrust = collective_thrust;

  state_.accel.linear() = OdometryViewF64(dx.data()).twist().linear();
  state_.accel.angular() = OdometryViewF64(dx.data()).twist().angular();

  // Wrench output for logging: Force in World Frame, Torque in Body Frame
  const Eigen::Vector3d f_body_thrust =
      Eigen::Vector3d::UnitZ() * collective_thrust;
  state_.wrench.force() = state_.odometry.pose().rotation() * f_body_thrust;
  state_.wrench.torque() = torque;
}

SensorData QuadrotorSimulator::getSensorMeasurements(double dt) {
  SensorData data;

  // --- IMU ---
  // 1. Ideal Measurements
  // Specific Force: f_b = R.inv() * (a_world - g_world)
  const Eigen::Vector3d ideal_accel =
      state_.odometry.pose().rotation().inverse() *
      (state_.accel.linear() - model()->grav_vector());

  const Eigen::Vector3d& ideal_gyro = state_.odometry.twist().angular();

  // 2. Corrupt Measurements
  data.imu.accel = ideal_accel;
  std::ignore = accel_noise_.corrupt(data.imu.accel, dt);
  data.imu.gyro = ideal_gyro;
  std::ignore = gyro_noise_.corrupt(data.imu.gyro, dt);

  // --- GPS ---
  // Simple white noise
  std::normal_distribution dist_xy(0.0, config_->gps.hor_pos_std_dev);
  std::normal_distribution dist_z(0.0, config_->gps.ver_pos_std_dev);
  std::normal_distribution dist_vel_xy(0.0, config_->gps.hor_vel_std_dev);
  std::normal_distribution dist_vel_z(0.0, config_->gps.ver_vel_std_dev);

  auto generate_position_noise = [&dist_xy, &dist_z, this](auto i) {
    return i < 2 ? dist_xy(gps_rng_) : dist_z(gps_rng_);
  };
  data.gps.position = state_.odometry.pose().translation() +
                      Eigen::Vector3d::NullaryExpr(generate_position_noise);

  auto generate_velocity_noise = [&dist_vel_xy, &dist_vel_z, this](auto i) {
    return i < 2 ? dist_vel_xy(gps_rng_) : dist_vel_z(gps_rng_);
  };
  data.gps.velocity = state_.odometry.twist().linear() +
                      Eigen::Vector3d::NullaryExpr(generate_velocity_noise);
  return data;
}
}  // namespace autopilot
