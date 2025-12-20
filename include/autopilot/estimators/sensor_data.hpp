#ifndef AUTOPILOT_SENSOR_DATA_HPP_
#define AUTOPILOT_SENSOR_DATA_HPP_

#include "Eigen/Dense"
#include "autopilot/base/estimator_base.hpp"

namespace autopilot {

// =============================================================================
// Input: IMU Strapdown
// =============================================================================
struct ImuData : public InputBase {
  Eigen::Vector3d accel;  // Body frame (m/s^2)
  Eigen::Vector3d gyro;   // Body frame (rad/s)
  double timestamp;       // Seconds
  std::string source_id;

  ImuData(double t, const Eigen::Vector3d& a, const Eigen::Vector3d& g,
          std::string id = "imu0")
      : accel(a), gyro(g), timestamp(t), source_id(std::move(id)) {}

  // EstimatorData Implementation
  double timestamp_secs() const override { return timestamp; }
  std::string sourceId() const override { return source_id; }
  std::string frameId() const override { return "body"; }
  std::size_t dim() const override { return 6; }  // 3 Acc + 3 Gyro

  std::unique_ptr<EstimatorData> clone() const override {
    return std::make_unique<ImuData>(*this);
  }
};

// =============================================================================
// Measurement: GPS Position
// =============================================================================
struct LocalPositionData : public MeasurementBase {
  Eigen::Vector3d position_enu;  // Local ENU frame (m)
  Eigen::Matrix3d cov_storage;   // Persistant storage for Ref return
  double timestamp;
  std::string source_id;

  LocalPositionData(double t, const Eigen::Vector3d& pos,
                    const Eigen::Matrix3d& cov, std::string id = "gps0")
      : position_enu(pos),
        cov_storage(cov),
        timestamp(t),
        source_id(std::move(id)) {}

  // EstimatorData Implementation
  double timestamp_secs() const override { return timestamp; }
  std::string sourceId() const override { return source_id; }
  std::string frameId() const override { return "world_enu"; }
  std::size_t dim() const override { return 3; }

  std::unique_ptr<EstimatorData> clone() const override {
    return std::make_unique<LocalPositionData>(*this);
  }

  // MeasurementBase Implementation
  Eigen::Ref<const Eigen::MatrixXd> covariance() const override {
    return cov_storage;
  }
};

// =============================================================================
// Measurement: Magnetometer
// =============================================================================
struct MagData : public MeasurementBase {
  Eigen::Vector3d field_body;  // Body frame
  Eigen::Matrix3d cov_storage;
  Eigen::Vector3d ref_field_enu;  // Reference field at this location
  double timestamp;
  std::string source_id;

  MagData(double t, const Eigen::Vector3d& mag, const Eigen::Matrix3d& cov,
          const Eigen::Vector3d& ref, std::string id = "mag0")
      : field_body(mag),
        cov_storage(cov),
        ref_field_enu(ref),
        timestamp(t),
        source_id(std::move(id)) {}

  double timestamp_secs() const override { return timestamp; }
  std::string sourceId() const override { return source_id; }
  std::string frameId() const override { return "body"; }
  std::size_t dim() const override { return 3; }

  std::unique_ptr<EstimatorData> clone() const override {
    return std::make_unique<MagData>(*this);
  }

  Eigen::Ref<const Eigen::MatrixXd> covariance() const override {
    return cov_storage;
  }
};

}  // namespace autopilot

#endif  // AUTOPILOT_SENSOR_DATA_HPP_
