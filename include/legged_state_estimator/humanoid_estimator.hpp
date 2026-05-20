#pragma once

#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace legged_state_estimator {

struct HumanoidEstimatorParams {
  // IMU-to-body transform (body = pelvis COM frame)
  gtsam::Pose3 body_P_imu = gtsam::Pose3();

  // IMU noise parameters
  double sigma_gyro = 8e-4;
  double sigma_acc = 2e-2;
  double sigma_integration = 1e-3;
  double bias_acc_rw = 5e-3;
  double bias_omega_rw = 1e-4;

  // Contact noise
  double contact_sigma_xy = 0.03;
  double contact_sigma_z = 0.02;

  // Initial state
  double initial_height = 0.787;  // G1 standing height (pelvis COM above ground)

  // Fixed-lag smoother lag window
  double lag_seconds = 1.0;

  // Gravity vector (world frame, NED convention: z points up)
  gtsam::Vector3 gravity = gtsam::Vector3(0.0, 0.0, -9.81);

  // Contact force threshold to consider foot in contact [N]
  double contact_force_threshold = 10.0;

  // Foot names (must match order used in ContactMeasurement::foot index)
  std::vector<std::string> foot_names = {"left", "right"};

  // Estimator variant: "invariant_ekf", "invariant_graph",
  //                    "fixed_lag_single_bias", "fixed_lag_combined_bias"
  std::string estimator_type = "invariant_ekf";
};

/**
 * Wraps gtsam::LeggedEstimator for real-time use.
 *
 * Call sequence each control cycle:
 *   1. predict(omega, accel, dt)         — after each IMU sample
 *   2. processContacts(measurements)     — when contact state changes or
 *                                          periodically (touchdowns + updates)
 *   3. navState() / position() / ...     — read current estimate
 */
class HumanoidEstimator {
 public:
  explicit HumanoidEstimator(const HumanoidEstimatorParams& params);

  // Initialize with known initial IMU bias and initial attitude
  void initialize(const gtsam::imuBias::ConstantBias& bias,
                  const gtsam::Rot3& initial_attitude = gtsam::Rot3());

  // Propagate with one IMU measurement [rad/s, m/s^2]
  void predict(const gtsam::Vector3& omega, const gtsam::Vector3& accel,
               double dt);

  // Process currently active foot contacts; measurements in body (pelvis) frame
  void processContacts(
      const std::vector<gtsam::ContactMeasurement>& active_contacts);

  // Query current estimate
  gtsam::NavState navState() const;
  gtsam::imuBias::ConstantBias bias() const;
  gtsam::Rot3 orientation() const;
  gtsam::Point3 position() const;
  gtsam::Vector3 velocity() const;

  bool isInitialized() const { return estimator_ != nullptr; }

 private:
  HumanoidEstimatorParams params_;
  std::unique_ptr<gtsam::LeggedEstimator> estimator_;

  gtsam::LeggedEstimatorParams makeGtsamParams(
      const gtsam::imuBias::ConstantBias& bias) const;
};

}  // namespace legged_state_estimator
