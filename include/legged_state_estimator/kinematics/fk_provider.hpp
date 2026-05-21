#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace legged_state_estimator {

/**
 * Abstract forward-kinematics provider.
 *
 * This header intentionally has NO pinocchio or GTSAM includes so that
 * translation units that use only this interface stay free of the
 * Boost-serialization Eigen conflicts that arise when pinocchio and GTSAM
 * are included in the same TU.
 */
class FkProvider {
 public:
  virtual ~FkProvider() = default;

  // Foot contact position in pelvis (body) frame.
  // q_leg: 6-DOF joint positions [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]
  virtual Eigen::Vector3d footPosition(const Eigen::VectorXd& q_leg,
                                       bool left) const = 0;
};

// Factory declared here, implemented in g1_fk_provider.cpp (pinocchio TU).
std::unique_ptr<FkProvider> makeG1FkProvider(const std::string& urdf_path);

}  // namespace legged_state_estimator
