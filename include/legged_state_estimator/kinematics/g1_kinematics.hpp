#pragma once

#include <array>
#include <string>

#include <Eigen/Dense>

// pinocchio must be included before Eigen-using headers to avoid macro conflicts
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace legged_state_estimator {

/**
 * FK for Unitree G1 (29-DOF) legs via Pinocchio, loaded from URDF at runtime.
 *
 * All positions are expressed in the pelvis (root) frame.
 *
 * Kinematic chain order expected in q_leg[0..5]:
 *   0: hip_pitch   1: hip_roll   2: hip_yaw
 *   3: knee        4: ankle_pitch  5: ankle_roll
 */
class G1Kinematics {
 public:
  explicit G1Kinematics(const std::string& urdf_path);

  // ankle_roll_link origin in pelvis frame
  Eigen::Vector3d footPosition(const Eigen::Ref<const Eigen::VectorXd>& q_leg, bool left) const;

  // hip_yaw_link origin in pelvis frame (visual knee landmark)
  Eigen::Vector3d kneePosition(const Eigen::Ref<const Eigen::VectorXd>& q_leg, bool left) const;

  Eigen::Vector3d footVelocity(const Eigen::Ref<const Eigen::VectorXd>& q_leg, const Eigen::Ref<const Eigen::VectorXd>& qd_leg, bool left) const;

  Eigen::Vector3d kneeVelocity(const Eigen::Ref<const Eigen::VectorXd>& q_leg, const Eigen::Ref<const Eigen::VectorXd>& qd_leg, bool left) const;

 private:
  pinocchio::Model        model_;
  mutable pinocchio::Data data_;

  // Index into Pinocchio q/v vectors for each of the 6 leg joints
  std::array<int, 6> left_q_idx_,  right_q_idx_;
  std::array<int, 6> left_v_idx_,  right_v_idx_;

  // Pinocchio frame IDs
  std::size_t left_foot_id_,  right_foot_id_;
  std::size_t left_knee_id_,  right_knee_id_;

  // Build full-model configuration/velocity vectors with leg joints set
  Eigen::VectorXd buildQ(const Eigen::Ref<const Eigen::VectorXd>& q_leg, bool left) const;
  Eigen::VectorXd buildV(const Eigen::Ref<const Eigen::VectorXd>& qd_leg, bool left) const;

  Eigen::Vector3d framePosition(std::size_t frame_id, const Eigen::VectorXd& q) const;
  Eigen::Vector3d frameVelocity(std::size_t frame_id, const Eigen::VectorXd& q, const Eigen::VectorXd& v) const;
};

}  // namespace legged_state_estimator
