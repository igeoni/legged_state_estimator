#include "legged_state_estimator/g1_kinematics.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace legged_state_estimator {

namespace {

const std::array<const char*, 6> kLeftJointNames = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint",      "left_ankle_pitch_joint", "left_ankle_roll_joint",
};

const std::array<const char*, 6> kRightJointNames = {
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint",      "right_ankle_pitch_joint", "right_ankle_roll_joint",
};

}  // namespace

G1Kinematics::G1Kinematics(const std::string& urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);

  // Isaac Sim (PhysX) publishes TF at body COM frames, not URDF link origins.
  // model_.inertias[0] holds the root body (pelvis) inertia; lever() is its COM
  // in the pelvis link frame. Subtracting this from FK positions converts from
  // pelvis-link-origin frame to pelvis-COM frame, matching Isaac Sim's TF.
  root_com_offset_ = model_.inertias[0].lever();

  // Map joint names to q/v indices in the Pinocchio configuration vector
  for (int i = 0; i < 6; ++i) {
    const auto lid = model_.getJointId(kLeftJointNames[i]);
    const auto rid = model_.getJointId(kRightJointNames[i]);
    if (lid >= (std::size_t)model_.njoints)
      throw std::runtime_error(std::string("Joint not found in URDF: ") + kLeftJointNames[i]);
    if (rid >= (std::size_t)model_.njoints)
      throw std::runtime_error(std::string("Joint not found in URDF: ") + kRightJointNames[i]);
    left_q_idx_[i]  = model_.joints[lid].idx_q();
    left_v_idx_[i]  = model_.joints[lid].idx_v();
    right_q_idx_[i] = model_.joints[rid].idx_q();
    right_v_idx_[i] = model_.joints[rid].idx_v();
  }

  // ankle_roll_link = foot end-effector
  left_foot_id_  = model_.getFrameId("left_ankle_roll_link");
  right_foot_id_ = model_.getFrameId("right_ankle_roll_link");

  // hip_yaw_link = visual knee landmark (origin after hip_pitch+roll+yaw)
  left_knee_id_  = model_.getFrameId("left_knee_joint");
  right_knee_id_ = model_.getFrameId("right_knee_joint");

  if (left_foot_id_  >= (std::size_t)model_.nframes ||
      right_foot_id_ >= (std::size_t)model_.nframes ||
      left_knee_id_  >= (std::size_t)model_.nframes ||
      right_knee_id_ >= (std::size_t)model_.nframes) {
    throw std::runtime_error("One or more expected link frames not found in URDF model.");
  }
}

Eigen::VectorXd G1Kinematics::buildQ(const Eigen::Ref<const Eigen::VectorXd>& q_leg, bool left) const {
  Eigen::VectorXd q = pinocchio::neutral(model_);
  const auto& idx = left ? left_q_idx_ : right_q_idx_;
  for (int i = 0; i < 6; ++i) q[idx[i]] = q_leg[i];
  return q;
}

Eigen::VectorXd G1Kinematics::buildV(const Eigen::Ref<const Eigen::VectorXd>& qd_leg, bool left) const {
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  const auto& idx = left ? left_v_idx_ : right_v_idx_;
  for (int i = 0; i < 6; ++i) v[idx[i]] = qd_leg[i];
  return v;
}

Eigen::Vector3d G1Kinematics::framePosition(std::size_t frame_id, const Eigen::VectorXd& q) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[frame_id].translation();
}

Eigen::Vector3d G1Kinematics::frameVelocity(std::size_t frame_id,const Eigen::VectorXd& q,const Eigen::VectorXd& v) const {
  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
  return pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
}

Eigen::Vector3d G1Kinematics::footPosition(const Eigen::Ref<const Eigen::VectorXd>& q_leg, bool left) const {
  return framePosition(left ? left_foot_id_ : right_foot_id_, buildQ(q_leg, left));
}

Eigen::Vector3d G1Kinematics::kneePosition(const Eigen::Ref<const Eigen::VectorXd>& q_leg, bool left) const {
  return framePosition(left ? left_knee_id_ : right_knee_id_, buildQ(q_leg, left));
}

Eigen::Vector3d G1Kinematics::footVelocity(const Eigen::Ref<const Eigen::VectorXd>& q_leg,const Eigen::Ref<const Eigen::VectorXd>& qd_leg, bool left) const {
  return frameVelocity(left ? left_foot_id_ : right_foot_id_,buildQ(q_leg, left), buildV(qd_leg, left));
}

Eigen::Vector3d G1Kinematics::kneeVelocity(const Eigen::Ref<const Eigen::VectorXd>& q_leg,const Eigen::Ref<const Eigen::VectorXd>& qd_leg, bool left) const {
  return frameVelocity(left ? left_knee_id_ : right_knee_id_,buildQ(q_leg, left), buildV(qd_leg, left));
}

}  // namespace legged_state_estimator
