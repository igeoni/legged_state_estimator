#include "legged_state_estimator/visualizer/fk_visualizer.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace legged_state_estimator {

const std::vector<std::string> FkVisualizer::kLegJointNames = {
  "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
  "left_knee_joint",       "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint",      "right_ankle_pitch_joint", "right_ankle_roll_joint",
};

FkVisualizer::FkVisualizer(const rclcpp::NodeOptions& options): rclcpp::Node("fk_visualizer", options) {

  const std::string urdf_path = declare_parameter<std::string>("urdf_path", "");
  base_frame_ = declare_parameter<std::string>("base_frame", "pelvis");

  if (urdf_path.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'urdf_path' is required but not set.");
    throw std::runtime_error("urdf_path parameter not set");
  }

  RCLCPP_INFO(get_logger(), "Loading URDF from: %s", urdf_path.c_str());
  kinematics_ = std::make_unique<G1Kinematics>(urdf_path);

  // Publish /robot_description with transient_local QoS (latched)
  auto qos_latched = rclcpp::QoS(1).transient_local();
  robot_desc_pub_ = create_publisher<StringMsg>("/robot_description", qos_latched);

  std::ifstream urdf_file(urdf_path);
  if (!urdf_file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Cannot open URDF file for publishing: %s", urdf_path.c_str());
  } 
  else {
    std::ostringstream ss;
    ss << urdf_file.rdbuf();
    StringMsg msg;
    msg.data = ss.str();
    robot_desc_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published /robot_description (%zu bytes)", msg.data.size());
  }

  joint_sub_ = create_subscription<JointMsg>(
      "/joint_states", 50,
      std::bind(&FkVisualizer::jointCallback, this, std::placeholders::_1));

  marker_pub_ = create_publisher<MarkerArray>("/fk_markers", 10);

  RCLCPP_INFO(get_logger(), "FkVisualizer started. base_frame='%s'", base_frame_.c_str());
}

// ---------------------------------------------------------------------------

void FkVisualizer::jointCallback(const JointMsg::ConstSharedPtr& msg) {
  if (!joint_map_ready_) {
    buildJointIndexMap(*msg);
    if (!joint_map_ready_) return;
  }

  std::vector<double> q(12), qd(12);
  if (!extractLegJoints(*msg, q, qd)) return;

  Eigen::VectorXd q_left  = Eigen::Map<Eigen::VectorXd>(q.data(),     6);
  Eigen::VectorXd q_right = Eigen::Map<Eigen::VectorXd>(q.data() + 6, 6);

  Eigen::Vector3d p_lf = kinematics_->footPosition(q_left,  true);
  Eigen::Vector3d p_rf = kinematics_->footPosition(q_right, false);
  Eigen::Vector3d p_lk = kinematics_->kneePosition(q_left,  true);
  Eigen::Vector3d p_rk = kinematics_->kneePosition(q_right, false);

  // Debug: print joint angles and FK positions every ~1s (throttle by count)
  if (++debug_count_ % 50 == 0) {
    RCLCPP_INFO(get_logger(),
      "[joints] lhp=%.3f lhr=%.3f lhy=%.3f lk=%.3f lap=%.3f lar=%.3f",
      q[0], q[1], q[2], q[3], q[4], q[5]);
    RCLCPP_INFO(get_logger(),
      "[FK] left_ankle_roll_link  xyz=(%.4f, %.4f, %.4f)",
      p_lf.x(), p_lf.y(), p_lf.z());
    RCLCPP_INFO(get_logger(),
      "[FK] left_hip_yaw_link(knee) xyz=(%.4f, %.4f, %.4f)",
      p_lk.x(), p_lk.y(), p_lk.z());
  }

  MarkerArray arr;
  arr.markers.push_back(makeSphere(0, p_lf, 0.0f, 1.0f, 0.0f, 0.02f, "left_foot"));   // green
  arr.markers.push_back(makeSphere(1, p_rf, 0.0f, 0.4f, 1.0f, 0.02f, "right_foot"));  // blue
  arr.markers.push_back(makeSphere(2, p_lk, 0.5f, 1.0f, 0.5f, 0.06f, "left_knee"));   // light green
  arr.markers.push_back(makeSphere(3, p_rk, 0.5f, 0.7f, 1.0f, 0.06f, "right_knee"));  // light blue

  for (auto& m : arr.markers) {
    m.header.stamp = msg->header.stamp;
  }

  marker_pub_->publish(arr);
}

// ---------------------------------------------------------------------------

FkVisualizer::Marker FkVisualizer::makeSphere(int id, const Eigen::Vector3d& pos,float r, float g, float b, float radius,const std::string& ns) const {

  Marker m;
  m.header.frame_id    = base_frame_;
  m.ns                 = ns;
  m.id                 = id;
  m.type               = Marker::SPHERE;
  m.action             = Marker::ADD;
  m.pose.position.x    = pos.x();
  m.pose.position.y    = pos.y();
  m.pose.position.z    = pos.z();
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = radius;
  m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.85f;
  m.lifetime = rclcpp::Duration::from_seconds(0.2);
  return m;
}

// ---------------------------------------------------------------------------

void FkVisualizer::buildJointIndexMap(const JointMsg& msg) {
  joint_index_map_.clear();

  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    joint_index_map_[msg.name[i]] = i;
  }

  for (const auto& name : kLegJointNames) {
    if (joint_index_map_.find(name) == joint_index_map_.end()) {
      RCLCPP_ERROR_ONCE(get_logger(),
                        "Joint '%s' not found — check kLegJointNames", name.c_str());
      return;
    }
  }
  joint_map_ready_ = true;
  RCLCPP_INFO(get_logger(), "Joint map ready.");
}

bool FkVisualizer::extractLegJoints(const JointMsg& msg, std::vector<double>& pos, std::vector<double>& vel) {

  for (std::size_t i = 0; i < kLegJointNames.size(); ++i) {

    auto it = joint_index_map_.find(kLegJointNames[i]);
    if (it == joint_index_map_.end()) return false;

    std::size_t idx = it->second;
    pos[i] = (idx < msg.position.size()) ? msg.position[idx] : 0.0;
    vel[i] = (idx < msg.velocity.size()) ? msg.velocity[idx] : 0.0;
  
  }
  return true;
}

}  // namespace legged_state_estimator
