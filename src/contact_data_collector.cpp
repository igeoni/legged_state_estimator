#include "legged_state_estimator/contact_data_collector.hpp"

namespace legged_state_estimator {

const std::vector<std::string> ContactDataCollector::kLegJointNames = {
  "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
  "left_knee_joint",       "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint",      "right_ankle_pitch_joint", "right_ankle_roll_joint",
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

ContactDataCollector::ContactDataCollector(const rclcpp::NodeOptions& options)
    : rclcpp::Node("contact_data_collector", options) {

  const std::string urdf_path = declare_parameter<std::string>("urdf_path", "");
  output_dir_          = declare_parameter<std::string>("output_dir",         "./dataset");
  max_samples_         = declare_parameter<int>("max_samples",                10000);
  time_slop_ns_        = declare_parameter<int>("time_slop_ns",               50'000'000);
  contact_threshold_n_ = declare_parameter<double>("contact_threshold_n",     10.0);

  if (urdf_path.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'urdf_path' is required but not set.");
    throw std::runtime_error("urdf_path parameter not set");
  }

  RCLCPP_INFO(get_logger(), "Loading URDF from: %s", urdf_path.c_str());
  kinematics_ = std::make_unique<G1Kinematics>(urdf_path);

  buffer_ = std::make_unique<DataBuffer>(output_dir_);

  imu_sub_ = create_subscription<ImuMsg>("/imu", 100, std::bind(&ContactDataCollector::imuCallback, this, std::placeholders::_1));
  joint_sub_ = create_subscription<JointMsg>("/joint_states", 100, std::bind(&ContactDataCollector::jointCallback, this, std::placeholders::_1));
  left_contact_sub_ = create_subscription<WrenchMsg>("/contact/left_foot", 100, std::bind(&ContactDataCollector::leftContactCallback, this, std::placeholders::_1));
  right_contact_sub_ = create_subscription<WrenchMsg>("/contact/right_foot", 100, std::bind(&ContactDataCollector::rightContactCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "ContactDataCollector started.\n"
              "  output_dir : %s\n"
              "  max_samples: %d\n"
              "  time_slop  : %.0f ms\n"
              "  contact thr: %.1f N",
              output_dir_.c_str(), max_samples_,
              static_cast<double>(time_slop_ns_) / 1e6,
              contact_threshold_n_);
}

// ---------------------------------------------------------------------------
// Subscribers
// ---------------------------------------------------------------------------

void ContactDataCollector::imuCallback(const ImuMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_imu_ = msg;
  tryProcess();
}

void ContactDataCollector::jointCallback(const JointMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_joint_ = msg;
  tryProcess();
}

void ContactDataCollector::leftContactCallback(const WrenchMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_left_contact_ = msg;
  tryProcess();
}

void ContactDataCollector::rightContactCallback(const WrenchMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_right_contact_ = msg;
  tryProcess();
}

// ---------------------------------------------------------------------------
// Synchronization and processing (called with mutex held)
// ---------------------------------------------------------------------------

void ContactDataCollector::tryProcess() {
  if (!collecting_) return;
  if (!latest_imu_ || !latest_joint_ || !latest_left_contact_ || !latest_right_contact_) return;

  // All 4 timestamps must be within time_slop_ns_ of each other.
  int64_t ts[4] = {
    stampNs(latest_imu_->header.stamp),
    stampNs(latest_joint_->header.stamp),
    stampNs(latest_left_contact_->header.stamp),
    stampNs(latest_right_contact_->header.stamp),
  };
  int64_t t_max = *std::max_element(ts, ts + 4);
  int64_t t_min = *std::min_element(ts, ts + 4);
  if (t_max - t_min > time_slop_ns_) return;

  if (!joint_map_ready_) {
    buildJointIndexMap(*latest_joint_);
    if (!joint_map_ready_) return;
  }

  std::vector<double> q_all(12), qd_all(12);
  if (!extractLegJoints(*latest_joint_, q_all, qd_all)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Missing leg joints — skipping");
    return;
  }

  bool left_contact  = latest_left_contact_->wrench.force.z  > contact_threshold_n_;
  bool right_contact = latest_right_contact_->wrench.force.z > contact_threshold_n_;
  int32_t label = encodeContact(left_contact, right_contact);

  Eigen::VectorXd q_left   = Eigen::Map<Eigen::VectorXd>(q_all.data(),      6);
  Eigen::VectorXd q_right  = Eigen::Map<Eigen::VectorXd>(q_all.data() + 6,  6);
  Eigen::VectorXd qd_left  = Eigen::Map<Eigen::VectorXd>(qd_all.data(),     6);
  Eigen::VectorXd qd_right = Eigen::Map<Eigen::VectorXd>(qd_all.data() + 6, 6);

  Eigen::Vector3d p_left_foot  = kinematics_->footPosition(q_left,  true);
  Eigen::Vector3d p_right_foot = kinematics_->footPosition(q_right, false);
  Eigen::Vector3d v_left_foot  = kinematics_->footVelocity(q_left,  qd_left,  true);
  Eigen::Vector3d v_right_foot = kinematics_->footVelocity(q_right, qd_right, false);

  // Feature vector: q[12] qd[12] acc[3] omega[3] p[6] v[6] = 42 floats
  std::vector<float> feat;
  feat.reserve(DataBuffer::kFeatureDim);

  for (double v : q_all)  feat.push_back(static_cast<float>(v));
  for (double v : qd_all) feat.push_back(static_cast<float>(v));

  feat.push_back(static_cast<float>(latest_imu_->linear_acceleration.x));
  feat.push_back(static_cast<float>(latest_imu_->linear_acceleration.y));
  feat.push_back(static_cast<float>(latest_imu_->linear_acceleration.z));
  feat.push_back(static_cast<float>(latest_imu_->angular_velocity.x));
  feat.push_back(static_cast<float>(latest_imu_->angular_velocity.y));
  feat.push_back(static_cast<float>(latest_imu_->angular_velocity.z));

  for (int i = 0; i < 3; ++i) feat.push_back(static_cast<float>(p_left_foot(i)));
  for (int i = 0; i < 3; ++i) feat.push_back(static_cast<float>(p_right_foot(i)));
  for (int i = 0; i < 3; ++i) feat.push_back(static_cast<float>(v_left_foot(i)));
  for (int i = 0; i < 3; ++i) feat.push_back(static_cast<float>(v_right_foot(i)));

  buffer_->addSample(feat, label);

  latest_imu_           = nullptr;
  latest_joint_         = nullptr;
  latest_left_contact_  = nullptr;
  latest_right_contact_ = nullptr;

  const std::size_t n = buffer_->size();
  if (n % 500 == 0) {
    RCLCPP_INFO(get_logger(), "Collected %zu / %d  label=%d (L=%d R=%d)",
                n, max_samples_, label,
                static_cast<int>(left_contact), static_cast<int>(right_contact));
  }

  if (static_cast<int>(n) >= max_samples_) {
    collecting_ = false;
    RCLCPP_INFO(get_logger(), "Reached %d samples, saving to %s ...", max_samples_, output_dir_.c_str());
    buffer_->save();
    RCLCPP_INFO(get_logger(), "Saved train/val/test .npy files. Shutting down.");
    rclcpp::shutdown();
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void ContactDataCollector::buildJointIndexMap(const JointMsg& msg) {
  joint_index_map_.clear();
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    joint_index_map_[msg.name[i]] = i;
  }
  for (const auto& name : kLegJointNames) {
    if (joint_index_map_.find(name) == joint_index_map_.end()) {
      RCLCPP_ERROR(get_logger(),
                   "Joint '%s' not found in /joint_states. Update kLegJointNames.", name.c_str());
      return;
    }
  }
  joint_map_ready_ = true;
  RCLCPP_INFO(get_logger(), "Joint index map built successfully.");
}

bool ContactDataCollector::extractLegJoints(const JointMsg& msg,
                                            std::vector<double>& pos,
                                            std::vector<double>& vel) {
  for (std::size_t i = 0; i < kLegJointNames.size(); ++i) {
    auto it = joint_index_map_.find(kLegJointNames[i]);
    if (it == joint_index_map_.end()) return false;
    std::size_t idx = it->second;
    pos[i] = (idx < msg.position.size()) ? msg.position[idx] : 0.0;
    vel[i] = (idx < msg.velocity.size()) ? msg.velocity[idx] : 0.0;
  }
  return true;
}

int32_t ContactDataCollector::encodeContact(bool left_contact, bool right_contact) const {
  // 0=no contact, 1=right only, 2=left only, 3=both
  return (left_contact ? 2 : 0) + (right_contact ? 1 : 0);
}

}  // namespace legged_state_estimator
