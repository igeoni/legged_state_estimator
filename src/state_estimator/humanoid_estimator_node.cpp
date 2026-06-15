/*
 * Copyright (c) 2026, Geoni Lee, LAIR Lab, Sungkyunkwan University (SKKU)
 * All Rights Reserved
 *
 * This file is licensed under the BSD-3-Clause License.
 * This work builds upon GTSAM (Georgia Tech Smoothing and Mapping library),
 * developed at Georgia Tech Research Corporation. GTSAM is distributed under
 * its own BSD license; see https://github.com/borglab/gtsam for details.
 *
 * See LICENSE for the license information applicable to this file.
 */

#include "legged_state_estimator/state_estimator/humanoid_estimator_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace legged_state_estimator {

static constexpr double kInf = std::numeric_limits<double>::infinity();

HumanoidEstimatorNode::HumanoidEstimatorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("humanoid_state_estimator", options), estimator_(est_params_) {

  const std::string urdf_path   = declare_parameter<std::string>("urdf_path", "");
  est_params_.estimator_type    = declare_parameter<std::string>("estimator_type", "fixed_lag_single_bias");
  contact_force_threshold_      = declare_parameter<double>("contact_force_threshold", 10.0);
  est_params_.initial_height    = declare_parameter<double>("initial_height", 0.787);
  est_params_.sigma_gyro        = declare_parameter<double>("sigma_gyro", 8e-4);
  est_params_.sigma_acc         = declare_parameter<double>("sigma_acc", 5e-2);
  est_params_.sigma_integration = declare_parameter<double>("sigma_integration", 1e-3);
  est_params_.bias_acc_rw       = declare_parameter<double>("bias_acc_rw", 2e-2);
  est_params_.lag_seconds       = declare_parameter<double>("lag_seconds", 1.0);
  est_params_.contact_sigma_xy  = declare_parameter<double>("contact_sigma_xy", 0.005);
  est_params_.contact_sigma_z   = declare_parameter<double>("contact_sigma_z",  0.005);
  max_dead_reckoning_s_         = declare_parameter<double>("max_dead_reckoning_s", 0.02);
  reorderDelaySeconds_          = declare_parameter<double>("qos.reorder_delay_seconds", 0.02);
  disable_contact_              = declare_parameter<bool>("disable_contact", false);
  zero_accel_debug_             = declare_parameter<bool>("zero_accel_debug", false);
  zero_gyro_debug_              = declare_parameter<bool>("zero_gyro_debug", false);
  world_frame_                  = declare_parameter<std::string>("world_frame", "odom");
  base_frame_                   = declare_parameter<std::string>("base_frame",  "pelvis");

  const std::string topic_imu           = declare_parameter<std::string>("topic.imu",                "/imu");
  const std::string topic_joint         = declare_parameter<std::string>("topic.joint_states",       "/joint_states");
  const std::string topic_contact_left  = declare_parameter<std::string>("topic.contact_left",       "/contact/left_foot");
  const std::string topic_contact_right = declare_parameter<std::string>("topic.contact_right",      "/contact/right_foot");
  const std::string topic_odom          = declare_parameter<std::string>("topic.odom",               "/state_estimator/odom");
  const std::string topic_description   = declare_parameter<std::string>("topic.robot_description",  "/description");
  const std::string topic_path          = declare_parameter<std::string>("topic.path",               "/state_estimator/path");
  const std::string topic_foot_contacts = declare_parameter<std::string>("topic.foot_contacts",      "/state_estimator/foot_contacts");

  if (urdf_path.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'urdf_path' is required.");
    throw std::runtime_error("urdf_path not set");
  }

  estimator_   = HumanoidEstimator(est_params_);
  fk_provider_ = makeG1FkProvider(urdf_path);

  left_joint_names_  = {"left_hip_pitch_joint",  "left_hip_roll_joint",
                        "left_hip_yaw_joint",     "left_knee_joint",
                        "left_ankle_pitch_joint", "left_ankle_roll_joint"};
  right_joint_names_ = {"right_hip_pitch_joint",  "right_hip_roll_joint",
                        "right_hip_yaw_joint",     "right_knee_joint",
                        "right_ankle_pitch_joint", "right_ankle_roll_joint"};

  left_joint_idx_.assign(6, -1);
  right_joint_idx_.assign(6, -1);

  imu_sub_           = create_subscription<ImuMsg>(topic_imu, rclcpp::SensorDataQoS(), [this](const ImuMsg::ConstSharedPtr& m) { imuCallback(m); });
  joint_sub_         = create_subscription<JointMsg>(topic_joint, rclcpp::SensorDataQoS(), [this](const JointMsg::ConstSharedPtr& m) { jointCallback(m); });
  left_contact_sub_  = create_subscription<WrenchMsg>(topic_contact_left, rclcpp::SensorDataQoS(), [this](const WrenchMsg::ConstSharedPtr& m) { leftContactCallback(m); });
  right_contact_sub_ = create_subscription<WrenchMsg>(topic_contact_right, rclcpp::SensorDataQoS(), [this](const WrenchMsg::ConstSharedPtr& m) { rightContactCallback(m); });

  odom_pub_              = create_publisher<OdomMsg>(topic_odom, 10);
  robot_description_pub_ = create_publisher<StringMsg>(topic_description, rclcpp::QoS(1).transient_local());

  // path visualization
  path_pub_.push_back(create_publisher<nav_msgs::msg::Path>(topic_path, 10));
  nav_msgs::msg::Path path;
  path.header.frame_id = world_frame_;
  path_msgs_.push_back(std::move(path));
  path_state_conunters_.push_back(0);

  // foot contact visualization
  foot_contact_pub_.push_back(create_publisher<visualization_msgs::msg::MarkerArray>(topic_foot_contacts, 10));
  foot_contact_markerarrays_.push_back(FootContactMarkerArray());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  std::ifstream urdf_file(urdf_path);
  if (urdf_file.is_open()) {
    std::ostringstream ss;
    ss << urdf_file.rdbuf();
    StringMsg msg;
    msg.data = ss.str();
    robot_description_pub_->publish(msg);
  } 
  else {
    RCLCPP_ERROR(get_logger(), "Cannot open URDF for publishing: %s", urdf_path.c_str());
  }

  // Start the flush timer after all subscriptions are set up
  flushTimer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { flushReadyEvents(); });

  RCLCPP_INFO(get_logger(),
              "[init] estimator=%s  dead_reckoning=%.3fs  lag=%.2fs  reorder=%.3fs",
              est_params_.estimator_type.c_str(),
              max_dead_reckoning_s_, est_params_.lag_seconds,
              reorderDelaySeconds_);
}

HumanoidEstimatorNode::~HumanoidEstimatorNode() {
  if (flushTimer_) flushTimer_->cancel();
}

// ---------------------------------------------------------------------------
// Visualization helpers
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::publishPath(size_t index, const geometry_msgs::msg::PoseStamped& pose) {
  if (index >= path_pub_.size() || index >= path_msgs_.size()) {
    return;
  }

  ++path_state_conunters_[index];
  if ((path_state_conunters_[index] - 1) % static_cast<size_t>(path_publish_decimation_) != 0) {
    return;
  }

  nav_msgs::msg::Path& path = path_msgs_[index];
  path.header = pose.header;
  path.poses.push_back(pose);

  if (path_max_poses_ > 0 && path.poses.size() > static_cast<size_t>(path_max_poses_)) {
    const size_t extra = path.poses.size() - static_cast<size_t>(path_max_poses_);
    path.poses.erase(path.poses.begin(), path.poses.begin() + extra);
  }
  path_pub_[index]->publish(path);
}

visualization_msgs::msg::MarkerArray HumanoidEstimatorNode::FootContactMarkerArray() const {
  static const std::array<std::string, 2> foot_names = {"left_foot", "right_foot"};

  static const std::array<std::array<float, 4>, 2> foot_colors = {{
      {0.0f, 1.0f, 0.0f, 0.8f},
      {0.0f, 0.5f, 1.0f, 0.8f},
  }};
  static constexpr float kScale = 0.03f;

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(2);
  for (size_t i = 0; i < 2; ++i) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = world_frame_;
    m.ns = foot_names[i] + "/contacts";
    m.id = static_cast<int32_t>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = kScale;
    m.scale.y = kScale;
    m.scale.z = kScale;
    m.color.r = foot_colors[i][0];
    m.color.g = foot_colors[i][1];
    m.color.b = foot_colors[i][2];
    m.color.a = foot_colors[i][3];
    marker_array.markers.push_back(std::move(m));
  }
  return marker_array;
}



// ---------------------------------------------------------------------------
// Sensor callbacks — buffer only
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::imuCallback(const ImuMsg::ConstSharedPtr& msg) {
  LiveEvent event;
  event.type       = LiveEvent::Type::kImu;
  event.timestampS = rclcpp::Time(msg->header.stamp).seconds();
  event.imu.timestamp_s = event.timestampS;
  event.imu.omega = {msg->angular_velocity.x,
                     msg->angular_velocity.y,
                     msg->angular_velocity.z};
  event.imu.accel = {msg->linear_acceleration.x,
                     msg->linear_acceleration.y,
                     msg->linear_acceleration.z};
  enqueueEvent(std::move(event));
}

void HumanoidEstimatorNode::jointCallback(const JointMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lk(sensor_mutex_);
  if (!joint_map_ready_) buildJointIndexMap(*msg);
  joint_queue_.push_back(msg);
  const double cutoff = rclcpp::Time(msg->header.stamp).seconds() - kJointBufferSecs;
  while (!joint_queue_.empty() && rclcpp::Time(joint_queue_.front()->header.stamp).seconds() < cutoff) {
    joint_queue_.pop_front();
  }
}

void HumanoidEstimatorNode::leftContactCallback(const WrenchMsg::ConstSharedPtr& msg) {
  const bool in_contact = msg->wrench.force.z > contact_force_threshold_;
  LiveEvent event;
  event.type = LiveEvent::Type::kContact;
  event.timestampS = rclcpp::Time(msg->header.stamp).seconds();
  {
    std::lock_guard<std::mutex> lk(sensor_mutex_);
    event.contact.left_touchdown = in_contact && !left_in_contact_;
    left_in_contact_ = in_contact;
    event.contact.left_in_contact = in_contact;
    event.contact.right_in_contact = right_in_contact_;
  }
  event.contact.timestamp_s = event.timestampS;
  event.contact.right_touchdown = false;
  enqueueEvent(std::move(event));
}

void HumanoidEstimatorNode::rightContactCallback(const WrenchMsg::ConstSharedPtr& msg) {
  const bool in_contact = msg->wrench.force.z > contact_force_threshold_;
  LiveEvent event;
  event.type = LiveEvent::Type::kContact;
  event.timestampS = rclcpp::Time(msg->header.stamp).seconds();
  {
    std::lock_guard<std::mutex> lk(sensor_mutex_);
    event.contact.right_touchdown = in_contact && !right_in_contact_;
    right_in_contact_ = in_contact;
    event.contact.left_in_contact = left_in_contact_;
    event.contact.right_in_contact = in_contact;
  }
  event.contact.timestamp_s = event.timestampS;
  event.contact.left_touchdown = false;
  enqueueEvent(std::move(event));
}

// ---------------------------------------------------------------------------
// enqueueEvent — called from any callback thread
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::enqueueEvent(LiveEvent event) {
  std::lock_guard<std::mutex> lock(queueMutex_);
  event.sequence = nextSequence_++;  // sequence numbers ensure FIFO order for events with identical timestamps
  latestQueuedTimestampS_ = std::max(latestQueuedTimestampS_, event.timestampS); // record the latest timestamp 
  const int priority = (event.type == LiveEvent::Type::kImu) ? 0 : 1;            // priority IMU is processed before contact at the same timestamp
  eventQueue_.push(QueuedEvent{event.timestampS, priority, event.sequence, std::move(event)});
}

// ---------------------------------------------------------------------------
// flushReadyEvents — called from 10 ms wall timer
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::flushReadyEvents() {
  start_time_ = std::chrono::steady_clock::now();

  std::vector<LiveEvent> ready;
  {
    std::lock_guard<std::mutex> lock(queueMutex_);
    // Wait for late arriving events 
    const double cutoff = latestQueuedTimestampS_ - reorderDelaySeconds_;
    while (!eventQueue_.empty() && eventQueue_.top().timestampS <= cutoff) {
      ready.push_back(eventQueue_.top().event);
      eventQueue_.pop();
    }
  }
  for (LiveEvent& event : ready) { // Drops events with reversed timestamps
    if (event.timestampS < lastDispatchedTimestampS_ - 1e-9) {
      RCLCPP_WARN(get_logger(), "Dropping out-of-order event at %.9f", event.timestampS);
      continue;
    }
    lastDispatchedTimestampS_ = event.timestampS;
    handleOrderedEvent(std::move(event));
  }
  const auto end_time_ = std::chrono::steady_clock::now();
  const double wall_time_ms = std::chrono::duration<double, std::milli>(end_time_ - start_time_).count();
  // RCLCPP_INFO(get_logger(), "wall_time=%.3fms", wall_time_ms); 
}

// ---------------------------------------------------------------------------
// handleOrderedEvent — startup buffer until bias is ready, then dispatch
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::handleOrderedEvent(LiveEvent event) {
  if (!initialized_) { // Initialization
    startupBuffer_.push_back(event);
    if (event.type == LiveEvent::Type::kImu) {
      if (tryInitializeBias(event.imu)) { // bias initialized, can start processing events
        initializeFromStartupBuffer(); 
      }
    }
    return;
  }
  dispatch(event);
}
// The initial IMU buffer is used to without being discarded. 
void HumanoidEstimatorNode::initializeFromStartupBuffer() {
  initialized_     = true;
  have_held_imu_   = false;
  RCLCPP_INFO(get_logger(), "[init] replaying %zu startup events", startupBuffer_.size());
  for (const LiveEvent& ev : startupBuffer_) {
    dispatch(ev);
  }
  startupBuffer_.clear();
}

void HumanoidEstimatorNode::dispatch(const LiveEvent& event) {
  if (event.type == LiveEvent::Type::kImu) {
    processImu(event.imu);
  } 
  else {
    processContact(event.contact);
  }

  publishOdom(estimator_time_s_);
}

// ---------------------------------------------------------------------------
// processImu — zero-order-hold prediction, publishes odom after each step
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::processImu(const ImuSample& sample) {
  if (!have_held_imu_) { // Save th first IMU sample
    held_imu_             = sample;
    have_held_imu_        = true;
    estimator_time_s_     = sample.timestamp_s;
    last_contact_update_s_ = sample.timestamp_s;
    return;
  }

  const double new_t = sample.timestamp_s;
  if (new_t <= estimator_time_s_) { // Skip old IMU samples
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[imu] not monotonic, skipping");
    return;
  }

  const double dt = new_t - estimator_time_s_;
  if (dt < 0.5) predictHeld(dt); // If there is a big gap, skip prediction to avoid large jumps 
  estimator_time_s_ = new_t;
  held_imu_         = sample;

}

// ---------------------------------------------------------------------------
// processContact — touchdown or periodic update (mirrors reference logic)
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::processContact(const ContactEvent& event) {
  if (!have_held_imu_ || disable_contact_) return; // First IMU sample not yet received or contact updates disabled
  if (!event.left_in_contact && !event.right_in_contact) return; // No contact, no update

  const bool is_touchdown = event.left_touchdown || event.right_touchdown; // Touchdown update
  const bool periodic_due = (max_dead_reckoning_s_ > 0.0) &&               // More than max_dead_reckoning_s_ has passsed since the last update
                            (event.timestamp_s - last_contact_update_s_ >=
                             max_dead_reckoning_s_ - 1e-12);

  if (!is_touchdown && !periodic_due) return; // Not a touchdown and periodic update not due, skip to avoid excessive updates during long stances

  // Predict to the exact contact timestamp if it's ahead of estimator time.
  if (event.timestamp_s > estimator_time_s_ + 1e-9) {
    const double dt = event.timestamp_s - estimator_time_s_;
    if (dt < 0.5) predictHeld(dt);
    estimator_time_s_ = event.timestamp_s;
  }

  runContactUpdate(estimator_time_s_,
                   event.left_touchdown,  event.right_touchdown,
                   event.left_in_contact, event.right_in_contact);
  last_contact_update_s_ = estimator_time_s_;
}

// ---------------------------------------------------------------------------
// predictHeld — zero-order hold prediction using held_imu_
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::predictHeld(double dt) {
  const gtsam::Vector3 omega = zero_gyro_debug_
      ? gtsam::Vector3::Zero()
      : held_imu_.omega;
  const gtsam::Vector3 accel = zero_accel_debug_
      ? gtsam::Vector3(0.0, 0.0, est_params_.gravity.norm())
      : held_imu_.accel;
  estimator_.predict(omega, accel, dt);
}

// ---------------------------------------------------------------------------
// runContactUpdate
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::runContactUpdate(double now, bool left_td, bool right_td, bool left_c,  bool right_c) {
  // Find the timestamp closest to now amont the joint_states in the joint_queue_.
  JointMsg::ConstSharedPtr joint_snap;
  {
    std::lock_guard<std::mutex> lk(sensor_mutex_);
    double best_diff = kInf;
    for (const auto& msg : joint_queue_) {
      const double diff = std::abs(rclcpp::Time(msg->header.stamp).seconds() - now);
      if (diff < best_diff) {
        best_diff = diff;
        joint_snap = msg;
      }
    }
  }

  if (!joint_snap) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[contact] skipped: no joint_states");
    return;
  }
  if (!joint_map_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[contact] skipped: joint map not ready");
    return;
  }
  const double joint_dt = std::abs(rclcpp::Time(joint_snap->header.stamp).seconds() - now);

  if (joint_dt > kJointMaxStaleSecs) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[contact] joint_states %.3fs from IMU time — skip", joint_dt);
    return;
  }

  std::vector<gtsam::ContactMeasurement> contacts;
  Eigen::VectorXd q(6);
  gtsam::Vector3 left_body_fp = gtsam::Vector3::Zero();
  gtsam::Vector3 right_body_fp = gtsam::Vector3::Zero();
  bool left_fk_ok  = false;
  bool right_fk_ok = false;

  // body frame foot positions from FK are used for contact updates.
  if (left_c && extractLegJoints(*joint_snap, true, q)) {
    const Eigen::Vector3d fp = fk_provider_->footPosition(q, true);
    left_body_fp = {fp.x(), fp.y(), fp.z()};
    left_fk_ok = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[contact] left  body_pt=[%.3f %.3f %.3f] td=%d", fp.x(), fp.y(), fp.z(), left_td);
    contacts.push_back({0, left_body_fp, left_td});
  }
  if (right_c && extractLegJoints(*joint_snap, false, q)) {
    const Eigen::Vector3d fp = fk_provider_->footPosition(q, false);
    right_body_fp = {fp.x(), fp.y(), fp.z()};
    right_fk_ok = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[contact] right body_pt=[%.3f %.3f %.3f] td=%d", fp.x(), fp.y(), fp.z(), right_td);
    contacts.push_back({1, right_body_fp, right_td});
  }

  if (contacts.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[contact] in_contact but no valid FK");
    return;
  }

  RCLCPP_DEBUG(get_logger(), "[contact] joint_dt=%.4fs  left=%d(td=%d) right=%d(td=%d)", joint_dt, left_c, left_td, right_c, right_td);

  const gtsam::Point3 pos_before = estimator_.position();   // DEBUG: 
  estimator_.processContacts(contacts); // Optimizer update using contact measurements
  const gtsam::Point3 delta = estimator_.position() - pos_before;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[contact] t=%.3f delta=[%.4f %.4f %.4f]", now, delta.x(), delta.y(), delta.z());

  // Publish foot contact markers on touchdown (Visualization) 
  if ((left_td && left_fk_ok) || (right_td && right_fk_ok)) {
    const gtsam::Pose3 T_wb = estimator_.navState().pose();
    const double sec_floor = std::floor(now);
    const rclcpp::Time stamp(static_cast<int32_t>(sec_floor),
                             static_cast<uint32_t>((now - sec_floor) * 1e9));

    auto& markers = foot_contact_markerarrays_[0].markers;
    markers[0].header.stamp = stamp;
    markers[1].header.stamp = stamp;

    if (left_td && left_fk_ok) {
      const gtsam::Point3 wp = T_wb.transformFrom(left_body_fp);
      geometry_msgs::msg::Point pt;
      pt.x = wp.x(); pt.y = wp.y(); pt.z = wp.z();
      markers[0].points.push_back(pt);
    }
    if (right_td && right_fk_ok) {
      const gtsam::Point3 wp = T_wb.transformFrom(right_body_fp);
      geometry_msgs::msg::Point pt;
      pt.x = wp.x(); pt.y = wp.y(); pt.z = wp.z();
      markers[1].points.push_back(pt);
    }
    foot_contact_pub_[0]->publish(foot_contact_markerarrays_[0]);
  }
}

// ---------------------------------------------------------------------------
// Publish
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::publishOdom(double timestamp_s) {
  const gtsam::NavState state = estimator_.navState();
  const gtsam::Point3&  p     = state.position();
  const gtsam::Vector3& v     = state.velocity();
  const Eigen::Quaterniond q  = state.attitude().toQuaternion();

  const double sec_floor = std::floor(timestamp_s);
  const rclcpp::Time stamp(
      static_cast<int32_t>(sec_floor),
      static_cast<uint32_t>((timestamp_s - sec_floor) * 1e9));

  OdomMsg odom;
  odom.header.stamp    = stamp;
  odom.header.frame_id = world_frame_;
  odom.child_frame_id  = base_frame_;
  odom.pose.pose.position.x    = p.x();
  odom.pose.pose.position.y    = p.y();
  odom.pose.pose.position.z    = p.z();
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.twist.twist.linear.x = v.x();
  odom.twist.twist.linear.y = v.y();
  odom.twist.twist.linear.z = v.z();
  odom_pub_->publish(odom);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = odom.header;
  pose_stamped.pose   = odom.pose.pose;
  publishPath(0, pose_stamped);

}

// ---------------------------------------------------------------------------
// Bias initialization
// ---------------------------------------------------------------------------

bool HumanoidEstimatorNode::tryInitializeBias(const ImuSample& s) {
  bias_sum_omega_  += s.omega;
  bias_sum_accel_  += s.accel;
  bias_sum2_omega_ += s.omega.cwiseProduct(s.omega);
  bias_sum2_accel_ += s.accel.cwiseProduct(s.accel);
  ++bias_sample_count_;

  if (bias_sample_count_ % 100 == 0) {
    RCLCPP_INFO(get_logger(), "[bias] collecting... %d/%d", bias_sample_count_, kBiasSamples);
  }
  if (bias_sample_count_ < kBiasSamples) return false;

  const double n = static_cast<double>(kBiasSamples);
  const gtsam::Vector3 mean_omega = bias_sum_omega_ / n;
  const gtsam::Vector3 mean_accel = bias_sum_accel_ / n;

  const gtsam::Vector3 var_omega = bias_sum2_omega_ / n - mean_omega.cwiseProduct(mean_omega);
  const gtsam::Vector3 var_accel = bias_sum2_accel_ / n - mean_accel.cwiseProduct(mean_accel);
  const double std_omega = var_omega.cwiseSqrt().maxCoeff();
  const double std_accel = var_accel.cwiseSqrt().maxCoeff();

  if (std_omega > kMaxGyroStd || std_accel > kMaxAccStd) {
    RCLCPP_WARN(get_logger(), "[bias] not stationary (gyro_std=%.4f acc_std=%.3f) — restart. " "Hold robot still.", std_omega, std_accel);
    bias_sample_count_ = 0;
    bias_sum_omega_  = gtsam::Vector3::Zero();
    bias_sum_accel_  = gtsam::Vector3::Zero();
    bias_sum2_omega_ = gtsam::Vector3::Zero();
    bias_sum2_accel_ = gtsam::Vector3::Zero();
    return false;
  }

  const gtsam::Vector3 g_up(0.0, 0.0, est_params_.gravity.norm());
  const Eigen::Quaterniond q_align = Eigen::Quaterniond::FromTwoVectors(mean_accel, g_up);
  const gtsam::Rot3 R_init = gtsam::Rot3(q_align.toRotationMatrix());

  const gtsam::Vector3 expected = R_init.unrotate(-est_params_.gravity);
  const gtsam::imuBias::ConstantBias bias(mean_accel - expected, mean_omega);

  const gtsam::Vector3 rpy = R_init.rpy();
  RCLCPP_INFO(get_logger(),
              "[bias] accel=[%.4f %.4f %.4f] norm=%.4f",
              mean_accel.x(), mean_accel.y(), mean_accel.z(), mean_accel.norm());
  RCLCPP_INFO(get_logger(),
              "[bias] attitude: roll=%.2f° pitch=%.2f° yaw=%.2f°",
              rpy.x() * 180.0 / M_PI,
              rpy.y() * 180.0 / M_PI,
              rpy.z() * 180.0 / M_PI);
  RCLCPP_INFO(get_logger(),
              "[bias] acc_bias=[%.4f %.4f %.4f] gyro_bias=[%.4f %.4f %.4f]",
              bias.accelerometer().x(), bias.accelerometer().y(),
              bias.accelerometer().z(),
              bias.gyroscope().x(), bias.gyroscope().y(),
              bias.gyroscope().z());

  if (bias.accelerometer().norm() > 2.0) {
    RCLCPP_WARN(get_logger(), "[bias] acc_bias norm=%.2f > 2 m/s² — check body_P_imu", bias.accelerometer().norm());
  }

  estimator_.initialize(bias, R_init);
  bias_initialized_ = true;

  // Allow immediate first contact update
  {
    std::lock_guard<std::mutex> lk(sensor_mutex_);
    bias_initialized_ = true;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Joint helpers
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::buildJointIndexMap(const JointMsg& msg) {
  auto lookup = [&](const std::string& name) -> int {
    const auto it = std::find(msg.name.begin(), msg.name.end(), name);
    return it != msg.name.end()
               ? static_cast<int>(it - msg.name.begin())
               : -1;
  };
  for (int i = 0; i < 6; ++i) {
    left_joint_idx_[i]  = lookup(left_joint_names_[i]);
    right_joint_idx_[i] = lookup(right_joint_names_[i]);
  }
  joint_map_ready_ =
      std::none_of(left_joint_idx_.begin(), left_joint_idx_.end(),
                   [](int v) { return v < 0; }) &&
      std::none_of(right_joint_idx_.begin(), right_joint_idx_.end(),
                   [](int v) { return v < 0; });

  if (joint_map_ready_) {
    RCLCPP_INFO(get_logger(), "[joint] index map ready");
  } 
  else {
    RCLCPP_WARN(get_logger(), "[joint] some leg joints missing");
  }
}

bool HumanoidEstimatorNode::extractLegJoints(const JointMsg& msg,
                                              bool left,
                                              Eigen::VectorXd& pos) const {
  const std::vector<int>& idx = left ? left_joint_idx_ : right_joint_idx_;
  for (int i = 0; i < 6; ++i) {
    if (idx[i] < 0 || static_cast<size_t>(idx[i]) >= msg.position.size()) {
      return false;
    }
    pos[i] = msg.position[idx[i]];
  }
  return true;
}

} 