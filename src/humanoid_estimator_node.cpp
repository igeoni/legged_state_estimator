// Translation unit: GTSAM + ROS2 — NO pinocchio headers.
#include "legged_state_estimator/humanoid_estimator_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legged_state_estimator {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

HumanoidEstimatorNode::HumanoidEstimatorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("humanoid_state_estimator", options),
      estimator_(est_params_) {

  const std::string urdf_path = declare_parameter<std::string>("urdf_path", "");
  est_params_.estimator_type  = declare_parameter<std::string>("estimator_type", "fixed_lag_single_bias");
  contact_force_threshold_    = declare_parameter<double>("contact_force_threshold", 10.0);
  est_params_.initial_height  = declare_parameter<double>("initial_height", 0.787);
  est_params_.sigma_gyro  = declare_parameter<double>("sigma_gyro", 8e-4);
  est_params_.sigma_acc   = declare_parameter<double>("sigma_acc",  2e-2);
  est_params_.lag_seconds = declare_parameter<double>("lag_seconds", 1.0);
  max_dead_reckoning_s_   = declare_parameter<double>("max_dead_reckoning_s", 0.1);
  disable_contact_        = declare_parameter<bool>("disable_contact", false);
  zero_accel_debug_       = declare_parameter<bool>("zero_accel_debug", false);
  zero_gyro_debug_        = declare_parameter<bool>("zero_gyro_debug", false);
  world_frame_ = declare_parameter<std::string>("world_frame", "odom");
  base_frame_  = declare_parameter<std::string>("base_frame",  "pelvis");

  if (urdf_path.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'urdf_path' is required.");
    throw std::runtime_error("urdf_path not set");
  }

  // G1 imu_in_pelvis_joint: xyz=[0.04525, 0, -0.08339] rpy=[0,0,0] (from URDF)
  est_params_.body_P_imu = gtsam::Pose3(
      gtsam::Rot3(),
      gtsam::Point3(0.04525, 0.0, -0.08339));

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

  imu_sub_ = create_subscription<ImuMsg>("/imu", rclcpp::SensorDataQoS(), [this](const ImuMsg::ConstSharedPtr& m) { imuCallback(m); });
  joint_sub_ = create_subscription<JointMsg>("/joint_states", rclcpp::SensorDataQoS(), [this](const JointMsg::ConstSharedPtr& m) { jointCallback(m); });
  left_contact_sub_ = create_subscription<WrenchMsg>("/contact/left_foot", rclcpp::SensorDataQoS(), [this](const WrenchMsg::ConstSharedPtr& m) { leftContactCallback(m); });
  right_contact_sub_ = create_subscription<WrenchMsg>("/contact/right_foot", rclcpp::SensorDataQoS(), [this](const WrenchMsg::ConstSharedPtr& m) { rightContactCallback(m); });

  odom_pub_ = create_publisher<OdomMsg>("/state_estimator/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  processing_thread_ = std::thread(&HumanoidEstimatorNode::processingLoop, this);

  RCLCPP_INFO(get_logger(),
              "[init] estimator=%s  dead_reckoning=%.3fs  lag=%.2fs",
              est_params_.estimator_type.c_str(),
              max_dead_reckoning_s_, est_params_.lag_seconds);
}

HumanoidEstimatorNode::~HumanoidEstimatorNode() {
  running_ = false;
  imu_cv_.notify_all();
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// Sensor callbacks — buffer only
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::imuCallback(const ImuMsg::ConstSharedPtr& msg) {
  ImuSample s;
  s.timestamp_s = rclcpp::Time(msg->header.stamp).seconds();
  s.omega = {msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z};
  s.accel = {msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z};
  {
    std::lock_guard<std::mutex> lk(sensor_mutex_);
    imu_queue_.push_back(s);
  }
  imu_cv_.notify_one();
}

void HumanoidEstimatorNode::jointCallback(const JointMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lk(sensor_mutex_);
  if (!joint_map_ready_) {
    buildJointIndexMap(*msg);
  }
  latest_joint_ = msg;
}

void HumanoidEstimatorNode::leftContactCallback(
    const WrenchMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lk(sensor_mutex_);
  const bool in_contact = msg->wrench.force.z > contact_force_threshold_;
  if (in_contact && !left_in_contact_) {
    pending_left_touchdown_ = true;
  }
  left_in_contact_ = in_contact;
}

void HumanoidEstimatorNode::rightContactCallback(
    const WrenchMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lk(sensor_mutex_);
  const bool in_contact = msg->wrench.force.z > contact_force_threshold_;
  if (in_contact && !right_in_contact_) {
    pending_right_touchdown_ = true;
  }
  right_in_contact_ = in_contact;
}

// ---------------------------------------------------------------------------
// processingLoop — single thread: predict → contact update → publish
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::processingLoop() {
  RCLCPP_INFO(get_logger(), "[loop] processing thread started");

  try {
    while (running_) {
      // Wait for new IMU data
      ImuSample s;
      {
        std::unique_lock<std::mutex> lk(sensor_mutex_);
        imu_cv_.wait(lk, [this] {
          return !imu_queue_.empty() || !running_;
        });
        if (!running_) break;
        s = imu_queue_.front();
        imu_queue_.pop_front();
      }

      // ── Step 1: bias initialization ────────────────────────────────────
      if (!bias_initialized_) {
        if (tryInitializeBias(s)) {
          estimator_time_s_      = s.timestamp_s;
          last_contact_update_s_ = s.timestamp_s;
          RCLCPP_INFO(get_logger(),
                      "[init] estimator ready at t=%.3f", estimator_time_s_);
        }
        continue;
      }

      // ── Step 2: predict ────────────────────────────────────────────────
      RCLCPP_DEBUG(get_logger(), "[loop] predict t=%.4f", s.timestamp_s);
      runPredict(s);

      // ── Step 3: contact update ─────────────────────────────────────────
      if (!disable_contact_) {
        bool left_td, right_td, left_c, right_c;
        {
          std::lock_guard<std::mutex> lk(sensor_mutex_);
          left_td  = pending_left_touchdown_;
          right_td = pending_right_touchdown_;
          left_c   = left_in_contact_;
          right_c  = right_in_contact_;
          pending_left_touchdown_  = false;
          pending_right_touchdown_ = false;
        }

        const bool has_touchdown = left_td || right_td;
        const bool periodic_due  =
            (estimator_time_s_ - last_contact_update_s_) >= max_dead_reckoning_s_;

        if ((left_c || right_c) && (has_touchdown || periodic_due)) {
          runContactUpdate(estimator_time_s_, left_td, right_td);
          last_contact_update_s_ = estimator_time_s_;
        }
      }

      // ── Step 4: publish ────────────────────────────────────────────────
      publishOdom(estimator_time_s_);
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(),
                 "[loop] EXCEPTION — processing thread died: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(get_logger(),
                 "[loop] UNKNOWN EXCEPTION — processing thread died");
  }

  RCLCPP_INFO(get_logger(), "[loop] processing thread stopped");
}

// ---------------------------------------------------------------------------
// Estimator steps (only called from processingLoop)
// ---------------------------------------------------------------------------

bool HumanoidEstimatorNode::tryInitializeBias(const ImuSample& s) {
  bias_sum_omega_ += s.omega;
  bias_sum_accel_ += s.accel;
  ++bias_sample_count_;

  if (bias_sample_count_ % 50 == 0) {
    RCLCPP_INFO(get_logger(), "[bias] collecting... %d/%d samples", bias_sample_count_, kBiasSamples);
  }

  if (bias_sample_count_ < kBiasSamples) return false;

  const double n = static_cast<double>(kBiasSamples);
  const gtsam::Vector3 mean_omega = bias_sum_omega_ / n;
  const gtsam::Vector3 mean_accel = bias_sum_accel_ / n;

  // Estimate roll/pitch from gravity direction in IMU frame (yaw = 0).
  // specific force at rest = R_body_world^T * (-g), so mean_accel ≈ R^T * (0,0,9.81).
  // Use Eigen's quaternion alignment: find minimum-rotation R s.t. R * mean_accel = g_up.
  const gtsam::Vector3 g_up(0.0, 0.0, est_params_.gravity.norm());  // (0,0,9.81)
  const Eigen::Quaterniond q_align =
      Eigen::Quaterniond::FromTwoVectors(mean_accel, g_up);
  const gtsam::Rot3 R_init = gtsam::Rot3(q_align.toRotationMatrix());

  // Bias = measured - expected_in_body_frame (gyro bias = mean gyro at rest)
  const gtsam::Vector3 expected_in_body = R_init.unrotate(-est_params_.gravity);
  const gtsam::imuBias::ConstantBias bias(mean_accel - expected_in_body, mean_omega);

  const gtsam::Vector3 rpy = R_init.rpy();
  RCLCPP_INFO(get_logger(),
              "[bias] mean IMU at rest — "
              "omega=[%.4f %.4f %.4f] accel=[%.4f %.4f %.4f] norm=%.4f",
              mean_omega.x(), mean_omega.y(), mean_omega.z(),
              mean_accel.x(), mean_accel.y(), mean_accel.z(),
              mean_accel.norm());
  RCLCPP_INFO(get_logger(),
              "[bias] initial attitude from gravity — "
              "roll=%.2f° pitch=%.2f° yaw=%.2f°",
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
    RCLCPP_WARN(get_logger(),
                "[diag] acc_bias norm=%.2f > 2 m/s² — "
                "check body_P_imu rotation or IMU axis convention",
                bias.accelerometer().norm());
  }

  estimator_.initialize(bias, R_init);  // pass computed initial attitude
  bias_initialized_ = true;
  return true;
}

void HumanoidEstimatorNode::runPredict(const ImuSample& s) {
  const double dt = s.timestamp_s - estimator_time_s_;
  if (dt <= 0.0 || dt > 0.5) {
    if (dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[predict] skipped: dt=%.6f (non-positive)", dt);
    } else {
      RCLCPP_WARN(get_logger(), "[predict] skipped: dt=%.3f too large (clock jump?)", dt);
    }
    estimator_time_s_ = s.timestamp_s;
    return;
  }

  const gtsam::Vector3 accel = zero_accel_debug_
      ? gtsam::Vector3(0.0, 0.0, +est_params_.gravity.norm())  // static gravity reaction (Z-up)
      : s.accel;
  const gtsam::Vector3 omega = zero_gyro_debug_
      ? gtsam::Vector3::Zero()
      : s.omega;

  estimator_.predict(omega, accel, dt);
  estimator_time_s_ = s.timestamp_s;
}

void HumanoidEstimatorNode::runContactUpdate(double /*now*/,
                                              bool left_td, bool right_td) {
  JointMsg::ConstSharedPtr joint_snap;
  bool left_c, right_c;
  {
    std::lock_guard<std::mutex> lk(sensor_mutex_);
    joint_snap = latest_joint_;
    left_c     = left_in_contact_;
    right_c    = right_in_contact_;
  }

  if (!joint_snap) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "[contact] skipped: no joint_states");
    return;
  }
  if (!joint_map_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "[contact] skipped: joint index map not ready");
    return;
  }

  // Log position BEFORE update to measure update's contribution
  const gtsam::Point3 pos_before = estimator_.position();

  std::vector<gtsam::ContactMeasurement> contacts;

  Eigen::VectorXd q(6);
  if (left_c && extractLegJoints(*joint_snap, true, q)) {
    const Eigen::Vector3d fp = fk_provider_->footPosition(q, true);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[contact] left  body_pt=[%.3f %.3f %.3f] td=%d",
                         fp.x(), fp.y(), fp.z(), left_td);
    contacts.push_back({0, gtsam::Vector3(fp.x(), fp.y(), fp.z()), left_td});
  }
  if (right_c && extractLegJoints(*joint_snap, false, q)) {
    const Eigen::Vector3d fp = fk_provider_->footPosition(q, false);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[contact] right body_pt=[%.3f %.3f %.3f] td=%d",
                         fp.x(), fp.y(), fp.z(), right_td);
    contacts.push_back({1, gtsam::Vector3(fp.x(), fp.y(), fp.z()), right_td});
  }

  if (contacts.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[contact] in_contact but no valid FK result");
    return;
  }

  // Log foot world positions BEFORE update — should stay constant if constraint works
  {
    const gtsam::Point3  p = estimator_.position();
    const gtsam::Rot3    R = estimator_.orientation();
    for (const auto& c : contacts) {
      const gtsam::Point3 foot_world = p + R.rotate(c.bodyPoint);
      RCLCPP_INFO(get_logger(),
                  "[contact] foot[%zu] world=[%.4f %.4f %.4f] body=[%.4f %.4f %.4f]",
                  c.foot,
                  foot_world.x(), foot_world.y(), foot_world.z(),
                  c.bodyPoint.x(), c.bodyPoint.y(), c.bodyPoint.z());
    }
  }

  RCLCPP_DEBUG(get_logger(),"[contact] update: left=%d(td=%d) right=%d(td=%d)",left_c, left_td, right_c, right_td);

  estimator_.processContacts(contacts);

  // Log foot world positions AFTER update
  const gtsam::Point3 pos_after = estimator_.position();
  const gtsam::Point3 delta = pos_after - pos_before;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "[contact] pos before=[%.3f %.3f %.3f] after=[%.3f %.3f %.3f] "
                       "delta=[%.4f %.4f %.4f]",
                       pos_before.x(), pos_before.y(), pos_before.z(),
                       pos_after.x(), pos_after.y(), pos_after.z(),
                       delta.x(), delta.y(), delta.z());
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
  const rclcpp::Time stamp(static_cast<int32_t>(sec_floor),
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

  geometry_msgs::msg::TransformStamped tf;
  tf.header      = odom.header;
  tf.child_frame_id             = base_frame_;
  tf.transform.translation.x    = p.x();
  tf.transform.translation.y    = p.y();
  tf.transform.translation.z    = p.z();
  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  // tf_broadcaster_->sendTransform(tf);

  RCLCPP_DEBUG(get_logger(), "[odom] t=%.3f pos=[%.3f %.3f %.3f]", timestamp_s, p.x(), p.y(), p.z());
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "[odom] pos=[%.3f %.3f %.3f] vel=[%.3f %.3f %.3f] |vel|=%.3f",
                       p.x(), p.y(), p.z(),
                       v.x(), v.y(), v.z(),
                       v.norm());
}

// ---------------------------------------------------------------------------
// Joint helpers
// ---------------------------------------------------------------------------

void HumanoidEstimatorNode::buildJointIndexMap(const JointMsg& msg) {
  auto lookup = [&](const std::string& name) -> int {
    const auto it =
        std::find(msg.name.begin(), msg.name.end(), name);
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
    RCLCPP_WARN(get_logger(), "[joint] some leg joints missing from /joint_states");
  }
}

bool HumanoidEstimatorNode::extractLegJoints(const JointMsg& msg, bool left, Eigen::VectorXd& pos) const {
  const std::vector<int>& idx = left ? left_joint_idx_ : right_joint_idx_;
  for (int i = 0; i < 6; ++i) {
    if (idx[i] < 0 ||
        static_cast<size_t>(idx[i]) >= msg.position.size()) {
      return false;
    }
    pos[i] = msg.position[idx[i]];
  }
  return true;
}

}  // namespace legged_state_estimator
