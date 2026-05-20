#pragma once

// NOTE: This header intentionally does NOT include g1_kinematics.hpp (pinocchio)
// or humanoid_estimator.hpp (GTSAM) to avoid Boost-serialization conflicts.
// Both are included only in humanoid_estimator_node.cpp.

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "legged_state_estimator/fk_provider.hpp"
#include "legged_state_estimator/humanoid_estimator.hpp"

namespace legged_state_estimator {

/**
 * ROS2 node for real-time humanoid state estimation.
 *
 * Architecture:
 *   - Sensor callbacks: buffer-only (push to queue, update flags)
 *   - Single processing thread: drains IMU queue → predict → processContacts → publish
 *
 * Subscribes:
 *   /imu                 sensor_msgs/Imu
 *   /joint_states        sensor_msgs/JointState
 *   /contact/left_foot   geometry_msgs/WrenchStamped
 *   /contact/right_foot  geometry_msgs/WrenchStamped
 *
 * Publishes:
 *   /state_estimator/odom   nav_msgs/Odometry
 *   /tf                     world → base_link
 *
 * Parameters:
 *   urdf_path               (string)  required
 *   estimator_type          (string)  [default: "fixed_lag_single_bias"]
 *   contact_force_threshold (double)  [default: 10.0 N]
 *   initial_height          (double)  [default: 0.787 m]
 *   sigma_gyro              (double)  [default: 8e-4]
 *   sigma_acc               (double)  [default: 2e-2]
 *   lag_seconds             (double)  [default: 1.0]
 *   max_dead_reckoning_s    (double)  contact update period [default: 0.1 s]
 *   world_frame             (string)  [default: "odom"]
 *   base_frame              (string)  [default: "base_link"]
 */
class HumanoidEstimatorNode : public rclcpp::Node {
 public:
  explicit HumanoidEstimatorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~HumanoidEstimatorNode();

 private:
  using ImuMsg    = sensor_msgs::msg::Imu;
  using JointMsg  = sensor_msgs::msg::JointState;
  using WrenchMsg = geometry_msgs::msg::WrenchStamped;
  using OdomMsg   = nav_msgs::msg::Odometry;

  struct ImuSample {
    double timestamp_s;
    gtsam::Vector3 omega;
    gtsam::Vector3 accel;
  };

  // ── Sensor callbacks (buffer only, never call estimator) ──────────────────
  void imuCallback(const ImuMsg::ConstSharedPtr& msg);
  void jointCallback(const JointMsg::ConstSharedPtr& msg);
  void leftContactCallback(const WrenchMsg::ConstSharedPtr& msg);
  void rightContactCallback(const WrenchMsg::ConstSharedPtr& msg);

  // ── Processing thread ─────────────────────────────────────────────────────
  void processingLoop();

  // Steps called only from processingLoop()
  bool tryInitializeBias(const ImuSample& s);
  void runPredict(const ImuSample& s);
  void runContactUpdate(double now, bool left_td, bool right_td);
  void publishOdom(double timestamp_s);

  // Helpers
  bool extractLegJoints(const JointMsg& msg, bool left,
                         Eigen::VectorXd& pos) const;
  void buildJointIndexMap(const JointMsg& msg);

  // ── Estimator ─────────────────────────────────────────────────────────────
  HumanoidEstimatorParams est_params_;
  HumanoidEstimator estimator_;
  std::unique_ptr<FkProvider> fk_provider_;

  // ── ROS I/O ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<ImuMsg>::SharedPtr    imu_sub_;
  rclcpp::Subscription<JointMsg>::SharedPtr  joint_sub_;
  rclcpp::Subscription<WrenchMsg>::SharedPtr left_contact_sub_;
  rclcpp::Subscription<WrenchMsg>::SharedPtr right_contact_sub_;

  rclcpp::Publisher<OdomMsg>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string world_frame_;
  std::string base_frame_;

  // ── Shared sensor state (guarded by sensor_mutex_) ───────────────────────
  std::mutex              sensor_mutex_;
  std::condition_variable imu_cv_;       // notified when imu_queue_ gets data
  std::deque<ImuSample>   imu_queue_;

  JointMsg::ConstSharedPtr latest_joint_;

  bool left_in_contact_        = false;
  bool right_in_contact_       = false;
  bool pending_left_touchdown_  = false;
  bool pending_right_touchdown_ = false;
  double contact_force_threshold_ = 10.0;

  // Joint index lookup (built once from first JointState)
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;
  std::vector<int> left_joint_idx_;
  std::vector<int> right_joint_idx_;
  bool joint_map_ready_ = false;

  // ── Processing thread state (only accessed from processingLoop()) ─────────
  std::thread processing_thread_;
  std::atomic<bool> running_{true};

  bool   bias_initialized_       = false;
  int    bias_sample_count_      = 0;
  gtsam::Vector3 bias_sum_omega_ = gtsam::Vector3::Zero();
  gtsam::Vector3 bias_sum_accel_ = gtsam::Vector3::Zero();
  static constexpr int kBiasSamples = 200;

  double estimator_time_s_      = 0.0;
  double last_contact_update_s_ = -1e9;
  double max_dead_reckoning_s_  = 0.1;
  bool   disable_contact_       = false;
  bool   zero_accel_debug_      = false;
  bool   zero_gyro_debug_       = false;
};

}  // namespace legged_state_estimator
