/**
 * ROS2 node for real-time humanoid state estimation.
 *
 * Processing model (reference-style event-driven):
 *   For each new IMU sample, process all contact events that fall between
 *   the previous IMU time and the new IMU time in chronological order,
 *   predicting with zero-order-hold IMU between events — identical to the
 *   offline LeggedEstimatorReplayExample approach.
 *
 * Subscribes:
 *   /imu                 sensor_msgs/Imu
 *   /joint_states        sensor_msgs/JointState
 *   /contact/left_foot   geometry_msgs/WrenchStamped
 *   /contact/right_foot  geometry_msgs/WrenchStamped
 *
 * Publishes:
 *   /state_estimator/odom   nav_msgs/Odometry
 */

#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "legged_state_estimator/kinematics/fk_provider.hpp"
#include "legged_state_estimator/state_estimator/humanoid_estimator.hpp"

namespace legged_state_estimator {

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
  using StringMsg = std_msgs::msg::String;
  using BoolMsg   = std_msgs::msg::Bool;

  struct ImuSample {
    double timestamp_s = 0.0;
    gtsam::Vector3 omega = gtsam::Vector3::Zero();
    gtsam::Vector3 accel = gtsam::Vector3::Zero();
  };

  struct ContactEvent {
    double timestamp_s    = 0.0;
    bool left_in_contact  = false;
    bool right_in_contact = false;
    bool left_touchdown   = false;
    bool right_touchdown  = false;
  };

  struct LiveEvent {
    enum class Type { kImu, kContact };
    Type     type      = Type::kImu;
    double   timestampS = 0.0;
    uint64_t sequence  = 0;
    ImuSample    imu;
    ContactEvent contact;
  };

  struct QueuedEvent {
    double   timestampS;
    int      priority;   // 0=IMU, 1=contact (IMU first at equal timestamp)
    uint64_t sequence;
    LiveEvent event;
    bool operator>(const QueuedEvent& o) const {
      if (timestampS != o.timestampS) return timestampS > o.timestampS;
      if (priority   != o.priority)   return priority   > o.priority;
      return sequence > o.sequence;
    }
  };

  // Visualization helpers
  void publishPath(size_t index, const geometry_msgs::msg::PoseStamped& pose);
  visualization_msgs::msg::MarkerArray FootContactMarkerArray() const;

  // Sensor callbacks (buffer only, never call estimator) 
  void imuCallback(const ImuMsg::ConstSharedPtr& msg);
  void jointCallback(const JointMsg::ConstSharedPtr& msg);
  // MuJoCo Simulation True contact information
  void leftContactCallback(const WrenchMsg::ConstSharedPtr& msg);
  void rightContactCallback(const WrenchMsg::ConstSharedPtr& msg);
  // CNN contact estimator
  void leftContactEstimatorCallback(const WrenchMsg::ConstSharedPtr& msg);
  void rightContactEstimatorCallback(const WrenchMsg::ConstSharedPtr& msg);

  // Event queue (timer-driven, reorder-delay flush) 
  void enqueueEvent(LiveEvent event);
  void flushReadyEvents();
  void handleOrderedEvent(LiveEvent event);
  void initializeFromStartupBuffer();
  void dispatch(const LiveEvent& event);

  // Per-event processors (called from dispatch) 
  void processImu(const ImuSample& sample);
  void processContact(const ContactEvent& event);

  // Helpers 
  bool tryInitializeBias(const ImuSample& s);
  void predictHeld(double dt);
  void runContactUpdate(double now, bool left_td, bool right_td,
                        bool left_c, bool right_c);
  void publishOdom(double timestamp_s);

  // Joint helpers
  bool extractLegJoints(const JointMsg& msg, bool left,
                        Eigen::VectorXd& pos) const;
  void buildJointIndexMap(const JointMsg& msg);

  // Estimator 
  HumanoidEstimatorParams est_params_;
  HumanoidEstimator estimator_;
  std::unique_ptr<FkProvider> fk_provider_;

  // ROS Subscribers & Publishers 
  rclcpp::Subscription<ImuMsg>::SharedPtr    imu_sub_;
  rclcpp::Subscription<JointMsg>::SharedPtr  joint_sub_;
  // MuJoCo Simulation True contact information
  rclcpp::Subscription<WrenchMsg>::SharedPtr left_contact_sub_;
  rclcpp::Subscription<WrenchMsg>::SharedPtr right_contact_sub_;
  // CNN contact estimator 
  rclcpp::Subscription<WrenchMsg>::SharedPtr left_contact_estimator_sub_;
  rclcpp::Subscription<WrenchMsg>::SharedPtr right_contact_estimator_sub_;

  rclcpp::Publisher<OdomMsg>::SharedPtr    odom_pub_;
  rclcpp::Publisher<StringMsg>::SharedPtr  robot_description_pub_;

  // path visualization 
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pub_;
  std::vector<nav_msgs::msg::Path> path_msgs_;
  std::vector<size_t> path_state_conunters_;
  int path_publish_decimation_ = 10;
  int path_max_poses_ = 10000;  

  // foot contact visualization
  std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> foot_contact_pub_;
  std::vector<visualization_msgs::msg::MarkerArray> foot_contact_markerarrays_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string world_frame_;
  std::string base_frame_;

  // Shared sensor state (guarded by sensor_mutex_) 
  std::mutex sensor_mutex_;

  // Buffered joint states for timestamp-matched FK
  std::deque<JointMsg::ConstSharedPtr> joint_queue_;
  static constexpr double kJointBufferSecs   = 0.5;
  static constexpr double kJointMaxStaleSecs = 0.05;

  bool   left_in_contact_       = false;
  bool   right_in_contact_      = false;
  double contact_force_threshold_ = 10.0;

  // Joint index lookup (built once from first JointState)
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;
  std::vector<int> left_joint_idx_;
  std::vector<int> right_joint_idx_;
  bool joint_map_ready_ = false;

  // Priority queue (event reordering, guarded by queue_mutex_) 
  using EventQueue = std::priority_queue<QueuedEvent,
                                         std::vector<QueuedEvent>,
                                         std::greater<QueuedEvent>>;
  EventQueue   eventQueue_;
  std::mutex   queueMutex_;
  uint64_t     nextSequence_           = 0;
  double       reorderDelaySeconds_    = 0.02;
  double       latestQueuedTimestampS_ = -std::numeric_limits<double>::infinity();
  double       lastDispatchedTimestampS_ = -std::numeric_limits<double>::infinity();
  rclcpp::TimerBase::SharedPtr flushTimer_;

  // Startup buffer: events collected before initialization 
  std::vector<LiveEvent> startupBuffer_;
  bool initialized_ = false;

  // Zero-order-hold IMU: the last consumed IMU sample.
  ImuSample held_imu_;
  bool      have_held_imu_ = false;

  bool   bias_initialized_        = false;
  int    bias_sample_count_       = 0;
  gtsam::Vector3 bias_sum_omega_  = gtsam::Vector3::Zero();
  gtsam::Vector3 bias_sum_accel_  = gtsam::Vector3::Zero();
  gtsam::Vector3 bias_sum2_omega_ = gtsam::Vector3::Zero();
  gtsam::Vector3 bias_sum2_accel_ = gtsam::Vector3::Zero();
  static constexpr int    kBiasSamples = 100;
  static constexpr double kMaxGyroStd  = 10.06;
  static constexpr double kMaxAccStd   = 16.5;

  double estimator_time_s_      = 0.0;
  double last_contact_update_s_ = -1e9;
  double max_dead_reckoning_s_  = 0.1;
  bool   disable_contact_       = false;
  bool   zero_accel_debug_      = false;
  bool   zero_gyro_debug_       = false;
  bool   contact_estimator_     = false;

  std::chrono::steady_clock::time_point start_time_;
  
};

}  
