#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "legged_state_estimator/kinematics/g1_kinematics.hpp"
#include "legged_state_estimator/data_collector/data_buffer.hpp"

namespace legged_state_estimator {

/**
 * ROS2 node that subscribes to:
 *   /imu                  sensor_msgs/Imu
 *   /joint_states         sensor_msgs/JointState
 *   /contact/left_foot    geometry_msgs/WrenchStamped
 *   /contact/right_foot   geometry_msgs/WrenchStamped
 *
 * Contact detection: force.z > contact_threshold_n (default 10 N)
 * Label encoding: 0=none, 1=right only, 2=left only, 3=both
 *
 * ROS2 parameters:
 *   urdf_path            (string)  path to URDF file
 *   output_dir           (string)  [default: "./dataset"]
 *   max_samples          (int)     [default: 10000]
 *   time_slop_ns         (int)     sync tolerance ns  [default: 50000000 = 50ms]
 *   contact_threshold_n  (double)  contact force threshold N  [default: 10.0]
 */
class ContactDataCollector : public rclcpp::Node {
 public:
  explicit ContactDataCollector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~ContactDataCollector() override = default;

 private:
  using ImuMsg     = sensor_msgs::msg::Imu;
  using JointMsg   = sensor_msgs::msg::JointState;
  using WrenchMsg  = geometry_msgs::msg::WrenchStamped;

  static const std::vector<std::string> kLegJointNames;

  void imuCallback(const ImuMsg::ConstSharedPtr& msg);
  void jointCallback(const JointMsg::ConstSharedPtr& msg);
  void leftContactCallback(const WrenchMsg::ConstSharedPtr& msg);
  void rightContactCallback(const WrenchMsg::ConstSharedPtr& msg);

  void tryProcess();

  bool extractLegJoints(const JointMsg& msg,
                        std::vector<double>& pos,
                        std::vector<double>& vel);
  void buildJointIndexMap(const JointMsg& msg);

  int32_t encodeContact(bool left_contact, bool right_contact) const;

  static int64_t stampNs(const builtin_interfaces::msg::Time& t) {
    return static_cast<int64_t>(t.sec) * 1'000'000'000LL + t.nanosec;
  }

  // -- parameters --
  std::string output_dir_;
  int         max_samples_;
  int64_t     time_slop_ns_;
  double      contact_threshold_n_;

  // -- sub-objects --
  std::unique_ptr<G1Kinematics> kinematics_;
  std::unique_ptr<DataBuffer>   buffer_;

  // -- subscribers --
  rclcpp::Subscription<ImuMsg>::SharedPtr    imu_sub_;
  rclcpp::Subscription<JointMsg>::SharedPtr  joint_sub_;
  rclcpp::Subscription<WrenchMsg>::SharedPtr left_contact_sub_;
  rclcpp::Subscription<WrenchMsg>::SharedPtr right_contact_sub_;

  // -- latest messages --
  std::mutex             mutex_;
  ImuMsg::ConstSharedPtr    latest_imu_;
  JointMsg::ConstSharedPtr  latest_joint_;
  WrenchMsg::ConstSharedPtr latest_left_contact_;
  WrenchMsg::ConstSharedPtr latest_right_contact_;

  std::map<std::string, std::size_t> joint_index_map_;
  bool joint_map_ready_ = false;
  bool collecting_      = true;
};

}  // namespace legged_state_estimator
