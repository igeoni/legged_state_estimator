#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "legged_state_estimator/g1_kinematics.hpp"

namespace legged_state_estimator {

/**
 * Subscribes to /joint_states and publishes FK foot/knee positions
 * as MarkerArray spheres for RViz2 verification.
 *
 * Topics published:
 *   /fk_markers         visualization_msgs/MarkerArray
 *   /robot_description  std_msgs/String  (transient_local, latched)
 *
 * ROS2 parameters:
 *   urdf_path   (string)  path to URDF file  [required]
 *   base_frame  (string)  frame_id for markers  [default: "pelvis"]
 */
class FkVisualizer : public rclcpp::Node {
 public:
  explicit FkVisualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 private:
  using JointMsg    = sensor_msgs::msg::JointState;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Marker      = visualization_msgs::msg::Marker;
  using StringMsg   = std_msgs::msg::String;

  static const std::vector<std::string> kLegJointNames;

  void jointCallback(const JointMsg::ConstSharedPtr& msg);
  void buildJointIndexMap(const JointMsg& msg);
  bool extractLegJoints(const JointMsg& msg,
                        std::vector<double>& pos,
                        std::vector<double>& vel);

  Marker makeSphere(int id, const Eigen::Vector3d& pos,
                    float r, float g, float b,
                    float radius,
                    const std::string& ns) const;

  std::string  base_frame_;

  std::unique_ptr<G1Kinematics> kinematics_;

  rclcpp::Subscription<JointMsg>::SharedPtr  joint_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr  marker_pub_;
  rclcpp::Publisher<StringMsg>::SharedPtr    robot_desc_pub_;

  std::map<std::string, std::size_t> joint_index_map_;
  bool joint_map_ready_ = false;
  int  debug_count_ = 0;
};

}  // namespace legged_state_estimator
