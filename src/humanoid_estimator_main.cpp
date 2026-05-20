#include <rclcpp/rclcpp.hpp>
#include "legged_state_estimator/humanoid_estimator_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<legged_state_estimator::HumanoidEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
