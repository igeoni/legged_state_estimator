#include <rclcpp/rclcpp.hpp>
#include "legged_state_estimator/visualizer/fk_visualizer.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<legged_state_estimator::FkVisualizer>());
  rclcpp::shutdown();
  return 0;
}
