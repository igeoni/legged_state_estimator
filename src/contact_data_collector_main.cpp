#include <rclcpp/rclcpp.hpp>
#include "legged_state_estimator/contact_data_collector.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<legged_state_estimator::ContactDataCollector>());
  rclcpp::shutdown();
  return 0;
}
