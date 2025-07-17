#include <rclcpp/rclcpp.hpp>
#include "nmea_serial_driver.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReachROSNode>();
  node->init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}