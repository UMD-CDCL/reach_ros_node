#include <rclcpp/rclcpp.hpp>
#include "nmea_serial_driver.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReachROSNode>();
  node->init(node);
  rclcpp::spin(node);

  // A non-zero exit tells the launcher this was a fault, not a clean shutdown. The node
  // asks for this when the serial port has been gone long enough that restarting the
  // process is the only thing left to try; launch respawns it after respawn_delay.
  const bool restart = node->wants_restart();
  node.reset();
  rclcpp::shutdown();
  return restart ? 1 : 0;
}
