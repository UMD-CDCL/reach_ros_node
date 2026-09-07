#ifndef REACH_SERIAL_CPP__DRIVER_HPP_
#define REACH_SERIAL_CPP__DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "parser.hpp"
#include "checksum_utils.hpp"

class RosNMEADriver {
public:
  explicit RosNMEADriver(rclcpp::Node::SharedPtr node);
  void process_line(const std::string &line);

private:
  void parse_GGA(const ParsedSentence &ps);
  void parse_GST(const ParsedSentence &ps);
  void parse_VTG(const ParsedSentence &ps);
  void parse_RMC(const ParsedSentence &ps);
  void parse_time(const ParsedSentence &ps);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_low_cov_only_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr timeref_pub_;

  std::string frame_timeref_;
  std::string frame_gps_;
  bool use_rostime_;
  bool use_rmc_;

  bool has_fix_;
  bool has_std_;
  bool has_vel_;
  bool has_timeref_;

  // if this is set to true then fix/low_cov_only will contain all messages from /fix
  bool relax_gps_low_cov_requirement_;
  double low_cov_threshold_;
  bool require_rtk_fix_;
  int gga_quality_ = 0;  // raw GGA quality indicator: 4 = RTK fixed, 5 = RTK float

  sensor_msgs::msg::NavSatFix msg_fix_;
  geometry_msgs::msg::TwistStamped msg_vel_;
  sensor_msgs::msg::TimeReference msg_timeref_;
};

#endif  // REACH_SERIAL_CPP__DRIVER_HPP_