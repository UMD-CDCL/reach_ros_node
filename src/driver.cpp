#include "driver.hpp"
#include <cmath>
#include <sstream>
#include <iostream>

RosNMEADriver::RosNMEADriver(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  has_fix_(false), has_std_(false), has_vel_(false), has_timeref_(false) {
  // Publishers
  fix_pub_     = node_->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
  fix_low_cov_only_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("fix/low_cov_only", 10);
  vel_pub_     = node_->create_publisher<geometry_msgs::msg::TwistStamped>("vel", 10);
  timeref_pub_ = node_->create_publisher<sensor_msgs::msg::TimeReference>("time", 10);

  // Parameters
  frame_timeref_ = node_->declare_parameter("frame_timeref", "gps_time");
  frame_gps_     = node_->declare_parameter("frame_gps", "gps");
  use_rmc_       = node_->declare_parameter("use_rmc", false);
 
  relax_gps_low_cov_requirement_ = node_->declare_parameter("relax_gps_low_cov_requirement", false);
  relax_gps_low_cov_requirement_ = node_->get_parameter("relax_gps_low_cov_requirement").as_bool();

  low_cov_threshold_ = node_->declare_parameter("low_cov_topic_threshold", 0.1);
  low_cov_threshold_ = node_->get_parameter("low_cov_topic_threshold").as_double();

  // Float and standalone solutions carry multi-metre biases while reporting sub-metre
  // covariance, so covariance alone is not a safe gate. Require an RTK-fixed status too.
  require_rtk_fix_ = node_->declare_parameter("require_rtk_fix", true);

  // Initialize blank messages
  msg_fix_.position_covariance_type = 
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

void RosNMEADriver::process_line(const std::string &line) {

  // std::cout << line << std::endl;

  // Validate checksum
  if (!check_nmea_checksum(line)) {
    RCLCPP_WARN_SKIPFIRST(node_->get_logger(),
      "Invalid checksum: '%s'", line.c_str());
    return;
  }

  ParsedSentence ps = parse_nmea_sentence(line);

  if (ps.type.empty()) {
    RCLCPP_WARN(node_->get_logger(),
      "Failed to parse NMEA sentence: '%s'", line.c_str());
    return;
  }

  // make sure some of the fields in the parsed sentence are non-empty 
  // fields remain empty when GPS does not have a fix
  bool all_empty = true;
  for (const auto &entry : ps.fields) {
      if (!entry.empty()) {
          all_empty = false;
          break;
      }
  }

  if(all_empty) {
    RCLCPP_WARN(node_->get_logger(), "GPS likely does not have a fix. Received empty NMEA sentence from GPS module.");
    return;
  }

  // parse each sentence type further
  parse_GGA(ps);
  parse_GST(ps);
  parse_VTG(ps);
  // parse_RMC(ps);
  parse_time(ps);
  
  // Publish as ready
  if (has_fix_ && has_std_) {
    fix_pub_->publish(msg_fix_);

    // republish on the gated topic only when the solution is RTK-fixed (if required) and its
    // covariance is below threshold
    // NavSatStatus cannot tell RTK float from RTK fixed (GGA quality 4 and 5 both map to
    // STATUS_GBAS_FIX above), so gate on the raw quality indicator: 4 is RTK fixed only.
    const bool rtk_ok = !require_rtk_fix_ || gga_quality_ == 4;
    if ((rtk_ok &&
        msg_fix_.position_covariance[0] < low_cov_threshold_ &&
        msg_fix_.position_covariance[4] < low_cov_threshold_) ||
        relax_gps_low_cov_requirement_) {

      fix_low_cov_only_pub_->publish(msg_fix_);

    }

    msg_fix_ = sensor_msgs::msg::NavSatFix();
    msg_fix_.position_covariance_type = 
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    has_fix_ = has_std_ = false;
  }
  if (has_vel_) {
    vel_pub_->publish(msg_vel_);
    msg_vel_ = geometry_msgs::msg::TwistStamped();
    has_vel_ = false;
  }
  if (has_timeref_) {
    timeref_pub_->publish(msg_timeref_);
    msg_timeref_ = sensor_msgs::msg::TimeReference();
    has_timeref_ = false;
  }
}

// Helper: convert ddmm.mmmm + dir to decimal degrees
static double convert_deg(const std::string &raw, char dir) {
  double val = std::stod(raw);
  int d = int(val / 100);
  double m = val - d*100;
  double deg = d + m/60.0;
  return (dir=='S' || dir=='W') ? -deg : deg;
}

void RosNMEADriver::parse_GGA(const ParsedSentence &ps) {
  if (ps.type != "GGA" || use_rmc_) return;
  auto &f = ps.fields;
  if (f.size() < 11 ||
      f[1].empty() ||
      f[2].empty() ||
      f[3].empty() ||
      f[4].empty() ||
      f[5].empty() ||
      f[8].empty() ||
      f[10].empty()) {
        return;
  } 

  // std::cout << "GGA fields.size() = " << f.size() << std::endl;

  // for(auto& t : f) {
  //   std::cout << t << std::endl;
  // }

  // Header
  msg_fix_.header.stamp = node_->get_clock()->now();
  msg_fix_.header.frame_id = frame_gps_;

  // Status
  int qual = std::stoi(f[5]);
  gga_quality_ = qual;
  using Status = sensor_msgs::msg::NavSatStatus;
  switch (qual) {
    case 0: msg_fix_.status.status = Status::STATUS_NO_FIX; break;
    case 1: msg_fix_.status.status = Status::STATUS_FIX;    break;
    case 2: msg_fix_.status.status = Status::STATUS_SBAS_FIX; break;
    case 4:
    case 5: msg_fix_.status.status = Status::STATUS_GBAS_FIX; break;
    default: msg_fix_.status.status = Status::STATUS_NO_FIX;
  }
  msg_fix_.status.service = Status::SERVICE_GPS;

  // Position
  msg_fix_.latitude  = convert_deg(f[1], f[2][0]);
  msg_fix_.longitude = convert_deg(f[3], f[4][0]);

  // Altitude + geoid offset
  double alt    = std::stod(f[8]);
  double geoid = std::stod(f[10]);
  msg_fix_.altitude = alt + geoid;

  has_fix_ = true;
}

void RosNMEADriver::parse_GST(const ParsedSentence &ps) {
  if (ps.type != "GST" || use_rmc_) return;
  auto &f = ps.fields;
  if (f.size() < 8 ||
      f[5].empty() ||
      f[6].empty() ||
      f[7].empty()) {
        return;
  } 

  // std deviations
  double s_lat = std::stod(f[5]);
  double s_lon = std::stod(f[6]);
  double s_alt = std::stod(f[7]);
  msg_fix_.position_covariance[0] = s_lat*s_lat;
  msg_fix_.position_covariance[4] = s_lon*s_lon;
  msg_fix_.position_covariance[8] = s_alt*s_alt;
  msg_fix_.position_covariance_type = 
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

  has_std_ = true;
}

void RosNMEADriver::parse_VTG(const ParsedSentence &ps) {
  if (ps.type != "VTG" || use_rmc_) return;
  auto &f = ps.fields;
  if (f.size() < 7 ||
      f[4].empty() ||
      f[0].empty()) {
        return;
  } 

  msg_vel_.header.stamp = node_->get_clock()->now();
  msg_vel_.header.frame_id = frame_gps_;

  // VTG field 5 (f[4] once the header is stripped) is speed over ground in knots. It was being
  // published unconverted, which is why gps/vel read 1.94x Spot's own speed in every bag.
  double speed = std::stod(f[4]) * 0.514444;  // knots -> m/s
  double course = std::stod(f[0]);   // true track
  msg_vel_.twist.linear.x = speed * std::sin(course * M_PI/180.0);
  msg_vel_.twist.linear.y = speed * std::cos(course * M_PI/180.0);

  has_vel_ = true;
}

// void RosNMEADriver::parse_RMC(const ParsedSentence &ps) {
//   if (ps.type != "RMC" || !use_rmc_) return;
//   auto &f = ps.fields;
//   if (f.size() < 9) return;

//   // std::cout << "RMC fields.size() = " << f.size() << std::endl;

//   // for(auto& t : f) {
//   //   std::cout << t << std::endl;
//   // }

//   msg_fix_.header.stamp = node_->get_clock()->now();
//   msg_fix_.header.frame_id = frame_gps_;

//   bool valid = (f[1] == "A");
//   msg_fix_.status.status  = valid
//     ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
//     : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
//   msg_fix_.status.service = 
//     sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

//   msg_fix_.latitude  = convert_deg(f[2], f[3][0]);
//   msg_fix_.longitude = convert_deg(f[3], f[5][0]);

//   msg_fix_.altitude = NAN;
//   msg_fix_.position_covariance_type = 
//     sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

//   has_fix_ = true;
//   has_std_ = true;

//   msg_vel_.header.stamp = msg_fix_.header.stamp;
//   msg_vel_.header.frame_id = frame_gps_;
//   double speed = std::stod(f[4]);    // speed over ground
//   double course = std::stod(f[5]);   // true course
//   msg_vel_.twist.linear.x = speed * std::sin(course * M_PI/180.0);
//   msg_vel_.twist.linear.y = speed * std::cos(course * M_PI/180.0);
//   has_vel_ = true;
// }

void RosNMEADriver::parse_time(const ParsedSentence &ps) {
  const std::string *utc_field = nullptr;
  if (!use_rmc_ && ps.type == "GGA") {
    utc_field = &ps.fields[0];
  } else if (use_rmc_ && ps.type == "RMC") {
    utc_field = &ps.fields[0];
  } else {
    return;
  }
  if (utc_field->empty()) return;

  // Convert hhmmss.ss to seconds since midnight
  double t = std::stod(*utc_field);
  int hh = int(t/10000);
  int mm = int((t - hh*10000)/100);
  double ss = t - hh*10000 - mm*100;

  // Fill TimeReference
  msg_timeref_.header.stamp = node_->get_clock()->now();

  msg_timeref_.header.frame_id = frame_timeref_;

  builtin_interfaces::msg::Time br;
  br.sec  = hh*3600 + mm*60 + int(ss);
  br.nanosec = int((ss - int(ss)) * 1e9);
  msg_timeref_.time_ref = br;
  msg_timeref_.source   = frame_timeref_;

  has_timeref_ = true;
}
