#include "nmea_serial_driver.hpp"
#include <libudev.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <functional>

using boost::asio::ip::udp;


ReachROSNode::ReachROSNode()
: Node("reach_ros_node"),
  io_context_(std::make_shared<boost::asio::io_context>()),
  serial_(*io_context_),
  udp_sock_(*io_context_)
{

  // declare parameters
  udp_port_ = declare_parameter<int>("udp_port", 9018);
  udp_port_ = get_parameter("udp_port").as_int();
  
  // Find serial port
  std::string port;
  while (rclcpp::ok()) {
    port = find_serial_device("FTDI");  // Emlid serial port connected through PPIM has vendor ID FTDI.
                                        // If connected directly over USB, this would be "Emlid"
    if (!port.empty()) {
      RCLCPP_INFO(get_logger(), "Emlid Reach GPS device found at %s", port.c_str());
      break;
    }
    RCLCPP_INFO(get_logger(), "Emlid Reach GPS device not found, retrying in 1s...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  int baud = declare_parameter("baud_rate", 38400);
  baud = get_parameter("baud_rate").as_int();

  try {
    serial_.open(port);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial %s: %s", port.c_str(), e.what());
    throw;
  }

  // open & bind UDP socket
  boost::system::error_code bec;
  udp_sock_.open(udp::v4(), bec);
  if (bec) {
    RCLCPP_ERROR(get_logger(), "UDP open failed: %s", bec.message().c_str());
    throw std::runtime_error("udp open failed");
  }
  udp_sock_.bind(udp::endpoint(udp::v4(), udp_port_), bec);
  if (bec) {
    RCLCPP_ERROR(get_logger(), "UDP bind on port %d failed: %s", udp_port_, bec.message().c_str());
    throw std::runtime_error("udp bind failed");
  }
  
}

ReachROSNode::~ReachROSNode() {
  io_context_->stop();
}

void ReachROSNode::init(const rclcpp::Node::SharedPtr &self) {
  driver_ = std::make_shared<RosNMEADriver>(self);

  start_udp_receive();

  // create ROS2 timer to poll Boost Asio
  asio_pump_timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                             std::bind(&ReachROSNode::asio_pump_tick, this));

  first_serial_read_call_ = true;

  start_serial_read();
}

void ReachROSNode::asio_pump_tick() {
  // non-blobking; processes any ready handlers from Boost Asio 
  
  try {
  io_context_->poll();
  } catch (...) { 
    RCLCPP_ERROR(get_logger(), "Some exception when io_context->poll()");
  }
}

std::string ReachROSNode::find_serial_device(const std::string &vendor_filter) {
  struct udev *udev = udev_new();
  if (!udev) {
    RCLCPP_ERROR(rclcpp::get_logger("reach_ros_node"), "Failed to init libudev");
    return "";
  }
  struct udev_enumerate *en = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(en, "tty");
  udev_enumerate_scan_devices(en);
  udev_list_entry *devs = udev_enumerate_get_list_entry(en), *ent;
  udev_list_entry_foreach(ent, devs) {
    const char *syspath = udev_list_entry_get_name(ent);
    udev_device *dev = udev_device_new_from_syspath(udev, syspath);
    const char *devnode = udev_device_get_devnode(dev);
    if (devnode) {
      if (const char *v = udev_device_get_property_value(dev, "ID_VENDOR")) {
        // std::cout << v << std::endl;
        if (vendor_filter == v) {
          std::string result(devnode);
          udev_device_unref(dev);
          udev_enumerate_unref(en);
          udev_unref(udev);
          return result;
        }
      }
    }
    udev_device_unref(dev);
  }
  udev_enumerate_unref(en);
  udev_unref(udev);
  return "";
}

void ReachROSNode::start_udp_receive() {
  try {
  udp_sock_.async_receive_from(
    boost::asio::buffer(udp_buffer_), sender_ep_,
    std::bind(&ReachROSNode::handle_udp_receive, this,
              std::placeholders::_1, std::placeholders::_2));
  } catch (...) { 
    RCLCPP_ERROR(get_logger(), "Some exception when start_udp_receive()");
  }
}

void ReachROSNode::handle_udp_receive(const boost::system::error_code &ec, std::size_t bytes) {
  if (!ec && bytes > 0) {
    // forward corrections to serial
   try {
    boost::asio::write(serial_, boost::asio::buffer(udp_buffer_.data(), bytes));
   } catch (...) { 
    RCLCPP_ERROR(get_logger(), "Some exception when writing to RTK serial port");
  }

  } else if (ec != boost::asio::error::operation_aborted) {
    RCLCPP_WARN(get_logger(), "udp receive error: %s", ec.message().c_str());
  }
  // queue next read regardless
  start_udp_receive();
}

void ReachROSNode::start_serial_read() {
  
  if (first_serial_read_call_) {
   RCLCPP_INFO(rclcpp::get_logger("reach_ros_node"),"Sleeping GPS fix publishing thread for 10 seconds to allow for RTK corrections to take effect...");
   std::this_thread::sleep_for(std::chrono::seconds(10));
   first_serial_read_call_ = false;
  }

  try {
  boost::asio::async_read_until(
    serial_, serial_buf_, '\n',
    std::bind(&ReachROSNode::handle_serial_read, this,
              std::placeholders::_1, std::placeholders::_2));
  } catch (...) { 
    RCLCPP_ERROR(get_logger(), "Some exception when async serial read");
  }

}

void ReachROSNode::handle_serial_read(const boost::system::error_code &ec, std::size_t) {
  if (!ec) {

    
    std::istream is(&serial_buf_);
    std::string line;
    std::getline(is, line);

    try {
    // strip any escape characters
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
      line.pop_back();
    }
   } catch (...) { 
    RCLCPP_ERROR(get_logger(), "Some exception when parsing the received line from serial");
    return;
  }


    // process it
  try {
    driver_->process_line(line);
  } catch (...) { 
    RCLCPP_ERROR(get_logger(), "Some exception when driver process line");
  }

  }
  start_serial_read();
}
