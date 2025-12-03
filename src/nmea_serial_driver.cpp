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
  serial_(*io_context_) {
    
  std::string serial_port = declare_parameter("serial_port", "/dev/ttyGPSFIX");
  serial_port = get_parameter("serial_port").as_string();

  int baud = declare_parameter("baud_rate", 38400);
  baud = get_parameter("baud_rate").as_int();

  try {
    serial_.open(serial_port);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
    
    serial_dev_ = serial_port;  // remember for reopen
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial %s: %s", serial_port.c_str(), e.what());
    throw;
  }
  
}

ReachROSNode::~ReachROSNode() {
  io_context_->stop();
}

void ReachROSNode::init(const rclcpp::Node::SharedPtr &self) {
  driver_ = std::make_shared<RosNMEADriver>(self);

  // create ROS2 timer to poll Boost Asio
  asio_pump_timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                             std::bind(&ReachROSNode::asio_pump_tick, this));

  start_serial_read();
}

void ReachROSNode::asio_pump_tick() {
  // non-blobking; processes any ready handlers from Boost Asio 
  
  try {
    if (io_context_->stopped()) {
      io_context_->restart();
    }
    io_context_->poll();
  } catch (...) { 
    RCLCPP_ERROR(get_logger(), "Some exception when io_context->poll()");
  }
}


void ReachROSNode::start_serial_read() {

  if (!serial_.is_open()) {
    RCLCPP_WARN(get_logger(), "start_serial_read called but serial is not open");
    schedule_serial_reopen_ms(2000);
    return;
  }
  
  try {
    boost::asio::async_read_until(
      serial_, serial_buf_, '\n',
      std::bind(&ReachROSNode::handle_serial_read, this,
                std::placeholders::_1, std::placeholders::_2));
  } catch (...) { 
    RCLCPP_ERROR(get_logger(), "Some exception when async serial read");
    schedule_serial_reopen_ms(2000);
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
      start_serial_read();
      return;
    }
    // process it
    try {
      driver_->process_line(line);
    } catch (...) { 
      RCLCPP_ERROR(get_logger(), "Some exception when driver process line");
    }

  } else {
      if (ec == boost::asio::error::operation_aborted) {
        // cancelled due to close()/cancel() during shutdown or reopen
        RCLCPP_DEBUG(get_logger(),
                    "serial read cancelled (operation_aborted)");
      } else {
        RCLCPP_WARN(get_logger(),
                    "serial read error on %s: %s (%d)",
                    serial_dev_.c_str(),
                    ec.message().c_str(),
                    static_cast<int>(ec.value()));
        // stop using this broken descriptor; try to reopen later
        schedule_serial_reopen_ms(2000);
      }
      return;  
  }
  start_serial_read();
}

void ReachROSNode::schedule_serial_reopen_ms(int ms) {
  if (serial_reopen_timer_) {
    return; // already scheduled
  }
  RCLCPP_WARN(get_logger(),
              "scheduling serial reopen in %d ms for %s",
              ms,
              serial_dev_.c_str());

  serial_reopen_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(ms),
      std::bind(&ReachROSNode::do_serial_reopen, this));
}


void ReachROSNode::do_serial_reopen() {
  serial_reopen_timer_.reset(); // one-shot

  // close if open; cancel any pending operations on the old fd
  boost::system::error_code ec;
  serial_.cancel(ec);  // ignore errors
  serial_.close(ec);

  // if ROS is shutting down, don't bother reopening
  if (!rclcpp::ok()) {
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200)); 

  try {
    serial_.open(serial_dev_);
    const int baud = get_parameter("baud_rate").as_int();
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));

    RCLCPP_INFO(get_logger(),
                "serial reopened: %s @ %d",
                serial_dev_.c_str(),
                baud);
    
    // drop any partial junk from the old session
    serial_buf_.consume(serial_buf_.size());

    if (io_context_->stopped()) {
      io_context_->restart();
    }

    // start reading again on the fresh descriptor
    start_serial_read();
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(),
                "serial reopen failed for %s: %s — retrying",
                serial_dev_.c_str(),
                e.what());
    schedule_serial_reopen_ms(2000);
  }
}
