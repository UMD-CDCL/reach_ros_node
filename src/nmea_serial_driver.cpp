#include "nmea_serial_driver.hpp"
#include <libudev.h>
#include <chrono>
#include <cstring>
#include <iostream>

using boost::asio::ip::tcp;

void ReachROSNode::run_io(boost::asio::io_context *io) {
  io->run();
}

ReachROSNode::ReachROSNode()
: Node("reach_ros_node"),
  io_context_(std::make_shared<boost::asio::io_context>()),
  serial_(*io_context_),
  tcp_sock_(*io_context_),
  resolver_(*io_context_),
  reconnect_timer_(*io_context_)
{
  // Find serial port
  std::string port;
  while (rclcpp::ok()) {

    tcp_port_ = declare_parameter<int>("tcp_port", 9018);
    tcp_port_ = get_parameter("tcp_port").as_int();

    // host parameter for the TCP server (RTK Base Pi)
    tcp_host_ = declare_parameter<std::string>("tcp_host", "10.200.142.54");
    tcp_host_ = get_parameter("tcp_host").as_string();

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
  
}

ReachROSNode::~ReachROSNode() {
  io_context_->stop();
  for (auto &t : threads_) t.join();
}

void ReachROSNode::init(const rclcpp::Node::SharedPtr &self) {
  driver_ = std::make_shared<RosNMEADriver>(self);

  start_tcp_receive();

  threads_.reserve(2);
  for (int i = 0; i < 2; ++i) {
    threads_.emplace_back(run_io, 
                          io_context_.get());
  }

  first_serial_read_call_ = true;

  start_serial_read();
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

void ReachROSNode::start_tcp_receive() {
  // If not connected, resolve + connect; otherwise start reading
  if (!tcp_sock_.is_open()) {
    auto results = resolver_.resolve(tcp_host_, std::to_string(tcp_port_));
    boost::asio::async_connect(
      tcp_sock_, results,
      [this](const boost::system::error_code &ec, const tcp::endpoint &ep) {
        if (ec) {
          RCLCPP_ERROR(get_logger(), "tcp connect to %s:%d failed: %s",
                       tcp_host_.c_str(), tcp_port_, ec.message().c_str());
          schedule_reconnect();
          return;
        }
        RCLCPP_INFO(get_logger(), "tcp connected to %s", ep.address().to_string().c_str());

        // basic socket tuning (non-fatal if any fail)
        boost::system::error_code ec2;
        tcp_sock_.set_option(tcp::no_delay(true), ec2);

        // DSCP EF (46) -> TOS 184 so WMM can prioritize on Wi-Fi
        int fd = tcp_sock_.native_handle();
        int tos = 46 << 2;
        ::setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

        // Keepalive to detect dead links faster
        int ka = 1; ::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        int idle = 15, intvl = 5, cnt = 3;
        ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &idle, sizeof(idle));
        ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,  sizeof(cnt));

        // Now that we're connected, immediately begin reading
        start_tcp_receive();
      });
    return;
  }

  tcp_sock_.async_read_some(
    boost::asio::buffer(tcp_buffer_),
    std::bind(&ReachROSNode::handle_tcp_receive, this,
              std::placeholders::_1, std::placeholders::_2));
}

void ReachROSNode::handle_tcp_receive(const boost::system::error_code &ec, std::size_t bytes) {
  if (!ec && bytes > 0) {
    // std::cout << "sending " << bytes << " bytes" << std::endl;
    // forward corrections to serial
    boost::asio::write(serial_, boost::asio::buffer(tcp_buffer_.data(), bytes));
    // queue next read
    start_tcp_receive();
    return;
  }

  if (ec == boost::asio::error::operation_aborted) {
    return; 
  }

  RCLCPP_WARN(get_logger(), "tcp read error: %s", ec.message().c_str());
  boost::system::error_code ignored;
  tcp_sock_.close(ignored);
  schedule_reconnect();
}

void ReachROSNode::schedule_reconnect() {
  reconnect_timer_.expires_after(std::chrono::seconds(2));
  reconnect_timer_.async_wait([this](const boost::system::error_code &to_ec) {
    if (to_ec == boost::asio::error::operation_aborted) return;
    RCLCPP_INFO(get_logger(), "retrying tcp connect to %s:%d", tcp_host_.c_str(), tcp_port_);
    start_tcp_receive();
  });
}

void ReachROSNode::start_serial_read() {
  
  if (first_serial_read_call_) {
   RCLCPP_INFO(rclcpp::get_logger("reach_ros_node"),"Sleeping GPS fix publishing thread for 10 seconds to allow for RTK corrections to take effect...");
   std::this_thread::sleep_for(std::chrono::seconds(10));
   first_serial_read_call_ = false;
  }

  boost::asio::async_read_until(
    serial_, serial_buf_, '\n',
    std::bind(&ReachROSNode::handle_serial_read, this,
              std::placeholders::_1, std::placeholders::_2));
}

void ReachROSNode::handle_serial_read(const boost::system::error_code &ec, std::size_t) {
  if (!ec) {
    std::istream is(&serial_buf_);
    std::string line;
    std::getline(is, line);

    // strip any escape characters
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
      line.pop_back();
    }

    // process it
    driver_->process_line(line);
  }
  start_serial_read();
}
