#include "nmea_serial_driver.hpp"
#include <libudev.h>
#include <chrono>

using boost::asio::ip::udp;

void ReachROSNode::run_io(boost::asio::io_context *io) {
  io->run();
}

ReachROSNode::ReachROSNode()
: Node("reach_ros_node"),
  io_context_(std::make_shared<boost::asio::io_context>()),
  serial_(*io_context_),
  udp_sock_(*io_context_, udp::endpoint(udp::v4(), declare_parameter("udp_port", 9008)))
{
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

  int baud = declare_parameter("baud_rate", 115200);

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

  start_udp_receive();
  start_serial_read();

  threads_.reserve(2);
  for (int i = 0; i < 2; ++i) {
    threads_.emplace_back(run_io, 
                          io_context_.get());
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
  udp_sock_.async_receive_from(
    boost::asio::buffer(udp_buffer_), remote_ep_,
    std::bind(&ReachROSNode::handle_udp_receive, this,
              std::placeholders::_1, std::placeholders::_2));
}

void ReachROSNode::handle_udp_receive(const boost::system::error_code &ec, std::size_t bytes) {
  if (!ec && bytes > 0) {
    // std::lock_guard<std::mutex> lock(serial_mtx_);
    boost::asio::write(serial_, boost::asio::buffer(udp_buffer_.data(), bytes));
  }
  start_udp_receive();
}

void ReachROSNode::start_serial_read() {
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
