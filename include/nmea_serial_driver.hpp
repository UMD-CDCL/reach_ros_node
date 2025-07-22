#ifndef REACH_SERIAL_CPP__SERIAL_UDP_NODE_HPP_
#define REACH_SERIAL_CPP__SERIAL_UDP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include "driver.hpp"

class ReachROSNode
: public rclcpp::Node
, public std::enable_shared_from_this<ReachROSNode>
{
public:
  ReachROSNode();
  ~ReachROSNode();
  void init(const rclcpp::Node::SharedPtr &self);

private:
  static std::string find_serial_device(const std::string &vendor);
  static void run_io(boost::asio::io_context *io);

  void start_udp_receive();
  void handle_udp_receive(const boost::system::error_code &ec, std::size_t n);
  void start_serial_read();
  void handle_serial_read(const boost::system::error_code &ec, std::size_t n);
  
  bool first_serial_read_call_;
  std::shared_ptr<boost::asio::io_context> io_context_;
  boost::asio::serial_port serial_;
  boost::asio::ip::udp::socket udp_sock_;
  boost::asio::ip::udp::endpoint remote_ep_;
  std::array<char,1024> udp_buffer_;
  boost::asio::streambuf serial_buf_;
  std::vector<std::thread> threads_;
  std::shared_ptr<RosNMEADriver> driver_;
};

#endif  // REACH_SERIAL_CPP__SERIAL_UDP_NODE_HPP_
