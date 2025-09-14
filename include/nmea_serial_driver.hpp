#ifndef REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_
#define REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <array>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>
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
  
  void asio_pump_tick();
  void start_udp_receive();
  void handle_udp_receive(const boost::system::error_code &ec, std::size_t n);
  void start_serial_read();
  void handle_serial_read(const boost::system::error_code &ec, std::size_t n);
  void schedule_serial_reopen_ms(int ms);
  void do_serial_reopen();

  bool first_serial_read_call_{true};
  std::shared_ptr<boost::asio::io_context> io_context_;
  
  boost::asio::serial_port serial_;
  boost::asio::streambuf serial_buf_;
  std::shared_ptr<RosNMEADriver> driver_;

  boost::asio::ip::udp::socket udp_sock_;
  boost::asio::ip::udp::endpoint sender_ep_;
  std::array<char, 1024> udp_buffer_;

  
  rclcpp::TimerBase::SharedPtr asio_pump_timer_;
  rclcpp::TimerBase::SharedPtr serial_reopen_timer_;
  
  // connection params
  int udp_port_;
  std::string serial_dev_;
};

#endif  // REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_
