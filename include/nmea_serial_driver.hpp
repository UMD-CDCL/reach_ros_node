#ifndef REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_
#define REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
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
  void start_tcp_receive();
  void handle_tcp_receive(const boost::system::error_code &ec, std::size_t n);
  void start_serial_read();
  void handle_serial_read(const boost::system::error_code &ec, std::size_t n);
  void schedule_reconnect();

  bool first_serial_read_call_{true};
  std::shared_ptr<boost::asio::io_context> io_context_;
  boost::asio::serial_port serial_;

  boost::asio::ip::tcp::socket tcp_sock_;
  boost::asio::ip::tcp::resolver resolver_;
  boost::asio::steady_timer reconnect_timer_;

  std::array<char, 1024> tcp_buffer_;

  boost::asio::streambuf serial_buf_;
  std::shared_ptr<RosNMEADriver> driver_;

  rclcpp::TimerBase::SharedPtr asio_pump_timer_;
  
  // connection params
  std::string tcp_host_;
  int tcp_port_;
};

#endif  // REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_
