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
  void asio_pump_tick();
  void start_serial_read();
  void handle_serial_read(const boost::system::error_code &ec, std::size_t n);
  void schedule_serial_reopen_ms(int ms);
  void do_serial_reopen();

  std::shared_ptr<boost::asio::io_context> io_context_;
  
  boost::asio::serial_port serial_;
  boost::asio::streambuf serial_buf_;
  std::shared_ptr<RosNMEADriver> driver_;
  
  rclcpp::TimerBase::SharedPtr asio_pump_timer_;
  rclcpp::TimerBase::SharedPtr serial_reopen_timer_;
  
  std::string serial_dev_;
};

#endif  // REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_
