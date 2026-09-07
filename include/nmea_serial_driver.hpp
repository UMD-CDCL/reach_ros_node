#ifndef REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_
#define REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <array>
#include <chrono>
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

  // True when the node has given up on the port and wants the process to exit so the
  // launcher can respawn it. See reopen_timeout_s_.
  bool wants_restart() const { return wants_restart_; }

private:
  void asio_pump_tick();
  void start_serial_read();
  void handle_serial_read(const boost::system::error_code &ec, std::size_t n);
  void schedule_serial_reopen_ms(int ms);
  void do_serial_reopen();

  // Close and cancel the current descriptor. Safe to call when already closed.
  void close_serial();
  // Apply baud and 8N1 raw framing to a freshly opened port.
  void configure_serial();
  // Throttled reporting for a fault that can last minutes.
  void report_reopen_failure(const std::string &what);

  std::shared_ptr<boost::asio::io_context> io_context_;

  boost::asio::serial_port serial_;
  boost::asio::streambuf serial_buf_;
  std::shared_ptr<RosNMEADriver> driver_;

  rclcpp::TimerBase::SharedPtr asio_pump_timer_;
  rclcpp::TimerBase::SharedPtr serial_reopen_timer_;

  std::string serial_dev_;
  int baud_ = 38400;

  // ---- reopen backoff -------------------------------------------------------
  // The port can be absent for minutes when the FTDI adapter drops off the USB bus, so
  // retries back off and their logging is throttled. The previous version retried at a
  // flat 2s and logged every attempt, which produced 50 identical lines per outage in
  // the 2026-08-24 field bags.
  int reopen_delay_ms_ = 500;
  int reopen_min_delay_ms_ = 500;
  int reopen_max_delay_ms_ = 8000;

  // Wall-clock bookkeeping for the outage, used for backoff and for the restart decision.
  std::chrono::steady_clock::time_point failing_since_{};
  std::chrono::steady_clock::time_point last_failure_log_{};
  bool in_failure_ = false;

  // Give up and let the launcher respawn us after this long with no port. Zero disables.
  // The package README calls this out as "Gate 3": a driver that neither recovers nor
  // exits cannot be rescued by a restart policy, because nothing ever exits. The launch
  // file already sets respawn:=true, so exiting is what closes that loop.
  double reopen_timeout_s_ = 120.0;
  bool wants_restart_ = false;
};

#endif  // REACH_SERIAL_CPP__SERIAL_TCP_NODE_HPP_
