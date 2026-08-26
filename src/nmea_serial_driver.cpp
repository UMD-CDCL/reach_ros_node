#include "nmea_serial_driver.hpp"
#include <libudev.h>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <functional>

using boost::asio::ip::udp;

// The port this driver reads, /dev/ttyGPSFIX, is a socat PTY on the far side of an FTDI
// adapter shared with the PPIM. Two distinct things can take it away:
//
//   * socat recycling the PTY, which returns as a *different* /dev/pts/N behind the same
//     symlink. Reopening by path handles that.
//   * the FTDI adapter leaving the USB bus, which takes /dev/ttyGPS with it. The splitter
//     then removes both PTY links, so this port does not merely break -- it ceases to
//     exist, sometimes for minutes. That is what the 2026-08-24 field bags show.
//
// Everything below is shaped by the second case: opening must be allowed to fail for a
// long time without the node dying noisily, spinning the CPU, or flooding the log.

ReachROSNode::ReachROSNode()
: Node("reach_ros_node"),
  io_context_(std::make_shared<boost::asio::io_context>()),
  serial_(*io_context_) {

  std::string serial_port = declare_parameter("serial_port", "/dev/ttyGPSFIX");
  serial_port = get_parameter("serial_port").as_string();

  baud_ = declare_parameter("baud_rate", 38400);
  baud_ = get_parameter("baud_rate").as_int();

  reopen_min_delay_ms_ = declare_parameter("reopen_min_delay_ms", 500);
  reopen_max_delay_ms_ = declare_parameter("reopen_max_delay_ms", 8000);
  reopen_timeout_s_ = declare_parameter("reopen_timeout_s", 120.0);
  reopen_delay_ms_ = reopen_min_delay_ms_;

  serial_dev_ = serial_port;  // remember for reopen

  // Deliberately does NOT throw when the port is missing.
  //
  // It used to. main() did not catch it, so an absent port meant an uncaught exception,
  // std::terminate and SIGABRT -- and because the launch file sets respawn:=true, that
  // became an abort loop every two seconds, one core dump per iteration, for as long as
  // the adapter was gone. Starting in the disconnected state and letting the ordinary
  // reopen path do its job is both quieter and the same amount of recovery.
  try {
    serial_.open(serial_dev_);
    configure_serial();
    RCLCPP_INFO(get_logger(), "serial opened: %s @ %d", serial_dev_.c_str(), baud_);
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(),
                "could not open %s at startup: %s - will keep trying",
                serial_dev_.c_str(), e.what());
    close_serial();
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

  if (serial_.is_open()) {
    start_serial_read();
  } else {
    schedule_serial_reopen_ms(reopen_delay_ms_);
  }
}

void ReachROSNode::configure_serial() {
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_));
  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));
  serial_.set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one));
  serial_.set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
}

void ReachROSNode::close_serial() {
  boost::system::error_code ec;
  serial_.cancel(ec);  // ignore errors; the descriptor may already be invalid
  serial_.close(ec);
  // Drop any partial sentence from the old session so the next read starts clean.
  serial_buf_.consume(serial_buf_.size());
}

void ReachROSNode::asio_pump_tick() {
  // non-blocking; processes any ready handlers from Boost Asio

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
    schedule_serial_reopen_ms(reopen_delay_ms_);
    return;
  }

  try {
    boost::asio::async_read_until(
      serial_, serial_buf_, '\n',
      std::bind(&ReachROSNode::handle_serial_read, this,
                std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Some exception when async serial read");
    close_serial();
    schedule_serial_reopen_ms(reopen_delay_ms_);
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

    // A good line means the link is alive again; clear any outage bookkeeping.
    if (in_failure_) {
      RCLCPP_INFO(get_logger(), "serial data flowing again on %s", serial_dev_.c_str());
      in_failure_ = false;
      reopen_delay_ms_ = reopen_min_delay_ms_;
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
        // Stop using this broken descriptor immediately. Closing here rather than inside
        // the reopen callback is what lets the reopen path drop its blocking sleep: by
        // the time the timer fires, the descriptor has been gone for a full backoff
        // interval already.
        close_serial();
        schedule_serial_reopen_ms(reopen_delay_ms_);
      }
      return;
  }
  start_serial_read();
}

void ReachROSNode::schedule_serial_reopen_ms(int ms) {
  if (serial_reopen_timer_) {
    return; // already scheduled
  }

  // Only announce the first attempt of an outage; report_reopen_failure() handles the
  // rest at a throttled rate.
  if (!in_failure_) {
    RCLCPP_WARN(get_logger(),
                "scheduling serial reopen in %d ms for %s",
                ms,
                serial_dev_.c_str());
  }

  serial_reopen_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(ms),
      std::bind(&ReachROSNode::do_serial_reopen, this));
}

void ReachROSNode::report_reopen_failure(const std::string &what) {
  const auto now = std::chrono::steady_clock::now();

  if (!in_failure_) {
    in_failure_ = true;
    failing_since_ = now;
    last_failure_log_ = now;
    RCLCPP_WARN(get_logger(),
                "serial reopen failed for %s: %s - retrying with backoff",
                serial_dev_.c_str(), what.c_str());
    return;
  }

  const auto since_log =
      std::chrono::duration_cast<std::chrono::seconds>(now - last_failure_log_).count();
  if (since_log >= 30) {
    last_failure_log_ = now;
    const auto down =
        std::chrono::duration_cast<std::chrono::seconds>(now - failing_since_).count();
    RCLCPP_WARN(get_logger(),
                "%s still unavailable after %lds (last error: %s)",
                serial_dev_.c_str(), static_cast<long>(down), what.c_str());
  }
}

void ReachROSNode::do_serial_reopen() {
  // One-shot: cancel explicitly rather than relying on the shared_ptr drop alone, so the
  // periodic wall timer cannot fire again while we are working.
  if (serial_reopen_timer_) {
    serial_reopen_timer_->cancel();
    serial_reopen_timer_.reset();
  }

  // if ROS is shutting down, don't bother reopening
  if (!rclcpp::ok()) {
    return;
  }

  // The old descriptor was closed the moment the error was seen, so there is nothing to
  // wait for here. This function previously slept 200ms inline, which blocked the whole
  // single-threaded executor -- including the 1ms asio pump -- on every retry.
  close_serial();

  try {
    serial_.open(serial_dev_);
    configure_serial();

    RCLCPP_INFO(get_logger(),
                "serial reopened: %s @ %d",
                serial_dev_.c_str(),
                baud_);

    in_failure_ = false;
    reopen_delay_ms_ = reopen_min_delay_ms_;

    if (io_context_->stopped()) {
      io_context_->restart();
    }

    // start reading again on the fresh descriptor
    start_serial_read();
    return;
  } catch (const std::exception &e) {
    report_reopen_failure(e.what());
    close_serial();
  }

  // Back off, so a port that is gone for minutes costs a handful of attempts rather than
  // one every two seconds.
  reopen_delay_ms_ = std::min(reopen_delay_ms_ * 2, reopen_max_delay_ms_);

  if (reopen_timeout_s_ > 0.0 && in_failure_) {
    const auto down = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now() - failing_since_).count();
    if (down >= reopen_timeout_s_) {
      RCLCPP_ERROR(get_logger(),
                   "%s has been unavailable for %.0fs; exiting so the launcher can "
                   "respawn this node",
                   serial_dev_.c_str(), down);
      wants_restart_ = true;
      rclcpp::shutdown();
      return;
    }
  }

  schedule_serial_reopen_ms(reopen_delay_ms_);
}
