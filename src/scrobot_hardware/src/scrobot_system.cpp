#include "scrobot_hardware/scrobot_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace scrobot_hardware
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;

std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ','))
  {
    fields.push_back(field);
  }
  return fields;
}

speed_t baud_to_termios(int baud)
{
  switch (baud)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
    default:
      return static_cast<speed_t>(0);
  }
}
}  // namespace

ScrobotSystemHardware::~ScrobotSystemHardware()
{
  close_serial();
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto get_param = [this](const std::string & key, const std::string & fallback) {
      const auto it = info_.hardware_parameters.find(key);
      return it == info_.hardware_parameters.end() ? fallback : it->second;
    };

  try
  {
    serial_port_ = get_param("serial_port", serial_port_);
    baud_rate_ = std::stoi(get_param("baud_rate", std::to_string(baud_rate_)));
    counts_per_wheel_rev_ =
      std::stod(get_param("counts_per_wheel_rev", std::to_string(counts_per_wheel_rev_)));
    left_command_sign_ =
      std::stod(get_param("left_command_sign", std::to_string(left_command_sign_)));
    right_command_sign_ =
      std::stod(get_param("right_command_sign", std::to_string(right_command_sign_)));
    left_state_sign_ =
      std::stod(get_param("left_state_sign", std::to_string(left_state_sign_)));
    right_state_sign_ =
      std::stod(get_param("right_state_sign", std::to_string(right_state_sign_)));
    max_wheel_rpm_ = std::stod(get_param("max_wheel_rpm", std::to_string(max_wheel_rpm_)));
    max_brush_rpm_ = std::stod(get_param("max_brush_rpm", std::to_string(max_brush_rpm_)));
    max_conveyor_rpm_ =
      std::stod(get_param("max_conveyor_rpm", std::to_string(max_conveyor_rpm_)));
    state_timeout_ms_ =
      std::stoi(get_param("state_timeout_ms", std::to_string(state_timeout_ms_)));
    collector_timeout_ms_ =
      std::stoi(get_param("collector_timeout_ms", std::to_string(collector_timeout_ms_)));
  }
  catch (const std::exception & ex)
  {
    RCLCPP_FATAL(get_logger(), "Invalid SC Robot hardware parameter: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (counts_per_wheel_rev_ <= 0.0)
  {
    RCLCPP_FATAL(get_logger(), "counts_per_wheel_rev must be positive");
    return hardware_interface::CallbackReturn::ERROR;
  }

  bool have_left = false;
  bool have_right = false;
  for (const auto & joint : info_.joints)
  {
    if (joint.name == "left_wheel_joint")
    {
      have_left = true;
    }
    if (joint.name == "right_wheel_joint")
    {
      have_right = true;
    }

    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' must expose one velocity command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' must expose position and velocity state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!have_left || !have_right || info_.joints.size() != 2)
  {
    RCLCPP_FATAL(
      get_logger(),
      "SC Robot hardware expects exactly left_wheel_joint and right_wheel_joint");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & [name, description] : joint_state_interfaces_)
  {
    (void)description;
    set_state(name, 0.0);
  }
  for (const auto & [name, description] : joint_command_interfaces_)
  {
    (void)description;
    set_command(name, 0.0);
  }

  if (get_node())
  {
    collector_sub_ = get_node()->create_subscription<scrobot_interfaces::msg::CollectorCommand>(
      "/hardware/collector_command",
      rclcpp::QoS(1).reliable(),
      [this](const scrobot_interfaces::msg::CollectorCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(collector_mutex_);
        collector_command_ = *msg;
        last_collector_command_ = std::chrono::steady_clock::now();
      });

    status_pub_ = get_node()->create_publisher<scrobot_interfaces::msg::HardwareStatus>(
      "/hardware/status", rclcpp::QoS(10).reliable());
  }

  connected_ = false;
  estop_ = false;
  fault_code_ = 0;
  tx_sequence_ = 0;
  rx_sequence_ = 0;
  rx_buffer_.clear();
  last_state_ = std::chrono::steady_clock::now();
  last_status_publish_ = last_state_;

  RCLCPP_INFO(
    get_logger(),
    "Configured STM32 serial interface on %s at %d baud, %.1f counts/wheel-rev",
    serial_port_.c_str(), baud_rate_, counts_per_wheel_rev_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  collector_sub_.reset();
  status_pub_.reset();
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_ = true;
  last_state_ = std::chrono::steady_clock::now();
  send_command(0.0, 0.0, 0.0, 0.0, 0.0, false, false);
  RCLCPP_INFO(get_logger(), "SC Robot hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  send_command(0.0, 0.0, 0.0, 0.0, 0.0, false, false);
  RCLCPP_INFO(get_logger(), "SC Robot hardware deactivated; motor enables cleared");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ScrobotSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  char buffer[512];
  while (true)
  {
    const ssize_t count = ::read(serial_fd_, buffer, sizeof(buffer));
    if (count > 0)
    {
      rx_buffer_.append(buffer, static_cast<size_t>(count));
      continue;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK)
    {
      break;
    }

    RCLCPP_ERROR(get_logger(), "Serial read failed: %s", std::strerror(errno));
    connected_ = false;
    publish_status(true);
    return hardware_interface::return_type::ERROR;
  }

  size_t newline = std::string::npos;
  while ((newline = rx_buffer_.find('\n')) != std::string::npos)
  {
    std::string line = rx_buffer_.substr(0, newline);
    rx_buffer_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    if (!line.empty())
    {
      parse_state_line(line);
    }
  }

  if (rx_buffer_.size() > 4096)
  {
    RCLCPP_WARN(get_logger(), "Discarding oversized serial receive buffer");
    rx_buffer_.clear();
  }

  const auto now = std::chrono::steady_clock::now();
  const auto age_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_state_).count();

  if (active_ && age_ms > state_timeout_ms_)
  {
    if (connected_)
    {
      RCLCPP_ERROR(
        get_logger(), "STM32 feedback timeout: no valid STATE frame for %ld ms",
        static_cast<long>(age_ms));
    }
    connected_ = false;
    publish_status(true);
    return hardware_interface::return_type::ERROR;
  }

  publish_status(false);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ScrobotSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  double left_rpm =
    rad_s_to_rpm(get_command("left_wheel_joint/velocity")) * left_command_sign_;
  double right_rpm =
    rad_s_to_rpm(get_command("right_wheel_joint/velocity")) * right_command_sign_;

  if (!std::isfinite(left_rpm))
  {
    left_rpm = 0.0;
  }
  if (!std::isfinite(right_rpm))
  {
    right_rpm = 0.0;
  }

  left_rpm = std::clamp(left_rpm, -max_wheel_rpm_, max_wheel_rpm_);
  right_rpm = std::clamp(right_rpm, -max_wheel_rpm_, max_wheel_rpm_);

  double brush_left_rpm = 0.0;
  double brush_right_rpm = 0.0;
  double conveyor_rpm = 0.0;
  bool collector_enable = false;
  bool collector_fresh = false;

  {
    std::lock_guard<std::mutex> lock(collector_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (last_collector_command_.time_since_epoch().count() != 0)
    {
      const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_collector_command_).count();
      collector_fresh = age_ms <= collector_timeout_ms_;
    }

    if (collector_fresh && collector_command_.enable)
    {
      brush_left_rpm =
        std::clamp(collector_command_.brush_left_rpm, -max_brush_rpm_, max_brush_rpm_);
      brush_right_rpm =
        std::clamp(collector_command_.brush_right_rpm, -max_brush_rpm_, max_brush_rpm_);
      conveyor_rpm =
        std::clamp(collector_command_.conveyor_rpm, -max_conveyor_rpm_, max_conveyor_rpm_);
      collector_enable = true;
    }
  }

  bool drive_enable = active_;
  if (estop_ || fault_code_ != 0)
  {
    left_rpm = 0.0;
    right_rpm = 0.0;
    brush_left_rpm = 0.0;
    brush_right_rpm = 0.0;
    conveyor_rpm = 0.0;
    drive_enable = false;
    collector_enable = false;
  }

  if (!send_command(
      left_rpm, right_rpm, brush_left_rpm, brush_right_rpm, conveyor_rpm,
      drive_enable, collector_enable))
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool ScrobotSystemHardware::open_serial()
{
  close_serial();

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      get_logger(), "Cannot open %s: %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  if (!configure_serial(baud_rate_))
  {
    close_serial();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void ScrobotSystemHardware::close_serial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ScrobotSystemHardware::configure_serial(int baud_rate)
{
  const speed_t speed = baud_to_termios(baud_rate);
  if (speed == static_cast<speed_t>(0))
  {
    RCLCPP_ERROR(get_logger(), "Unsupported baud rate: %d", baud_rate);
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
    return false;
  }

  return true;
}

bool ScrobotSystemHardware::send_command(
  double left_rpm,
  double right_rpm,
  double brush_left_rpm,
  double brush_right_rpm,
  double conveyor_rpm,
  bool drive_enable,
  bool collector_enable)
{
  if (serial_fd_ < 0)
  {
    return false;
  }

  std::ostringstream stream;
  stream << "CMD," << tx_sequence_++ << ','
         << std::fixed << std::setprecision(3)
         << left_rpm << ',' << right_rpm << ','
         << brush_left_rpm << ',' << brush_right_rpm << ',' << conveyor_rpm << ','
         << (drive_enable ? 1 : 0) << ',' << (collector_enable ? 1 : 0) << "\n";
  const std::string payload = stream.str();

  size_t sent = 0;
  while (sent < payload.size())
  {
    const ssize_t count =
      ::write(serial_fd_, payload.data() + sent, payload.size() - sent);
    if (count > 0)
    {
      sent += static_cast<size_t>(count);
      continue;
    }

    if (count < 0 && errno == EINTR)
    {
      continue;
    }

    RCLCPP_ERROR(get_logger(), "Serial write failed: %s", std::strerror(errno));
    return false;
  }

  return true;
}

bool ScrobotSystemHardware::parse_state_line(const std::string & line)
{
  const auto fields = split_csv(line);
  if (fields.size() != 8 || fields[0] != "STATE")
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Ignoring malformed STM32 frame: '%s'",
      line.c_str());
    return false;
  }

  try
  {
    const uint32_t sequence = static_cast<uint32_t>(std::stoul(fields[1]));
    const int64_t left_count = std::stoll(fields[2]);
    const int64_t right_count = std::stoll(fields[3]);
    const double left_rpm = std::stod(fields[4]);
    const double right_rpm = std::stod(fields[5]);
    const bool estop = std::stoi(fields[6]) != 0;
    const uint32_t fault = static_cast<uint32_t>(std::stoul(fields[7]));

    const double left_position =
      left_state_sign_ * static_cast<double>(left_count) * kTwoPi / counts_per_wheel_rev_;
    const double right_position =
      right_state_sign_ * static_cast<double>(right_count) * kTwoPi / counts_per_wheel_rev_;

    set_state("left_wheel_joint/position", left_position);
    set_state("right_wheel_joint/position", right_position);
    set_state("left_wheel_joint/velocity", left_state_sign_ * rpm_to_rad_s(left_rpm));
    set_state("right_wheel_joint/velocity", right_state_sign_ * rpm_to_rad_s(right_rpm));

    rx_sequence_ = sequence;
    estop_ = estop;
    fault_code_ = fault;
    connected_ = true;
    last_state_ = std::chrono::steady_clock::now();
    return true;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to parse STM32 STATE frame: %s",
      ex.what());
    return false;
  }
}

void ScrobotSystemHardware::publish_status(bool force)
{
  if (!status_pub_)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!force &&
    std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_status_publish_).count() < 100)
  {
    return;
  }
  last_status_publish_ = now;

  bool collector_fresh = false;
  {
    std::lock_guard<std::mutex> lock(collector_mutex_);
    if (last_collector_command_.time_since_epoch().count() != 0)
    {
      collector_fresh =
        std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_collector_command_).count() <= collector_timeout_ms_;
    }
  }

  scrobot_interfaces::msg::HardwareStatus msg;
  msg.connected = connected_;
  msg.estop = estop_;
  msg.fault_code = fault_code_;
  msg.rx_sequence = rx_sequence_;
  msg.left_position_rad = get_state("left_wheel_joint/position");
  msg.right_position_rad = get_state("right_wheel_joint/position");
  msg.left_velocity_rad_s = get_state("left_wheel_joint/velocity");
  msg.right_velocity_rad_s = get_state("right_wheel_joint/velocity");
  msg.collector_command_fresh = collector_fresh;
  status_pub_->publish(msg);
}

double ScrobotSystemHardware::rpm_to_rad_s(double rpm)
{
  return rpm * kTwoPi / 60.0;
}

double ScrobotSystemHardware::rad_s_to_rpm(double rad_s)
{
  return rad_s * 60.0 / kTwoPi;
}

}  // namespace scrobot_hardware

PLUGINLIB_EXPORT_CLASS(
  scrobot_hardware::ScrobotSystemHardware,
  hardware_interface::SystemInterface)
