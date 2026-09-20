#include "scrobot_hardware/scrobot_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace scrobot_hardware
{
namespace
{
constexpr uint8_t kSof1 = 0xAA;
constexpr uint8_t kSof2 = 0x55;

constexpr uint8_t kTypeRpmEstop = 0x00;
constexpr uint8_t kTypeRpmNormal = 0x01;

constexpr uint8_t kTypeDriveRef = 0xA0;
constexpr uint8_t kTypeAuxRef = 0xA1;

constexpr uint8_t kTypePidWrEcho = 0xCA;
constexpr uint8_t kTypePidWlEcho = 0xCB;
constexpr uint8_t kTypePidBrEcho = 0xC0;
constexpr uint8_t kTypePidBlEcho = 0xC1;
constexpr uint8_t kTypePidCvEcho = 0xC2;

constexpr uint8_t kTypeTuningRpm = 0xF2;

constexpr size_t kDriveFrameLength = 12;
constexpr size_t kAuxFrameLength = 16;
constexpr size_t kPidEchoFrameLength = 20;
constexpr size_t kNormalRpmFrameLength = 24;
constexpr size_t kTuningFrameLength = 26;

constexpr double kTwoPi = 6.28318530717958647692;

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
#ifdef B500000
    case 500000:
      return B500000;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
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
  rx_frame_count_ = 0;
  rx_buffer_.clear();
  rx_buffer_.reserve(256);
  last_state_ = std::chrono::steady_clock::now();
  last_status_publish_ = last_state_;

  RCLCPP_INFO(
    get_logger(),
    "Configured STM32 USART6 protocol on %s at %d baud",
    serial_port_.c_str(), baud_rate_);

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

  // Send zero normal-mode commands before accepting motion.
  send_drive_frame(0.0, 0.0);
  send_aux_frame(0.0, 0.0, 0.0);

  RCLCPP_INFO(get_logger(), "SC Robot hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;

  // Explicit zero frames are followed by the STM32's own 200 ms A0 watchdog.
  send_drive_frame(0.0, 0.0);
  send_aux_frame(0.0, 0.0, 0.0);

  RCLCPP_INFO(get_logger(), "SC Robot hardware deactivated; zero references sent");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ScrobotSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  uint8_t buffer[256];
  while (true)
  {
    const ssize_t count = ::read(serial_fd_, buffer, sizeof(buffer));
    if (count > 0)
    {
      rx_buffer_.insert(rx_buffer_.end(), buffer, buffer + count);
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

  process_rx_buffer();

  // STM32 normal telemetry contains measured RPM, not cumulative encoder counts.
  // ros2_control's diff_drive_controller is configured for position feedback, so
  // integrate the measured wheel velocity here to expose wheel position.
  if (connected_)
  {
    const double dt = std::max(0.0, period.seconds());
    set_state(
      "left_wheel_joint/position",
      get_state("left_wheel_joint/position") +
      get_state("left_wheel_joint/velocity") * dt);
    set_state(
      "right_wheel_joint/position",
      get_state("right_wheel_joint/position") +
      get_state("right_wheel_joint/velocity") * dt);
  }

  const auto now = std::chrono::steady_clock::now();
  const auto age_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_state_).count();

  if (active_ && age_ms > state_timeout_ms_)
  {
    if (connected_)
    {
      RCLCPP_ERROR(
        get_logger(), "STM32 feedback timeout: no valid 0x00/0x01 RPM frame for %ld ms",
        static_cast<long>(age_ms));
    }
    connected_ = false;
    set_state("left_wheel_joint/velocity", 0.0);
    set_state("right_wheel_joint/velocity", 0.0);
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

  double left_rpm = rad_s_to_rpm(get_command("left_wheel_joint/velocity"));
  double right_rpm = rad_s_to_rpm(get_command("right_wheel_joint/velocity"));

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
    }
  }

  if (!active_ || estop_)
  {
    left_rpm = 0.0;
    right_rpm = 0.0;
    brush_left_rpm = 0.0;
    brush_right_rpm = 0.0;
    conveyor_rpm = 0.0;
  }

  // A0 is the STM32's normal-mode heartbeat. Its payload order is WR, WL.
  if (!send_drive_frame(right_rpm, left_rpm))
  {
    return hardware_interface::return_type::ERROR;
  }

  // A1 payload order is BR, BL, CV. Send it every cycle, including zeros, so
  // a stale collector command cannot leave the auxiliary motors running.
  if (!send_aux_frame(brush_right_rpm, brush_left_rpm, conveyor_rpm))
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

bool ScrobotSystemHardware::send_drive_frame(double right_rpm, double left_rpm)
{
  uint8_t frame[kDriveFrameLength] = {};
  frame[0] = kSof1;
  frame[1] = kSof2;
  frame[2] = kTypeDriveRef;
  write_float_le(&frame[3], static_cast<float>(right_rpm));
  write_float_le(&frame[7], static_cast<float>(left_rpm));
  frame[kDriveFrameLength - 1] = crc8(&frame[2], kDriveFrameLength - 3);
  return write_all(frame, sizeof(frame));
}

bool ScrobotSystemHardware::send_aux_frame(
  double brush_right_rpm, double brush_left_rpm, double conveyor_rpm)
{
  uint8_t frame[kAuxFrameLength] = {};
  frame[0] = kSof1;
  frame[1] = kSof2;
  frame[2] = kTypeAuxRef;
  write_float_le(&frame[3], static_cast<float>(brush_right_rpm));
  write_float_le(&frame[7], static_cast<float>(brush_left_rpm));
  write_float_le(&frame[11], static_cast<float>(conveyor_rpm));
  frame[kAuxFrameLength - 1] = crc8(&frame[2], kAuxFrameLength - 3);
  return write_all(frame, sizeof(frame));
}

bool ScrobotSystemHardware::write_all(const uint8_t * data, size_t size)
{
  size_t sent = 0;
  while (sent < size)
  {
    const ssize_t count = ::write(serial_fd_, data + sent, size - sent);
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

void ScrobotSystemHardware::process_rx_buffer()
{
  while (rx_buffer_.size() >= 4)
  {
    size_t sof = 0;
    while (sof + 1 < rx_buffer_.size() &&
      !(rx_buffer_[sof] == kSof1 && rx_buffer_[sof + 1] == kSof2))
    {
      ++sof;
    }

    if (sof > 0)
    {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(sof));
    }

    if (rx_buffer_.size() < 4)
    {
      return;
    }

    if (rx_buffer_[0] != kSof1 || rx_buffer_[1] != kSof2)
    {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const size_t frame_length = expected_frame_length(rx_buffer_[2]);
    if (frame_length == 0)
    {
      // Unknown TYPE: drop one byte and search for the next SOF.
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    if (rx_buffer_.size() < frame_length)
    {
      return;
    }

    const uint8_t expected_crc = crc8(&rx_buffer_[2], frame_length - 3);
    const uint8_t received_crc = rx_buffer_[frame_length - 1];
    if (expected_crc != received_crc)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "STM32 CRC mismatch (type 0x%02X)", rx_buffer_[2]);
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    process_frame(rx_buffer_.data(), frame_length);
    rx_buffer_.erase(
      rx_buffer_.begin(),
      rx_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length));
  }

  if (rx_buffer_.size() > 1024)
  {
    RCLCPP_WARN(get_logger(), "Discarding oversized STM32 receive buffer");
    rx_buffer_.clear();
  }
}

bool ScrobotSystemHardware::process_frame(const uint8_t * frame, size_t size)
{
  const uint8_t type = frame[2];

  if ((type == kTypeRpmNormal || type == kTypeRpmEstop) &&
    size == kNormalRpmFrameLength)
  {
    // STM32 telemetry order: WR, WL, BR, BL, CV measured RPM.
    const double right_rpm = static_cast<double>(read_float_le(&frame[3]));
    const double left_rpm = static_cast<double>(read_float_le(&frame[7]));

    if (!std::isfinite(right_rpm) || !std::isfinite(left_rpm))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-finite STM32 RPM telemetry");
      return false;
    }

    set_state("right_wheel_joint/velocity", rpm_to_rad_s(right_rpm));
    set_state("left_wheel_joint/velocity", rpm_to_rad_s(left_rpm));

    estop_ = type == kTypeRpmEstop;
    connected_ = true;
    ++rx_frame_count_;
    last_state_ = std::chrono::steady_clock::now();
    return true;
  }

  // PID echo (C*) and synchronized tuning telemetry (F2) belong to the tuning
  // tools, not normal ros2_control operation. Their lengths are recognized by
  // the stream parser so they can be skipped without losing framing.
  return false;
}

uint8_t ScrobotSystemHardware::crc8(const uint8_t * data, size_t size)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x80U) != 0U ?
        static_cast<uint8_t>((crc << 1U) ^ 0x07U) :
        static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

float ScrobotSystemHardware::read_float_le(const uint8_t * data)
{
  uint32_t raw =
    static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8U) |
    (static_cast<uint32_t>(data[2]) << 16U) |
    (static_cast<uint32_t>(data[3]) << 24U);

  float value = 0.0f;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void ScrobotSystemHardware::write_float_le(uint8_t * data, float value)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));

  data[0] = static_cast<uint8_t>(raw & 0xFFU);
  data[1] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
  data[2] = static_cast<uint8_t>((raw >> 16U) & 0xFFU);
  data[3] = static_cast<uint8_t>((raw >> 24U) & 0xFFU);
}

size_t ScrobotSystemHardware::expected_frame_length(uint8_t type)
{
  switch (type)
  {
    case kTypeRpmNormal:
    case kTypeRpmEstop:
      return kNormalRpmFrameLength;

    case kTypePidWrEcho:
    case kTypePidWlEcho:
    case kTypePidBrEcho:
    case kTypePidBlEcho:
    case kTypePidCvEcho:
      return kPidEchoFrameLength;

    case kTypeTuningRpm:
      return kTuningFrameLength;

    default:
      return 0;
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
  msg.rx_frame_count = rx_frame_count_;
  msg.left_position_rad = get_state("left_wheel_joint/position");
  msg.right_position_rad = get_state("right_wheel_joint/position");
  msg.left_velocity_rad_s = get_state("left_wheel_joint/velocity");
  msg.right_velocity_rad_s = get_state("right_wheel_joint/velocity");
  msg.collector_command_fresh = collector_fresh;
  status_pub_->publish(msg);
}

uint8_t ScrobotSystemHardware::crc8(const uint8_t * data, size_t size);

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
