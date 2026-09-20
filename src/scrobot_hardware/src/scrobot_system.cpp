#include "scrobot_hardware/scrobot_system.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <thread>
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
constexpr uint8_t kProtocolVersion = 2;
constexpr size_t kHeaderLength = 7;
constexpr size_t kCrcLength = 2;
constexpr size_t kMaxPayload = 64;

constexpr uint8_t kTypeSetpoint = 0x10;
constexpr uint8_t kTypeArm = 0x11;
constexpr uint8_t kTypeDisarm = 0x12;
constexpr uint8_t kTypeFeedback = 0x20;
constexpr uint8_t kTypeDiagnostics = 0x21;
constexpr uint8_t kTypeInfoRequest = 0x50;
constexpr uint8_t kTypeInfoResponse = 0x51;
constexpr uint8_t kTypeError = 0x7F;

constexpr uint32_t kStatusArmed = 1U << 0;
constexpr uint32_t kStatusEstop = 1U << 1;
constexpr uint32_t kStatusCommTimeout = 1U << 2;
constexpr uint32_t kStatusSysid = 1U << 3;
constexpr uint32_t kStatusUartError = 1U << 4;
constexpr uint32_t kStatusInvalidOutput = 1U << 5;
constexpr uint32_t kStatusTxDrop = 1U << 6;
constexpr uint32_t kStatusInvalidCommand = 1U << 7;

constexpr double kTwoPi = 6.28318530717958647692;

speed_t baud_to_termios(int baud)
{
  switch (baud)
  {
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
#ifdef B576000
    case 576000:
      return B576000;
#endif
#ifdef B921600
    case 921600:
      return B921600;
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
    counts_per_wheel_rev_ =
      std::stod(get_param("counts_per_wheel_rev", std::to_string(counts_per_wheel_rev_)));

    left_count_sign_ =
      std::stod(get_param("left_count_sign", std::to_string(left_count_sign_)));
    right_count_sign_ =
      std::stod(get_param("right_count_sign", std::to_string(right_count_sign_)));
    left_command_sign_ =
      std::stod(get_param("left_command_sign", std::to_string(left_command_sign_)));
    right_command_sign_ =
      std::stod(get_param("right_command_sign", std::to_string(right_command_sign_)));

    max_wheel_rpm_ = std::stod(get_param("max_wheel_rpm", std::to_string(max_wheel_rpm_)));
    max_brush_rpm_ = std::stod(get_param("max_brush_rpm", std::to_string(max_brush_rpm_)));
    max_conveyor_rpm_ =
      std::stod(get_param("max_conveyor_rpm", std::to_string(max_conveyor_rpm_)));

    feedback_timeout_ms_ =
      std::stoi(get_param("feedback_timeout_ms", std::to_string(feedback_timeout_ms_)));
    collector_timeout_ms_ =
      std::stoi(get_param("collector_timeout_ms", std::to_string(collector_timeout_ms_)));
    handshake_timeout_ms_ =
      std::stoi(get_param("handshake_timeout_ms", std::to_string(handshake_timeout_ms_)));
    arm_timeout_ms_ =
      std::stoi(get_param("arm_timeout_ms", std::to_string(arm_timeout_ms_)));
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
    have_left = have_left || joint.name == "left_wheel_joint";
    have_right = have_right || joint.name == "right_wheel_joint";

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

  reset_protocol_state();

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

  tcflush(serial_fd_, TCIOFLUSH);

  if (!send_info_request())
  {
    close_serial();
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(handshake_timeout_ms_);

  while (std::chrono::steady_clock::now() < deadline && !info_received_)
  {
    if (!poll_serial())
    {
      close_serial();
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (!info_received_)
  {
    RCLCPP_ERROR(
      get_logger(), "STM32 protocol handshake timed out on %s", serial_port_.c_str());
    close_serial();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (protocol_version_ != kProtocolVersion)
  {
    RCLCPP_ERROR(
      get_logger(), "STM32 protocol version %u does not match ROS driver version %u",
      static_cast<unsigned>(protocol_version_), static_cast<unsigned>(kProtocolVersion));
    close_serial();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(),
    "Connected to STM32 firmware %u.%u.%u, protocol v%u on %s at %d baud",
    static_cast<unsigned>(firmware_major_),
    static_cast<unsigned>(firmware_minor_),
    static_cast<unsigned>(firmware_patch_),
    static_cast<unsigned>(protocol_version_),
    serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  collector_sub_.reset();
  status_pub_.reset();
  close_serial();
  reset_protocol_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;

  if (!connected_)
  {
    RCLCPP_ERROR(get_logger(), "Cannot arm: no valid STM32 feedback");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (estop_)
  {
    RCLCPP_ERROR(get_logger(), "Cannot arm: STM32 reports E-stop active");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!send_arm())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(arm_timeout_ms_);

  while (std::chrono::steady_clock::now() < deadline)
  {
    if (!poll_serial())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (estop_)
    {
      RCLCPP_ERROR(get_logger(), "STM32 E-stop became active while arming");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (armed_)
    {
      if (!send_setpoint(0.0, 0.0, 0.0, 0.0, 0.0))
      {
        return hardware_interface::CallbackReturn::ERROR;
      }

      active_ = true;
      RCLCPP_INFO(get_logger(), "STM32 armed; ros2_control hardware active");
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  RCLCPP_ERROR(get_logger(), "Timed out waiting for STM32 ARMED feedback");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn ScrobotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;

  if (serial_fd_ >= 0)
  {
    if (armed_)
    {
      (void)send_setpoint(0.0, 0.0, 0.0, 0.0, 0.0);
    }
    (void)send_disarm();
  }

  RCLCPP_INFO(get_logger(), "STM32 DISARM requested");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ScrobotSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0 || !poll_serial())
  {
    return hardware_interface::return_type::ERROR;
  }

  const auto now = std::chrono::steady_clock::now();
  if (last_feedback_.time_since_epoch().count() != 0)
  {
    const auto age_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feedback_).count();

    if (age_ms > feedback_timeout_ms_)
    {
      if (connected_)
      {
        RCLCPP_ERROR(
          get_logger(), "STM32 FEEDBACK timeout: %ld ms",
          static_cast<long>(age_ms));
      }
      connected_ = false;
    }
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

  if (!active_ || !connected_ || !armed_ || estop_ || comm_timeout_)
  {
    return hardware_interface::return_type::OK;
  }

  double wl_rpm =
    rad_s_to_rpm(get_command("left_wheel_joint/velocity")) * left_command_sign_;
  double wr_rpm =
    rad_s_to_rpm(get_command("right_wheel_joint/velocity")) * right_command_sign_;

  if (!std::isfinite(wl_rpm))
  {
    wl_rpm = 0.0;
  }
  if (!std::isfinite(wr_rpm))
  {
    wr_rpm = 0.0;
  }

  wl_rpm = std::clamp(wl_rpm, -max_wheel_rpm_, max_wheel_rpm_);
  wr_rpm = std::clamp(wr_rpm, -max_wheel_rpm_, max_wheel_rpm_);

  double br_rpm = 0.0;
  double bl_rpm = 0.0;
  double cv_rpm = 0.0;

  {
    std::lock_guard<std::mutex> lock(collector_mutex_);
    bool fresh = false;

    if (last_collector_command_.time_since_epoch().count() != 0)
    {
      const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_collector_command_).count();
      fresh = age_ms <= collector_timeout_ms_;
    }

    if (fresh && collector_command_.enable)
    {
      br_rpm = std::clamp(
        collector_command_.brush_right_rpm, -max_brush_rpm_, max_brush_rpm_);
      bl_rpm = std::clamp(
        collector_command_.brush_left_rpm, -max_brush_rpm_, max_brush_rpm_);
      cv_rpm = std::clamp(
        collector_command_.conveyor_rpm, -max_conveyor_rpm_, max_conveyor_rpm_);
    }
  }

  if (!send_setpoint(wr_rpm, wl_rpm, br_rpm, bl_rpm, cv_rpm))
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool ScrobotSystemHardware::open_serial()
{
  close_serial();

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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

bool ScrobotSystemHardware::send_packet(
  uint8_t type, const std::vector<uint8_t> & payload)
{
  if (serial_fd_ < 0 || payload.size() > kMaxPayload)
  {
    return false;
  }

  std::vector<uint8_t> frame;
  frame.reserve(kHeaderLength + payload.size() + kCrcLength);

  frame.push_back(kSof1);
  frame.push_back(kSof2);
  frame.push_back(kProtocolVersion);
  frame.push_back(type);

  const uint16_t sequence = tx_sequence_++;
  frame.push_back(static_cast<uint8_t>(sequence & 0xFFU));
  frame.push_back(static_cast<uint8_t>((sequence >> 8) & 0xFFU));
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  const uint16_t crc = crc16_ccitt_false(&frame[2], 5 + payload.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));

  size_t written = 0;
  while (written < frame.size())
  {
    const ssize_t count =
      ::write(serial_fd_, frame.data() + written, frame.size() - written);

    if (count > 0)
    {
      written += static_cast<size_t>(count);
      continue;
    }

    if (count < 0 && errno == EINTR)
    {
      continue;
    }

    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }

    RCLCPP_ERROR(get_logger(), "Serial write failed: %s", std::strerror(errno));
    return false;
  }

  return true;
}

bool ScrobotSystemHardware::send_setpoint(
  double wr_rpm,
  double wl_rpm,
  double br_rpm,
  double bl_rpm,
  double cv_rpm)
{
  std::vector<uint8_t> payload;
  payload.reserve(20);
  append_float_le(payload, static_cast<float>(wr_rpm));
  append_float_le(payload, static_cast<float>(wl_rpm));
  append_float_le(payload, static_cast<float>(br_rpm));
  append_float_le(payload, static_cast<float>(bl_rpm));
  append_float_le(payload, static_cast<float>(cv_rpm));
  return send_packet(kTypeSetpoint, payload);
}

bool ScrobotSystemHardware::send_arm()
{
  return send_packet(kTypeArm);
}

bool ScrobotSystemHardware::send_disarm()
{
  return send_packet(kTypeDisarm);
}

bool ScrobotSystemHardware::send_info_request()
{
  return send_packet(kTypeInfoRequest);
}

bool ScrobotSystemHardware::poll_serial()
{
  char buffer[512];

  while (true)
  {
    const ssize_t count = ::read(serial_fd_, buffer, sizeof(buffer));

    if (count > 0)
    {
      rx_buffer_.insert(
        rx_buffer_.end(),
        reinterpret_cast<uint8_t *>(buffer),
        reinterpret_cast<uint8_t *>(buffer) + count);
      continue;
    }

    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK)
    {
      break;
    }

    if (errno == EINTR)
    {
      continue;
    }

    RCLCPP_ERROR(get_logger(), "Serial read failed: %s", std::strerror(errno));
    connected_ = false;
    return false;
  }

  parse_rx_buffer();
  return true;
}

void ScrobotSystemHardware::parse_rx_buffer()
{
  while (true)
  {
    size_t sof_index = rx_buffer_.size();
    for (size_t i = 0; i + 1 < rx_buffer_.size(); ++i)
    {
      if (rx_buffer_[i] == kSof1 && rx_buffer_[i + 1] == kSof2)
      {
        sof_index = i;
        break;
      }
    }

    if (sof_index == rx_buffer_.size())
    {
      if (!rx_buffer_.empty() && rx_buffer_.back() == kSof1)
      {
        const uint8_t last = rx_buffer_.back();
        rx_buffer_.clear();
        rx_buffer_.push_back(last);
      }
      else
      {
        rx_buffer_.clear();
      }
      return;
    }

    if (sof_index != 0)
    {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sof_index);
    }

    if (rx_buffer_.size() < kHeaderLength)
    {
      return;
    }

    const uint8_t version = rx_buffer_[2];
    const uint8_t type = rx_buffer_[3];
    const uint16_t sequence = read_u16_le(&rx_buffer_[4]);
    const uint8_t payload_len = rx_buffer_[6];

    if (payload_len > kMaxPayload)
    {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const size_t total_len = kHeaderLength + payload_len + kCrcLength;
    if (rx_buffer_.size() < total_len)
    {
      return;
    }

    const uint16_t expected_crc = read_u16_le(&rx_buffer_[7 + payload_len]);
    const uint16_t actual_crc = crc16_ccitt_false(&rx_buffer_[2], 5 + payload_len);

    if (expected_crc != actual_crc)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "CRC mismatch from STM32; resynchronizing");
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    handle_frame(version, type, sequence, &rx_buffer_[7], payload_len);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_len);
  }
}

void ScrobotSystemHardware::handle_frame(
  uint8_t version,
  uint8_t type,
  uint16_t sequence,
  const uint8_t * payload,
  uint8_t payload_len)
{
  if (version != kProtocolVersion)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received protocol version %u, expected %u",
      static_cast<unsigned>(version), static_cast<unsigned>(kProtocolVersion));
    return;
  }

  switch (type)
  {
    case kTypeFeedback:
      handle_feedback(sequence, payload, payload_len);
      break;
    case kTypeInfoResponse:
      handle_info_response(payload, payload_len);
      break;
    case kTypeDiagnostics:
      break;
    case kTypeError:
      handle_error_frame(payload, payload_len);
      break;
    default:
      break;
  }
}

void ScrobotSystemHardware::handle_feedback(
  uint16_t sequence,
  const uint8_t * payload,
  uint8_t payload_len)
{
  if (payload_len != 50)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Unexpected FEEDBACK payload length %u", static_cast<unsigned>(payload_len));
    return;
  }

  control_tick_ = read_u32_le(&payload[0]);
  status_flags_ = read_u32_le(&payload[4]);
  last_setpoint_seq_ = read_u16_le(&payload[8]);

  const int32_t wr_count = read_i32_le(&payload[10]);
  const int32_t wl_count = read_i32_le(&payload[14]);

  const float wr_rpm = read_float_le(&payload[30]);
  const float wl_rpm = read_float_le(&payload[34]);
  feedback_br_rpm_ = read_float_le(&payload[38]);
  feedback_bl_rpm_ = read_float_le(&payload[42]);
  feedback_cv_rpm_ = read_float_le(&payload[46]);

  if (!wheel_counts_initialized_)
  {
    previous_wr_count_ = wr_count;
    previous_wl_count_ = wl_count;
    wheel_counts_initialized_ = true;
  }
  else
  {
    accumulated_wr_count_ +=
      static_cast<int64_t>(right_count_sign_ * wrapped_count_delta(wr_count, previous_wr_count_));
    accumulated_wl_count_ +=
      static_cast<int64_t>(left_count_sign_ * wrapped_count_delta(wl_count, previous_wl_count_));
    previous_wr_count_ = wr_count;
    previous_wl_count_ = wl_count;
  }

  set_state(
    "left_wheel_joint/position",
    static_cast<double>(accumulated_wl_count_) * kTwoPi / counts_per_wheel_rev_);
  set_state(
    "right_wheel_joint/position",
    static_cast<double>(accumulated_wr_count_) * kTwoPi / counts_per_wheel_rev_);

  // STM32 motor->rpm is already corrected using APP_ENCODER_SIGN_*.
  set_state("left_wheel_joint/velocity", rpm_to_rad_s(wl_rpm));
  set_state("right_wheel_joint/velocity", rpm_to_rad_s(wr_rpm));

  armed_ = (status_flags_ & kStatusArmed) != 0;
  estop_ = (status_flags_ & kStatusEstop) != 0;
  comm_timeout_ = (status_flags_ & kStatusCommTimeout) != 0;
  sysid_ = (status_flags_ & kStatusSysid) != 0;
  uart_error_ = (status_flags_ & kStatusUartError) != 0;
  invalid_output_ = (status_flags_ & kStatusInvalidOutput) != 0;
  tx_queue_drop_ = (status_flags_ & kStatusTxDrop) != 0;
  invalid_command_ = (status_flags_ & kStatusInvalidCommand) != 0;

  feedback_sequence_ = sequence;
  connected_ = true;
  last_feedback_ = std::chrono::steady_clock::now();
}

void ScrobotSystemHardware::handle_info_response(
  const uint8_t * payload, uint8_t payload_len)
{
  if (payload_len != 4)
  {
    return;
  }

  firmware_major_ = payload[0];
  firmware_minor_ = payload[1];
  firmware_patch_ = payload[2];
  protocol_version_ = payload[3];
  info_received_ = true;
}

void ScrobotSystemHardware::handle_error_frame(
  const uint8_t * payload, uint8_t payload_len)
{
  if (payload_len != 2)
  {
    return;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "STM32 rejected request type 0x%02X with error code %u",
    static_cast<unsigned>(payload[0]), static_cast<unsigned>(payload[1]));
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
  msg.armed = armed_;
  msg.estop = estop_;
  msg.comm_timeout = comm_timeout_;
  msg.sysid = sysid_;
  msg.uart_error = uart_error_;
  msg.invalid_output = invalid_output_;
  msg.tx_queue_drop = tx_queue_drop_;
  msg.invalid_command = invalid_command_;

  msg.status_flags = status_flags_;
  msg.control_tick = control_tick_;
  msg.last_setpoint_seq = last_setpoint_seq_;
  msg.feedback_sequence = feedback_sequence_;

  msg.firmware_major = firmware_major_;
  msg.firmware_minor = firmware_minor_;
  msg.firmware_patch = firmware_patch_;
  msg.protocol_version = protocol_version_;

  msg.left_position_rad = get_state("left_wheel_joint/position");
  msg.right_position_rad = get_state("right_wheel_joint/position");
  msg.left_velocity_rad_s = get_state("left_wheel_joint/velocity");
  msg.right_velocity_rad_s = get_state("right_wheel_joint/velocity");

  msg.brush_right_rpm = feedback_br_rpm_;
  msg.brush_left_rpm = feedback_bl_rpm_;
  msg.conveyor_rpm = feedback_cv_rpm_;
  msg.collector_command_fresh = collector_fresh;

  status_pub_->publish(msg);
}

void ScrobotSystemHardware::reset_protocol_state()
{
  active_ = false;
  connected_ = false;
  armed_ = false;
  estop_ = false;
  comm_timeout_ = true;
  sysid_ = false;
  uart_error_ = false;
  invalid_output_ = false;
  tx_queue_drop_ = false;
  invalid_command_ = false;

  status_flags_ = 0;
  control_tick_ = 0;
  last_setpoint_seq_ = 0;
  feedback_sequence_ = 0;
  tx_sequence_ = 0;

  firmware_major_ = 0;
  firmware_minor_ = 0;
  firmware_patch_ = 0;
  protocol_version_ = 0;
  info_received_ = false;

  wheel_counts_initialized_ = false;
  previous_wr_count_ = 0;
  previous_wl_count_ = 0;
  accumulated_wr_count_ = 0;
  accumulated_wl_count_ = 0;

  feedback_br_rpm_ = 0.0;
  feedback_bl_rpm_ = 0.0;
  feedback_cv_rpm_ = 0.0;

  rx_buffer_.clear();
  last_feedback_ = {};
  last_status_publish_ = std::chrono::steady_clock::now();
}

uint16_t ScrobotSystemHardware::crc16_ccitt_false(
  const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFFU;

  for (size_t i = 0; i < len; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8;

    for (int bit = 0; bit < 8; ++bit)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

uint16_t ScrobotSystemHardware::read_u16_le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t ScrobotSystemHardware::read_u32_le(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int32_t ScrobotSystemHardware::read_i32_le(const uint8_t * data)
{
  return static_cast<int32_t>(read_u32_le(data));
}

float ScrobotSystemHardware::read_float_le(const uint8_t * data)
{
  const uint32_t bits = read_u32_le(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void ScrobotSystemHardware::append_float_le(
  std::vector<uint8_t> & payload, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));

  payload.push_back(static_cast<uint8_t>(bits & 0xFFU));
  payload.push_back(static_cast<uint8_t>((bits >> 8) & 0xFFU));
  payload.push_back(static_cast<uint8_t>((bits >> 16) & 0xFFU));
  payload.push_back(static_cast<uint8_t>((bits >> 24) & 0xFFU));
}

int64_t ScrobotSystemHardware::wrapped_count_delta(
  int32_t current, int32_t previous)
{
  const uint32_t delta =
    static_cast<uint32_t>(current) - static_cast<uint32_t>(previous);
  return static_cast<int64_t>(static_cast<int32_t>(delta));
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
